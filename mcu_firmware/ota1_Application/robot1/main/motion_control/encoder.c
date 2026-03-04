#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "Encoder";

// Array to hold all encoder instances
static encoder_t encoders[NUM_ENCODERS];

// GPIO pin configuration arrays
static const int phase_a_pins[NUM_ENCODERS] = {
    ENCODER_1_PHASE_A_GPIO,
    ENCODER_2_PHASE_A_GPIO,
    ENCODER_3_PHASE_A_GPIO,
    ENCODER_4_PHASE_A_GPIO};

static const int phase_b_pins[NUM_ENCODERS] = {
    ENCODER_1_PHASE_B_GPIO,
    ENCODER_2_PHASE_B_GPIO,
    ENCODER_3_PHASE_B_GPIO,
    ENCODER_4_PHASE_B_GPIO};

esp_err_t encoder_init(void)
{
    esp_err_t ret = ESP_OK;

    // Initialize all encoders
    for (int i = 0; i < NUM_ENCODERS; i++)
    {
        // Clear encoder structure
        memset(&encoders[i], 0, sizeof(encoder_t));

        encoders[i].phase_a_gpio = phase_a_pins[i];
        encoders[i].phase_b_gpio = phase_b_pins[i];

        // PCNT unit configuration
        pcnt_unit_config_t unit_config = {
            .high_limit = 32767,
            .low_limit = -32768,
            .flags.accum_count = 1, // Enable accumulation to handle overflow automatically
        };

        ret = pcnt_new_unit(&unit_config, &encoders[i].pcnt_unit);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to create PCNT unit %d: %s", i, esp_err_to_name(ret));
            return ret;
        }

        // Configure glitch filter (optional, helps with noisy signals)
        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = 1000, // 1us glitch filter
        };
        ret = pcnt_unit_set_glitch_filter(encoders[i].pcnt_unit, &filter_config);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set glitch filter for encoder %d: %s", i, esp_err_to_name(ret));
        }

        // Configure channel 0 (Phase A as signal, Phase B as control)
        pcnt_chan_config_t chan_a_config = {
            .edge_gpio_num = encoders[i].phase_a_gpio,
            .level_gpio_num = encoders[i].phase_b_gpio,
        };
        pcnt_channel_handle_t pcnt_chan_a = NULL;
        ret = pcnt_new_channel(encoders[i].pcnt_unit, &chan_a_config, &pcnt_chan_a);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to create PCNT channel A for encoder %d: %s", i, esp_err_to_name(ret));
            return ret;
        }

        // Configure channel 1 (Phase B as signal, Phase A as control)
        pcnt_chan_config_t chan_b_config = {
            .edge_gpio_num = encoders[i].phase_b_gpio,
            .level_gpio_num = encoders[i].phase_a_gpio,
        };
        pcnt_channel_handle_t pcnt_chan_b = NULL;
        ret = pcnt_new_channel(encoders[i].pcnt_unit, &chan_b_config, &pcnt_chan_b);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to create PCNT channel B for encoder %d: %s", i, esp_err_to_name(ret));
            return ret;
        }

        // Set edge and level actions for quadrature decoding
        // Channel A: Increment on positive edge when B is low, decrement when B is high
        // pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        // pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        // // Channel B: Increment on positive edge when A is high, decrement when A is low
        // pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        // pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        // Enable and clear the unit
        ret = pcnt_unit_enable(encoders[i].pcnt_unit);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to enable PCNT unit %d: %s", i, esp_err_to_name(ret));
            return ret;
        }

        ret = pcnt_unit_clear_count(encoders[i].pcnt_unit);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to clear PCNT unit %d: %s", i, esp_err_to_name(ret));
            return ret;
        }

        ret = pcnt_unit_start(encoders[i].pcnt_unit);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start PCNT unit %d: %s", i, esp_err_to_name(ret));
            return ret;
        }

        encoders[i].last_update_time = esp_timer_get_time();

        ESP_LOGI(TAG, "Encoder %d initialized (Phase A: GPIO%d, Phase B: GPIO%d)",
                 i, encoders[i].phase_a_gpio, encoders[i].phase_b_gpio);
    }

    ESP_LOGI(TAG, "All %d encoders initialized successfully", NUM_ENCODERS);
    return ESP_OK;
}

esp_err_t encoder_update(void)
{
    int64_t current_time = esp_timer_get_time();
    esp_err_t ret = ESP_OK;

    for (int i = 0; i < NUM_ENCODERS; i++)
    {
        // Read current pulse count
        int pulse_count;
        ret = pcnt_unit_get_count(encoders[i].pcnt_unit, &pulse_count);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to get count for encoder %d: %s", i, esp_err_to_name(ret));
            continue;
        }

        // Calculate time delta in seconds
        float dt = (current_time - encoders[i].last_update_time) / 1000000.0f;

        if (dt > 0.0f)
        {
            // Calculate delta pulses (handles overflow automatically)
            int32_t delta_pulses = pulse_count - encoders[i].prev_pulse_count;

            // Detect and correct overflow/underflow
            // If delta is too large (> half range), it's likely an overflow
            if (delta_pulses > 16384) // More than half of 32767
            {
                delta_pulses -= 65536; // Correct overflow
            }
            else if (delta_pulses < -16384) // Less than half of -32768
            {
                delta_pulses += 65536; // Correct underflow
            }

            // Calculate raw velocity
            float raw_velocity = (delta_pulses / PULSES_PER_RADIAN) / dt;

            // Glitch filter: Reject physically impossible changes
            // Max acceleration of DC motor is typically ~1000 rad/s^2
            // At 100Hz (dt=0.01s), max delta_v = 10 rad/s (~95 RPM)
            float delta_velocity = raw_velocity - encoders[i].velocity;

            if (fabs(delta_velocity) < 15.0f) // 15 rad/s threshold (~143 RPM jump)
            {
                // Valid reading - apply EMA (Exponential Moving Average) filter
                // Alpha = 0.3 (aggressive smoothing to reduce ±3 RPM oscillation)
                float alpha = 0.3f;
                encoders[i].velocity = alpha * raw_velocity + (1.0f - alpha) * encoders[i].velocity;

                // Update accumulated pulse count
                encoders[i].pulse_count += delta_pulses;

                // Update position (radians)
                encoders[i].position += (delta_pulses / PULSES_PER_RADIAN);
            }
            else
            {
                // Glitch detected - keep previous velocity, don't update position
                // ESP_LOGW(TAG, "Encoder %d glitch: %.2f -> %.2f rad/s (delta: %.2f)",
                //          i, encoders[i].velocity, raw_velocity, delta_velocity);
            }
        }

        // Update previous values
        encoders[i].prev_pulse_count = pulse_count;
        encoders[i].last_update_time = current_time;
    }

    return ESP_OK;
}

float get_encoder_velocity(uint8_t index)
{
    if (index >= NUM_ENCODERS)
    {
        ESP_LOGE(TAG, "Invalid encoder index: %d", index);
        return 0.0f;
    }

    return encoders[index].velocity;
}

float get_encoder_position(uint8_t index)
{
    if (index >= NUM_ENCODERS)
    {
        ESP_LOGE(TAG, "Invalid encoder index: %d", index);
        return 0.0f;
    }

    return encoders[index].position;
}

esp_err_t encoder_reset(uint8_t index)
{
    if (index >= NUM_ENCODERS)
    {
        ESP_LOGE(TAG, "Invalid encoder index: %d", index);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = pcnt_unit_clear_count(encoders[index].pcnt_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to clear encoder %d: %s", index, esp_err_to_name(ret));
        return ret;
    }

    encoders[index].pulse_count = 0;
    encoders[index].prev_pulse_count = 0;
    encoders[index].position = 0.0f;
    encoders[index].velocity = 0.0f;
    encoders[index].last_update_time = esp_timer_get_time();

    ESP_LOGI(TAG, "Encoder %d reset", index);
    return ESP_OK;
}

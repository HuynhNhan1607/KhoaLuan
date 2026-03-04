#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "bno055.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LPF.h"

static const char *TAG_LPF = "LPF";

bool lpf_init(lpf_filter_t *filter, int order, float *a_coeff, float *b_coeff)
{
    if (!filter || order < 1 || order > LPF_MAX_ORDER || !a_coeff || !b_coeff)
    {
        ESP_LOGE(TAG_LPF, "Invalid filter parameters");
        return false;
    }

    filter->order = order;
    filter->initialized = false;

    // Copy coefficients
    for (int i = 0; i <= order; i++)
    {
        if (i > 0) // a[0] is assumed to be 1.0
        {
            filter->a[i] = a_coeff[i];
        }
        filter->b[i] = b_coeff[i];
    }

    // Initialize history buffers
    for (int i = 0; i <= order; i++)
    {
        filter->input_history[i].x = 0.0f;
        filter->input_history[i].y = 0.0f;
        filter->input_history[i].z = 0.0f;

        filter->output_history[i].x = 0.0f;
        filter->output_history[i].y = 0.0f;
        filter->output_history[i].z = 0.0f;
    }

    return true;
}

void lpf_apply_vec3(lpf_filter_t *filter, bno055_vec3_t *vec)
{
    if (!filter || !vec)
    {
        return;
    }

    if (!filter->initialized)
    {
        // Initialize history with current value
        for (int i = 0; i <= filter->order; i++)
        {
            filter->input_history[i] = *vec;
            filter->output_history[i] = *vec;
        }
        filter->initialized = true;
        return;
    }

    // Shift input history
    for (int i = filter->order; i > 0; i--)
    {
        filter->input_history[i] = filter->input_history[i - 1];
    }
    filter->input_history[0] = *vec;

    // Apply filter for each axis
    bno055_vec3_t filtered = {0.0f, 0.0f, 0.0f};

    // X-axis filtering
    for (int i = 0; i <= filter->order; i++)
    {
        filtered.x += filter->b[i] * filter->input_history[i].x;
        if (i > 0)
        {
            filtered.x -= filter->a[i] * filter->output_history[i - 1].x;
        }
    }

    // Y-axis filtering
    for (int i = 0; i <= filter->order; i++)
    {
        filtered.y += filter->b[i] * filter->input_history[i].y;
        if (i > 0)
        {
            filtered.y -= filter->a[i] * filter->output_history[i - 1].y;
        }
    }

    // Z-axis filtering
    for (int i = 0; i <= filter->order; i++)
    {
        filtered.z += filter->b[i] * filter->input_history[i].z;
        if (i > 0)
        {
            filtered.z -= filter->a[i] * filter->output_history[i - 1].z;
        }
    }

    // Shift output history
    for (int i = filter->order; i > 0; i--)
    {
        filter->output_history[i] = filter->output_history[i - 1];
    }
    filter->output_history[0] = filtered;

    // Return filtered value
    *vec = filtered;
}

void lpf_reset(lpf_filter_t *filter)
{
    if (filter)
    {
        for (int i = 0; i <= filter->order; i++)
        {
            filter->input_history[i].x = 0.0f;
            filter->input_history[i].y = 0.0f;
            filter->input_history[i].z = 0.0f;

            filter->output_history[i].x = 0.0f;
            filter->output_history[i].y = 0.0f;
            filter->output_history[i].z = 0.0f;
        }
        filter->initialized = false;
    }
}

// Helper function to create a second-order filter with standard coefficients
bool lpf_init_default_second_order(lpf_filter_t *filter)
{
    float a_coeff[3] = {1.0f, -1.928942f, 0.931382f}; // Note: a0 is 1.0, a1 is negated from display format
    float b_coeff[3] = {0.000610f, 0.001220f, 0.000610f};

    return lpf_init(filter, 2, a_coeff, b_coeff);
}
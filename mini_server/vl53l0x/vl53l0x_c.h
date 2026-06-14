/**
 * @file vl53l0x_c.h
 * @brief VL53L0X ToF sensor driver for Linux/Jetson (Pure C version)
 *
 * Converted from the C++ JetsonHacks VL53L0X library.
 * Most of the functionality is based on the VL53L0X API provided by ST
 * (STSW-IMG005).
 */

#ifndef VL53L0X_C_H
#define VL53L0X_C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Register addresses (from API vl53l0x_device.h) ────────────────────── */
#define VL53L0X_REG_SYSRANGE_START                              0x00
#define VL53L0X_REG_SYSTEM_THRESH_HIGH                          0x0C
#define VL53L0X_REG_SYSTEM_THRESH_LOW                           0x0E
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG                      0x01
#define VL53L0X_REG_SYSTEM_RANGE_CONFIG                         0x09
#define VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD              0x04
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH                    0x84
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR                      0x0B
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS                     0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS                         0x14
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       0xBC
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        0xC0
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       0xD0
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF        0xD4
#define VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF                 0xB6
#define VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM           0x28
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS                    0x8A
#define VL53L0X_REG_MSRC_CONFIG_CONTROL                         0x60
#define VL53L0X_REG_PRE_RANGE_CONFIG_MIN_SNR                    0x27
#define VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56
#define VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57
#define VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          0x64
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_SNR                  0x67
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW          0x47
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         0x48
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI            0x61
#define VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO            0x62
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x52
#define VL53L0X_REG_SYSTEM_HISTOGRAM_BIN                        0x81
#define VL53L0X_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       0x33
#define VL53L0X_REG_HISTOGRAM_CONFIG_READOUT_CTRL               0x55
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        0x72
#define VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20
#define VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP                  0x46
#define VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N                 0xBF
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID                     0xC0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID                  0xC2
#define VL53L0X_REG_OSC_CALIBRATE_VAL                           0xF8
#define VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH                   0x32
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1            0xB1
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2            0xB2
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4            0xB4
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5            0xB5
#define VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE            0x80
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89
#define VL53L0X_REG_ALGO_PHASECAL_LIM                           0x30
#define VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT                0x30

/* ── VCSEL period type ─────────────────────────────────────────────────── */
typedef enum {
    VL53L0X_VCSEL_PERIOD_PRE_RANGE,
    VL53L0X_VCSEL_PERIOD_FINAL_RANGE
} VL53L0X_VcselPeriodType;

/* ── Internal helper structs ───────────────────────────────────────────── */
typedef struct {
    bool tcc;
    bool msrc;
    bool dss;
    bool pre_range;
    bool final_range;
} VL53L0X_SequenceStepEnables;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
    uint16_t final_range_mclks;
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_us;
    uint32_t final_range_us;
} VL53L0X_SequenceStepTimeouts;

/* ── Main device struct (replaces the C++ class) ───────────────────────── */
typedef struct {
    /* Public fields */
    uint8_t  last_status;            /* status of last I2C transmission */
    uint8_t  i2c_bus;                /* I2C bus number (e.g. 1 for /dev/i2c-1) */
    int      i2c_fd;                 /* file descriptor for the I2C device */
    int      error;                  /* errno from last failed operation */

    /* Private fields (used internally) */
    uint8_t  address;                /* 7-bit I2C address */
    uint16_t io_timeout;             /* timeout in ms (0 = no timeout) */
    bool     did_timeout;
    uint16_t timeout_start_ms;
    uint8_t  stop_variable;          /* read by init, used when starting measurement */
    uint32_t measurement_timing_budget_us;
} VL53L0X;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/** Initialise the struct to default values. Call this before anything else. */
void vl53l0x_create(VL53L0X *dev);

/** Open the I2C bus. Returns true on success. */
bool vl53l0x_open(VL53L0X *dev);

/** Close the I2C bus. */
void vl53l0x_close(VL53L0X *dev);

/** Full sensor initialisation sequence. io_2v8 = true for 2V8 I/O mode. */
bool vl53l0x_init(VL53L0X *dev, bool io_2v8);

/* ── Address ───────────────────────────────────────────────────────────── */
void    vl53l0x_set_address(VL53L0X *dev, uint8_t new_addr);
uint8_t vl53l0x_get_address(const VL53L0X *dev);

/* ── Low-level register I/O ────────────────────────────────────────────── */
void     vl53l0x_write_reg      (VL53L0X *dev, uint8_t reg, uint8_t  value);
void     vl53l0x_write_reg16    (VL53L0X *dev, uint8_t reg, uint16_t value);
void     vl53l0x_write_reg32    (VL53L0X *dev, uint8_t reg, uint32_t value);
uint8_t  vl53l0x_read_reg       (VL53L0X *dev, uint8_t reg);
uint16_t vl53l0x_read_reg16     (VL53L0X *dev, uint8_t reg);
uint32_t vl53l0x_read_reg32     (VL53L0X *dev, uint8_t reg);
void     vl53l0x_write_multi    (VL53L0X *dev, uint8_t reg, const uint8_t *src, uint8_t count);
void     vl53l0x_read_multi     (VL53L0X *dev, uint8_t reg, uint8_t *dst, uint8_t count);

/* ── Configuration ─────────────────────────────────────────────────────── */
bool     vl53l0x_set_signal_rate_limit        (VL53L0X *dev, float limit_Mcps);
float    vl53l0x_get_signal_rate_limit        (VL53L0X *dev);
bool     vl53l0x_set_measurement_timing_budget(VL53L0X *dev, uint32_t budget_us);
uint32_t vl53l0x_get_measurement_timing_budget(VL53L0X *dev);
bool     vl53l0x_set_vcsel_pulse_period       (VL53L0X *dev, VL53L0X_VcselPeriodType type, uint8_t period_pclks);
uint8_t  vl53l0x_get_vcsel_pulse_period       (VL53L0X *dev, VL53L0X_VcselPeriodType type);

/* ── Ranging ───────────────────────────────────────────────────────────── */
void     vl53l0x_start_continuous        (VL53L0X *dev, uint32_t period_ms);
void     vl53l0x_stop_continuous         (VL53L0X *dev);
uint16_t vl53l0x_read_range_continuous_mm(VL53L0X *dev);
uint16_t vl53l0x_read_range_single_mm   (VL53L0X *dev);

/* ── Timeout ───────────────────────────────────────────────────────────── */
void     vl53l0x_set_timeout     (VL53L0X *dev, uint16_t timeout_ms);
uint16_t vl53l0x_get_timeout     (const VL53L0X *dev);
bool     vl53l0x_timeout_occurred(VL53L0X *dev);

/* ── Ranging profiles (convenience) ────────────────────────────────────── */

/**
 * Pre-defined ranging profiles based on the ST VL53L0X API User Manual.
 *
 *   Profile         Budget    Range    Accuracy   Speed
 *   ─────────────── ──────── ──────── ────────── ────────
 *   DEFAULT          33 ms    ~1.2 m   ±5%        ~30 Hz
 *   HIGH_SPEED       20 ms    ~1.2 m   ±10%       ~50 Hz
 *   HIGH_ACCURACY   200 ms    ~1.2 m   ±3%        ~5 Hz
 *   LONG_RANGE       33 ms    ~2.0 m   ±5%        ~30 Hz
 */
typedef enum {
    VL53L0X_PROFILE_DEFAULT,
    VL53L0X_PROFILE_HIGH_SPEED,
    VL53L0X_PROFILE_HIGH_ACCURACY,
    VL53L0X_PROFILE_LONG_RANGE
} VL53L0X_RangingProfile;

/**
 * Apply a ranging profile. Call AFTER vl53l0x_init().
 *
 * This sets the timing budget, VCSEL pulse periods, and signal rate
 * limit as appropriate for the selected profile. You do NOT need to
 * calculate or set these individually.
 *
 * @return true on success.
 */
bool vl53l0x_set_ranging_profile(VL53L0X *dev, VL53L0X_RangingProfile profile);

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_C_H */

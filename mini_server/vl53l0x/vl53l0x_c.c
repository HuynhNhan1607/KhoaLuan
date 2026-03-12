/**
 * @file vl53l0x_c.c
 * @brief VL53L0X ToF sensor driver for Linux/Jetson (Pure C implementation)
 *
 * Converted from the C++ JetsonHacks VL53L0X library.
 * Most of the functionality is based on the VL53L0X API provided by ST
 * (STSW-IMG005).
 *
 * Requires: sudo apt install libi2c-dev
 */

#include "vl53l0x_c.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h> /* i2c_smbus_* functions (from libi2c-dev) */

/* ── Defines ───────────────────────────────────────────────────────────── */

#define ADDRESS_DEFAULT 0x29

/* Decode VCSEL pulse period in PCLKs from register value */
#define decodeVcselPeriod(reg_val) (((reg_val) + 1) << 1)

/* Encode VCSEL pulse period register value from period in PCLKs */
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

/* Calculate macro period in nanoseconds from VCSEL period in PCLKs */
#define calcMacroPeriod(vcsel_period_pclks) \
    ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

/* ── Internal helper: get current time in milliseconds ─────────────────── */
static int vl53l0x_millis(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int)((tv.tv_sec) * 1000 + (tv.tv_usec) / 1000);
}

/* ── Internal helper: start timeout tracking ───────────────────────────── */
static inline void start_timeout(VL53L0X *dev)
{
    dev->timeout_start_ms = (uint16_t)vl53l0x_millis();
}

/* ── Internal helper: check if timeout has expired ─────────────────────── */
static inline bool check_timeout_expired(const VL53L0X *dev)
{
    return (dev->io_timeout > 0 &&
            ((uint16_t)vl53l0x_millis() - dev->timeout_start_ms) > dev->io_timeout);
}

/* ── Forward declarations of internal (static) functions ───────────────── */
static bool get_spad_info(VL53L0X *dev, uint8_t *count, bool *type_is_aperture);
static void get_sequence_step_enables(VL53L0X *dev, VL53L0X_SequenceStepEnables *enables);
static void get_sequence_step_timeouts(VL53L0X *dev,
                                       const VL53L0X_SequenceStepEnables *enables,
                                       VL53L0X_SequenceStepTimeouts *timeouts);
static bool perform_single_ref_calibration(VL53L0X *dev, uint8_t vhv_init_byte);
static uint16_t decode_timeout(uint16_t value);
static uint16_t encode_timeout(uint16_t timeout_mclks);
static uint32_t timeout_mclks_to_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeout_us_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

void vl53l0x_create(VL53L0X *dev)
{
    memset(dev, 0, sizeof(VL53L0X));
    dev->address = ADDRESS_DEFAULT;
    dev->io_timeout = 0; /* no timeout */
    dev->did_timeout = false;
    dev->i2c_bus = 1; /* default: /dev/i2c-1 */
    dev->i2c_fd = -1;
}

bool vl53l0x_open(VL53L0X *dev)
{
    char fname[32];
    snprintf(fname, sizeof(fname), "/dev/i2c-%d", dev->i2c_bus);

    dev->i2c_fd = open(fname, O_RDWR);
    if (dev->i2c_fd < 0)
    {
        dev->error = errno;
        return false;
    }
    if (ioctl(dev->i2c_fd, I2C_SLAVE, dev->address) < 0)
    {
        dev->error = errno;
        return false;
    }
    return true;
}

void vl53l0x_close(VL53L0X *dev)
{
    if (dev->i2c_fd > 0)
    {
        close(dev->i2c_fd);
        dev->i2c_fd = -1;
    }
}

/* ── Address ───────────────────────────────────────────────────────────── */

void vl53l0x_set_address(VL53L0X *dev, uint8_t new_addr)
{
    /* Write the new address to the sensor's internal register (while still
     * communicating on the OLD address). */
    vl53l0x_write_reg(dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    dev->address = new_addr;

    /* Tell the Linux I2C driver to communicate with the NEW address from now on. */
    if (ioctl(dev->i2c_fd, I2C_SLAVE, dev->address) < 0)
    {
        dev->error = errno;
    }
}

uint8_t vl53l0x_get_address(const VL53L0X *dev)
{
    return dev->address;
}

/* ── Low-level register I/O ────────────────────────────────────────────── */

void vl53l0x_write_reg(VL53L0X *dev, uint8_t reg, uint8_t value)
{
    int ret = i2c_smbus_write_byte_data(dev->i2c_fd, reg, value);
    if (ret < 0) {
        dev->error = errno;
        ret = -1;
    }
    dev->last_status = (uint8_t)ret;
}

void vl53l0x_write_reg16(VL53L0X *dev, uint8_t reg, uint16_t value)
{
    int ret = i2c_smbus_write_word_data(dev->i2c_fd, reg, value);
    if (ret < 0) {
        dev->error = errno;
        ret = -1;
    }
    dev->last_status = (uint8_t)ret;
}

void vl53l0x_write_reg32(VL53L0X *dev, uint8_t reg, uint32_t value)
{
    uint8_t buffer[4];
    buffer[0] = (value >> 24) & 0xFF;
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = (value) & 0xFF;

    int ret = i2c_smbus_write_block_data(dev->i2c_fd, reg, 4, buffer);
    if (ret < 0) {
        dev->error = errno;
        ret = -1;
    }
    dev->last_status = (uint8_t)ret;
}

uint8_t vl53l0x_read_reg(VL53L0X *dev, uint8_t reg)
{
    int32_t value = i2c_smbus_read_byte_data(dev->i2c_fd, reg);
    if (value < 0) {
        dev->error = errno;
        dev->last_status = (uint8_t)-1;
    }
    return (uint8_t)value;
}

uint16_t vl53l0x_read_reg16(VL53L0X *dev, uint8_t reg)
{
    int32_t value = i2c_smbus_read_word_data(dev->i2c_fd, reg);
    if (value < 0) {
        dev->error = errno;
        dev->last_status = (uint8_t)-1;
    }
    return (uint16_t)value;
}

uint32_t vl53l0x_read_reg32(VL53L0X *dev, uint8_t reg)
{
    uint8_t data[4];
    int ret = i2c_smbus_read_block_data(dev->i2c_fd, reg, data);
    if (ret < 0) {
        dev->error = errno;
    }
    dev->last_status = (uint8_t)ret;

    uint32_t result = (uint32_t)data[0] << 24;
    result |= (uint32_t)data[1] << 16;
    result |= (uint32_t)data[2] << 8;
    result |= (uint32_t)data[3];
    return result;
}

void vl53l0x_write_multi(VL53L0X *dev, uint8_t reg, const uint8_t *src, uint8_t count)
{
    uint8_t buffer[256]; /* max SMBus block is 32, but keep safe */
    uint8_t i;
    for (i = 0; i < count; i++)
    {
        buffer[i] = src[i];
    }

    int ret = i2c_smbus_write_block_data(dev->i2c_fd, reg, count, buffer);
    if (ret < 0) {
        dev->error = errno;
    }
    dev->last_status = (uint8_t)ret;
}

void vl53l0x_read_multi(VL53L0X *dev, uint8_t reg, uint8_t *dst, uint8_t count)
{
    while (count-- > 0)
    {
        *(dst++) = vl53l0x_read_reg(dev, reg++);
    }
}

/* ── Configuration ─────────────────────────────────────────────────────── */

bool vl53l0x_set_signal_rate_limit(VL53L0X *dev, float limit_Mcps)
{
    if (limit_Mcps < 0 || limit_Mcps > 511.99f)
    {
        return false;
    }

    /* Q9.7 fixed point format (9 integer bits, 7 fractional bits) */
    vl53l0x_write_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                        (uint16_t)(limit_Mcps * (1 << 7)));
    return true;
}

float vl53l0x_get_signal_rate_limit(VL53L0X *dev)
{
    return (float)vl53l0x_read_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (float)(1 << 7);
}

bool vl53l0x_set_measurement_timing_budget(VL53L0X *dev, uint32_t budget_us)
{
    VL53L0X_SequenceStepEnables enables;
    VL53L0X_SequenceStepTimeouts timeouts;

    const uint16_t StartOverhead = 1320;
    const uint16_t EndOverhead = 960;
    const uint16_t MsrcOverhead = 660;
    const uint16_t TccOverhead = 590;
    const uint16_t DssOverhead = 690;
    const uint16_t PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;
    const uint32_t MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget)
    {
        return false;
    }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    get_sequence_step_enables(dev, &enables);
    get_sequence_step_timeouts(dev, &enables, &timeouts);

    if (enables.tcc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }
    if (enables.dss)
    {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }
    if (enables.pre_range)
    {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        used_budget_us += FinalRangeOverhead;

        if (used_budget_us > budget_us)
        {
            return false; /* requested timeout too big */
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        uint16_t final_range_timeout_mclks =
            (uint16_t)timeout_us_to_mclks(final_range_timeout_us,
                                          timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        vl53l0x_write_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                            encode_timeout(final_range_timeout_mclks));

        dev->measurement_timing_budget_us = budget_us;
    }
    return true;
}

uint32_t vl53l0x_get_measurement_timing_budget(VL53L0X *dev)
{
    VL53L0X_SequenceStepEnables enables;
    VL53L0X_SequenceStepTimeouts timeouts;

    const uint16_t StartOverhead = 1910;
    const uint16_t EndOverhead = 960;
    const uint16_t MsrcOverhead = 660;
    const uint16_t TccOverhead = 590;
    const uint16_t DssOverhead = 690;
    const uint16_t PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t budget_us = StartOverhead + EndOverhead;

    get_sequence_step_enables(dev, &enables);
    get_sequence_step_timeouts(dev, &enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }
    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }
    if (enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }
    if (enables.final_range)
    {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    dev->measurement_timing_budget_us = budget_us;
    return budget_us;
}

bool vl53l0x_set_vcsel_pulse_period(VL53L0X *dev, VL53L0X_VcselPeriodType type, uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    VL53L0X_SequenceStepEnables enables;
    VL53L0X_SequenceStepTimeouts timeouts;

    get_sequence_step_enables(dev, &enables);
    get_sequence_step_timeouts(dev, &enables, &timeouts);

    if (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE)
    {
        /* Set phase check limits */
        switch (period_pclks)
        {
        case 12:
            vl53l0x_write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
            break;
        case 14:
            vl53l0x_write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
            break;
        case 16:
            vl53l0x_write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
            break;
        case 18:
            vl53l0x_write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
            break;
        default:
            return false; /* invalid period */
        }
        vl53l0x_write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

        /* Apply new VCSEL period */
        vl53l0x_write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        /* Update timeouts */
        uint16_t new_pre_range_timeout_mclks =
            (uint16_t)timeout_us_to_mclks(timeouts.pre_range_us, period_pclks);

        vl53l0x_write_reg16(dev, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                            encode_timeout(new_pre_range_timeout_mclks));

        uint16_t new_msrc_timeout_mclks =
            (uint16_t)timeout_us_to_mclks(timeouts.msrc_dss_tcc_us, period_pclks);

        vl53l0x_write_reg(dev, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                          (new_msrc_timeout_mclks > 256) ? 255 : (uint8_t)(new_msrc_timeout_mclks - 1));
    }
    else if (type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE)
    {
        switch (period_pclks)
        {
        case 8:
            vl53l0x_write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
            vl53l0x_write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            vl53l0x_write_reg(dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
            vl53l0x_write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
            vl53l0x_write_reg(dev, 0xFF, 0x01);
            vl53l0x_write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x30);
            vl53l0x_write_reg(dev, 0xFF, 0x00);
            break;
        case 10:
            vl53l0x_write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
            vl53l0x_write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            vl53l0x_write_reg(dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            vl53l0x_write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
            vl53l0x_write_reg(dev, 0xFF, 0x01);
            vl53l0x_write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
            vl53l0x_write_reg(dev, 0xFF, 0x00);
            break;
        case 12:
            vl53l0x_write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
            vl53l0x_write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            vl53l0x_write_reg(dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            vl53l0x_write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
            vl53l0x_write_reg(dev, 0xFF, 0x01);
            vl53l0x_write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
            vl53l0x_write_reg(dev, 0xFF, 0x00);
            break;
        case 14:
            vl53l0x_write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
            vl53l0x_write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            vl53l0x_write_reg(dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            vl53l0x_write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
            vl53l0x_write_reg(dev, 0xFF, 0x01);
            vl53l0x_write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
            vl53l0x_write_reg(dev, 0xFF, 0x00);
            break;
        default:
            return false; /* invalid period */
        }

        /* Apply new VCSEL period */
        vl53l0x_write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        /* Update timeouts */
        uint16_t new_final_range_timeout_mclks =
            (uint16_t)timeout_us_to_mclks(timeouts.final_range_us, period_pclks);

        if (enables.pre_range)
        {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        vl53l0x_write_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                            encode_timeout(new_final_range_timeout_mclks));
    }
    else
    {
        return false; /* invalid type */
    }

    /* Re-apply timing budget */
    vl53l0x_set_measurement_timing_budget(dev, dev->measurement_timing_budget_us);

    /* Perform phase calibration (needed after changing VCSEL period) */
    uint8_t sequence_config = vl53l0x_read_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    perform_single_ref_calibration(dev, 0x0);
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config);

    return true;
}

uint8_t vl53l0x_get_vcsel_pulse_period(VL53L0X *dev, VL53L0X_VcselPeriodType type)
{
    if (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE)
    {
        return decodeVcselPeriod(vl53l0x_read_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD));
    }
    else if (type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE)
    {
        return decodeVcselPeriod(vl53l0x_read_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    }
    return 255;
}

/* ── Ranging ───────────────────────────────────────────────────────────── */

void vl53l0x_start_continuous(VL53L0X *dev, uint32_t period_ms)
{
    vl53l0x_write_reg(dev, 0x80, 0x01);
    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x00, 0x00);
    vl53l0x_write_reg(dev, 0x91, dev->stop_variable);
    vl53l0x_write_reg(dev, 0x00, 0x01);
    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x80, 0x00);

    if (period_ms != 0)
    {
        /* Continuous timed mode */
        uint16_t osc_calibrate_val = vl53l0x_read_reg16(dev, VL53L0X_REG_OSC_CALIBRATE_VAL);
        if (osc_calibrate_val != 0)
        {
            period_ms *= osc_calibrate_val;
        }
        vl53l0x_write_reg32(dev, VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        vl53l0x_write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x04); /* timed mode */
    }
    else
    {
        /* Continuous back-to-back mode */
        vl53l0x_write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x02); /* back-to-back */
    }
}

void vl53l0x_stop_continuous(VL53L0X *dev)
{
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x01); /* single-shot */

    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x00, 0x00);
    vl53l0x_write_reg(dev, 0x91, 0x00);
    vl53l0x_write_reg(dev, 0x00, 0x01);
    vl53l0x_write_reg(dev, 0xFF, 0x00);
}

uint16_t vl53l0x_read_range_continuous_mm(VL53L0X *dev)
{
    start_timeout(dev);
    while ((vl53l0x_read_reg(dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (check_timeout_expired(dev))
        {
            dev->did_timeout = true;
            return 65535;
        }
    }

    /* Linearity Corrective Gain assumed 1000 (default); fractional ranging disabled */
    uint16_t range = (uint16_t)vl53l0x_read_reg16(dev, VL53L0X_REG_RESULT_RANGE_STATUS + 10) << 8;
    range |= vl53l0x_read_reg(dev, VL53L0X_REG_RESULT_RANGE_STATUS + 11);

    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
}

uint16_t vl53l0x_read_range_single_mm(VL53L0X *dev)
{
    vl53l0x_write_reg(dev, 0x80, 0x01);
    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x00, 0x00);
    vl53l0x_write_reg(dev, 0x91, dev->stop_variable);
    vl53l0x_write_reg(dev, 0x00, 0x01);
    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x80, 0x00);

    vl53l0x_write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x01);

    /* Wait until start bit has been cleared */
    start_timeout(dev);
    while (vl53l0x_read_reg(dev, VL53L0X_REG_SYSRANGE_START) & 0x01)
    {
        if (check_timeout_expired(dev))
        {
            dev->did_timeout = true;
            /* ── Cleanup: abort measurement so next read starts fresh ── */
            vl53l0x_write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x00);
            vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
            return 65535;
        }
        usleep(1000); /* 1ms — reduce I2C bus stress, prevent EMI-induced lockup */
    }

    return vl53l0x_read_range_continuous_mm(dev);
}

/* ── Timeout ───────────────────────────────────────────────────────────── */

void vl53l0x_set_timeout(VL53L0X *dev, uint16_t timeout_ms)
{
    dev->io_timeout = timeout_ms;
}

uint16_t vl53l0x_get_timeout(const VL53L0X *dev)
{
    return dev->io_timeout;
}

bool vl53l0x_timeout_occurred(VL53L0X *dev)
{
    bool tmp = dev->did_timeout;
    dev->did_timeout = false;
    return tmp;
}

/* ── Ranging profiles ──────────────────────────────────────────────────── */

bool vl53l0x_set_ranging_profile(VL53L0X *dev, VL53L0X_RangingProfile profile)
{
    /*
     * Giá trị dựa trên ST API User Manual (UM2039) và Application Note AN4846.
     *
     *   Profile         Budget(µs)  VCSEL_Pre  VCSEL_Final  SignalRate(MCPS)
     *   ─────────────── ────────── ────────── ──────────── ────────────────
     *   DEFAULT           33000       14          10           0.25
     *   HIGH_SPEED        20000       14          10           0.25
     *   HIGH_ACCURACY    200000       14          10           0.25
     *   LONG_RANGE        33000       18          14           0.10
     */

    switch (profile)
    {

    case VL53L0X_PROFILE_DEFAULT:
        /* Reset về mặc định — VCSEL periods + signal rate đã đúng sau init */
        if (!vl53l0x_set_measurement_timing_budget(dev, 33000))
            return false;
        break;

    case VL53L0X_PROFILE_HIGH_SPEED:
        /* 20 ms budget → ~50 Hz nhưng kém chính xác hơn (±10%) */
        if (!vl53l0x_set_measurement_timing_budget(dev, 20000))
            return false;
        break;

    case VL53L0X_PROFILE_HIGH_ACCURACY:
        /* 200 ms budget → ~5 Hz nhưng rất chính xác (±3%) */
        if (!vl53l0x_set_measurement_timing_budget(dev, 200000))
            return false;
        break;

    case VL53L0X_PROFILE_LONG_RANGE:
        /* Tăng VCSEL pulse period → tia laser mạnh hơn → xa hơn (~2m)
         * Giảm signal rate limit → chấp nhận tín hiệu yếu hơn */
        if (!vl53l0x_set_signal_rate_limit(dev, 0.1f))
            return false;
        if (!vl53l0x_set_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18))
            return false;
        if (!vl53l0x_set_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14))
            return false;
        if (!vl53l0x_set_measurement_timing_budget(dev, 33000))
            return false;
        break;

    default:
        return false;
    }

    return true;
}

/* ── Full init sequence ────────────────────────────────────────────────── */

bool vl53l0x_init(VL53L0X *dev, bool io_2v8)
{
    /* VL53L0X_DataInit() begin */

    if (io_2v8)
    {
        vl53l0x_write_reg(dev, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                          vl53l0x_read_reg(dev, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
    }

    /* "Set I2C standard mode" */
    vl53l0x_write_reg(dev, 0x88, 0x00);

    vl53l0x_write_reg(dev, 0x80, 0x01);
    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x00, 0x00);
    dev->stop_variable = vl53l0x_read_reg(dev, 0x91);
    vl53l0x_write_reg(dev, 0x00, 0x01);
    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x80, 0x00);

    /* disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks */
    vl53l0x_write_reg(dev, VL53L0X_REG_MSRC_CONFIG_CONTROL,
                      vl53l0x_read_reg(dev, VL53L0X_REG_MSRC_CONFIG_CONTROL) | 0x12);

    /* set final range signal rate limit to 0.25 MCPS */
    vl53l0x_set_signal_rate_limit(dev, 0.25f);

    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* VL53L0X_DataInit() end */

    /* VL53L0X_StaticInit() begin */

    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!get_spad_info(dev, &spad_count, &spad_type_is_aperture))
    {
        return false;
    }

    uint8_t ref_spad_map[6];
    vl53l0x_read_multi(dev, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* -- VL53L0X_set_reference_spads() begin (assume NVM values are valid) */

    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    vl53l0x_write_reg(dev, VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;

    uint8_t i;
    for (i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }

    vl53l0x_write_multi(dev, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* -- VL53L0X_set_reference_spads() end */

    /* -- VL53L0X_load_tuning_settings() begin */
    /* DefaultTuningSettings from vl53l0x_tuning.h */

    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x00, 0x00);

    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x09, 0x00);
    vl53l0x_write_reg(dev, 0x10, 0x00);
    vl53l0x_write_reg(dev, 0x11, 0x00);

    vl53l0x_write_reg(dev, 0x24, 0x01);
    vl53l0x_write_reg(dev, 0x25, 0xFF);
    vl53l0x_write_reg(dev, 0x75, 0x00);

    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x4E, 0x2C);
    vl53l0x_write_reg(dev, 0x48, 0x00);
    vl53l0x_write_reg(dev, 0x30, 0x20);

    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x30, 0x09);
    vl53l0x_write_reg(dev, 0x54, 0x00);
    vl53l0x_write_reg(dev, 0x31, 0x04);
    vl53l0x_write_reg(dev, 0x32, 0x03);
    vl53l0x_write_reg(dev, 0x40, 0x83);
    vl53l0x_write_reg(dev, 0x46, 0x25);
    vl53l0x_write_reg(dev, 0x60, 0x00);
    vl53l0x_write_reg(dev, 0x27, 0x00);
    vl53l0x_write_reg(dev, 0x50, 0x06);
    vl53l0x_write_reg(dev, 0x51, 0x00);
    vl53l0x_write_reg(dev, 0x52, 0x96);
    vl53l0x_write_reg(dev, 0x56, 0x08);
    vl53l0x_write_reg(dev, 0x57, 0x30);
    vl53l0x_write_reg(dev, 0x61, 0x00);
    vl53l0x_write_reg(dev, 0x62, 0x00);
    vl53l0x_write_reg(dev, 0x64, 0x00);
    vl53l0x_write_reg(dev, 0x65, 0x00);
    vl53l0x_write_reg(dev, 0x66, 0xA0);

    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x22, 0x32);
    vl53l0x_write_reg(dev, 0x47, 0x14);
    vl53l0x_write_reg(dev, 0x49, 0xFF);
    vl53l0x_write_reg(dev, 0x4A, 0x00);

    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x7A, 0x0A);
    vl53l0x_write_reg(dev, 0x7B, 0x00);
    vl53l0x_write_reg(dev, 0x78, 0x21);

    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x23, 0x34);
    vl53l0x_write_reg(dev, 0x42, 0x00);
    vl53l0x_write_reg(dev, 0x44, 0xFF);
    vl53l0x_write_reg(dev, 0x45, 0x26);
    vl53l0x_write_reg(dev, 0x46, 0x05);
    vl53l0x_write_reg(dev, 0x40, 0x40);
    vl53l0x_write_reg(dev, 0x0E, 0x06);
    vl53l0x_write_reg(dev, 0x20, 0x1A);
    vl53l0x_write_reg(dev, 0x43, 0x40);

    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x34, 0x03);
    vl53l0x_write_reg(dev, 0x35, 0x44);

    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x31, 0x04);
    vl53l0x_write_reg(dev, 0x4B, 0x09);
    vl53l0x_write_reg(dev, 0x4C, 0x05);
    vl53l0x_write_reg(dev, 0x4D, 0x04);

    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x44, 0x00);
    vl53l0x_write_reg(dev, 0x45, 0x20);
    vl53l0x_write_reg(dev, 0x47, 0x08);
    vl53l0x_write_reg(dev, 0x48, 0x28);
    vl53l0x_write_reg(dev, 0x67, 0x00);
    vl53l0x_write_reg(dev, 0x70, 0x04);
    vl53l0x_write_reg(dev, 0x71, 0x01);
    vl53l0x_write_reg(dev, 0x72, 0xFE);
    vl53l0x_write_reg(dev, 0x76, 0x00);
    vl53l0x_write_reg(dev, 0x77, 0x00);

    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x0D, 0x01);

    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x80, 0x01);
    vl53l0x_write_reg(dev, 0x01, 0xF8);

    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x8E, 0x01);
    vl53l0x_write_reg(dev, 0x00, 0x01);
    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x80, 0x00);

    /* -- VL53L0X_load_tuning_settings() end */

    /* "Set interrupt config to new sample ready" */
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    vl53l0x_write_reg(dev, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH,
                      vl53l0x_read_reg(dev, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); /* active low */
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    dev->measurement_timing_budget_us = vl53l0x_get_measurement_timing_budget(dev);

    /* "Disable MSRC and TCC by default" */
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* "Recalculate timing budget" */
    vl53l0x_set_measurement_timing_budget(dev, dev->measurement_timing_budget_us);

    /* VL53L0X_StaticInit() end */

    /* VL53L0X_PerformRefCalibration() begin */

    /* -- VL53L0X_perform_vhv_calibration() begin */
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!perform_single_ref_calibration(dev, 0x40))
    {
        return false;
    }

    /* -- VL53L0X_perform_phase_calibration() begin */
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!perform_single_ref_calibration(dev, 0x00))
    {
        return false;
    }

    /* "restore the previous Sequence Config" */
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* VL53L0X_PerformRefCalibration() end */

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * INTERNAL (STATIC) FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

static bool get_spad_info(VL53L0X *dev, uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp;

    vl53l0x_write_reg(dev, 0x80, 0x01);
    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x00, 0x00);

    vl53l0x_write_reg(dev, 0xFF, 0x06);
    vl53l0x_write_reg(dev, 0x83, vl53l0x_read_reg(dev, 0x83) | 0x04);
    vl53l0x_write_reg(dev, 0xFF, 0x07);
    vl53l0x_write_reg(dev, 0x81, 0x01);

    vl53l0x_write_reg(dev, 0x80, 0x01);

    vl53l0x_write_reg(dev, 0x94, 0x6b);
    vl53l0x_write_reg(dev, 0x83, 0x00);
    start_timeout(dev);
    while (vl53l0x_read_reg(dev, 0x83) == 0x00)
    {
        if (check_timeout_expired(dev))
        {
            return false;
        }
    }
    vl53l0x_write_reg(dev, 0x83, 0x01);
    tmp = vl53l0x_read_reg(dev, 0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    vl53l0x_write_reg(dev, 0x81, 0x00);
    vl53l0x_write_reg(dev, 0xFF, 0x06);
    vl53l0x_write_reg(dev, 0x83, vl53l0x_read_reg(dev, 0x83 & ~0x04));
    vl53l0x_write_reg(dev, 0xFF, 0x01);
    vl53l0x_write_reg(dev, 0x00, 0x01);

    vl53l0x_write_reg(dev, 0xFF, 0x00);
    vl53l0x_write_reg(dev, 0x80, 0x00);

    return true;
}

static void get_sequence_step_enables(VL53L0X *dev, VL53L0X_SequenceStepEnables *enables)
{
    uint8_t sequence_config = vl53l0x_read_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);

    enables->tcc = (sequence_config >> 4) & 0x1;
    enables->dss = (sequence_config >> 3) & 0x1;
    enables->msrc = (sequence_config >> 2) & 0x1;
    enables->pre_range = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

static void get_sequence_step_timeouts(VL53L0X *dev,
                                       const VL53L0X_SequenceStepEnables *enables,
                                       VL53L0X_SequenceStepTimeouts *timeouts)
{
    timeouts->pre_range_vcsel_period_pclks =
        vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE);

    timeouts->msrc_dss_tcc_mclks =
        vl53l0x_read_reg(dev, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us =
        timeout_mclks_to_us(timeouts->msrc_dss_tcc_mclks,
                            timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks =
        decode_timeout(vl53l0x_read_reg16(dev, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us =
        timeout_mclks_to_us(timeouts->pre_range_mclks,
                            timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks =
        vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE);

    timeouts->final_range_mclks =
        decode_timeout(vl53l0x_read_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us =
        timeout_mclks_to_us(timeouts->final_range_mclks,
                            timeouts->final_range_vcsel_period_pclks);
}

static uint16_t decode_timeout(uint16_t reg_val)
{
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encode_timeout(uint16_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return 0;
}

static uint32_t timeout_mclks_to_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeout_us_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static bool perform_single_ref_calibration(VL53L0X *dev, uint8_t vhv_init_byte)
{
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x01 | vhv_init_byte);

    start_timeout(dev);
    while ((vl53l0x_read_reg(dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (check_timeout_expired(dev))
        {
            return false;
        }
    }

    vl53l0x_write_reg(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    vl53l0x_write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x00);

    return true;
}

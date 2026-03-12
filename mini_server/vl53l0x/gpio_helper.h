/**
 * @file gpio_helper.h
 * @brief GPIO control via libgpiod for Jetson Xavier NX
 *
 * Used to control XSHUT pins of VL53L0X sensors.
 * Requires libgpiod v1: sudo apt install libgpiod-dev
 */

#ifndef GPIO_HELPER_H
#define GPIO_HELPER_H

#include <stdbool.h>
#include <gpiod.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Handle for a single GPIO output line managed via libgpiod.
 */
typedef struct {
    struct gpiod_chip *chip;
    struct gpiod_line *line;
    unsigned int       line_offset;
    const char        *chip_name;
} GpioHandle;

/**
 * Open a GPIO line as output.
 *
 * @param handle       Pointer to a GpioHandle to initialise.
 * @param chip_name    Name of the GPIO chip (e.g. "gpiochip1").
 *                     You can also pass a device path like "/dev/gpiochip0".
 * @param line_offset  Line offset within the chip (NOT the board pin number).
 * @param initial_val  Initial output value (0 = LOW, 1 = HIGH).
 * @param consumer     Label for this usage (e.g. "vl53l0x-xshut-left").
 * @return true on success.
 */
bool gpio_open_output(GpioHandle *handle,
                      const char *chip_name,
                      unsigned int line_offset,
                      int initial_val,
                      const char *consumer);

/**
 * Write a value (0 or 1) to an open GPIO line.
 * @return true on success.
 */
bool gpio_write(GpioHandle *handle, int value);

/**
 * Close and release a GPIO line and its chip.
 */
void gpio_close(GpioHandle *handle);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_HELPER_H */

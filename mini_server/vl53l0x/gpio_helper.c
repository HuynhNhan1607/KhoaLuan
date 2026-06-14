/**
 * @file gpio_helper.c
 * @brief GPIO control via libgpiod for Jetson Xavier NX
 */

#include "gpio_helper.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

bool gpio_open_output(GpioHandle *handle, const char *chip_name,
                      unsigned int line_offset, int initial_val,
                      const char *consumer) {
  memset(handle, 0, sizeof(GpioHandle));
  handle->chip_name = chip_name;
  handle->line_offset = line_offset;

  /* Mở GPIO chip — thử bằng tên trước, nếu bắt đầu bằng '/' thì mở bằng path */
  if (chip_name[0] == '/') {
    handle->chip = gpiod_chip_open(chip_name);
  } else {
    handle->chip = gpiod_chip_open_by_name(chip_name);
  }

  if (!handle->chip) {
    fprintf(stderr, "gpio_open_output: Khong the mo chip '%s'\n", chip_name);
    return false;
  }

  /* Lấy line theo offset */
  handle->line = gpiod_chip_get_line(handle->chip, line_offset);
  if (!handle->line) {
    fprintf(stderr, "gpio_open_output: Khong the lay line %u tren chip '%s'\n",
            line_offset, chip_name);
    gpiod_chip_close(handle->chip);
    handle->chip = NULL;
    return false;
  }

  /* Yêu cầu line làm output với giá trị ban đầu */
  if (gpiod_line_request_output(handle->line, consumer, initial_val) < 0) {
    fprintf(stderr,
            "gpio_open_output: Khong the request output line %u ('%s'): "
            "errno=%d (%s)\n",
            line_offset, consumer, errno, strerror(errno));
    gpiod_chip_close(handle->chip);
    handle->chip = NULL;
    handle->line = NULL;
    return false;
  }

  return true;
}

bool gpio_write(GpioHandle *handle, int value) {
  if (!handle->line) {
    fprintf(stderr, "gpio_write: Line chua duoc mo!\n");
    return false;
  }

  if (gpiod_line_set_value(handle->line, value) < 0) {
    fprintf(stderr, "gpio_write: Khong the set value %d\n", value);
    return false;
  }
  return true;
}

void gpio_close(GpioHandle *handle) {
  if (handle->line) {
    gpiod_line_release(handle->line);
    handle->line = NULL;
  }
  if (handle->chip) {
    gpiod_chip_close(handle->chip);
    handle->chip = NULL;
  }
}

/**
 * @file vl53l0x_addr_init.c
 * @brief Set dia chi I2C cho 2 cam bien VL53L0X - chay 1 lan khi Jetson boot.
 *
 * Build: cd vl53l0x && make
 * Chay:  sudo ./vl53l0x_addr_init
 */

#include "gpio_helper.h"
#include "vl53l0x_c.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/* ---- Hardcode cau hinh phan cung ---- */
#define I2C_BUS 1
#define GPIO_CHIP "gpiochip1"
#define XSHUT_LEFT 108u  /* Board Pin 31 = GPIO11 */
#define XSHUT_RIGHT 118u /* Board Pin 33 = GPIO13 */
#define ADDR_LEFT 0x30u  /* Dia chi moi cho cam bien TRAI */
/* Cam bien PHAI giu nguyen dia chi mac dinh 0x29 */
#define SENSOR_TIMEOUT 500

int main(void)
{
    GpioHandle gpio_l = {0};
    GpioHandle gpio_r = {0};
    VL53L0X sensor_l, sensor_r;
    int ret = EXIT_FAILURE;

    printf("=== VL53L0X ADDR INIT ===\n");
    printf("LEFT=0x%02X (XSHUT line %u)  RIGHT=0x29 (XSHUT line %u)\n\n",
           ADDR_LEFT, XSHUT_LEFT, XSHUT_RIGHT);

    /* Mo GPIO XSHUT, ban dau = LOW (tat ca hai) */
    if (!gpio_open_output(&gpio_l, GPIO_CHIP, XSHUT_LEFT, 0, "xshut_left") ||
        !gpio_open_output(&gpio_r, GPIO_CHIP, XSHUT_RIGHT, 0, "xshut_right"))
    {
        fprintf(stderr, "LOI: Khong mo GPIO (can sudo? gpio chip dung?)\n");
        goto done;
    }

    /* B1: Reset ca hai ve 0x29 */
    gpio_write(&gpio_l, 0);
    gpio_write(&gpio_r, 0);
    usleep(10000);

    /* B2: Danh thuc TRAI */
    gpio_write(&gpio_l, 1);
    usleep(5000);

    vl53l0x_create(&sensor_l);
    sensor_l.i2c_bus = I2C_BUS;
    if (!vl53l0x_open(&sensor_l))
    {
        fprintf(stderr, "LOI: Mo I2C LEFT that bai (errno=%d)\n", sensor_l.error);
        goto done;
    }

    /* B3: Doi dia chi TRAI 0x29 -> ADDR_LEFT, roi init */
    vl53l0x_set_address(&sensor_l, ADDR_LEFT);
    usleep(2000);
    vl53l0x_set_timeout(&sensor_l, SENSOR_TIMEOUT);
    if (!vl53l0x_init(&sensor_l, true))
    {
        fprintf(stderr, "LOI: Init LEFT that bai\n");
        goto done;
    }
    printf("LEFT  OK - addr=0x%02X\n", vl53l0x_get_address(&sensor_l));

    /* B4: Danh thuc PHAI (se dung tai 0x29, TRAI da chiem ADDR_LEFT) */
    gpio_write(&gpio_r, 1);
    usleep(5000);

    vl53l0x_create(&sensor_r);
    sensor_r.i2c_bus = I2C_BUS;
    if (!vl53l0x_open(&sensor_r))
    {
        fprintf(stderr, "LOI: Mo I2C RIGHT that bai (errno=%d)\n", sensor_r.error);
        goto done;
    }

    vl53l0x_set_timeout(&sensor_r, SENSOR_TIMEOUT);
    if (!vl53l0x_init(&sensor_r, true))
    {
        fprintf(stderr, "LOI: Init RIGHT that bai\n");
        goto done;
    }
    printf("RIGHT OK - addr=0x%02X\n", vl53l0x_get_address(&sensor_r));

    printf("\nTHANH CONG! Gio co the rut chan XSHUT.\n");
    ret = EXIT_SUCCESS;

done:
    vl53l0x_close(&sensor_l);
    vl53l0x_close(&sensor_r);
    gpio_close(&gpio_l);
    gpio_close(&gpio_r);
    return ret;
}

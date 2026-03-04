#ifndef GPIO_DEF_H_
#define GPIO_DEF_H_

#include "driver/gpio.h"

// Motor 1 Encoder
#define ENCODER_1_PHASE_A_GPIO GPIO_NUM_4
#define ENCODER_1_PHASE_B_GPIO GPIO_NUM_5

// Motor 2 Encoder
#define ENCODER_2_PHASE_A_GPIO GPIO_NUM_6
#define ENCODER_2_PHASE_B_GPIO GPIO_NUM_7

// Motor 3 Encoder
#define ENCODER_3_PHASE_A_GPIO GPIO_NUM_15
#define ENCODER_3_PHASE_B_GPIO GPIO_NUM_16

// Motor 4 Encoder
#define ENCODER_4_PHASE_A_GPIO GPIO_NUM_17
#define ENCODER_4_PHASE_B_GPIO GPIO_NUM_18

/*----------------------------------------------*/

// GPIO pin definitions for Motor 1 (H-Bridge BTS7960)
#define MOTOR_1_L_PWM_GPIO GPIO_NUM_10
#define MOTOR_1_R_PWM_GPIO GPIO_NUM_11

// GPIO pin definitions for Motor 2 (H-Bridge BTS7960)
#define MOTOR_2_L_PWM_GPIO GPIO_NUM_38
#define MOTOR_2_R_PWM_GPIO GPIO_NUM_39

// GPIO pin definitions for Motor 3 (H-Bridge BTS7960)
#define MOTOR_3_L_PWM_GPIO GPIO_NUM_21
#define MOTOR_3_R_PWM_GPIO GPIO_NUM_14

// GPIO pin definitions for Motor 4 (H-Bridge BTS7960)
#define MOTOR_4_L_PWM_GPIO GPIO_NUM_12
#define MOTOR_4_R_PWM_GPIO GPIO_NUM_13

#endif // GPIO_DEF_H_
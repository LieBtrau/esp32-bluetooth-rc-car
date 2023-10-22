#pragma once

#include <stdint.h>
#include <driver/gpio.h>

static const uint32_t PIN_STEERING_MOTOR_A = GPIO_NUM_18;
static const uint32_t PIN_STEERING_MOTOR_B = GPIO_NUM_19;
static const uint32_t PIN_THRUST_MOTOR_A = GPIO_NUM_22;
static const uint32_t PIN_THRUST_MOTOR_B = GPIO_NUM_23;

static const gpio_num_t PIN_PWR_EN = GPIO_NUM_25;
static const gpio_num_t PIN_LED = GPIO_NUM_26;
static const gpio_num_t PIN_SWITCH = GPIO_NUM_27;

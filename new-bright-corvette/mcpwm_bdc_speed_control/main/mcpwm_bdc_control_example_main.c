/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bdc_motor.h"

static const char *TAG = "example";
#define CONFIG_FREERTOS_HZ 100 // 10ms tick (as in menuconfig), @todo : find a way to get this value from menuconfig

// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG CONFIG_SERIAL_STUDIO_DEBUG

void app_main(void)
{
    const uint32_t BDC_MCPWM_TIMER_RESOLUTION_HZ = 10e6;                                        // 10MHz
    const uint32_t BDC_MCPWM_FREQ_HZ = 25e3;                                                    // 25KHz PWM
    const uint32_t BDC_MCPWM_DUTY_TICK_MAX = BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ; // maximum value we can set for the duty cycle, in ticks

    ESP_LOGI(TAG, "Create DC motors");
    bdc_motor_config_t motor1_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = 18,
        .pwmb_gpio_num = 19,
    };
    bdc_motor_config_t motor2_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = 22,
        .pwmb_gpio_num = 23,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor1 = NULL, motor2 = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor1_config, &mcpwm_config, &motor1));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor2_config, &mcpwm_config, &motor2));
    ESP_LOGI(TAG, "Enable motor1");
    ESP_ERROR_CHECK(bdc_motor_enable(motor1));
    ESP_LOGI(TAG, "Enable motor2");
    ESP_ERROR_CHECK(bdc_motor_enable(motor2));
     
    ESP_LOGI(TAG, "Set motor speed");
    bdc_motor_set_speed(motor1, BDC_MCPWM_DUTY_TICK_MAX / 2 /*50% duty cycle*/);
    bdc_motor_set_speed(motor2, 0 /*25% duty cycle*/);

    while (1)
    {
        // bdc_motor_brake() causes both outputs to be high.  Not allowed for our H-bridge configuration.
        //  bdc_motor_coast() causes both outputs to be low.  This is what we want, equivalent to setting motor speed to 0.
        ESP_ERROR_CHECK(bdc_motor_forward(motor2));
        // ESP_ERROR_CHECK(bdc_motor_forward(motor1));
        // vTaskDelay(pdMS_TO_TICKS(100));
        // ESP_ERROR_CHECK(bdc_motor_reverse(motor1));
        // vTaskDelay(pdMS_TO_TICKS(100));
        // ESP_ERROR_CHECK(bdc_motor_forward(motor2));
        // vTaskDelay(pdMS_TO_TICKS(100));
        // ESP_ERROR_CHECK(bdc_motor_reverse(motor2));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

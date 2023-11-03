/**
 * @file esp_hid_host_main.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2023
 * @details
 *  * heavily based on [https://github.com/espressif/esp-idf/tree/3befd5fff7/examples/bluetooth/esp_hid_host]
 *  * other bluetooth HID implementation, as described in the ESP bluetooth-hid API [https://github.com/nlemoing/bt_kb_receiver/tree/master]
 */

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "pins.h"
#include "driver/gpio.h"
#include "iot_button.h"
#include "LED.h"
#include "SteerMotor.h"
#include "ThrustMotor.h"
#include "BluetoothController.h"

static const char *TAG = "ESP_HIDH_DEMO";

static void button_long_press_cb(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "BUTTON_LONG_PRESS_UP");
}

static void button_single_click_cb(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "BUTTON_SINGLE_CLICK");
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Version : %s", VERSION);

    SteerMotor steerMotor;
    steerMotor.init(PIN_STEERING_MOTOR_A, PIN_STEERING_MOTOR_B);
    ThrustMotor thrustMotor;
    thrustMotor.init(PIN_THRUST_MOTOR_A, PIN_THRUST_MOTOR_B);
    BluetoothController btcontroller;
    btcontroller.init();

    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = PIN_SWITCH,
            .active_level = 0,
        },
    };
    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if (NULL == gpio_btn)
    {
        ESP_LOGE(TAG, "Button create failed");
    }

    button_event_config_t cfg_long_press_up;
    cfg_long_press_up.event = BUTTON_LONG_PRESS_UP;
    cfg_long_press_up.event_data.long_press.press_time = 2000;
    iot_button_register_event_cb(gpio_btn, cfg_long_press_up, button_long_press_cb, NULL);

    button_event_config_t cfg_single_press_up;
    cfg_single_press_up.event = BUTTON_SINGLE_CLICK;
    cfg_single_press_up.event_data.long_press.press_time = 2000;
    iot_button_register_event_cb(gpio_btn, cfg_single_press_up, button_single_click_cb, NULL);

    LED led;
    led.init(PIN_LED);
    for (;;)
    {
        switch (btcontroller.getState())
        {
        case Bluetooth_states::UNPAIRED:
            led.blinkFast();
            break;
        case Bluetooth_states::CONNECTING:
            led.blinkSlow();
            break;
        case Bluetooth_states::CONNECTED:
            led.on();
            uni_gamepad_t gamepad;
            if (btcontroller.eventReady(&gamepad))
            {
                ESP_LOGI(TAG, "Button state: %d, Axis x: %ld, Axis y: %ld, Axis rx: %ld, Axis ry: %ld", gamepad.dpad, gamepad.axis_x, gamepad.axis_y, gamepad.axis_rx, gamepad.axis_ry);
                thrustMotor.setSpeed(gamepad.axis_y);
                Direction direction = gamepad.axis_rx < 0 ? Direction::LEFT : (gamepad.axis_rx > 0 ? Direction::RIGHT : Direction::STRAIGHT);
                steerMotor.setDirection(direction);
            }
            break;
        default:
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
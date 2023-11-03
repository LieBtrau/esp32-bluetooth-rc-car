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

#include "LED.h"
#include "SteerMotor.h"
#include "ThrustMotor.h"
#include "BluetoothController.h"
#include "Button.h"

static const char *TAG = "ESP_HIDH_DEMO";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Version : %s", VERSION);

    EventGroupHandle_t xButtonEventGroup;
    xButtonEventGroup = xEventGroupCreate();
    if( xButtonEventGroup == NULL )
    {
        /* The event group was not created because there was insufficient
        FreeRTOS heap available. */
        return;
    }

    SteerMotor steerMotor;
    steerMotor.init(PIN_STEERING_MOTOR_A, PIN_STEERING_MOTOR_B);
    ThrustMotor thrustMotor;
    thrustMotor.init(PIN_THRUST_MOTOR_A, PIN_THRUST_MOTOR_B);
    BluetoothController btcontroller;
    btcontroller.init();
    initButton(PIN_SWITCH, xButtonEventGroup);


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
        EventBits_t uxBits = xEventGroupWaitBits(xButtonEventGroup, 0x3, pdTRUE, pdFALSE, pdMS_TO_TICKS(10));
        if ((uxBits & 0x1) != 0)
        {
            ESP_LOGI(TAG, "Button short pressed");
        }
        if ((uxBits & 0x2) != 0)
        {
            ESP_LOGI(TAG, "Button long pressed");
        }
    }
}
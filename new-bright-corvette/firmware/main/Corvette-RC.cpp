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
static bool isAlive = true;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Version : %s", VERSION);

    // Enable switch takeover first, so that the user doesn't have to press the power on switch for too long.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_PWR_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(PIN_PWR_EN, 1);
    // Configure button (needed to detect the start of the key press at power on)
    Button button;
    button.init(PIN_SWITCH);
    // Configure LED (so that user can see the device is alive)
    LED led;
    led.init(PIN_LED);

    SteerMotor steerMotor;
    steerMotor.init(PIN_STEERING_MOTOR_A, PIN_STEERING_MOTOR_B);
    ThrustMotor thrustMotor;
    thrustMotor.init(PIN_THRUST_MOTOR_A, PIN_THRUST_MOTOR_B);
    BluetoothController btcontroller;
    btcontroller.init();

    TimerHandle_t xNoConnectTimer = xTimerCreate("Timer", pdMS_TO_TICKS(60000), pdFALSE, NULL, [](TimerHandle_t xTimer)
                                                 {
        ESP_LOGI(TAG, "Timer expired");
        isAlive = false; });
    xTimerStart(xNoConnectTimer, 0);

    while (isAlive)
    {
        switch (btcontroller.getState())
        {
        case Bluetooth_states::UNPAIRED:
            thrustMotor.stop();
            led.blinkFast();
            break;
        case Bluetooth_states::CONNECTING:
            thrustMotor.stop();
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
            if (xTimerReset(xNoConnectTimer, 10) != pdPASS)
            {
                /* The reset command was not executed successfully.  Take appropriate
                action here. */
                ESP_LOGE(TAG, "Timer reset failed");
            }
            break;
        default:
            break;
        }
        switch (button.waitEvent())
        {
        case ButtonEvent::SingleClick:
            ESP_LOGI(TAG, "Button short pressed");
            isAlive = false;
            break;
        case ButtonEvent::LongPress:
            ESP_LOGI(TAG, "Button long pressed");
            btcontroller.unpair();
            break;
        default:
            break;
        }
    }
    ESP_LOGI(TAG, "Exiting...");
    thrustMotor.stop();
    gpio_set_level(PIN_PWR_EN, 0);
}
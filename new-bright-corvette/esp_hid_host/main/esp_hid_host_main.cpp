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
#include "nvs_flash.h"
#include "nintendo_switch_controller.h"
#include "hid.h"
#include "bdc_motor.h"
#include "pins.h"
#include "driver/gpio.h"
#include "iot_button.h"
#include "NonVolatileStorage.h"
#include "LED.h"

static const char *TAG = "ESP_HIDH_DEMO";
static QueueHandle_t xSteerQueue = NULL;
static QueueHandle_t xThrustQueue = NULL;
static QueueHandle_t xBluetoothQueue = NULL;
static NonVolatileStorage prefs;
static NintendoSwitchController controller;

enum class Direction
{
    LEFT,
    RIGHT,
    STRAIGHT
};

enum class Bluetooth_states
{
    UNPAIRED,
    PAIRED,
    CONNECTED,
    CONNECTING
};

/**
 * @brief
 *
 * @param pvParameters
 * @todo add a timer that gets reset continuously by the "connected" state. If the timer expires, i.e. no connection for more than
 * one minute, then notify the user.
 */
void hid_demo_task(void *pvParameters)
{
    const int COD_MAJOR = 0x05; // Peripheral
    const int COD_MINOR = 0x02; // Gamepad
    esp_bd_addr_t bluetooth_address;
    Bluetooth_states bluetooth_state = Bluetooth_states::UNPAIRED;

    // Bluetooth driver requires NVS to connect to previously paired devices, initialize it before Bluetooth
    // Execute "esptool.py --port /dev/ttyUSB1 erase_flash" to erase the flash
    if (prefs.init() == ESP_OK)
    {
        if (prefs.getBDA(&bluetooth_address))
        {
            ESP_LOGI(TAG, "Found bluetooth address in NVS:" ESP_BD_ADDR_STR " ", ESP_BD_ADDR_HEX(bluetooth_address));
            controller.setBda(bluetooth_address);
            // controller.setBda((uint8_t *)"\x98\xb6\xe9\x54\x85\x38");
            bluetooth_state = Bluetooth_states::PAIRED;
        }
        else
        {
            ESP_LOGI(TAG, "No bluetooth address found in NVS");
            bluetooth_state = Bluetooth_states::UNPAIRED;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize NVS");
    }

    hid_init();

    uni_gamepad_t gamepad;
    while (bluetooth_state == Bluetooth_states::UNPAIRED)
    {
        ESP_LOGI(TAG, "Waiting for pairing");
        if (scan_hid_device(COD_MAJOR, COD_MINOR, 10, &bluetooth_address))
        {
            ESP_LOGI(TAG, "Found bluetooth device:  " ESP_BD_ADDR_STR " ", ESP_BD_ADDR_HEX(bluetooth_address));
            prefs.setBDA(&bluetooth_address);
            controller.setBda(bluetooth_address);
            bluetooth_state = Bluetooth_states::PAIRED;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to find bluetooth device");
        }
    }
    controller.connect(); // Reconnection will be attempted automatically by the driver.  Don't put this inside the loop.
    bluetooth_state = Bluetooth_states::CONNECTING;
    for (;;)
    {
        switch (bluetooth_state)
        {
        case Bluetooth_states::CONNECTING:
            vTaskDelay(pdMS_TO_TICKS(100));
            if (controller.isConnected())
            {
                ESP_LOGI(TAG, "Connected");
                bluetooth_state = Bluetooth_states::CONNECTED;
            }
            break;
        case Bluetooth_states::CONNECTED:
            if (controller.isUpdateAvailable(&gamepad))
            {
                if (xBluetoothQueue != NULL)
                {
                    if (xQueueSend(xBluetoothQueue, &gamepad, (TickType_t)1) != pdPASS)
                    {
                        ESP_LOGE(TAG, "Failed to send to queue");
                    }
                }
            }
            if (!controller.isConnected())
            {
                bluetooth_state = Bluetooth_states::CONNECTING;
                ESP_LOGI(TAG, "Disconnected");
            }
            break;
        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
    vTaskDelete(NULL);
}

void thrust_motor_task(void *pvParameters)
{
    const int32_t BDC_MAX_THRUST = 512;
    const uint32_t BDC_MCPWM_TIMER_RESOLUTION_HZ = 10e6;                                        // 10MHz
    const uint32_t BDC_MCPWM_FREQ_HZ = 20000;                                                   // PWM frequency
    const uint32_t BDC_MCPWM_DUTY_TICK_MAX = BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ; // maximum value we can set for the duty cycle, in ticks
    const uint32_t BDC_MCPWM_DUTY_TICK_MIN = BDC_MCPWM_DUTY_TICK_MAX / 2;                       // minimum PWM value needed for the motor to spin
    ESP_LOGI(TAG, "Create DC motors");
    bdc_motor_config_t motor1_config = {
        .pwma_gpio_num = PIN_THRUST_MOTOR_A,
        .pwmb_gpio_num = PIN_THRUST_MOTOR_B,
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t thrust_motor = NULL /*, motor2 = NULL*/;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor1_config, &mcpwm_config, &thrust_motor));
    ESP_LOGI(TAG, "Enable thrust");
    ESP_ERROR_CHECK(bdc_motor_enable(thrust_motor));
    ESP_ERROR_CHECK(bdc_motor_forward(thrust_motor));
    int sollThrust = 0;
    int currentThrust = 0;
    for (;;)
    {
        if (xThrustQueue != NULL)
        {
            if (xQueueReceive(xThrustQueue, &sollThrust, (TickType_t)1) == pdPASS)
            {
                sollThrust = sollThrust > BDC_MAX_THRUST ? BDC_MAX_THRUST : (sollThrust < -BDC_MAX_THRUST ? -BDC_MAX_THRUST : sollThrust);
                ESP_LOGI(TAG, "SollThrust: %d", sollThrust);

                // Might implement some
                currentThrust = sollThrust;

                // It's no use setting PWM-values below the minimum value needed for the motor to spin.
                ESP_ERROR_CHECK(bdc_motor_set_speed(thrust_motor, BDC_MCPWM_DUTY_TICK_MIN +
                                                                      (currentThrust < 0 ? -currentThrust : currentThrust) * (BDC_MCPWM_DUTY_TICK_MAX - BDC_MCPWM_DUTY_TICK_MIN) / BDC_MAX_THRUST));

                if (sollThrust > 0)
                {
                    ESP_ERROR_CHECK(bdc_motor_forward(thrust_motor));
                }
                else if (sollThrust < 0)
                {
                    ESP_ERROR_CHECK(bdc_motor_reverse(thrust_motor));
                }
                else
                {
                    ESP_ERROR_CHECK(bdc_motor_coast(thrust_motor));
                }
                ESP_LOGI(TAG, "Current thrust: %d", currentThrust);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void steering_motor_task(void *pvParameters)
{
    const uint32_t BDC_MCPWM_TIMER_RESOLUTION_HZ = 10e6;                                        // 10MHz
    const uint32_t BDC_MCPWM_FREQ_HZ = 20000;                                                   // PWM frequency
    const uint32_t BDC_MCPWM_DUTY_TICK_MAX = BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ; // maximum value we can set for the duty cycle, in ticks
    const uint32_t BDC_MCPWM_DUTY_TICK_MIN_HOLD = 30 * BDC_MCPWM_DUTY_TICK_MAX / 100;           // minimum PWM value needed for the motor to hold position

    ESP_LOGI(TAG, "Create DC motors");
    bdc_motor_config_t motor1_config = {
        .pwma_gpio_num = PIN_STEERING_MOTOR_A,
        .pwmb_gpio_num = PIN_STEERING_MOTOR_B,
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t steering_motor = NULL /*, motor2 = NULL*/;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor1_config, &mcpwm_config, &steering_motor));
    ESP_LOGI(TAG, "Enable steering");
    ESP_ERROR_CHECK(bdc_motor_enable(steering_motor));
    ESP_ERROR_CHECK(bdc_motor_coast(steering_motor));
    int pwm_duty = 0;

    for (;;)
    {
        if (xSteerQueue != NULL)
        {
            Direction direction;
            if (xQueueReceive(xSteerQueue, &direction, (TickType_t)1) == pdPASS)
            {
                switch (direction)
                {
                case Direction::LEFT:
                    ESP_ERROR_CHECK(bdc_motor_forward(steering_motor));
                    ESP_LOGI(TAG, "Steer left");
                    pwm_duty = BDC_MCPWM_DUTY_TICK_MAX;
                    break;
                case Direction::RIGHT:
                    ESP_LOGI(TAG, "Steer right");
                    ESP_ERROR_CHECK(bdc_motor_reverse(steering_motor));
                    pwm_duty = BDC_MCPWM_DUTY_TICK_MAX;
                    break;
                case Direction::STRAIGHT:
                    ESP_LOGI(TAG, "Steer straight");
                    ESP_ERROR_CHECK(bdc_motor_coast(steering_motor));
                    pwm_duty = 0;
                    break;
                }
            }
        }
        // To save power during the turning, the PWM duty cycle is gradually reduced to the minimum value needed to hold the motor in place.
        if (pwm_duty >= BDC_MCPWM_DUTY_TICK_MIN_HOLD)
        {
            ESP_ERROR_CHECK(bdc_motor_set_speed(steering_motor, pwm_duty));
            pwm_duty -= 10;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

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

    // todo
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    xSteerQueue = xQueueCreate(10, sizeof(Direction));
    if (xSteerQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue");
    }

    xThrustQueue = xQueueCreate(10, sizeof(int));
    if (xThrustQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue");
    }

    xBluetoothQueue = xQueueCreate(10, sizeof(uni_gamepad_t));
    if (xBluetoothQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue");
    }

    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, nullptr);
    xTaskCreate(&steering_motor_task, "steering_motor_task", 6 * 1024, NULL, 3, nullptr); // make sure to allocate enough stack space for the task
    xTaskCreate(&thrust_motor_task, "thrust_motor_task", 6 * 1024, NULL, 3, nullptr);

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

    bool connected=false;
    led.blink();

    for (;;)
    {
        uni_gamepad_t gamepad;
        if (xBluetoothQueue != NULL)
        {
            if (xQueueReceive(xBluetoothQueue, &gamepad, (TickType_t)1) == pdPASS)
            {
                ESP_LOGI(TAG, "Button state: %d, Axis x: %ld, Axis y: %ld, Axis rx: %ld, Axis ry: %ld", gamepad.dpad, gamepad.axis_x, gamepad.axis_y, gamepad.axis_rx, gamepad.axis_ry);
                if (xThrustQueue != NULL)
                {
                    if (xQueueSend(xThrustQueue, &gamepad.axis_y, (TickType_t)1) != pdPASS)
                    {
                        ESP_LOGE(TAG, "Failed to send to xThrustQueue");
                    }
                }
                if (xSteerQueue != NULL)
                {
                    Direction direction = gamepad.axis_rx < 0 ? Direction::LEFT : (gamepad.axis_rx > 0 ? Direction::RIGHT : Direction::STRAIGHT);
                    if (xQueueSend(xSteerQueue, &direction, (TickType_t)1) != pdPASS)
                    {
                        ESP_LOGE(TAG, "Failed to send to xSteerQueue");
                    }
                }
            }
            if (controller.isConnected() && !connected)
            {
                led.on();
                connected=true;
            }
            else if (!controller.isConnected() && connected)
            {
                led.blink();
                connected=false;
            }
        }
        else
        {
            ESP_LOGE(TAG, "xBluetoothQueue is NULL");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    vTaskDelay(portMAX_DELAY);
}

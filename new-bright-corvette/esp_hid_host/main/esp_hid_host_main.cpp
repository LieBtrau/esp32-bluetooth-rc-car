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
#include "led_indicator.h"
#include "led_indicator_blink_default.h"
#include "iot_button.h"

static const char *TAG = "ESP_HIDH_DEMO";
static QueueHandle_t xSteerQueue = NULL;
static QueueHandle_t xThrustQueue = NULL;

enum class Direction
{
    LEFT,
    RIGHT,
    STRAIGHT
};

void hid_demo_task(void *pvParameters)
{
    // scan_hid_device(bluetooth_address, 10, NULL);
    const esp_bd_addr_t bluetooth_address = {0x98, 0xb6, 0xe9, 0x54, 0x85, 0x38};
    NintendoSwitchController controller(bluetooth_address);

    controller.connect();

    uni_gamepad_t gamepad;
    for (;;)
    {
        if (controller.isUpdateAvailable(&gamepad))
        {
            ESP_LOGI(TAG, "Button state: %d", gamepad.dpad);
        }
    }
    vTaskDelete(NULL);
}

void thrust_motor_task(void *pvParameters)
{
    const uint32_t BDC_MCPWM_TIMER_RESOLUTION_HZ = 10e6;                                        // 10MHz
    const uint32_t BDC_MCPWM_FREQ_HZ = 20000;                                                   // PWM frequency
    const uint32_t BDC_MCPWM_DUTY_TICK_MAX = BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ; // maximum value we can set for the duty cycle, in ticks
    const uint32_t BDC_MCPWM_DUTY_TICK_MIN = 50 * BDC_MCPWM_DUTY_TICK_MAX / 100;                // minimum PWM value needed for the motor to spin
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
                sollThrust = sollThrust > 100 ? 100 : (sollThrust < -100 ? -100 : sollThrust);
                ESP_LOGI(TAG, "SollThrust: %d", sollThrust);
            }
        }
        // Set thrust
        if (sollThrust > currentThrust)
        {
            currentThrust++;
        }
        else if (sollThrust < currentThrust)
        {
            currentThrust--;
        }
        // It's no use setting PWM-values below the minimum value needed for the motor to spin.
        ESP_ERROR_CHECK(bdc_motor_set_speed(thrust_motor, BDC_MCPWM_DUTY_TICK_MIN +
                                                              (currentThrust < 0 ? -currentThrust : currentThrust) * (BDC_MCPWM_DUTY_TICK_MAX - BDC_MCPWM_DUTY_TICK_MIN) / 100));

        // Set motor direction
        if (currentThrust == 0)
        {
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
                    pwm_duty = BDC_MCPWM_DUTY_TICK_MIN_HOLD;
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
    esp_err_t ret;

    // Bluetooth requires NVS to connect to previously paired devices, initialize it before Bluetooth
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    hid_init();

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

    // xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, &xTask1);
    xTaskCreate(&steering_motor_task, "steering_motor_task", 6 * 1024, NULL, 3, nullptr); // make sure to allocate enough stack space for the task
    xTaskCreate(&thrust_motor_task, "thrust_motor_task", 6 * 1024, NULL, 3, nullptr);

    led_indicator_gpio_config_t led_indicator_gpio_config = {
        .is_active_level_high = true,
        .gpio_num = GPIO_NUM_26, /**< num of GPIO */
    };

    led_indicator_config_t config = {
        .mode = LED_GPIO_MODE,
        .led_indicator_gpio_config = &led_indicator_gpio_config,
        .blink_lists = default_led_indicator_blink_lists,
        .blink_list_num = (uint16_t)DEFAULT_BLINK_LIST_NUM,
    };
    led_indicator_handle_t led_handle = led_indicator_create(&config);
    led_indicator_start(led_handle, BLINK_FACTORY_RESET);

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

    for (;;)
    {
        // Direction direction;

        // if (xSteerQueue != NULL)
        // {
        //     direction = Direction::LEFT;
        //     if (xQueueSend(xSteerQueue, &direction, (TickType_t)1) != pdPASS)
        //     {
        //         ESP_LOGE(TAG, "Failed to send to queue");
        //     }
        // }
        ESP_ERROR_CHECK(gpio_set_level(PIN_LED, 1));
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_ERROR_CHECK(gpio_set_level(PIN_LED, 0));
        vTaskDelay(pdMS_TO_TICKS(1000));
        // if (xSteerQueue != NULL)
        // {
        //     direction = Direction::STRAIGHT;
        //     if (xQueueSend(xSteerQueue, &direction, (TickType_t)1) != pdPASS)
        //     {
        //         ESP_LOGE(TAG, "Failed to send to queue");
        //     }
        // }

        // if(xThrustQueue != NULL)
        // {
        //     int thrust = 100;
        //     if (xQueueSend(xThrustQueue, &thrust, (TickType_t)1) != pdPASS)
        //     {
        //         ESP_LOGE(TAG, "Failed to send to queue");
        //     }
        // }
        // vTaskDelay(pdMS_TO_TICKS(5000));
        // if(xThrustQueue != NULL)
        // {
        //     int thrust = -100;
        //     if (xQueueSend(xThrustQueue, &thrust, (TickType_t)1) != pdPASS)
        //     {
        //         ESP_LOGE(TAG, "Failed to send to queue");
        //     }
        // }
        // vTaskDelay(pdMS_TO_TICKS(5000));

        // bdc_motor_brake() causes both outputs to be high.
        //  bdc_motor_coast() causes both outputs to be low.  equivalent to setting motor speed to 0.
        //       ESP_ERROR_CHECK(bdc_motor_forward(motor2));
    }

    vTaskDelay(portMAX_DELAY);
}

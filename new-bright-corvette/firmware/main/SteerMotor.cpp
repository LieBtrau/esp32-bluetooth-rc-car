#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "SteerMotor.h"
#include "bdc_motor.h"

static const char *TAG = "SteerMotor";

static QueueHandle_t xSteerQueue = NULL;
static bdc_motor_config_t motor1_config;
static const uint32_t BDC_MCPWM_FREQ_HZ = 20000;        // PWM frequency

static void steering_motor_task(void *pvParameters);


bool SteerMotor::init(uint32_t pin_A, uint32_t pin_B)
{
    motor1_config = {
        .pwma_gpio_num = pin_A,
        .pwmb_gpio_num = pin_B,
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    };

    xSteerQueue = xQueueCreate(10, sizeof(Direction));
    if (xSteerQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue");
        return false;
    }
    xTaskCreate(&steering_motor_task, "steering_motor_task", 6 * 1024, NULL, 3, nullptr); // make sure to allocate enough stack space for the task
    return true;
}

bool SteerMotor::setDirection(Direction direction)
{
    if (xSteerQueue == NULL)
    {
        return false;
    }
    if (xQueueSend(xSteerQueue, &direction, (TickType_t)1) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to send to xSteerQueue");
        return false;
    }
    return true;
}

void steering_motor_task(void *pvParameters)
{
    const uint32_t BDC_MCPWM_TIMER_RESOLUTION_HZ = 10e6;                                        // 10MHz
    const uint32_t BDC_MCPWM_DUTY_TICK_MAX = BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ; // maximum value we can set for the duty cycle, in ticks
    const uint32_t BDC_MCPWM_DUTY_TICK_MIN_HOLD = 80 * BDC_MCPWM_DUTY_TICK_MAX / 100;           // minimum PWM value needed for the motor to hold position

    ESP_LOGI(TAG, "Create DC motors");
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
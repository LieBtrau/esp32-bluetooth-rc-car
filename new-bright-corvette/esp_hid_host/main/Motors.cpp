#include "bdc_motor.h"
#include "Motors.h"
#include "pins.h"

#include "freertos/FreeRTOS.h"

static const char *TAG = "Motors";
static QueueHandle_t xSteerQueue = NULL;
static QueueHandle_t xThrustQueue = NULL;

void initMotors()
{
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
}

QueueHandle_t getSteerQueue()
{
    return xSteerQueue;
}

QueueHandle_t getThrustQueue()
{
    return xThrustQueue;
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
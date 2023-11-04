#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ThrustMotor.h"
#include "bdc_motor.h"
#include "esp_log.h"

static const char *TAG = "SteerMotor";

static QueueHandle_t xThrustQueue = NULL;
static bdc_motor_config_t motor1_config;
static const uint32_t BDC_MCPWM_FREQ_HZ = 20000;        // PWM frequency

static void thrust_motor_task(void *pvParameters);

bool ThrustMotor::init(uint32_t pin_A, uint32_t pin_B)
{
    xThrustQueue = xQueueCreate(10, sizeof(int));
    if (xThrustQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue");
        return false;
    }
    motor1_config = {
        .pwma_gpio_num = pin_A,
        .pwmb_gpio_num = pin_B,
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    };
    xTaskCreate(&thrust_motor_task, "thrust_motor_task", 6 * 1024, NULL, 3, nullptr);
    return true;
}

void ThrustMotor::stop()
{
    setSpeed(0);
}

bool ThrustMotor::setSpeed(int speed)
{
    if (xThrustQueue == NULL)
    {
        return false;
    }
    if(speed == _currentSpeed)
    {
        return true;
    }
    _currentSpeed = speed;
    if (xQueueSend(xThrustQueue, &speed, (TickType_t)1) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to send to xThrustQueue");
        return false;
    }
    return true;
}

void thrust_motor_task(void *pvParameters)
{
    const int32_t BDC_MAX_THRUST = 512;
    const uint32_t BDC_MCPWM_TIMER_RESOLUTION_HZ = 10e6;                                        // 10MHz
    const uint32_t BDC_MCPWM_FREQ_HZ = 20000;                                                   // PWM frequency
    const uint32_t BDC_MCPWM_DUTY_TICK_MAX = BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ; // maximum value we can set for the duty cycle, in ticks
    const uint32_t BDC_MCPWM_DUTY_TICK_MIN = BDC_MCPWM_DUTY_TICK_MAX / 2;                       // minimum PWM value needed for the motor to spin
    ESP_LOGI(TAG, "Create DC motors");
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

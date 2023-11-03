#include "Button.h"
#include "iot_button.h"
#include "esp_log.h"

static const char *TAG = "Button";

static void button_long_press_cb(void *arg, void *usr_data);
static void button_single_click_cb(void *arg, void *usr_data);

void initButton(uint32_t pinButton)
{
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = pinButton,
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

}

void button_long_press_cb(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "BUTTON_LONG_PRESS_UP");
}

void button_single_click_cb(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "BUTTON_SINGLE_CLICK");
}
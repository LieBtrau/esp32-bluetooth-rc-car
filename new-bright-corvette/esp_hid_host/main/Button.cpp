#include "Button.h"
#include "esp_log.h"
#include "iot_button.h"

static const char *TAG = "Button";

static void button_long_press_cb(void *arg, void *usr_data);
static void button_single_click_cb(void *arg, void *usr_data);
static EventGroupHandle_t buttonEvents = NULL;

enum ButtonEventBitmask
{
    BITMASK_SINGLE_CLICK = 0x1,
    BITMASK_LONG_PRESS = 0x2
};

void Button::init(int32_t pinButton)
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
    buttonEvents = xEventGroupCreate();
    if( buttonEvents == NULL )
    {
        /* The event group was not created because there was insufficient
        FreeRTOS heap available. */
        ESP_LOGE(TAG, "Button event group create failed");
        return;
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

ButtonEvent Button::waitEvent()
{
    EventBits_t bits = xEventGroupWaitBits(buttonEvents, BITMASK_SINGLE_CLICK | BITMASK_LONG_PRESS, pdTRUE, pdFALSE, pdMS_TO_TICKS(10));
    if (bits & BITMASK_SINGLE_CLICK)
    {
        return ButtonEvent::SingleClick;
    }
    else if (bits & BITMASK_LONG_PRESS)
    {
        return ButtonEvent::LongPress;
    }
    else
    {
        return ButtonEvent::None;
    }
}

void button_single_click_cb(void *arg, void *usr_data)
{
    xEventGroupSetBits(buttonEvents, BITMASK_SINGLE_CLICK);
}

void button_long_press_cb(void *arg, void *usr_data)
{
    xEventGroupSetBits(buttonEvents, BITMASK_LONG_PRESS);
}


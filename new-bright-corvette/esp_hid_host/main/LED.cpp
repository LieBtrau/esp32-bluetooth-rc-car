#include "LED.h"
#include "led_indicator_blink_default.h"

void LED::init(int32_t pin)
{
    led_indicator_gpio_config_t led_indicator_gpio_config = {
        .is_active_level_high = true,
        .gpio_num = pin, /**< num of GPIO */
    };

    led_indicator_config_t config = {
        .mode = LED_GPIO_MODE,
        .led_indicator_gpio_config = &led_indicator_gpio_config,
        .blink_lists = default_led_indicator_blink_lists,
        .blink_list_num = (uint16_t)DEFAULT_BLINK_LIST_NUM,
    };
    _led_handle = led_indicator_create(&config);
}

void LED::on()
{
    led_indicator_stop(_led_handle, _blink_type);
    led_indicator_start(_led_handle, BLINK_CONNECTED);
    _blink_type = BLINK_CONNECTED;
}

void LED::blink()
{
    led_indicator_stop(_led_handle, _blink_type);
    led_indicator_start(_led_handle, BLINK_CONNECTING);
    _blink_type = BLINK_CONNECTING;
}

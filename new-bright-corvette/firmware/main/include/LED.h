#pragma once
#include "led_indicator.h"
#include "led_indicator_blink_default.h"

class LED
{
public:
    void init(int32_t pin);
    void on();
    void off();
    void blinkSlow();
    void blinkFast();

private:
    led_indicator_handle_t _led_handle = nullptr;
    int _blink_type = BLINK_MAX;
};
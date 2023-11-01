#pragma once
#include "led_indicator.h"

class LED
{
public:
    void init(int32_t pin);
    void on();
    void off();
    void blink();

private:
    led_indicator_handle_t _led_handle = nullptr;
    int _blink_type = 0;
};
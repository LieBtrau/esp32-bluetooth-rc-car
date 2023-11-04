#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

enum class ButtonEvent
{
    None,
    SingleClick,
    LongPress
};

class Button
{
public:
    void init(int32_t pinButton);
    ButtonEvent waitEvent();
private:
    int _startPressCount = 0;
};

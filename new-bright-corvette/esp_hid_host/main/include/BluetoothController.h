#pragma once

#include "nintendo_switch_controller.h"

enum class Bluetooth_states
{
    UNPAIRED,
    PAIRED,
    CONNECTED,
    CONNECTING
};

class BluetoothController
{
public:
    bool init();
    bool eventReady(uni_gamepad_t *gamepad);
    Bluetooth_states getState();
    void unpair();
};

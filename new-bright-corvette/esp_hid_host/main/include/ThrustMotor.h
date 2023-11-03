#pragma once


#include "stdint.h"

class ThrustMotor
{
public:
    bool init(uint32_t pin_A, uint32_t pin_B);
    bool setSpeed(int speed);
private:
    
};



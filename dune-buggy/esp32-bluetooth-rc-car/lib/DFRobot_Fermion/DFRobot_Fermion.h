#pragma once

#include "Arduino.h"

class DFRobot_Fermion
{
private:
    int _pinPwm;
    int _pinDir;
    int _pwmChannel;
    int _maxSpeed;
public:
    DFRobot_Fermion(int pinPwm, int pinDir);
    ~DFRobot_Fermion();
    void init(int pwmChannel, int pwmFrequency, int pwmResolution);
    void forward(int speed);
    void reverse(int speed);
    void brake();
};



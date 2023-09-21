/**
 * Control for the DFRobot Fermion 2x1.2A DC Motor Driver TB6612FNG SKU DRI0044
 */
#include "DFRobot_Fermion.h"

///@param pinPwm PWM-pin
///@param pinDir direction pin
DFRobot_Fermion::DFRobot_Fermion(int pinPwm, int pinDir) : _pinPwm(pinPwm), _pinDir(pinDir)
{
}

DFRobot_Fermion::~DFRobot_Fermion()
{
}

///@param pwmChannel [0 - 15]
///@param pwmFrequency frequency in Hz
///@param pwmResolution [1 - 16]
void DFRobot_Fermion::init(int pwmChannel, int pwmFrequency, int pwmResolution)
{
    _pwmChannel = pwmChannel;
    _maxSpeed = (1 << pwmResolution) - 1;
    ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(_pinPwm, pwmChannel);
    ledcWrite(pwmChannel, 0);
    pinMode(_pinDir, OUTPUT);
    digitalWrite(_pinDir, LOW);
}

void DFRobot_Fermion::forward(int speed)
{
    speed = speed > _maxSpeed ? _maxSpeed : speed;
    ledcWrite(_pwmChannel, speed);
    digitalWrite(_pinDir, HIGH);
}

void DFRobot_Fermion::reverse(int speed)
{
    speed = speed > _maxSpeed ? _maxSpeed : speed;
    ledcWrite(_pwmChannel, speed);
    digitalWrite(_pinDir, LOW);
}

void DFRobot_Fermion::brake()
{
    ledcWrite(_pwmChannel, 0);
}

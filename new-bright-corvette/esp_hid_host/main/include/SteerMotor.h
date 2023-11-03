#pragma once

enum class Direction
{
    LEFT,
    RIGHT,
    STRAIGHT
};

class SteerMotor
{
public:
    bool init(uint32_t pin_A, uint32_t pin_B);
    bool setDirection(Direction direction);
private:
    
};

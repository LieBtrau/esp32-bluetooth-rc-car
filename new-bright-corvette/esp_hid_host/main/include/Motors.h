#pragma once

#include "nintendo_switch_controller.h" // wrong header, but it works....

enum class Direction
{
    LEFT,
    RIGHT,
    STRAIGHT
};

void initMotors();
void steering_motor_task(void *pvParameters);
void thrust_motor_task(void *pvParameters);
QueueHandle_t getSteerQueue();
QueueHandle_t getThrustQueue();
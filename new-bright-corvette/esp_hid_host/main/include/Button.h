#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif



void initButton(uint32_t pinButton, EventGroupHandle_t buttonEvents);

#ifdef __cplusplus
}
#endif
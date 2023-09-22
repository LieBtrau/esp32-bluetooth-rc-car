/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2019 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <esp_bt_defs.h>
#include <esp_hid_common.h>
#include "esp_hidh.h"
#include "uni_gamepad.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    esp_bd_addr_t bda;
    esp_hid_transport_t transport;
    bool is_connected;
    uni_gamepad_t gamepad;
    uni_gamepad_t last_gamepad;
    QueueHandle_t buttonStateQueue;
}nintendo_switch_controller_t;

// Taken from Linux kernel: hid-nintendo.c
enum switch_proto_reqs {
    /* Input Reports */
    SWITCH_INPUT_SUBCMD_REPLY = 0x21,
    SWITCH_INPUT_IMU_DATA = 0x30,
    SWITCH_INPUT_MCU_DATA = 0x31,
    SWITCH_INPUT_BUTTON_EVENT = 0x3F,
};

void nintendo_switch_controller_init(nintendo_switch_controller_t* controller, esp_bd_addr_t address);
void nintendo_switch_controller_callback(esp_hidh_event_t event, esp_hidh_event_data_t *param);
void nintendo_switch_controller_connect(nintendo_switch_controller_t* controller);


#ifdef __cplusplus
}
#endif
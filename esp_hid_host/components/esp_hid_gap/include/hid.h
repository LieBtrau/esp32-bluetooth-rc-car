#pragma once

#include "freertos/task.h"
#include "esp_bt_defs.h"
#include "esp_hid_common.h"

void hid_init(TaskHandle_t* task);
bool scan_hid_device(esp_bd_addr_t address_to_find, uint32_t scan_duration_seconds, esp_ble_addr_type_t* addr_type);
void hid_connect(esp_bd_addr_t address, esp_hid_transport_t transport, esp_ble_addr_type_t addr_type);
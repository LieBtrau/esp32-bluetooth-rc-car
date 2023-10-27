#pragma once

#include "freertos/task.h"
#include "esp_bt_defs.h"
#include "esp_hid_common.h"
#include "esp_hidh.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    esp_bd_addr_t bda;
    void (*callback)(esp_hidh_event_t event, esp_hidh_event_data_t *param);
} callback_entry_t;

void hid_init();
bool scan_hid_device(uint32_t cod_major, uint32_t cod_minor, uint32_t scan_duration_seconds, esp_bd_addr_t* found_bda);
void hid_connect(esp_bd_addr_t address, esp_hid_transport_t transport, esp_ble_addr_type_t addr_type);
void add_callback(esp_bd_addr_t address, void (*callback)(esp_hidh_event_t event, esp_hidh_event_data_t *param));


#ifdef __cplusplus
}
#endif
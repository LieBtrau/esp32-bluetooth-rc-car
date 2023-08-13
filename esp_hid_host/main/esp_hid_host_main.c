/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "uni_hid_parser_switch.h"
#include "hid.h"

static const char *TAG = "ESP_HIDH_DEMO";
static TaskHandle_t xTask1;

void hid_demo_task(void *pvParameters)
{
    nintendo_switch_controller_t controller;
    esp_bd_addr_t bluetooth_address = {0x98, 0xb6, 0xe9, 0x54, 0x85, 0x38};
    nintendo_switch_controller_init(&controller, bluetooth_address);

    
    hid_connect(controller.bda, controller.transport, 0);

    for (;;)
    {
        // Keep trying to find the controller until it is found
        // If not yet paired, press home button on controller 10+ seconds so it can be found.
        // while (!scan_hid_device(bluetooth_address, 2, &addr_type))
        //     ;

        // Wait for the controller to be disconnected
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret;

    // Bluetooth requires NVS, initialize it before Bluetooth
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    hid_init(&xTask1);

    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, &xTask1);
}

/**
 * @file esp_hid_host_main.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-14
 * 
 * @copyright Copyright (c) 2023
 * @details
 *  * heavily based on [https://github.com/espressif/esp-idf/tree/3befd5fff7/examples/bluetooth/esp_hid_host]
 *  * other bluetooth HID implementation, as described in the ESP bluetooth-hid API [https://github.com/nlemoing/bt_kb_receiver/tree/master]
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

#include "nintendo_switch_controller.h"
#include "hid.h"
#include "esp_hidh.h"

static const char *TAG = "ESP_HIDH_DEMO";
static TaskHandle_t xTask1;

nintendo_switch_controller_t controller;

void hid_demo_task(void *pvParameters)
{
    esp_bd_addr_t bluetooth_address = {0x98, 0xb6, 0xe9, 0x54, 0x85, 0x38};

    scan_hid_device(bluetooth_address, 10, NULL);
    nintendo_switch_controller_init(&controller, bluetooth_address);
    nintendo_switch_controller_connect(&controller);

    uni_gamepad_t gamepad;
    for (;;)
    {
        if (xQueueReceive(controller.buttonStateQueue, &gamepad, (TickType_t)10) == pdPASS)
        {
            ESP_LOGI(TAG, "Button state: %d", gamepad.dpad);
        }

    }
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
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
    hid_init();

    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, &xTask1);
}

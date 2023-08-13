#include "uni_hid_parser_switch.h"
#include "string.h"
#include "esp_log.h"

const char* TAG="uni_hid_parser_switch";

struct switch_report_3f_s {
    uint8_t buttons_main;
    uint8_t buttons_aux;
    uint8_t hat;
    uint8_t x_lsb;
    uint8_t x_msb;
    uint8_t y_lsb;
    uint8_t y_msb;
    uint8_t rx_lsb;
    uint8_t rx_msb;
    uint8_t ry_lsb;
    uint8_t ry_msb;
} __attribute__((packed));

void nintendo_switch_controller_init(nintendo_switch_controller_t* controller, esp_bd_addr_t address)
{
    memcpy(controller->bda, address, sizeof(esp_bd_addr_t));
    ESP_LOG_BUFFER_HEX(TAG, controller->bda, sizeof(esp_bd_addr_t));
    controller->transport = ESP_HID_TRANSPORT_BT;
}
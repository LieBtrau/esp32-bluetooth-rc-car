#include "nintendo_switch_controller.h"
#include "string.h"
#include "esp_log.h"
#include "hid.h"

const char *TAG = "nintendo_switch_controller";

struct switch_report_3f_s
{
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

static void parse_input_buttons(nintendo_switch_controller_t *controller, uint8_t *data, size_t len);

static nintendo_switch_controller_t **controllerTable;
static uint8_t controllerTableSize = 0;

void nintendo_switch_controller_init(nintendo_switch_controller_t *controller, esp_bd_addr_t address)
{
    memcpy(controller->bda, address, sizeof(esp_bd_addr_t));
    ESP_LOG_BUFFER_HEX(TAG, controller->bda, sizeof(esp_bd_addr_t));
    controller->transport = ESP_HID_TRANSPORT_BT;
    add_callback(controller->bda, nintendo_switch_controller_callback);

    // Add the controller to the table
    controllerTableSize++;
    controllerTable = realloc(controllerTable, controllerTableSize * sizeof(nintendo_switch_controller_t *));
    memcpy(&controllerTable[controllerTableSize - 1], &controller, sizeof(nintendo_switch_controller_t *));
}

void nintendo_switch_controller_connect(nintendo_switch_controller_t *controller)
{
    hid_connect(controller->bda, controller->transport, 0);
}

void nintendo_switch_controller_callback(esp_hidh_event_t event, esp_hidh_event_data_t *param)
{
    const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
    nintendo_switch_controller_t *controller = NULL;

    // Find the controller in the table
    for (int i = 0; i < controllerTableSize; i++)
    {
        if (memcmp(controllerTable[i]->bda, bda, sizeof(esp_bd_addr_t)) == 0)
        {
            controller = controllerTable[i];
            break;
        }
    }
    if (controller == NULL)
    {
        ESP_LOGE(TAG, "Controller not found in table");
        return;
    }

    switch (event)
    {
    case ESP_HIDH_OPEN_EVENT:
    {
        controller->is_connected = (param->open.status == ESP_OK);
        if (param->open.status == ESP_OK)
        {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            // esp_hidh_dev_dump(param->open.dev, stdout);
            controller->is_connected = true;
        }
        else
        {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT:
    {
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT:
    {
        switch (param->input.report_id)
        {
        case SWITCH_INPUT_BUTTON_EVENT:
        {
            parse_input_buttons(controller, param->input.data, param->input.length);
            break;
        }
        default:
        {
            ESP_LOGI(TAG, "Unknown usage: %02x", param->input.usage);
            break;
        }
        }
        break;
    }
    case ESP_HIDH_FEATURE_EVENT:
    {
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT:
    {
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        controller->is_connected = false;
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

void parse_input_buttons(nintendo_switch_controller_t *controller, uint8_t *data, size_t len)
{
    if (len != 11)
    {
        ESP_LOGE(TAG, "Invalid input length: %d", len);
        return;
    }
    ESP_LOG_BUFFER_HEX(TAG, data, len);
}
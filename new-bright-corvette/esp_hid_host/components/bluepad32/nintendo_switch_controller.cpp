#include "nintendo_switch_controller.h"
#include "string.h"
#include "esp_log.h"
#include "hid.h"

static const char *TAG = "nintendo_switch_controller";
const int AXIS_NORMALIZE_RANGE = 1024; // 10-bit resolution (1024)
static NintendoSwitchController **controllerTable;
static uint8_t controllerTableSize = 0;

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

static void nintendo_switch_controller_callback(esp_hidh_event_t event, esp_hidh_event_data_t *param);

NintendoSwitchController::NintendoSwitchController(const esp_bd_addr_t address) : transport(ESP_HID_TRANSPORT_BT), is_connected(false)
{
    memcpy(bda, address, sizeof(esp_bd_addr_t));
    add_callback(bda, nintendo_switch_controller_callback);
    buttonStateQueue = xQueueCreate(10, sizeof(uni_gamepad_t));
    if (buttonStateQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create buttonStateQueue");
    }

    // Add the controller to the table
    NintendoSwitchController **newTable = (NintendoSwitchController **)realloc(controllerTable, sizeof(NintendoSwitchController *) * (controllerTableSize + 1));
    if (newTable == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for controller table");
        return;
    }
    controllerTable = newTable;
    controllerTable[controllerTableSize] = this;
    controllerTableSize++;
}

void NintendoSwitchController::connect()
{
    ESP_LOGI(TAG, "Connecting to controller");
    hid_connect(bda, transport, BLE_ADDR_TYPE_PUBLIC);
}

bool NintendoSwitchController::hasAddress(const uint8_t *address)
{
    return memcmp(bda, address, sizeof(esp_bd_addr_t)) == 0;
}

/**
 * @brief Parse the input report for buttons
 *
 * @param data
 * @param len
 * @details Source code from [Bluepad32](https://github.com/ricardoquesada/bluepad32/blob/main/src/components/bluepad32/uni_hid_parser_switch.c)
 */
void NintendoSwitchController::parse_input_buttons(uint8_t *report, size_t len)
{
    if (len != 11)
    {
        ESP_LOGE(TAG, "Invalid input length: %d", len);
        return;
    }
    // ESP_LOG_BUFFER_HEX(TAG, report, len);

    uni_gamepad_t gamepad;
    memset(&gamepad, 0, sizeof(gamepad));

    const struct switch_report_3f_s *r = (const struct switch_report_3f_s *)report;

    // Button main
    gamepad.buttons |= (r->buttons_main & 0b00000001) ? BUTTON_A : 0;          // B
    gamepad.buttons |= (r->buttons_main & 0b00000010) ? BUTTON_B : 0;          // A
    gamepad.buttons |= (r->buttons_main & 0b00000100) ? BUTTON_X : 0;          // Y
    gamepad.buttons |= (r->buttons_main & 0b00001000) ? BUTTON_Y : 0;          // X
    gamepad.buttons |= (r->buttons_main & 0b00010000) ? BUTTON_SHOULDER_L : 0; // L
    gamepad.buttons |= (r->buttons_main & 0b00100000) ? BUTTON_SHOULDER_R : 0; // R
    gamepad.buttons |= (r->buttons_main & 0b01000000) ? BUTTON_TRIGGER_L : 0;  // ZL
    gamepad.buttons |= (r->buttons_main & 0b10000000) ? BUTTON_TRIGGER_R : 0;  // ZR

    // Button aux
    gamepad.misc_buttons |= (r->buttons_aux & 0b00000001) ? MISC_BUTTON_BACK : 0;   // -
    gamepad.misc_buttons |= (r->buttons_aux & 0b00000010) ? MISC_BUTTON_HOME : 0;   // +
    gamepad.buttons |= (r->buttons_aux & 0b00000100) ? BUTTON_THUMB_L : 0;          // Thumb L
    gamepad.buttons |= (r->buttons_aux & 0b00001000) ? BUTTON_THUMB_R : 0;          // Thumb R
    gamepad.misc_buttons |= (r->buttons_aux & 0b00010000) ? MISC_BUTTON_SYSTEM : 0; // Home
    gamepad.misc_buttons |= (r->buttons_aux & 0b00100000) ? 0 : 0;                  // Capture (unused)

    // Dpad
    switch (r->hat)
    {
    case 0x0f:
    case 0xff:
    case 0x08:
        // joy.up = joy.down = joy.left = joy.right = 0;
        break;
    case 0:
        gamepad.dpad = DPAD_UP;
        break;
    case 1:
        gamepad.dpad = DPAD_UP | DPAD_RIGHT;
        break;
    case 2:
        gamepad.dpad = DPAD_RIGHT;
        break;
    case 3:
        gamepad.dpad = DPAD_RIGHT | DPAD_DOWN;
        break;
    case 4:
        gamepad.dpad = DPAD_DOWN;
        break;
    case 5:
        gamepad.dpad = DPAD_DOWN | DPAD_LEFT;
        break;
    case 6:
        gamepad.dpad = DPAD_LEFT;
        break;
    case 7:
        gamepad.dpad = DPAD_LEFT | DPAD_UP;
        break;
    default:
        ESP_LOGE(TAG, "Error parsing hat value: 0x%02x\n", r->hat);
        break;
    }

    // Axis
    gamepad.axis_x = ((r->x_msb << 8) | r->x_lsb) * AXIS_NORMALIZE_RANGE / 65536 - AXIS_NORMALIZE_RANGE / 2;
    gamepad.axis_y = ((r->y_msb << 8) | r->y_lsb) * AXIS_NORMALIZE_RANGE / 65536 - AXIS_NORMALIZE_RANGE / 2;
    gamepad.axis_rx = ((r->rx_msb << 8) | r->rx_lsb) * AXIS_NORMALIZE_RANGE / 65536 - AXIS_NORMALIZE_RANGE / 2;
    gamepad.axis_ry = ((r->ry_msb << 8) | r->ry_lsb) * AXIS_NORMALIZE_RANGE / 65536 - AXIS_NORMALIZE_RANGE / 2;

    // Every button press causes 5 reports to be sent. We only want to add one event per button press to the queue.
    if (memcmp(&gamepad, &last_gamepad, sizeof(gamepad)) != 0)
    {
        xQueueSend(buttonStateQueue, (void *)&gamepad, (TickType_t)0);
    }
    memcpy(&last_gamepad, &gamepad, sizeof(gamepad));
}

bool NintendoSwitchController::isUpdateAvailable(uni_gamepad_t *gamepad)
{
    return xQueueReceive(buttonStateQueue, gamepad, (TickType_t)10) == pdPASS;
}

void nintendo_switch_controller_callback(esp_hidh_event_t event, esp_hidh_event_data_t *param)
{
    const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
    NintendoSwitchController *controller = NULL;

    // Find the controller in the table
    for (int i = 0; i < controllerTableSize; i++)
    {
        if (controllerTable[i]->hasAddress(bda))
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
        controller->setConnected(param->open.status == ESP_OK);
        if (param->open.status == ESP_OK)
        {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            // esp_hidh_dev_dump(param->open.dev, stdout);
            controller->setConnected(true);
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
            controller->parse_input_buttons(param->input.data, param->input.length);
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
        controller->setConnected(false);
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}
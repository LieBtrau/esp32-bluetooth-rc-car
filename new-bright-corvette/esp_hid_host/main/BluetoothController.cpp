#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "hid.h"
#include "NonVolatileStorage.h"
#include "BluetoothController.h"

static const char *TAG = "BluetoothController";

static NonVolatileStorage prefs;
static NintendoSwitchController controller;
static QueueHandle_t xBluetoothQueue = NULL;
static Bluetooth_states bluetooth_state = Bluetooth_states::UNPAIRED;

static void hid_demo_task(void *pvParameters);

bool BluetoothController::init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    xBluetoothQueue = xQueueCreate(10, sizeof(uni_gamepad_t));
    if (xBluetoothQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue");
        return false;
    }
    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, nullptr);

    return true;
}

bool BluetoothController::eventReady(uni_gamepad_t *gamepad)
{
    if (xBluetoothQueue != NULL)
    {
        if (xQueueReceive(xBluetoothQueue, gamepad, (TickType_t)1) == pdPASS)
        {
            return true;
        }
    }
    else
    {
        ESP_LOGE(TAG, "xBluetoothQueue is NULL");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    return false;
}

Bluetooth_states BluetoothController::getState()
{
    return bluetooth_state;
}

void BluetoothController::unpair()
{
    prefs.setBDA((const esp_bd_addr_t *)"\x00\x00\x00\x00\x00\x00");
    bluetooth_state = Bluetooth_states::UNPAIRED;
}

void hid_demo_task(void *pvParameters)
{
    const int COD_MAJOR = 0x05; // Peripheral
    const int COD_MINOR = 0x02; // Gamepad
    esp_bd_addr_t bluetooth_address;
    bluetooth_state = Bluetooth_states::UNPAIRED;

    // Bluetooth driver requires NVS to connect to previously paired devices, initialize it before Bluetooth
    // Execute "esptool.py --port /dev/ttyUSB1 erase_flash" to erase the flash
    if (prefs.init() == ESP_OK)
    {
        if (prefs.getBDA(&bluetooth_address))
        {
            ESP_LOGI(TAG, "Found bluetooth address in NVS:" ESP_BD_ADDR_STR " ", ESP_BD_ADDR_HEX(bluetooth_address));
            controller.setBda(bluetooth_address);
            // controller.setBda((uint8_t *)"\x98\xb6\xe9\x54\x85\x38");
            bluetooth_state = Bluetooth_states::PAIRED;
        }
        else
        {
            ESP_LOGI(TAG, "No bluetooth address found in NVS");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize NVS");
    }

    hid_init();
    uni_gamepad_t gamepad;
    for (;;)
    {
        switch (bluetooth_state)
        {
        case Bluetooth_states::UNPAIRED:
            ESP_LOGI(TAG, "Waiting for pairing");
            if (scan_hid_device(COD_MAJOR, COD_MINOR, 10, &bluetooth_address))
            {
                ESP_LOGI(TAG, "Found bluetooth device:  " ESP_BD_ADDR_STR " ", ESP_BD_ADDR_HEX(bluetooth_address));
                prefs.setBDA(&bluetooth_address);
                bluetooth_state = Bluetooth_states::PAIRED;
            }
            else
            {
                ESP_LOGE(TAG, "Failed to find bluetooth device");
            }
            break;
        case Bluetooth_states::PAIRED:
            controller.setBda(bluetooth_address);
            controller.connect(); // Reconnection will be attempted automatically by the driver.  Don't put this inside the loop.
            bluetooth_state = Bluetooth_states::CONNECTING;
            break;
        case Bluetooth_states::CONNECTING:
            vTaskDelay(pdMS_TO_TICKS(100));
            if (controller.isConnected())
            {
                ESP_LOGI(TAG, "Connected");
                bluetooth_state = Bluetooth_states::CONNECTED;
            }
            break;
        case Bluetooth_states::CONNECTED:
            if (controller.isUpdateAvailable(&gamepad))
            {
                if (xBluetoothQueue != NULL)
                {
                    if (xQueueSend(xBluetoothQueue, &gamepad, (TickType_t)1) != pdPASS)
                    {
                        ESP_LOGE(TAG, "Failed to send to queue");
                    }
                }
            }
            if (!controller.isConnected())
            {
                bluetooth_state = Bluetooth_states::CONNECTING;
                ESP_LOGI(TAG, "Disconnected");
            }
            break;
        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
    vTaskDelete(NULL);
}
#include <string.h>

#include "esp_hidh.h"
#include "esp_hid_gap.h"
#include "esp_hidh_bluedroid.h"
#include "hid.h"

static const char *TAG = "HID";

static callback_entry_t *callback_table = NULL;
static int callback_table_size = 0;

void add_callback(esp_bd_addr_t address, void (*callback)(esp_hidh_event_t event, esp_hidh_event_data_t *param))
{
    callback_table_size++;
    callback_table = realloc(callback_table, callback_table_size * sizeof(callback_entry_t));
    callback_table[callback_table_size - 1].callback = callback;
    memcpy(callback_table[callback_table_size - 1].bda, address, sizeof(esp_bd_addr_t));
}

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    for (int i = 0; i < callback_table_size; i++)
    {
        if (memcmp(callback_table[i].bda, esp_hidh_dev_bda_get(param->open.dev), sizeof(esp_bd_addr_t)) == 0)
        {
            callback_table[i].callback(event, param);
        }
    }
}


/**
 * @brief Initialize HID host
 * @todo check to replace with new API as shown in the [hid device example](https://github.com/espressif/esp-idf/blob/db4308888d30ccaa93d5492a323cd85665f24831/examples/bluetooth/bluedroid/classic_bt/bt_hid_mouse_device/main/main.c)
 * @param taskHandle 
 */
void hid_init()
{
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&config));

}

bool scan_hid_device(esp_bd_addr_t address_to_find, uint32_t scan_duration_seconds, esp_ble_addr_type_t* addr_type)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;

    ESP_LOGI(TAG, "SCAN...");
    // start scan for HID devices
    esp_hid_scan(scan_duration_seconds, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (!results_len)
    {
        return false;
    }
    esp_hid_scan_result_t *r = results;
    esp_hid_scan_result_t *cr = NULL;
    while (r)
    {
        if (!memcmp(r->bda, address_to_find, sizeof(esp_bd_addr_t)))
        {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE)
            {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT)
            {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            if(addr_type != NULL)
            {
                *addr_type = r->ble.addr_type;
            }
            printf("\n");
            break;
        }
        r = r->next;
    }
    // free the results
    esp_hid_scan_results_free(results);
    return cr != NULL;
}


/**
 * @brief Connect to HID device
 * 
 * @param address 
 * @param transport 
 * @param addr_type 
 * @details Only need to call this once.  The HID host will automatically try to reconnect to the device if it disconnects.
 */
void hid_connect(esp_bd_addr_t address, esp_hid_transport_t transport, esp_ble_addr_type_t addr_type)
{
    ESP_LOGI(TAG, "CONNECT: " ESP_BD_ADDR_STR ", ADDR_TYPE: '%s'", ESP_BD_ADDR_HEX(address), ble_addr_type_str(addr_type));
    esp_hidh_dev_open(address, transport, addr_type);
}
#include "NonVolatileStorage.h"
#include "nvs_flash.h"
#include "string.h"
#include "esp_log.h"

static constexpr const char *STORAGE_NAMESPACE = "storage";

esp_err_t NonVolatileStorage::init()
{
    esp_err_t err;
    nvs_handle_t my_handle;

    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK)
        return err;

    // Check if NVS contents is valid
    err = nvs_get_u8(my_handle, STORAGE_KEY_NVS_INITIALIZED, &nvsInitialized);
    if (err != ESP_OK)
        return err;

     // If NVS contents is valid, read BDA
    size_t required_size = STORAGE_BDA_SIZE;
    err = nvs_get_blob(my_handle, STORAGE_KEY_BDA, _bda, &required_size);
    if (err != ESP_OK)
        return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

bool NonVolatileStorage::getBDA(esp_bd_addr_t *bda)
{
    if (nvsInitialized == 0)
        return false;
    memcpy(bda, _bda, STORAGE_BDA_SIZE);
    return true;
}

bool NonVolatileStorage::setBDA(const esp_bd_addr_t *bda)
{
    esp_err_t err;
    nvs_handle_t my_handle;

    // Open NVS
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
        return false;

    // Write BDA
    err = nvs_set_blob(my_handle, STORAGE_KEY_BDA, bda, STORAGE_BDA_SIZE);
    if (err != ESP_OK)
        return false;

    // Write NVS initialized flag
    nvsInitialized = 1;
    err = nvs_set_u8(my_handle, STORAGE_KEY_NVS_INITIALIZED, nvsInitialized);
    if (err != ESP_OK)
        return false;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
        return false;

    // Close
    nvs_close(my_handle);
    memcpy(_bda, bda, STORAGE_BDA_SIZE);
    return true;
}
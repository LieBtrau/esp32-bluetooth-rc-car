#pragma once
#include "esp_bt_defs.h"
#include "esp_err.h"

class NonVolatileStorage
{
public:
    esp_err_t init();
    bool getBDA(esp_bd_addr_t *bda);
    bool setBDA(const esp_bd_addr_t *bda);
private:
    static constexpr const char *STORAGE_NAMESPACE = "storage";
    static constexpr const char *STORAGE_KEY_BDA = "bda";
    static constexpr const char *STORAGE_KEY_NVS_INITIALIZED = "nvsInitialized";
    static constexpr size_t STORAGE_BDA_SIZE = sizeof(esp_bd_addr_t);

    uint8_t nvsInitialized = 0;
    esp_bd_addr_t _bda={0};
};
set(srcs
        "BluetoothController.cpp"
        "esp_hid_host_main.cpp" 
        "NonVolatileStorage.cpp"
        "LED.cpp"
        "SteerMotor.cpp"
        "ThrustMotor.cpp"
        )
set(include_dirs "include")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES bluepad32 driver esp_hid esp_hid_gap 
                       PRIV_REQUIRES nvs_flash)

# https://stackoverflow.com/questions/58333319/cmakelists-add-definition-add-double-quotes-around-variable
execute_process(COMMAND git describe --always --dirty OUTPUT_VARIABLE GIT_VERSION)
string(REGEX REPLACE "\n$" "" GIT_VERSION "${GIT_VERSION}")
add_compile_definitions(VERSION="${GIT_VERSION}")



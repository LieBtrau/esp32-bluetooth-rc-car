set(srcs "esp_hid_host_main.c"
        )
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid bluepad32 esp_hid_gap
                       PRIV_REQUIRES nvs_flash)

# https://github.com/TobleMiner/oled-nametag-firmware/blob/main/main/CMakeLists.txt
# execute_process(COMMAND git describe --always --dirty OUTPUT_VARIABLE badge_app_version)
# add_compile_definitions(BADGE_APP_VERSION=${badge_app_version})

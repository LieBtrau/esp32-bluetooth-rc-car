set(srcs "nintendo_switch_controller.cpp")
set(include_dirs "include")

idf_component_register(SRCS  "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid esp_hid_gap)
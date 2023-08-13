set(srcs "esp_hid_gap.c")
set(include_dirs "include")

idf_component_register(SRCS "hid.c" "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid)
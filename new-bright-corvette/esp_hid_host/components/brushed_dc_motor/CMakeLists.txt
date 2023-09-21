set(srcs "dc_motor.c")
set(include_dirs "include")

idf_component_register(SRCS  "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES )
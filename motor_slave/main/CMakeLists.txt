set(srcs
    "app_main.c"
    "app_driver.c"
)

set(include_dirs
    "include"
)

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "${include_dirs}"
    REQUIRES
        motor_driver
        encoder_driver
        can_driver
        freertos
)

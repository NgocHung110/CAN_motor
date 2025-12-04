set(srcs "app_main.c"
                    "app_driver.c")
set(INCLUDE_DIRS "include")
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${INCLUDE_DIRS}"
                    ) 

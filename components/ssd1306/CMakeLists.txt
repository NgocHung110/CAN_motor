idf_component_register(
  SRCS "ssd1306.c" "fonts.c"
  INCLUDE_DIRS "include"
  REQUIRES esp_driver_i2c
)
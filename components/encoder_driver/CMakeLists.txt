idf_component_register(
  SRCS "encoder_driver.c"
  INCLUDE_DIRS "include"
  REQUIRES esp_driver_gpio esp_timer
)
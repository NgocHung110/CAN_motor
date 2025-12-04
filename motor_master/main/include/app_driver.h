#ifndef APP_DRIVER_H
#define APP_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "can_driver.h"  // Thêm include cho can_mode_t

// Cấu trúc dữ liệu góc
typedef struct {
    uint16_t current;
    uint16_t desired;
} angle_data_t;

// Khởi tạo driver
void app_driver_init(can_mode_t mode);

// Lấy giá trị encoder
uint16_t app_driver_encoder_get_desired(void);
uint16_t app_driver_encoder_get_current(void);

// CAN functions
void app_driver_can_receive_handler(void);
esp_err_t app_driver_can_send_desired_encoder(uint16_t value);
bool app_driver_can_get_current_encoder(uint16_t *value);
bool app_driver_has_new_current_encoder(void);

// Queue functions
bool app_driver_send_angle_data(uint16_t current, uint16_t desired);
bool app_driver_receive_angle_data(angle_data_t *data, TickType_t timeout);
bool app_driver_try_receive_angle_data(angle_data_t *data);

// Display
esp_err_t app_driver_display_angle(uint16_t current, uint16_t desired);

#endif // APP_DRIVER_H
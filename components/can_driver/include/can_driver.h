#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Chế độ của ESP
typedef enum {
    CAN_MODE_MASTER,  // ESP1: gửi encoder mong muốn, nhận encoder hiện tại
    CAN_MODE_SLAVE    // ESP2: nhận encoder mong muốn, gửi encoder hiện tại
} can_mode_t;

// Khởi tạo CAN (không tạo task)
esp_err_t can_init(can_mode_t mode);

// Hàm xử lý nhận CAN - gọi định kỳ từ task
void can_receive_handler(void);

// ESP1: Gửi encoder mong muốn đến ESP2
esp_err_t can_send_desired_encoder(uint16_t encoder_value);

// ESP2: Gửi encoder hiện tại đến ESP1
esp_err_t can_send_current_encoder(uint16_t encoder_value);

// ESP2: Lấy encoder mong muốn đã nhận
bool can_get_desired_encoder(uint16_t *encoder_value);

// ESP1: Lấy encoder hiện tại đã nhận
bool can_get_current_encoder(uint16_t *encoder_value);

// Kiểm tra có dữ liệu mới không
bool can_has_new_desired_data(void);
bool can_has_new_current_data(void);

#endif
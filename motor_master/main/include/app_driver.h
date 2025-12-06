#ifndef APP_DRIVER_H
#define APP_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ================== BOARD CONFIG (MASTER) ==================

// Encoder 1 
#define ENC1_CLK_GPIO        7
#define ENC1_DT_GPIO         4
#define ENC1_SW_GPIO        -1     
#define ENC1_REVERSE_DIR     0

// Encoder 2 
#define ENC2_CLK_GPIO        0      
#define ENC2_DT_GPIO         10
#define ENC2_SW_GPIO        -1
#define ENC2_REVERSE_DIR     0

#define ENCODER_DEBOUNCE_US  2000
#define ANGLE_MIN            0
#define ANGLE_MAX            180

// OLED SSD1306 (I2C)
#define I2C_MASTER_SDA_IO    2
#define I2C_MASTER_SCL_IO    3
#define I2C_MASTER_FREQ_HZ   400000

// CAN TX/RX MASTER 
#define MASTER_CAN_TX_PIN    GPIO_NUM_5
#define MASTER_CAN_RX_PIN    GPIO_NUM_6

// Cấu trúc dữ liệu góc cho hiển thị
typedef struct {
    uint16_t current;   // angle_actual
    uint16_t desired;   // angle_setpoint
} angle_data_t;

// Khởi tạo toàn bộ driver trên MASTER (2 encoder + OLED + CAN)
esp_err_t app_driver_init(void);

// Lấy giá trị encoder mong muốn (đọc từ núm xoay trên MASTER)
uint16_t app_driver_encoder_get_desired(void);

// Lấy giá trị encoder hiện tại (encoder 2 gắn trên trục gương)
uint16_t app_driver_encoder_get_current(void);

// Queue cho task display
bool app_driver_send_angle_data(uint16_t current, uint16_t desired);
bool app_driver_receive_angle_data(angle_data_t *data, TickType_t timeout);
bool app_driver_try_receive_angle_data(angle_data_t *data);

// Hiển thị số lên OLED
esp_err_t app_driver_display_angle(uint16_t current, uint16_t desired);

#endif 

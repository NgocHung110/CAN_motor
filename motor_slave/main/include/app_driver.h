#ifndef __APP_DRIVER_H__
#define __APP_DRIVER_H__

#include "esp_err.h"
#include <stdint.h>

// ================== BOARD PIN CONFIG ==================
// -------- Motor L298N pins --------
#define MOTOR_PWM_PIN       1
#define MOTOR_FWD_PIN       5
#define MOTOR_BWD_PIN       6

// -------- Encoder pins --------
#define ENC_CLK_PIN         7
#define ENC_DT_PIN          4
#define ENC_SW_PIN         -1
#define ENC_REVERSE_DIR     0
#define ENC_ANGLE_MIN       0
#define ENC_ANGLE_MAX       180

// -------- CAN (ESP32C3 -> MCP2551) --------
#define CAN_TX_PIN          GPIO_NUM_2
#define CAN_RX_PIN          GPIO_NUM_3

// ================== DRIVER CONFIG STRUCT ==================
typedef struct {
    int motor_pwm_pin;
    int motor_forward_pin;
    int motor_backward_pin;

    int enc_clk_pin;
    int enc_dt_pin;
    int enc_sw_pin;
    int enc_reverse_dir;
    int enc_angle_min;
    int enc_angle_max;

    int can_tx_pin;
    int can_rx_pin;
} app_driver_config_t;

// ================== PUBLIC API ==================

/**
 * @brief Khởi tạo toàn bộ driver (CAN + motor + encoder)
 */
esp_err_t app_driver_init(const app_driver_config_t *cfg);

/**
 * @brief Đọc góc hiện tại từ encoder (đơn vị độ)
 */
int16_t app_driver_get_encoder_angle(void);

#endif // __APP_DRIVER_H__

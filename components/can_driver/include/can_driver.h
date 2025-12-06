#ifndef __CAN_DRIVER_H__
#define __CAN_DRIVER_H__

#include "esp_err.h"
#include "driver/twai.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== Protocol ID dùng chung cho Master/Slave =====
#define CAN_ID_SETPOINT    0x101   // (KHÔNG dùng nữa, để đó nếu cần)
#define CAN_ID_FEEDBACK    0x102   // (KHÔNG dùng nữa, để đó nếu cần)
#define CAN_ID_MOTOR_CMD   0x103   // Master -> Slave: lệnh motor (dir + duty)

/**
 * @brief Khởi tạo TWAI (CAN) cho ESP32-C3
 * @param tx_pin GPIO TX nối với CTX của MCP2551
 * @param rx_pin GPIO RX nối với CRX của MCP2551
 */
esp_err_t can_driver_init(gpio_num_t tx_pin, gpio_num_t rx_pin);

/**
 * @brief Gửi frame CAN raw
 */
esp_err_t can_driver_transmit(const twai_message_t *msg);

/**
 * @brief Nhận frame CAN (block với timeout)
 */
esp_err_t can_driver_receive(twai_message_t *msg, TickType_t timeout);

/* ========== Cũ (góc setpoint/feedback) – có thể bỏ nếu không dùng ========== */
esp_err_t can_driver_send_setpoint(int16_t angle);
esp_err_t can_driver_send_feedback(int16_t angle);

/* ========== MỚI: Lệnh motor (Master -> Slave) ========== */
/**
 * Byte 0: dir  (0 = backward, 1 = forward)
 * Byte 1: duty LSB (0–1023)
 * Byte 2: duty MSB
 */
esp_err_t can_driver_send_motor_cmd(bool dir, uint16_t duty);

/**
 * Parse frame lệnh motor
 */
esp_err_t can_driver_parse_motor_cmd(const twai_message_t *msg,
                                     bool *dir,
                                     uint16_t *duty);

#ifdef __cplusplus
}
#endif

#endif // __CAN_DRIVER_H__

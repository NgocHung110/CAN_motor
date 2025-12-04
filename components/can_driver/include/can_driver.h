#ifndef __CAN_DRIVER_H__
#define __CAN_DRIVER_H__

#include "esp_err.h"
#include "driver/twai.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== Protocol ID dùng chung cho Master/Slave =====
#define CAN_ID_SETPOINT   0x101   // Master -> Slave: góc mong muốn
#define CAN_ID_FEEDBACK   0x102   // Slave  -> Master: góc hiện tại

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

/**
 * @brief Gửi góc setpoint (Master -> Slave)
 */
esp_err_t can_driver_send_setpoint(int16_t angle);

/**
 * @brief Gửi góc feedback (Slave -> Master)
 */
esp_err_t can_driver_send_feedback(int16_t angle);

#ifdef __cplusplus
}
#endif

#endif // __CAN_DRIVER_H__

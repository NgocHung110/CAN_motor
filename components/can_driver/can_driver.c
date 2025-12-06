#include "can_driver.h"
#include "esp_log.h"

static const char *TAG = "CAN_DRIVER";

esp_err_t can_driver_init(gpio_num_t tx_pin, gpio_num_t rx_pin)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "CAN initialized (TX=%d, RX=%d)", tx_pin, rx_pin);
    return ESP_OK;
}

esp_err_t can_driver_transmit(const twai_message_t *msg)
{
    esp_err_t ret = twai_transmit(msg, pdMS_TO_TICKS(20));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN transmit failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t can_driver_receive(twai_message_t *msg, TickType_t timeout)
{
    return twai_receive(msg, timeout);
}

/* ========= Legacy helpers (có thể bỏ nếu không dùng) ========= */

esp_err_t can_driver_send_setpoint(int16_t angle)
{
    twai_message_t msg = {0};
    msg.identifier       = CAN_ID_SETPOINT;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = 2;
    msg.data[0] = (uint8_t)(angle & 0xFF);
    msg.data[1] = (uint8_t)((angle >> 8) & 0xFF);

    return can_driver_transmit(&msg);
}

esp_err_t can_driver_send_feedback(int16_t angle)
{
    twai_message_t msg = {0};
    msg.identifier       = CAN_ID_FEEDBACK;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = 2;
    msg.data[0] = (uint8_t)(angle & 0xFF);
    msg.data[1] = (uint8_t)((angle >> 8) & 0xFF);

    return can_driver_transmit(&msg);
}

/* ================= NEW API: MOTOR COMMAND ================= */
// Byte 0: dir (0 = backward, 1 = forward)
// Byte 1: duty LSB
// Byte 2: duty MSB
esp_err_t can_driver_send_motor_cmd(bool dir, uint16_t duty)
{
    if (duty > 1023) {
        duty = 1023;
    }

    twai_message_t msg = {0};
    msg.identifier       = CAN_ID_MOTOR_CMD;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = 3;

    msg.data[0] = dir ? 1 : 0;
    msg.data[1] = (uint8_t)(duty & 0xFF);        // LSB
    msg.data[2] = (uint8_t)((duty >> 8) & 0xFF); // MSB

    return can_driver_transmit(&msg);
}

esp_err_t can_driver_parse_motor_cmd(const twai_message_t *msg,
                                     bool *dir,
                                     uint16_t *duty)
{
    if (!msg || !dir || !duty) {
        return ESP_ERR_INVALID_ARG;
    }

    if (msg->identifier != CAN_ID_MOTOR_CMD ||
        msg->extd != 0 ||
        msg->rtr  != 0 ||
        msg->data_length_code < 3)
    {
        return ESP_FAIL;
    }

    *dir  = (msg->data[0] != 0);
    *duty = (uint16_t)msg->data[1] | ((uint16_t)msg->data[2] << 8);
    if (*duty > 1023) {
        *duty = 1023;
    }
    return ESP_OK;
}

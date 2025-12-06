#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "app_driver.h"
#include "motor_driver.h"
#include "can_driver.h"

static const char *TAG = "SLAVE_APP";

void task_can_rx(void *arg);

// ================== app_main ==================

void app_main(void)
{
    ESP_LOGI(TAG, "SLAVE node starting (motor driver only)...");

    // ====== GET PIN FROM HEADER ======
    app_driver_config_t cfg = {
        .motor_pwm_pin      = MOTOR_PWM_PIN,
        .motor_forward_pin  = MOTOR_FWD_PIN,
        .motor_backward_pin = MOTOR_BWD_PIN,

        .enc_clk_pin        = ENC_CLK_PIN,      // encoder không dùng nữa,
        .enc_dt_pin         = ENC_DT_PIN,       // nhưng vẫn có thể giữ nguyên
        .enc_sw_pin         = ENC_SW_PIN,
        .enc_reverse_dir    = ENC_REVERSE_DIR,
        .enc_angle_min      = ENC_ANGLE_MIN,
        .enc_angle_max      = ENC_ANGLE_MAX,

        .can_tx_pin         = CAN_TX_PIN,
        .can_rx_pin         = CAN_RX_PIN,
    };

    // ====== INIT HARDWARE ======
    app_driver_init(&cfg);   // Khởi tạo CAN + motor (+ encoder nếu bạn vẫn để)

    // ====== Create CAN RX Task ======
    xTaskCreate(task_can_rx, "CAN_RX_TASK", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "SLAVE app started (CAN RX task running)");
}

// ================== TASK: CAN RX (nhận lệnh motor) ==================

void task_can_rx(void *arg)
{
    (void)arg;
    twai_message_t msg;

    ESP_LOGI(TAG, "CAN RX task started, waiting for motor commands...");

    while (1) {
        if (can_driver_receive(&msg, portMAX_DELAY) == ESP_OK) {
            bool dir;
            uint16_t duty;

            if (can_driver_parse_motor_cmd(&msg, &dir, &duty) == ESP_OK) {
                if (duty == 0) {
                    motor_stop();
                    ESP_LOGI(TAG, "Motor STOP");
                } else {
                    motor_set_direction(dir);
                    motor_set_speed(duty);
                    ESP_LOGI(TAG, "Motor CMD: dir=%d, duty=%u",
                             (int)dir, (unsigned)duty);
                }
            } else {
                // Không phải frame MOTOR_CMD, có thể log debug nếu cần
                ESP_LOGD(TAG, "Received non-motor frame: ID=0x%03X, DLC=%d",
                         msg.identifier, msg.data_length_code);
            }
        }
    }
}

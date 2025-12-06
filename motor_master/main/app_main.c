#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_driver.h"
#include "can_driver.h"

#define TAG "MASTER_MAIN"

#define CONTROL_PERIOD_MS      10      // 100 Hz
#define ANGLE_DEADBAND_DEG      2      // |error| <= 4° -> stop
#define ANGLE_FULL_SCALE_DEG  180      // dải điều khiển (0–180°)

#define DUTY_MAX             1023      // 10-bit PWM
#define DUTY_MIN              250      // duty tối thiểu để motor chạy
#define DUTY_STEP_MAX          20      // giới hạn thay đổi duty mỗi chu kỳ

// Task handles
static TaskHandle_t s_task_control  = NULL;
static TaskHandle_t s_task_display  = NULL;

// Task prototypes
static void task_control(void *pvParameters);
static void task_display(void *pvParameters);

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP1 - MASTER Node (2 encoders, CAN motor cmd)");

    // Khởi tạo driver cho MASTER (2 encoder + CAN + OLED)
    if (app_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init app_driver");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    xTaskCreate(task_control,
                "CTRL",  4096, NULL, 5, &s_task_control);

    xTaskCreate(task_display,
                "DISPLAY", 4096, NULL, 3, &s_task_display);

    ESP_LOGI(TAG, "All tasks created successfully");
}

// Task CONTROL
static void task_control(void *pvParameters)
{
    (void)pvParameters;

    uint16_t last_duty = 0;
    bool     last_dir  = true;

    ESP_LOGI(TAG, "Control Task started");

    while (1) {
        // 1. Đọc 2 encoder
        uint16_t desired = app_driver_encoder_get_desired(); // angle_setpoint
        uint16_t actual  = app_driver_encoder_get_current(); // angle_actual

        // 2. Tính sai số
        int16_t error   = (int16_t)desired - (int16_t)actual;
        int16_t abs_err = (error >= 0) ? error : -error;

        bool dir  = (error > 0);  // 1 = forward, 0 = backward
        uint16_t duty = 0;

        if (abs_err <= ANGLE_DEADBAND_DEG) {
           
            duty = 0;
        } else {
            // P-control đơn giản: duty ~ |error|
            float ratio = (float)abs_err / (float)ANGLE_FULL_SCALE_DEG;
            if (ratio > 1.0f) ratio = 1.0f;

            uint32_t d = (uint32_t)(DUTY_MIN + ratio * (float)(DUTY_MAX - DUTY_MIN));
            if (d > DUTY_MAX) d = DUTY_MAX;
            duty = (uint16_t)d;

            // Slew-rate limit
            if (duty > last_duty) {
                uint16_t diff = duty - last_duty;
                if (diff > DUTY_STEP_MAX) {
                    duty = last_duty + DUTY_STEP_MAX;
                }
            } else {
                uint16_t diff = last_duty - duty;
                if (diff > DUTY_STEP_MAX) {
                    duty = last_duty - DUTY_STEP_MAX;
                }
            }
        }

        // 3. Chỉ gửi CAN khi dir/duty thay đổi để giảm traffic
        if (duty != last_duty || dir != last_dir) {
            esp_err_t ret = can_driver_send_motor_cmd(dir, duty);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send motor cmd (dir=%d, duty=%u)",
                         (int)dir, (unsigned)duty);
            } else {
                ESP_LOGD(TAG, "Send motor cmd: desired=%u, actual=%u, err=%d, dir=%d, duty=%u",
                         desired, actual, (int)error, (int)dir, (unsigned)duty);
            }

            last_duty = duty;
            last_dir  = dir;
        }

        // 4. Gửi dữ liệu cho task display (OLED)
        app_driver_send_angle_data(actual, desired);

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

// Task DISPLAY
static void task_display(void *pvParameters)
{
    (void)pvParameters;

    angle_data_t angle_data;
    uint32_t display_count = 0;

    ESP_LOGI(TAG, "Display Task started");

    while (1) {
        // Thử lấy dữ liệu mới từ queue (non-blocking)
        if (app_driver_try_receive_angle_data(&angle_data)) {
            app_driver_display_angle(angle_data.current, angle_data.desired);
            display_count++;

            if (display_count % 20 == 0) {
                ESP_LOGI(TAG, "Displayed %u frames", display_count);
            }
        } else {
            // Nếu không có dữ liệu mới, vẫn cập nhật với giá trị hiện tại
            uint16_t current = app_driver_encoder_get_current();
            uint16_t desired = app_driver_encoder_get_desired();
            app_driver_display_angle(current, desired);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

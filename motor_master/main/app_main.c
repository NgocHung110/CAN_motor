#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "app_driver.h"

#define TAG "app_main"

// Task handles
static TaskHandle_t s_task_can_rx = NULL;
static TaskHandle_t s_task_encoder = NULL;
static TaskHandle_t s_task_display = NULL;

// Function prototypes
void task_can_receive(void *pvParameters);
void task_read_and_send_encoder(void *pvParameters);
void task_display(void *pvParameters);

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP1 - MASTER Node");
    
    // Khởi tạo driver với chế độ MASTER
    app_driver_init(CAN_MODE_MASTER);
    
    // Tạo các tasks
    xTaskCreate(task_can_receive, "CAN_RX", 2048, NULL, 5, &s_task_can_rx);
    xTaskCreate(task_read_and_send_encoder, "ENCODER", 2048, NULL, 4, &s_task_encoder);
    xTaskCreate(task_display, "DISPLAY", 4096, NULL, 3, &s_task_display);
    
    ESP_LOGI(TAG, "All tasks created successfully");
    
    // Main task - chỉ giám sát
    uint32_t tick_count = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        tick_count++;
        
        // Log trạng thái mỗi 5 giây
        if (tick_count % 5 == 0) {
            uint16_t current = app_driver_encoder_get_current();
            uint16_t desired = app_driver_encoder_get_desired();
            ESP_LOGI(TAG, "Status - Current: %d, Desired: %d", current, desired);
        }
    }
}

// Task 1: Xử lý nhận CAN message
void task_can_receive(void *pvParameters)
{
    ESP_LOGI(TAG, "CAN Receive Task started");
    
    while (1) {
        // Xử lý nhận CAN message (non-blocking)
        app_driver_can_receive_handler();
        
        // Chạy ở 100Hz (10ms)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task 2: Đọc encoder mong muốn và gửi qua CAN
void task_read_and_send_encoder(void *pvParameters)
{
    uint16_t last_desired_value = 0;
    uint16_t last_current_value = 0;
    uint32_t send_count = 0;
    
    ESP_LOGI(TAG, "Encoder Task started");
    
    while (1) {
        // 1. Đọc encoder mong muốn
        uint16_t desired_value = app_driver_encoder_get_desired();
        
        // 2. Kiểm tra nếu giá trị thay đổi thì gửi qua CAN
        if (desired_value != last_desired_value) {
            esp_err_t ret = app_driver_can_send_desired_encoder(desired_value);
            if (ret == ESP_OK) {
                send_count++;
                last_desired_value = desired_value;
                
                // Log mỗi 10 lần gửi
                if (send_count % 10 == 0) {
                    ESP_LOGI(TAG, "Sent %d encoder values", send_count);
                }
            }
        }
        
        // 3. Lấy giá trị encoder hiện tại từ ESP2
        uint16_t current_value = app_driver_encoder_get_current();
        
        // 4. Gửi dữ liệu vào queue nếu có thay đổi
        if (current_value != last_current_value || desired_value != last_desired_value) {
            if (app_driver_send_angle_data(current_value, desired_value)) {
                last_current_value = current_value;
            }
        }
        
        // Chạy ở 20Hz (50ms) - đủ nhanh cho encoder
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task 3: Hiển thị OLED
void task_display(void *pvParameters)
{
    angle_data_t angle_data;
    uint32_t display_count = 0;
    
    ESP_LOGI(TAG, "Display Task started");
    
    while (1) {
        // Thử lấy dữ liệu mới từ queue (non-blocking)
        if (app_driver_try_receive_angle_data(&angle_data)) {
            // Hiển thị dữ liệu mới
            app_driver_display_angle(angle_data.current, angle_data.desired);
            display_count++;
            
            // Log mỗi 20 lần hiển thị
            if (display_count % 20 == 0) {
                ESP_LOGI(TAG, "Displayed %d times", display_count);
            }
        } else {
            // Nếu không có dữ liệu mới, vẫn cập nhật với giá trị hiện tại
            uint16_t current = app_driver_encoder_get_current();
            uint16_t desired = app_driver_encoder_get_desired();
            app_driver_display_angle(current, desired);
        }
        
        // Chạy ở 10Hz (100ms) - đủ mượt cho OLED
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
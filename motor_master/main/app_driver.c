#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "app_driver.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "ssd1306.h"
#include "fonts.h"

#define TAG "app_driver"

static ky040_handle_t s_enc_desired = NULL;
static QueueHandle_t s_angle_queue = NULL;

// Khởi tạo driver cho master
void app_driver_init(can_mode_t mode)
{   
    ESP_LOGI(TAG, "Initializing Master Node...");
    
    // Khởi tạo encoder cho giá trị mong muốn
    ky040_config_t enc_config = {
        .gpio_clk = ENC1_CLK_GPIO,
        .gpio_dt = ENC1_DT_GPIO,
        .gpio_sw = ENC1_SW_GPIO,
        .reverse_dir = ENC1_REVERSE_DIR,
        .debounce_us = ENCODER_DEBOUNCE_US,
        .angle_min = ANGLE_MIN,
        .angle_max = ANGLE_MAX
    };
    
    ESP_ERROR_CHECK(ky040_install_isr_service_once(0));
    ESP_ERROR_CHECK(ky040_create(&enc_config, &s_enc_desired));
    ESP_LOGI(TAG, "Encoder for desired value initialized");

    // Khởi tạo CAN driver
    ESP_ERROR_CHECK(can_init(mode));
    ESP_LOGI(TAG, "CAN driver initialized in MASTER mode");

    // Tạo queue cho dữ liệu góc
    s_angle_queue = xQueueCreate(5, sizeof(angle_data_t));
    if (s_angle_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create angle queue");
    } else {
        ESP_LOGI(TAG, "Angle queue created");
    }

    // Khởi tạo OLED display
    ssd1306_i2c_config_t i2c_cfg = {
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .freq_hz = I2C_MASTER_FREQ_HZ,
    };

    SSD1306_Init(&i2c_cfg);
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Current:", &Font_11x18, 1);
    SSD1306_GotoXY(0, 30);
    SSD1306_Puts("Desired:", &Font_11x18, 1);
    SSD1306_UpdateScreen();
    ESP_LOGI(TAG, "OLED display initialized");
}

// Lấy giá trị encoder mong muốn
uint16_t app_driver_encoder_get_desired(void)
{
    if (s_enc_desired == NULL) {
        ESP_LOGW(TAG, "Encoder not initialized");
        return 0;
    }
    return ky040_get_angle(s_enc_desired);
}

// Lấy giá trị encoder hiện tại từ CAN
uint16_t app_driver_encoder_get_current(void)
{
    static uint16_t last_current = 0;
    uint16_t current = 0;
    
    if (app_driver_can_get_current_encoder(&current)) {
        last_current = current;
        ESP_LOGI(TAG, "Got new current encoder: %d", current);
    }
    
    return last_current;
}

// Xử lý nhận CAN message - gọi định kỳ từ task
void app_driver_can_receive_handler(void)
{
    can_receive_handler();
}

// Gửi encoder mong muốn qua CAN
esp_err_t app_driver_can_send_desired_encoder(uint16_t value)
{
    esp_err_t ret = can_send_desired_encoder(value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Successfully sent desired encoder: %d", value);
    } else {
        ESP_LOGW(TAG, "Failed to send desired encoder: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Lấy encoder hiện tại từ buffer CAN
bool app_driver_can_get_current_encoder(uint16_t *value)
{
    return can_get_current_encoder(value);
}

// Kiểm tra có dữ liệu encoder hiện tại mới không
bool app_driver_has_new_current_encoder(void)
{
    return can_has_new_current_data();
}

// Gửi dữ liệu góc vào queue
bool app_driver_send_angle_data(uint16_t current, uint16_t desired)
{
    if (s_angle_queue == NULL) {
        ESP_LOGW(TAG, "Angle queue not initialized");
        return false;
    }
    
    angle_data_t data = {
        .current = current,
        .desired = desired
    };
    
    bool success = xQueueSend(s_angle_queue, &data, 0) == pdTRUE;
    if (!success) {
        ESP_LOGW(TAG, "Angle queue full, data dropped");
    }
    
    return success;
}

// Lấy dữ liệu góc từ queue (blocking)
bool app_driver_receive_angle_data(angle_data_t *data, TickType_t timeout)
{
    if (s_angle_queue == NULL || data == NULL) {
        return false;
    }
    return xQueueReceive(s_angle_queue, data, timeout) == pdTRUE;
}

// Lấy dữ liệu góc từ queue (non-blocking)
bool app_driver_try_receive_angle_data(angle_data_t *data)
{
    if (s_angle_queue == NULL || data == NULL) {
        return false;
    }
    return xQueueReceive(s_angle_queue, data, 0) == pdTRUE;
}

// Hiển thị góc lên OLED
esp_err_t app_driver_display_angle(uint16_t current, uint16_t desired)
{
    char snum[5];
    char snum2[5];

    sprintf(snum, "%03d", current);
    sprintf(snum2, "%03d", desired);
    
    // Chỉ cập nhật phần số
    SSD1306_GotoXY(90, 0);
    SSD1306_Puts(snum, &Font_11x18, 1);
    SSD1306_GotoXY(90, 30);
    SSD1306_Puts(snum2, &Font_11x18, 1);
    SSD1306_UpdateScreen();

    ESP_LOGI(TAG, "Display - Current: %d, Desired: %d", current, desired);
    return ESP_OK;
}
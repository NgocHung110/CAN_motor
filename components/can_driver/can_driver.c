#include "can_driver.h"
#include "can_config.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "CAN_DRIVER";
static can_mode_t s_mode;
static SemaphoreHandle_t s_mutex = NULL;
static uint16_t s_last_desired = 0;
static uint16_t s_last_current = 0;
static bool s_new_desired = false;
static bool s_new_current = false;

// Hàm xử lý nhận CAN - gọi định kỳ từ task nhận
void can_receive_handler(void)
{
    twai_message_t msg;
    
    // Đọc tất cả message có sẵn (non-blocking)
    while (twai_receive(&msg, 0) == ESP_OK) {
        if (msg.data_length_code == 2) {  // encoder value là 2 bytes
            uint16_t value = (msg.data[0] << 8) | msg.data[1];
            
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            
            if (msg.identifier == CAN_MSG_DESIRED_ENCODER) {
                s_last_desired = value;
                s_new_desired = true;
                ESP_LOGI(TAG, "Received desired encoder: %d", value);
            }
            else if (msg.identifier == CAN_MSG_CURRENT_ENCODER) {
                s_last_current = value;
                s_new_current = true;
                ESP_LOGI(TAG, "Received current encoder: %d", value);
            }
            
            xSemaphoreGive(s_mutex);
        }
    }
}

// Khởi tạo CAN (không tạo task)
esp_err_t can_init(can_mode_t mode)
{
    s_mode = mode;
    
    // Tạo mutex bảo vệ dữ liệu chung
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    // Cấu hình CAN
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN,
        CAN_RX_PIN,
        TWAI_MODE_NORMAL
    );
    
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Cài đặt driver
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install driver");
        vSemaphoreDelete(s_mutex);
        return ret;
    }
    
    // Bắt đầu
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start");
        vSemaphoreDelete(s_mutex);
        return ret;
    }
    
    ESP_LOGI(TAG, "CAN initialized as %s", 
             (mode == CAN_MODE_MASTER) ? "MASTER" : "SLAVE");
    return ESP_OK;
}

// ESP1: Gửi encoder mong muốn
esp_err_t can_send_desired_encoder(uint16_t encoder_value)
{
    if (s_mode != CAN_MODE_MASTER) {
        ESP_LOGE(TAG, "Only master can send desired encoder");
        return ESP_ERR_INVALID_STATE;
    }
    
    twai_message_t msg;
    msg.identifier = CAN_MSG_DESIRED_ENCODER;
    msg.data_length_code = 2;
    msg.data[0] = (encoder_value >> 8) & 0xFF;
    msg.data[1] = encoder_value & 0xFF;
    msg.flags = 0;
    
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sent desired encoder: %d", encoder_value);
    } else {
        ESP_LOGW(TAG, "Failed to send desired encoder: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// ESP2: Gửi encoder hiện tại
esp_err_t can_send_current_encoder(uint16_t encoder_value)
{
    if (s_mode != CAN_MODE_SLAVE) {
        ESP_LOGE(TAG, "Only slave can send current encoder");
        return ESP_ERR_INVALID_STATE;
    }
    
    twai_message_t msg;
    msg.identifier = CAN_MSG_CURRENT_ENCODER;
    msg.data_length_code = 2;
    msg.data[0] = (encoder_value >> 8) & 0xFF;
    msg.data[1] = encoder_value & 0xFF;
    msg.flags = 0;
    
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sent current encoder: %d", encoder_value);
    }
    
    return ret;
}

// ESP2: Nhận encoder mong muốn
bool can_get_desired_encoder(uint16_t *encoder_value)
{
    if (s_mode != CAN_MODE_SLAVE || encoder_value == NULL) {
        return false;
    }
    
    bool result = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    
    if (s_new_desired) {
        *encoder_value = s_last_desired;
        s_new_desired = false;
        result = true;
    } else {
        *encoder_value = s_last_desired; // Vẫn trả về giá trị cũ
    }
    
    xSemaphoreGive(s_mutex);
    return result;
}

// ESP1: Nhận encoder hiện tại
bool can_get_current_encoder(uint16_t *encoder_value)
{
    if (s_mode != CAN_MODE_MASTER || encoder_value == NULL) {
        return false;
    }
    
    bool result = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    
    if (s_new_current) {
        *encoder_value = s_last_current;
        s_new_current = false;
        result = true;
    } else {
        *encoder_value = s_last_current; // Vẫn trả về giá trị cũ
    }
    
    xSemaphoreGive(s_mutex);
    return result;
}

// Kiểm tra có dữ liệu mới không (không xóa flag)
bool can_has_new_desired_data(void)
{
    bool result;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    result = s_new_desired;
    xSemaphoreGive(s_mutex);
    return result;
}

bool can_has_new_current_data(void)
{
    bool result;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    result = s_new_current;
    xSemaphoreGive(s_mutex);
    return result;
}
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "app_driver.h"
#include "encoder_driver.h"
#include "ssd1306.h"
#include "fonts.h"
#include "can_driver.h"   // dùng can_driver_init()

#define TAG "MASTER_APP_DRIVER"

// ================== STATIC VARIABLES ==================
static ky040_handle_t s_enc_desired = NULL;  // encoder 1 – góc mong muốn
static ky040_handle_t s_enc_actual  = NULL;  // encoder 2 – góc thực tế

static QueueHandle_t  s_angle_queue = NULL;  // queue cho task display

// ================== IMPLEMENTATION ==================

esp_err_t app_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing MASTER node (2 encoders).");

    // ===== ENCODER 1: mong muốn =====
    ky040_config_t enc1_config = {
        .gpio_clk    = ENC1_CLK_GPIO,
        .gpio_dt     = ENC1_DT_GPIO,
        .gpio_sw     = ENC1_SW_GPIO,
        .reverse_dir = ENC1_REVERSE_DIR,
        .debounce_us = ENCODER_DEBOUNCE_US,
        .angle_min   = ANGLE_MIN,
        .angle_max   = ANGLE_MAX,
    };

    // ===== ENCODER 2: góc thực tế =====
    ky040_config_t enc2_config = {
        .gpio_clk    = ENC2_CLK_GPIO,
        .gpio_dt     = ENC2_DT_GPIO,
        .gpio_sw     = ENC2_SW_GPIO,
        .reverse_dir = ENC2_REVERSE_DIR,
        .debounce_us = ENCODER_DEBOUNCE_US,
        .angle_min   = ANGLE_MIN,
        .angle_max   = ANGLE_MAX,
    };

    ESP_ERROR_CHECK(ky040_install_isr_service_once(0));
    ESP_ERROR_CHECK(ky040_create(&enc1_config, &s_enc_desired));
    ESP_ERROR_CHECK(ky040_create(&enc2_config, &s_enc_actual));
    ESP_LOGI(TAG, "Both encoders initialized (desired + actual)");

    // ===== CAN (TWAI) =====
    ESP_ERROR_CHECK(can_driver_init(MASTER_CAN_TX_PIN, MASTER_CAN_RX_PIN));
    ESP_LOGI(TAG, "CAN driver initialized on MASTER (TX=%d, RX=%d)",
             MASTER_CAN_TX_PIN, MASTER_CAN_RX_PIN);

    // ===== Queue cho dữ liệu góc =====
    s_angle_queue = xQueueCreate(1, sizeof(angle_data_t));
    if (!s_angle_queue) {
        ESP_LOGE(TAG, "Failed to create angle queue");
        return ESP_FAIL;
    }

    // ===== OLED =====
    ssd1306_i2c_config_t i2c_cfg = {
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .freq_hz    = I2C_MASTER_FREQ_HZ,
    };

    SSD1306_Init(&i2c_cfg);
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Current:", &Font_11x18, 1);
    SSD1306_GotoXY(0, 30);
    SSD1306_Puts("Desired:", &Font_11x18, 1);
    SSD1306_UpdateScreen();

    ESP_LOGI(TAG, "OLED display initialized");

    return ESP_OK;
}

// Lấy giá trị encoder mong muốn (núm xoay trên MASTER)
uint16_t app_driver_encoder_get_desired(void)
{
    if (!s_enc_desired) {
        ESP_LOGW(TAG, "Desired encoder not initialized");
        return 0;
    }
    return (uint16_t)ky040_get_angle(s_enc_desired);
}

// Lấy giá trị encoder thực tế (gắn trục gương/motor)
uint16_t app_driver_encoder_get_current(void)
{
    if (!s_enc_actual) {
        ESP_LOGW(TAG, "Actual encoder not initialized");
        return 0;
    }
    return (uint16_t)ky040_get_angle(s_enc_actual);
}

// Queue: gửi dữ liệu cho task display
bool app_driver_send_angle_data(uint16_t current, uint16_t desired)
{
    if (!s_angle_queue) {
        ESP_LOGW(TAG, "Angle queue not initialized");
        return false;
    }

    angle_data_t data = {
        .current = current,
        .desired = desired,
    };

    // Queue length = 1 → dùng overwrite để luôn giữ giá trị mới nhất
    if (xQueueOverwrite(s_angle_queue, &data) != pdTRUE) {
        // Không còn chuyện "full" nữa
        ESP_LOGW(TAG, "Angle queue overwrite failed");
        return false;
    }
    return true;
}

bool app_driver_receive_angle_data(angle_data_t *data, TickType_t timeout)
{
    if (!s_angle_queue || !data) return false;
    return (xQueueReceive(s_angle_queue, data, timeout) == pdTRUE);
}

bool app_driver_try_receive_angle_data(angle_data_t *data)
{
    if (!s_angle_queue || !data) return false;
    return (xQueueReceive(s_angle_queue, data, 0) == pdTRUE);
}

// Hiển thị góc lên OLED
esp_err_t app_driver_display_angle(uint16_t current, uint16_t desired)
{
    char snum[8];
    char snum2[8];

    snprintf(snum,  sizeof(snum),  "%03u", current);
    snprintf(snum2, sizeof(snum2), "%03u", desired);

    SSD1306_GotoXY(90, 0);
    SSD1306_Puts(snum, &Font_11x18, 1);
    SSD1306_GotoXY(90, 30);
    SSD1306_Puts(snum2, &Font_11x18, 1);
    SSD1306_UpdateScreen();

    ESP_LOGI(TAG, "Display - Current: %u, Desired: %u",
             current, desired);
    return ESP_OK;
}

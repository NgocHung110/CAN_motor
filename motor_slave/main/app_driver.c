#include "app_driver.h"

#include "esp_log.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "can_driver.h"

static const char *TAG = "APP_DRIVER_SLAVE";

// Encoder handle
static ky040_handle_t s_enc = NULL;

esp_err_t app_driver_init(const app_driver_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    // ===== CAN =====
    ESP_ERROR_CHECK(can_driver_init(cfg->can_tx_pin, cfg->can_rx_pin));

    // ===== MOTOR =====
    motor_config_t mcfg = {
        .pwm_pin      = cfg->motor_pwm_pin,
        .forward_pin  = cfg->motor_forward_pin,
        .backward_pin = cfg->motor_backward_pin,
    };
    ESP_ERROR_CHECK(motor_driver_init(&mcfg));

    // ===== ENCODER =====
    ky040_config_t ecfg = {
        .gpio_clk    = cfg->enc_clk_pin,
        .gpio_dt     = cfg->enc_dt_pin,
        .gpio_sw     = cfg->enc_sw_pin,
        .reverse_dir = cfg->enc_reverse_dir,
        .debounce_us = 2000,
        .angle_min   = cfg->enc_angle_min,
        .angle_max   = cfg->enc_angle_max,
    };
    ESP_ERROR_CHECK(ky040_install_isr_service_once(0));
    ESP_ERROR_CHECK(ky040_create(&ecfg, &s_enc));

    ESP_LOGI(TAG, "app_driver_init done (SLAVE)");
    return ESP_OK;
}

int16_t app_driver_get_encoder_angle(void)
{
    if (!s_enc) return 0;
    return (int16_t)ky040_get_angle(s_enc);
}

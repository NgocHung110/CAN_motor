#include <stdio.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#include "motor_driver.h"

#define TAG "motor_driver"

static motor_config_t motor = {0};

esp_err_t motor_driver_init(motor_config_t *config)
{
    esp_err_t ret = ESP_OK;

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed");
        return ret;
    }

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = config->pwm_pin,
        .duty           = 0,  // Set duty to 0%
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed");
        return ret;
    }
    motor.pwm_pin = config->pwm_pin;
    motor.forward_pin = config->forward_pin;
    motor.backward_pin = config->backward_pin;

    // Configure GPIO pins for direction control
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << motor.forward_pin) | (1ULL << motor.backward_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed");
        return ret;
    }

    return ESP_OK;
}

esp_err_t motor_set_direction(bool forward) {
    gpio_set_level(motor.forward_pin, forward ? 1 : 0);
    gpio_set_level(motor.backward_pin, forward ? 0 : 1);
    return ESP_OK;
}

// Set motor speed (0-1023)
esp_err_t motor_set_speed(uint32_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    return ESP_OK;
}

esp_err_t motor_stop(void)
{
    // Set duty cycle to 0 to stop the motor
    gpio_set_level(motor.forward_pin, 0);
    gpio_set_level(motor.backward_pin, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    return ESP_OK;
}
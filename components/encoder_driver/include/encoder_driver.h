#pragma once
#include "esp_err.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ky040_encoder* ky040_handle_t;

typedef struct {
    gpio_num_t gpio_clk;
    gpio_num_t gpio_dt;
    gpio_num_t gpio_sw;           // set to -1 if unused
    bool       reverse_dir;
    uint32_t   debounce_us;       // 0 = off
    uint16_t   angle_min;         // e.g., 0
    uint16_t   angle_max;         // e.g., 90
} ky040_config_t;

esp_err_t ky040_install_isr_service_once(int intr_flags);
esp_err_t ky040_create(const ky040_config_t* cfg, ky040_handle_t* out);
void      ky040_delete(ky040_handle_t h);
void      ky040_set_reverse(ky040_handle_t h, bool reverse);
void      ky040_reset_zero(ky040_handle_t h);
esp_err_t ky040_set_range(ky040_handle_t h, uint16_t angle_min, uint16_t angle_max);
int32_t   ky040_get_ticks(ky040_handle_t h);
uint16_t  ky040_get_angle(ky040_handle_t h);

#ifdef __cplusplus
}
#endif
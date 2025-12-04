#include "encoder_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"
#include <stdlib.h>

#define KY040_TAG "KY040DRV"

struct ky040_encoder {
    gpio_num_t clk, dt, sw;
    volatile int32_t ticks;
    volatile int64_t last_edge_us;
    uint32_t debounce_us;
    bool reverse;
    uint16_t ang_min, ang_max;   // inclusive
    uint16_t span;               // (ang_max - ang_min + 1)
    portMUX_TYPE mux;
};

static bool s_isr_service_installed = false;

static inline bool _debounce_ok(volatile int64_t* last_us, uint32_t min_us) {
    if (min_us == 0) return true;
    int64_t now = esp_timer_get_time();
    if (now - *last_us < (int64_t)min_us) return false;
    *last_us = now;
    return true;
}

static inline uint16_t _angle_mod(struct ky040_encoder* e, int32_t ticks) {
    int32_t t = ticks % e->span;
    if (t < 0) t += e->span;
    return (uint16_t)(e->ang_min + t);
}

static void IRAM_ATTR ky040_isr_clk(void* arg) {
    struct ky040_encoder* e = (struct ky040_encoder*)arg;
    if (!_debounce_ok(&e->last_edge_us, e->debounce_us)) return;

    int dt = gpio_get_level(e->dt);
    int delta = (dt == 0) ? +1 : -1;
    if (e->reverse) delta = -delta;

    portENTER_CRITICAL_ISR(&e->mux);
    e->ticks += delta;
    if (e->ticks >= (int32_t)e->span) e->ticks -= e->span;
    if (e->ticks < 0)                 e->ticks += e->span;
    portEXIT_CRITICAL_ISR(&e->mux);
}

static void IRAM_ATTR ky040_isr_sw(void* arg) {
    struct ky040_encoder* e = (struct ky040_encoder*)arg;
    portENTER_CRITICAL_ISR(&e->mux);
    e->ticks = 0;
    portEXIT_CRITICAL_ISR(&e->mux);
}

esp_err_t ky040_install_isr_service_once(int intr_flags) {
    if (s_isr_service_installed) return ESP_OK;
    esp_err_t err = gpio_install_isr_service(intr_flags);
    if (err == ESP_ERR_INVALID_STATE) {
        s_isr_service_installed = true;
        return ESP_OK;
    }
    if (err == ESP_OK) s_isr_service_installed = true;
    return err;
}

esp_err_t ky040_create(const ky040_config_t* cfg, ky040_handle_t* out) {
    if (!cfg || !out) return ESP_ERR_INVALID_ARG;
    if (cfg->angle_max < cfg->angle_min) return ESP_ERR_INVALID_ARG;
    uint32_t span = (uint32_t)cfg->angle_max - (uint32_t)cfg->angle_min + 1;
    if (span == 0 || span > 65535) return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(ky040_install_isr_service_once(0), KY040_TAG, "ISR service");

    struct ky040_encoder* e = (struct ky040_encoder*)calloc(1, sizeof(*e));
    if (!e) return ESP_ERR_NO_MEM;

    e->clk = cfg->gpio_clk;
    e->dt  = cfg->gpio_dt;
    e->sw  = cfg->gpio_sw;
    e->ticks = 0;
    e->last_edge_us = esp_timer_get_time();
    e->debounce_us = cfg->debounce_us;
    e->reverse = cfg->reverse_dir;
    e->ang_min = cfg->angle_min;
    e->ang_max = cfg->angle_max;
    e->span    = (uint16_t)span;
    e->mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << e->clk) | (1ULL << e->dt),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_set_intr_type(e->dt, GPIO_INTR_DISABLE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(e->clk, ky040_isr_clk, (void*)e));

    if (e->sw >= 0) {
        gpio_config_t io_sw = {
            .pin_bit_mask = (1ULL << e->sw),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE
        };
        ESP_ERROR_CHECK(gpio_config(&io_sw));
        ESP_ERROR_CHECK(gpio_isr_handler_add(e->sw, ky040_isr_sw, (void*)e));
    }

    *out = e;
    return ESP_OK;
}

void ky040_delete(ky040_handle_t h) {
    if (!h) return;
    gpio_isr_handler_remove(h->clk);
    if (h->sw >= 0) gpio_isr_handler_remove(h->sw);
    free(h);
}

void ky040_set_reverse(ky040_handle_t h, bool reverse) {
    if (!h) return;
    h->reverse = reverse;
}

void ky040_reset_zero(ky040_handle_t h) {
    if (!h) return;
    portENTER_CRITICAL(&h->mux);
    h->ticks = 0;
    portEXIT_CRITICAL(&h->mux);
}

esp_err_t ky040_set_range(ky040_handle_t h, uint16_t angle_min, uint16_t angle_max) {
    if (!h) return ESP_ERR_INVALID_ARG;
    if (angle_max < angle_min) return ESP_ERR_INVALID_ARG;
    uint32_t span = (uint32_t)angle_max - (uint32_t)angle_min + 1;
    if (span == 0 || span > 65535) return ESP_ERR_INVALID_ARG;

    portENTER_CRITICAL(&h->mux);
    h->ang_min = angle_min;
    h->ang_max = angle_max;
    h->span    = (uint16_t)span;
    if (h->ticks >= (int32_t)h->span) h->ticks %= h->span;
    if (h->ticks < 0)                 h->ticks = (h->ticks % h->span + h->span) % h->span;
    portEXIT_CRITICAL(&h->mux);
    return ESP_OK;
}

int32_t ky040_get_ticks(ky040_handle_t h) {
    if (!h) return 0;
    int32_t t;
    portENTER_CRITICAL(&h->mux);
    t = h->ticks;
    portEXIT_CRITICAL(&h->mux);
    return t;
}

uint16_t ky040_get_angle(ky040_handle_t h) {
    if (!h) return 0;
    int32_t t = ky040_get_ticks(h);
    return _angle_mod(h, t);
}
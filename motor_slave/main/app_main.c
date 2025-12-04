#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "esp_log.h"

#include "app_driver.h"
#include "motor_driver.h"
#include "can_driver.h"

static const char *TAG = "SLAVE_APP";

// ================== Control parameters ==================
#define CONTROL_PERIOD_MS   10
#define FEEDBACK_PERIOD_MS  50
#define ANGLE_DEADBAND_DEG  2
#define KP_SPEED            10.0f

// ================== RTOS Objects ==================
static QueueHandle_t     s_setpoint_queue = NULL;
static SemaphoreHandle_t s_angle_mutex    = NULL;
static TimerHandle_t     s_feedback_timer = NULL;

// ================== Shared state ==================
static int16_t g_setpoint_angle = 0;
static int16_t g_current_angle  = 0;

// ================== Forward Declaration ==================
static void task_control(void *arg);
static void task_can_rx(void *arg);
static void feedback_timer_cb(TimerHandle_t xTimer);

// ================== app_main ==================

void app_main(void)
{
    ESP_LOGI(TAG, "SLAVE node starting...");

    // ====== GET PIN FROM HEADER ======
    app_driver_config_t cfg = {
        .motor_pwm_pin      = MOTOR_PWM_PIN,
        .motor_forward_pin  = MOTOR_FWD_PIN,
        .motor_backward_pin = MOTOR_BWD_PIN,

        .enc_clk_pin        = ENC_CLK_PIN,
        .enc_dt_pin         = ENC_DT_PIN,
        .enc_sw_pin         = ENC_SW_PIN,
        .enc_reverse_dir    = ENC_REVERSE_DIR,
        .enc_angle_min      = ENC_ANGLE_MIN,
        .enc_angle_max      = ENC_ANGLE_MAX,

        .can_tx_pin         = CAN_TX_PIN,
        .can_rx_pin         = CAN_RX_PIN,
    };

    // ====== INIT HARDWARE ======
    app_driver_init(&cfg);
 // ====== CREATE RTOS OBJECTS ======
    s_setpoint_queue = xQueueCreate(1, sizeof(int16_t));
    s_angle_mutex    = xSemaphoreCreateMutex();

    s_feedback_timer = xTimerCreate(
        "fb_timer",
        pdMS_TO_TICKS(FEEDBACK_PERIOD_MS),
        pdTRUE,
        NULL,
        feedback_timer_cb
    );
    xTimerStart(s_feedback_timer, 0);

    // ====== Create tasks ======
    xTaskCreate(task_control, "CTRL_TASK",   4096, NULL, 5, NULL);
    xTaskCreate(task_can_rx,  "CAN_RX_TASK", 4096, NULL, 6, NULL);

    ESP_LOGI(TAG, "SLAVE app started (tasks + timer running)");
}

// ================== FEEDBACK TIMER CALLBACK ==================

static void feedback_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;

    int16_t angle;
    if (xSemaphoreTake(s_angle_mutex, 0) == pdTRUE) {
        angle = g_current_angle;
        xSemaphoreGive(s_angle_mutex);
    } else {
        return;
    }

    can_driver_send_feedback(angle);
}

// ================== TASK: CAN RX (nhận setpoint) ==================

static void task_can_rx(void *arg)
{
    (void)arg;
    twai_message_t msg;

    while (1) {
        if (can_driver_receive(&msg, portMAX_DELAY) == ESP_OK) {
            if (!msg.rtr && !msg.extd &&
                msg.identifier == CAN_ID_SETPOINT &&
                msg.data_length_code >= 2)
            {
                int16_t sp = (int16_t)((((int16_t)msg.data[1]) << 8) | msg.data[0]);
                xQueueOverwrite(s_setpoint_queue, &sp);
                ESP_LOGI(TAG, "Received setpoint = %d deg", sp);
            }
        }
    }
}

// ================== TASK: CONTROL ==================

static void task_control(void *arg)
{
    (void)arg;
    int16_t local_sp = 0;

    while (1) {
        // Lấy setpoint mới nhất
        int16_t sp;
        while (xQueueReceive(s_setpoint_queue, &sp, 0) == pdPASS) {
            local_sp = sp;
        }

        // Đọc góc hiện tại
        int16_t angle = app_driver_get_encoder_angle();

        // Cập nhật biến global
        if (xSemaphoreTake(s_angle_mutex, portMAX_DELAY) == pdTRUE) {
            g_current_angle  = angle;
            g_setpoint_angle = local_sp;
            xSemaphoreGive(s_angle_mutex);
        }

        // Điều khiển motor bám góc
        int16_t error   = local_sp - angle;
        int16_t abs_err = abs(error);

        if (abs_err <= ANGLE_DEADBAND_DEG) {
            motor_stop();
        } else {
            motor_set_direction(error > 0);
            uint32_t duty = (uint32_t)(KP_SPEED * abs_err);
            if (duty > 8191) duty = 8191;
            motor_set_speed(duty);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
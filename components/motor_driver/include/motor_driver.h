#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#include "esp_err.h"
#include "driver/gpio.h"

typedef struct
{
    gpio_num_t pwm_pin;
    gpio_num_t forward_pin;
    gpio_num_t backward_pin;

} motor_config_t;

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t motor_driver_init(motor_config_t *config);
    esp_err_t motor_set_direction(bool forward);
    esp_err_t motor_set_speed(uint32_t speed);
    esp_err_t motor_stop(void);

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_DRIVER_H__
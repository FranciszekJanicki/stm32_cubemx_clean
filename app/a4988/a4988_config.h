#ifndef A4988_A4988_CONFIG_H
#define A4988_A4988_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    A4988_ERR_OK = 0,
    A4988_ERR_FAIL = 1 << 0,
    A4988_ERR_NULL = 1 << 1,
} a4988_err_t;

typedef enum {
    A4988_MICROSTEP_FULL,
    A4988_MICROSTEP_HALF,
    A4988_MICROSTEP_QUARTER,
    A4988_MICROSTEP_EIGHTH,
    A4988_MICROSTEP_SIXTEENTH,
} a4988_microstep_t;

typedef enum {
    A4988_DIRECTION_FORWARD,
    A4988_DIRECTION_BACKWARD,
    A4988_DIRECTION_STOP,
} a4988_direction_t;

inline float a4988_microstep_to_fraction(a4988_microstep_t microstep)
{
    switch (microstep) {
        case A4988_MICROSTEP_FULL:
            return 1.0F;
        case A4988_MICROSTEP_HALF:
            return 0.5F;
        case A4988_MICROSTEP_QUARTER:
            return 0.25F;
        case A4988_MICROSTEP_EIGHTH:
            return 0.125F;
        case A4988_MICROSTEP_SIXTEENTH:
            return 0.0625F;
        default:
            return 0.0F;
    }
}

typedef struct {
    uint32_t pin_ms1;
    uint32_t pin_ms2;
    uint32_t pin_ms3;
    uint32_t pin_reset;
    uint32_t pin_sleep;
    uint32_t pin_dir;
    uint32_t pin_enable;
} a4988_config_t;

typedef struct {
    void* gpio_user;
    a4988_err_t (*gpio_init)(void*);
    a4988_err_t (*gpio_deinit)(void*);
    a4988_err_t (*gpio_write_pin)(void*, uint32_t, bool);

    void* pulse_user;
    a4988_err_t (*pulse_init)(void*);
    a4988_err_t (*pulse_deinit)(void*);
    a4988_err_t (*pulse_start)(void*);
    a4988_err_t (*pulse_stop)(void*);
    a4988_err_t (*pulse_set_freq)(void*, uint32_t);
} a4988_interface_t;

#endif // A4988_A4988_CONFIG_H
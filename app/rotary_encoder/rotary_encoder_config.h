#ifndef ROTARY_ENCODER_ROTARY_ENCODER_CONFIG_H
#define ROTARY_ENCODER_ROTARY_ENCODER_CONFIG_H

#include <stdint.h>

typedef float float32_t;

typedef enum {
    ROTARY_ENCODER_ERR_OK = 0,
    ROTARY_ENCODER_ERR_FAIL = 1 << 0,
} rotary_encoder_err_t;

typedef struct {
    int32_t step_count;
    float32_t prev_speed;
    float32_t prev_acceleration;
} rotary_encoder_state_t;

typedef struct {
    float32_t step_change;
    float32_t min_position;
    float32_t max_position;
    float32_t min_speed;
    float32_t max_speed;
    float32_t min_acceleration;
    float32_t max_acceleration;
} rotary_encoder_config_t;

typedef struct {
    void* device_user;
    step_motor_err_t (*device_init)(void*);
    step_motor_err_t (*device_deinit)(void*);
    step_motor_err_t (*device_get_step_count)(void*, int32_t*);
} rotary_encoder_interface_t;

#endif // ROTARY_ENCODER_ROTARY_ENCODER_CONFIG_H
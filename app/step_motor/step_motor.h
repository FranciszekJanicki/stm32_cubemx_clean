#ifndef STEP_MOTOR_STEP_MOTOR_H
#define STEP_MOTOR_STEP_MOTOR_H

#include "step_motor_config.h"
#include <stdbool.h>

typedef struct {
    step_motor_config_t config;
    step_motor_interface_t interface;

    bool is_running;
    uint32_t frequency;
    int64_t step_count;
    float32_t prev_position;
    float32_t prev_speed;
    float32_t prev_acceleration;
    step_motor_direction_t direction;
} step_motor_t;

step_motor_err_t step_motor_initialize(step_motor_t* step_motor,
                                       step_motor_config_t const* config,
                                       step_motor_interface_t const* interface);
step_motor_err_t step_motor_deinitialize(step_motor_t* step_motor);

step_motor_err_t step_motor_reset(step_motor_t* step_motor);
step_motor_err_t step_motor_start(step_motor_t* step_motor);
step_motor_err_t step_motor_stop(step_motor_t* step_motor);

void step_motor_update_step_count(step_motor_t* step_motor);

step_motor_err_t step_motor_set_position(step_motor_t* step_motor,
                                         float32_t position,
                                         float32_t sampling_time);
step_motor_err_t step_motor_set_speed(step_motor_t* step_motor, float32_t speed);
step_motor_err_t step_motor_set_acceleration(step_motor_t* step_motor,
                                             float32_t acceleration,
                                             float32_t sampling_time);

float32_t step_motor_get_position(step_motor_t* step_motor);
float32_t step_motor_get_speed(step_motor_t* step_motor, float32_t sampling_time);
float32_t step_motor_get_acceleration(step_motor_t* step_motor, float32_t sampling_time);

#endif // STEP_MOTOR_STEP_MOTOR_H
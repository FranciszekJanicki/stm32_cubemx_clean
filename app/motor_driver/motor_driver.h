#ifndef MOTOR_DRIVER_MOTOR_DRIVER_H
#define MOTOR_DRIVER_MOTOR_DRIVER_H

#include "pid_regulator.h"
#include "rotary_encoder.h"
#include "step_motor.h"

typedef struct {
    rotary_encoder_t encoder;
    pid_regulator_t regulator;
    step_motor_t motor;
} motor_driver_t;

void motor_driver_initialize(motor_driver_t* driver,
                             rotary_encoder_t const* encoder,
                             pid_regulator_t const* regulator,
                             step_motor_t const* motor);
void motor_driver_deinitialize(motor_driver_t* driver);

void motor_driver_set_position(motor_driver_t* driver, float32_t position, float32_t delta_time);

#endif // MOTOR_DRIVER_MOTOR_DRIVER_H

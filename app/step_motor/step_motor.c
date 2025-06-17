#include "step_motor.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <string.h>

static inline step_motor_direction_t step_motor_speed_to_direction(float32_t speed,
                                                                   float32_t stall_speed)
{
    return fabsf(speed) < fabsf(stall_speed)
               ? STEP_MOTOR_DIRECTION_STOP
               : (speed > 0.0F ? STEP_MOTOR_DIRECTION_FORWARD : STEP_MOTOR_DIRECTION_BACKWARD);
}

static inline uint32_t step_motor_speed_to_frequency(float32_t speed, float32_t step_change)
{
    assert(step_change > 0.0F);

    return (uint32_t)fabsf(speed / step_change);
}

static step_motor_err_t step_motor_driver_init(step_motor_t const* step_motor)
{
    return step_motor->interface.driver_init
               ? step_motor->interface.driver_init(step_motor->interface.driver_user)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_driver_deinit(step_motor_t const* step_motor)
{
    return step_motor->interface.driver_deinit
               ? step_motor->interface.driver_deinit(step_motor->interface.driver_user)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_driver_start(step_motor_t const* step_motor)
{
    return step_motor->interface.driver_start
               ? step_motor->interface.driver_start(step_motor->interface.driver_user)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_driver_stop(step_motor_t const* step_motor)
{
    return step_motor->interface.driver_stop
               ? step_motor->interface.driver_stop(step_motor->interface.driver_user)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_driver_set_frequency(step_motor_t const* step_motor,
                                                        uint32_t frequency)
{
    return step_motor->interface.driver_set_frequency
               ? step_motor->interface.driver_set_frequency(step_motor->interface.driver_user,
                                                            frequency)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_driver_set_direction(step_motor_t const* step_motor,
                                                        step_motor_direction_t direction)
{
    return step_motor->interface.driver_set_direction
               ? step_motor->interface.driver_set_direction(step_motor->interface.driver_user,
                                                            direction)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_set_direction(step_motor_t* step_motor,
                                                 step_motor_direction_t direction)
{
    if (direction == step_motor->direction) {
        return STEP_MOTOR_ERR_OK;
    }

    step_motor->direction = direction;

    return step_motor_driver_set_direction(step_motor, direction);
}

static step_motor_err_t step_motor_set_frequency(step_motor_t* step_motor, uint32_t frequency)
{
    if (frequency == step_motor->frequency) {
        return STEP_MOTOR_ERR_OK;
    }

    step_motor->frequency = frequency;

    return step_motor_driver_set_frequency(step_motor, frequency);
}

step_motor_err_t step_motor_initialize(step_motor_t* step_motor,
                                       step_motor_config_t const* config,
                                       step_motor_interface_t const* interface)
{
    assert(step_motor && config && interface);

    memset(step_motor, 0, sizeof(*step_motor));
    memcpy(&step_motor->config, config, sizeof(*config));
    memcpy(&step_motor->interface, interface, sizeof(*interface));

    return step_motor_driver_init(step_motor);
}

step_motor_err_t step_motor_deinitialize(step_motor_t* step_motor)
{
    assert(step_motor);

    step_motor_err_t err = step_motor_driver_deinit(step_motor);

    memset(step_motor, 0, sizeof(*step_motor));

    return err;
}

step_motor_err_t step_motor_reset(step_motor_t* step_motor)
{
    assert(step_motor);

    step_motor->direction = STEP_MOTOR_DIRECTION_STOP;
    step_motor->frequency = 0UL;
    step_motor->step_count = 0LL;
    step_motor->prev_position = 0.0F;
    step_motor->prev_acceleration = 0.0F;
    step_motor->is_running = false;

    return step_motor_driver_stop(step_motor);
}

step_motor_err_t step_motor_start(step_motor_t* step_motor)
{
    assert(step_motor);

    step_motor_err_t err = STEP_MOTOR_ERR_OK;

    if (!step_motor->is_running) {
        err |= step_motor_driver_start(step_motor);
        step_motor->is_running = true;
    }

    return err;
}

step_motor_err_t step_motor_stop(step_motor_t* step_motor)
{
    assert(step_motor);

    step_motor_err_t err = STEP_MOTOR_ERR_OK;

    if (step_motor->is_running) {
        err |= step_motor_driver_stop(step_motor);
        step_motor->is_running = false;
    }

    return err;
}

step_motor_err_t step_motor_update_step_count(step_motor_t* step_motor)
{
    assert(step_motor);

    if (step_motor->direction == STEP_MOTOR_DIRECTION_BACKWARD &&
        step_motor->step_count != LLONG_MIN) {
        step_motor->step_count--;
    } else if (step_motor->direction == STEP_MOTOR_DIRECTION_FORWARD &&
               step_motor->step_count != LLONG_MAX) {
        step_motor->step_count++;
    }

    return STEP_MOTOR_ERR_OK;
}

step_motor_err_t step_motor_set_position(step_motor_t* step_motor,
                                         float32_t position,
                                         float32_t sampling_time)
{
    assert(step_motor && sampling_time > 0.0F);

    position =
        fminf(step_motor->config.max_position, fmaxf(step_motor->config.min_position, position));

    float32_t speed = (position - step_motor->prev_position) / sampling_time;

    step_motor_err_t err = step_motor_set_speed(step_motor, speed, sampling_time);

    if (err != STEP_MOTOR_ERR_FAIL) {
        step_motor->prev_position = position;
        step_motor->prev_speed = speed;
    }

    return err;
}

step_motor_err_t step_motor_set_speed(step_motor_t* step_motor,
                                      float32_t speed,
                                      float32_t sampling_time)
{
    assert(step_motor && sampling_time > 0.0F);

    step_motor_direction_t direction =
        step_motor_speed_to_direction(speed, step_motor->config.stall_speed);

    step_motor_err_t err = step_motor_set_direction(step_motor, direction);
    if (direction == STEP_MOTOR_DIRECTION_STOP) {
        return err;
    }

    if (fabsf(speed) < step_motor->config.min_speed) {
        speed = copysignf(step_motor->config.min_speed, speed);
    } else if (fabsf(speed) > step_motor->config.max_speed) {
        speed = copysignf(step_motor->config.max_speed, speed);
    }

    uint32_t frequency = step_motor_speed_to_frequency(speed, step_motor->config.step_change);

    err |= step_motor_set_frequency(step_motor, frequency);

    if (err != STEP_MOTOR_ERR_FAIL) {
        step_motor->prev_speed = speed;
    }

    return err;
}

step_motor_err_t step_motor_set_acceleration(step_motor_t* step_motor,
                                             float32_t acceleration,
                                             float32_t sampling_time)
{
    assert(step_motor && sampling_time > 0.0F);

    if (fabsf(acceleration) < step_motor->config.min_acceleration) {
        acceleration = copysignf(step_motor->config.min_acceleration, acceleration);
    } else if (fabsf(acceleration) > step_motor->config.max_acceleration) {
        acceleration = copysignf(step_motor->config.max_acceleration, acceleration);
    }

    float32_t speed = (acceleration + step_motor->prev_acceleration) * sampling_time / 2.0F;

    step_motor_err_t err = step_motor_set_speed(step_motor, speed, sampling_time);

    if (err != STEP_MOTOR_ERR_FAIL) {
        step_motor->prev_speed = speed;
        step_motor->prev_acceleration = acceleration;
    }

    return err;
}

step_motor_err_t step_motor_set_position_ramp(step_motor_t* motor,
                                              float32_t target_position,
                                              float32_t sampling_time)
{
    assert(motor && sampling_time > 0.0F);

    // Clamp target position
    target_position =
        fminf(motor->config.max_position, fmaxf(motor->config.min_position, target_position));

    float32_t current_position = motor->prev_position;
    float32_t current_speed = motor->prev_speed;
    float32_t distance_to_go = target_position - current_position;

    // Desired direction
    step_motor_direction_t desired_dir =
        (distance_to_go >= 0.0F) ? STEP_MOTOR_DIRECTION_FORWARD : STEP_MOTOR_DIRECTION_BACKWARD;

    // Compute braking distance = v^2 / (2 * a)
    float32_t max_accel = motor->config.max_acceleration;
    float32_t braking_distance = (current_speed * current_speed) / (2.0F * max_accel);
    if (current_speed < 0.0F)
        braking_distance = -braking_distance;

    float32_t next_speed;

    // If we need to slow down (close to target)
    if (fabsf(distance_to_go) <= fabsf(braking_distance)) {
        // Decelerate to stop
        float32_t speed_step = max_accel * sampling_time;
        next_speed = current_speed - copysignf(speed_step, current_speed);

        // Stop if we're near zero
        if (fabsf(next_speed) < motor->config.stall_speed) {
            next_speed = 0.0F;
        }
    } else {
        // Accelerate toward max speed in the right direction
        float32_t target_speed = copysignf(motor->config.max_speed, distance_to_go);
        float32_t speed_step = max_accel * sampling_time;

        if (fabsf(current_speed - target_speed) < speed_step) {
            next_speed = target_speed;
        } else {
            next_speed = current_speed + copysignf(speed_step, target_speed - current_speed);
        }
    }

    // Apply direction change logic if needed
    if (current_speed != 0.0F && ((next_speed > 0.0F && current_speed < 0.0F) ||
                                  (next_speed < 0.0F && current_speed > 0.0F))) {
        // Enforce zero-crossing: fully stop before reversing
        next_speed = 0.0F;
    }

    // Save speed and position
    motor->prev_position += next_speed * sampling_time;
    motor->prev_speed = next_speed;
    motor->prev_acceleration = (next_speed - current_speed) / sampling_time;

    // Drive the motor
    return step_motor_set_speed(motor, next_speed, sampling_time);
}

step_motor_err_t step_motor_set_speed_ramp(step_motor_t* motor,
                                           float32_t target_speed,
                                           float32_t sampling_time)
{
    assert(motor && sampling_time > 0.0F);

    float32_t current_speed = motor->prev_speed;
    float32_t max_accel = motor->config.max_acceleration;

    // Clamp target speed to max limits
    if (fabsf(target_speed) < motor->config.min_speed && target_speed != 0.0F) {
        target_speed = copysignf(motor->config.min_speed, target_speed);
    } else if (fabsf(target_speed) > motor->config.max_speed) {
        target_speed = copysignf(motor->config.max_speed, target_speed);
    }

    // Compute allowed change in speed
    float32_t speed_step = max_accel * sampling_time;

    float32_t next_speed = current_speed;

    // Apply ramping
    if (fabsf(target_speed - current_speed) <= speed_step) {
        next_speed = target_speed;
    } else {
        next_speed = current_speed + copysignf(speed_step, target_speed - current_speed);
    }

    // Enforce stopping before reversing direction
    if ((current_speed > 0.0F && next_speed < 0.0F) ||
        (current_speed < 0.0F && next_speed > 0.0F)) {
        next_speed = 0.0F;
    }

    // Compute direction and frequency
    step_motor_direction_t direction =
        (fabsf(next_speed) < motor->config.stall_speed)
            ? STEP_MOTOR_DIRECTION_STOP
            : (next_speed > 0.0F ? STEP_MOTOR_DIRECTION_FORWARD : STEP_MOTOR_DIRECTION_BACKWARD);

    uint32_t frequency = (uint32_t)fabsf(next_speed / motor->config.step_change);

    // Update internal state
    float32_t acceleration = (next_speed - current_speed) / sampling_time;
    motor->prev_speed = next_speed;
    motor->prev_acceleration = acceleration;

    step_motor_err_t err = step_motor_set_direction(motor, direction);
    err |= step_motor_set_frequency(motor, frequency);

    return err;
}

step_motor_err_t step_motor_set_acceleration_ramp(step_motor_t* motor,
                                                  float32_t target_accel,
                                                  float32_t sampling_time)
{
    assert(motor && sampling_time > 0.0F);

    float32_t current_accel = motor->prev_acceleration;
    float32_t max_jerk = motor->config.max_acceleration / sampling_time;

    // Clamp target acceleration
    if (fabsf(target_accel) < motor->config.min_acceleration && target_accel != 0.0F) {
        target_accel = copysignf(motor->config.min_acceleration, target_accel);
    } else if (fabsf(target_accel) > motor->config.max_acceleration) {
        target_accel = copysignf(motor->config.max_acceleration, target_accel);
    }

    float32_t jerk_step = max_jerk * sampling_time;

    float32_t next_accel;
    if (fabsf(target_accel - current_accel) <= jerk_step) {
        next_accel = target_accel;
    } else {
        next_accel = current_accel + copysignf(jerk_step, target_accel - current_accel);
    }

    // Compute next speed from acceleration (trapezoidal integration)
    float32_t next_speed = motor->prev_speed + 0.5F * (current_accel + next_accel) * sampling_time;

    motor->prev_speed = next_speed;
    motor->prev_acceleration = next_accel;

    return step_motor_set_speed(motor, next_speed, sampling_time);
}

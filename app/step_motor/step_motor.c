#include "step_motor.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <string.h>

static inline float32_t step_motor_get_clamped_position(step_motor_t const* step_motor,
                                                        float32_t position)
{
    if (position < step_motor->config.min_position) {
        return step_motor->config.min_position;
    } else if (position > step_motor->config.max_position) {
        return step_motor->config.max_position;
    }

    return position;
}

static inline float32_t step_motor_get_clamped_speed(step_motor_t const* step_motor,
                                                     float32_t speed)
{
    if (fabsf(speed) < step_motor->config.min_speed) {
        return copysignf(step_motor->config.min_speed, speed);
    } else if (fabsf(speed) > step_motor->config.max_speed) {
        return copysignf(step_motor->config.max_speed, speed);
    }

    return speed;
}

static inline float32_t step_motor_get_clamped_acceleration(step_motor_t const* step_motor,
                                                            float32_t acceleration)
{
    if (fabsf(acceleration) < step_motor->config.min_acceleration) {
        return copysignf(step_motor->config.min_acceleration, acceleration);
    } else if (fabsf(acceleration) > step_motor->config.max_acceleration) {
        return copysignf(step_motor->config.max_acceleration, acceleration);
    }

    return acceleration;
}

static inline step_motor_direction_t step_motor_speed_to_direction(step_motor_t const* step_motor,
                                                                   float32_t speed)
{
    if (fabsf(speed) < step_motor->config.stall_speed) {
        return STEP_MOTOR_DIRECTION_STOP;
    }

    return speed > 0.0f ? STEP_MOTOR_DIRECTION_FORWARD : STEP_MOTOR_DIRECTION_BACKWARD;
}

static inline uint32_t step_motor_speed_to_frequency(step_motor_t const* step_motor,
                                                     float32_t speed)
{
    assert(step_motor->config.step_change > 0.0F);

    return (uint32_t)fabsf(speed / step_motor->config.step_change);
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

void step_motor_update_step_count(step_motor_t* step_motor)
{
    assert(step_motor);

    if (step_motor->direction == STEP_MOTOR_DIRECTION_BACKWARD &&
        step_motor->step_count != LLONG_MIN) {
        step_motor->step_count--;
    } else if (step_motor->direction == STEP_MOTOR_DIRECTION_FORWARD &&
               step_motor->step_count != LLONG_MAX) {
        step_motor->step_count++;
    }
}

step_motor_err_t step_motor_set_position(step_motor_t* step_motor,
                                         float32_t position,
                                         float32_t sampling_time)
{
    assert(step_motor && sampling_time > 0.0F);

    position = step_motor_get_clamped_position(step_motor, position);

    float32_t current_position = step_motor_get_position(step_motor);
    float32_t speed = (position - current_position) / sampling_time;

    return step_motor_set_speed(step_motor, speed);
}

step_motor_err_t step_motor_set_speed(step_motor_t* step_motor, float32_t speed)
{
    assert(step_motor);

    step_motor_direction_t direction = step_motor_speed_to_direction(step_motor, speed);

    step_motor_err_t err = step_motor_set_direction(step_motor, direction);
    if (err != STEP_MOTOR_ERR_OK || direction == STEP_MOTOR_DIRECTION_STOP) {
        return err;
    }

    speed = step_motor_get_clamped_speed(step_motor, speed);

    uint32_t frequency = step_motor_speed_to_frequency(step_motor, speed);

    return step_motor_set_frequency(step_motor, frequency);
}

step_motor_err_t step_motor_set_acceleration(step_motor_t* step_motor,
                                             float32_t acceleration,
                                             float32_t sampling_time)
{
    assert(step_motor && sampling_time > 0.0F);

    acceleration = step_motor_get_clamped_acceleration(step_motor, acceleration);

    float32_t current_acceleration = step_motor_get_acceleration(step_motor, sampling_time);
    float32_t speed = (acceleration + current_acceleration) * sampling_time / 2.0F;

    return step_motor_set_speed(step_motor, speed);
}

float32_t step_motor_get_position(step_motor_t* step_motor)
{
    assert(step_motor);

    float32_t position = (float32_t)step_motor->step_count * step_motor->config.step_change;
    if (position < 0.0F) {
        position += 360.0F;
    }

    return position;
}

float32_t step_motor_get_speed(step_motor_t* step_motor, float32_t sampling_time)
{
    assert(step_motor && sampling_time > 0.0F);

    float32_t position = step_motor_get_position(step_motor);
    float32_t speed = (position - step_motor->prev_position) / sampling_time;

    step_motor->prev_position = position;

    return speed;
}

float32_t step_motor_get_acceleration(step_motor_t* step_motor, float32_t sampling_time)
{
    assert(step_motor && sampling_time > 0.0F);

    float32_t speed = step_motor_get_speed(step_motor, sampling_time);
    float32_t acceleration = (speed - step_motor->prev_speed) / sampling_time;

    step_motor->prev_acceleration = acceleration;

    return acceleration;
}
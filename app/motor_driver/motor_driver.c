#include "motor_driver.h"
#include <assert.h>
#include <string.h>

void motor_driver_initialize(motor_driver_t* driver,
                             rotary_encoder_t const* encoder,
                             pid_regulator_t const* regulator,
                             step_motor_t const* motor)
{
    assert(driver && encoder && regulator && motor);

    memset(driver, 0, sizeof(*driver));
    memcpy(&driver->encoder, encoder, sizeof(*encoder));
    memcpy(&driver->regulator, regulator, sizeof(*regulator));
    memcpy(&driver->motor, motor, sizeof(*motor));
}

void motor_driver_deinitialize(motor_driver_t* driver)
{
    assert(driver);

    memset(driver, 0, sizeof(*driver));
}

void motor_set_position(motor_driver_t* driver, float32_t position, float32_t delta_time)
{
    assert(driver);

    float32_t measured_position;
    rotary_encoder_get_position(&driver->encoder, &measured_position);

    float32_t error_position = position - measured_position;
    float32_t control_speed =
        pid_regulator_get_sat_control(&driver->regulator, error_position, delta_time);

    step_motor_set_speed(&driver->motor, control_speed, delta_time);
}

#include "joint_manager.h"
#include "FreeRTOS.h"
#include "a4988.h"
#include "gpio.h"
#include "motor_driver.h"
#include "pid_regulator.h"
#include "step_motor.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static bool frequency_to_prescaler_and_period(uint32_t frequency,
                                              uint32_t clock_hz,
                                              uint32_t clock_div,
                                              uint32_t max_prescaler,
                                              uint32_t max_period,
                                              uint32_t* prescaler,
                                              uint32_t* period)
{
    if (frequency == 0U || !prescaler || !period) {
        return false;
    }

    uint32_t base_clock = clock_hz / (clock_div + 1U);
    uint32_t temp_prescaler = 0U;
    uint32_t temp_period = base_clock / frequency;

    while (temp_period > max_period && temp_prescaler < max_prescaler) {
        temp_prescaler++;
        temp_period = base_clock / ((temp_prescaler + 1U) * frequency);
    }
    if (temp_period > max_period) {
        temp_period = max_period;
        temp_prescaler = (base_clock / (temp_period * frequency)) - 1U;
    }
    if (temp_prescaler > max_prescaler) {
        temp_prescaler = max_prescaler;
    }

    *prescaler = temp_prescaler;
    *period = temp_period;

    return true;
}

static a4988_err_t a4988_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    HAL_GPIO_WritePin(manager->dir_port, pin, (GPIO_PinState)state);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_stop(void* user)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    HAL_TIM_PWM_Stop_IT(manager->pwm_timer, manager->pwm_channel);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_start(void* user)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    HAL_TIM_PWM_Start_IT(manager->pwm_timer, manager->pwm_channel);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_set_freq(void* user, uint32_t freq)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    uint32_t prescaler;
    uint32_t period;
    if (frequency_to_prescaler_and_period(freq,
                                          80000000U,
                                          0x0U,
                                          0xFFFFU,
                                          0xFFFFU,
                                          &prescaler,
                                          &period)) {
        __HAL_TIM_DISABLE(manager->pwm_timer);
        __HAL_TIM_SET_PRESCALER(manager->pwm_timer, prescaler);
        __HAL_TIM_SET_AUTORELOAD(manager->pwm_timer, period);
        __HAL_TIM_SET_COMPARE(manager->pwm_timer, manager->pwm_channel, period / 2U);
        __HAL_TIM_ENABLE(manager->pwm_timer);
    }

    return A4988_ERR_OK;
}

static step_motor_err_t step_motor_device_set_frequency(void* user, uint32_t frequency)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    a4988_set_frequency(&manager->a4988, frequency);

    return STEP_MOTOR_ERR_OK;
}

static step_motor_err_t step_motor_device_set_direction(void* user,
                                                        step_motor_direction_t direction)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    a4988_set_direction(&manager->a4988, (a4988_direction_t)direction);

    return STEP_MOTOR_ERR_OK;
}

static motor_driver_err_t motor_driver_joint_set_speed(void* user, float32_t speed)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    step_motor_set_speed(&manager->motor, speed);

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_encoder_get_position(void* user, float32_t* position)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    *position = step_motor_get_position(&manager->motor);

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_regulator_get_control(void* user,
                                                             float32_t error,
                                                             float32_t* control,
                                                             float32_t delta_time)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    *control = pid_regulator_get_sat_control(&manager->regulator, error, delta_time);

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_fault_get_current(void* user, float32_t* current)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    *current = 1.F;

    return MOTOR_DRIVER_ERR_OK;
}

static joint_err_t joint_manager_event_start_handler(joint_manager_t* manager)
{
    if (manager->is_running) {
        return JOINT_ERR_ALREADY_RUNNING;
    }

    manager->is_running = true;

    return JOINT_ERR_OK;
}

static joint_err_t joint_manager_event_stop_handler(joint_manager_t* manager)
{
    if (!manager->is_running) {
        return JOINT_ERR_NOT_RUNNING;
    }

    manager->is_running = false;

    return JOINT_ERR_OK;
}

static joint_err_t joint_manager_event_update_handler(joint_manager_t* manager,
                                                      joint_event_payload_t const* payload)
{
    if (!manager->is_running) {
        return JOINT_ERR_NOT_RUNNING;
    }

    manager->delta_time = payload->update.delta_time;
    manager->position = payload->update.position;

    return JOINT_ERR_OK;
}

static joint_err_t joint_manager_notify_delta_timer_handler(joint_manager_t* manager)
{
    // if (!manager->is_running) {
    //     return JOINT_ERR_NOT_RUNNING;
    // }

    motor_driver_set_position(&manager->driver, manager->position, manager->delta_time);

    return JOINT_ERR_OK;
}

static joint_err_t joint_manager_notify_pwm_pulse_handler(joint_manager_t* manager)
{
    // if (!manager->is_running) {
    //     return JOINT_ERR_NOT_RUNNING;
    // }

    step_motor_update_step_count(&manager->motor);

    return JOINT_ERR_OK;
}

joint_err_t joint_manager_notify_handler(joint_manager_t* manager, joint_notify_t notify)
{
    if (notify & JOINT_NOTIFY_DELTA_TIMER) {
        joint_err_t err = joint_manager_notify_delta_timer_handler(manager);
        if (err != JOINT_ERR_OK) {
            return err;
        }
    }

    if (notify & JOINT_NOTIFY_PWM_PULSE) {
        return joint_manager_notify_pwm_pulse_handler(manager);
    }

    return JOINT_ERR_UNKNOWN_NOTIFY;
}

joint_err_t joint_manager_event_handler(joint_manager_t* manager, joint_event_t const* event)
{
    switch (event->type) {
        case JOINT_EVENT_TYPE_START: {
            return joint_manager_event_start_handler(manager);
        }
        case JOINT_EVENT_TYPE_STOP: {
            return joint_manager_event_stop_handler(manager);
        }
        case JOINT_EVENT_TYPE_UPDATE: {
            return joint_manager_event_update_handler(manager, &event->payload);
        }
    }

    return JOINT_ERR_UNKNOWN_EVENT;
}

joint_err_t joint_manager_process(joint_manager_t* manager)
{
    joint_err_t err;

    uint32_t notify;
    if (xTaskNotifyWait(0x00, JOINT_NOTIFY_ALL, &notify, pdMS_TO_TICKS(1))) {
        err = joint_manager_notify_handler(manager, notify);
        if (err != JOINT_ERR_OK) {
            return err;
        }
    }

    // joint_event_t event;
    // if (xQueuePeek(manager->joint_queue, &event, pdMS_TO_TICKS(1))) {
    //     err = joint_manager_event_handler(manager, &event);
    // }

    return err;
}

joint_err_t joint_manager_initialize(joint_manager_t* manager)
{
    manager->is_running = false;

    HAL_TIM_Base_Start_IT(manager->delta_timer);
    HAL_TIM_PWM_Start_IT(manager->pwm_timer, manager->pwm_channel);

    a4988_initialize(&manager->a4988,
                     &(a4988_config_t){.pin_dir = manager->dir_pin},
                     &(a4988_interface_t){.gpio_user = manager,
                                          .gpio_write_pin = a4988_gpio_write_pin,
                                          .pwm_user = manager,
                                          .pwm_start = a4988_pwm_start,
                                          .pwm_stop = a4988_pwm_stop,
                                          .pwm_set_freq = a4988_pwm_set_freq});

    step_motor_initialize(
        &manager->motor,
        &(step_motor_config_t){.min_position = 0.0F,
                               .max_position = 360.0F,
                               .min_speed = 10.0F,
                               .max_speed = 500.0F,
                               .step_change = 1.8F},
        &(step_motor_interface_t){.device_user = manager,
                                  .device_set_frequency = step_motor_device_set_frequency,
                                  .device_set_direction = step_motor_device_set_direction},
        0.0F);

    pid_regulator_initialize(&manager->regulator,
                             &(pid_regulator_config_t){.prop_gain = 10.0F,
                                                       .int_gain = 0.0F,
                                                       .dot_gain = 0.0F,
                                                       .min_control = 10.0F,
                                                       .max_control = 500.0F,
                                                       .sat_gain = 0.0F});

    motor_driver_initialize(
        &manager->driver,
        &(motor_driver_config_t){.min_position = 0.0F,
                                 .max_position = 360.0F,
                                 .min_speed = 10.0F,
                                 .max_speed = 500.0F,
                                 .max_current = 2.0F},
        &(motor_driver_interface_t){.motor_user = manager,
                                    .motor_set_speed = motor_driver_joint_set_speed,
                                    .encoder_user = manager,
                                    .encoder_get_position = motor_driver_encoder_get_position,
                                    .regulator_user = manager,
                                    .regulator_get_control = motor_driver_regulator_get_control,
                                    .fault_user = manager,
                                    .fault_get_current = motor_driver_fault_get_current});

    return JOINT_ERR_OK;
}

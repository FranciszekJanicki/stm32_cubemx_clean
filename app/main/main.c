#include "main.h"
#include "a4988.h"
#include "gpio.h"
#include "motor_driver.h"
#include "pid_regulator.h"
#include "step_motor.h"
#include "stm32l4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>

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
    (void)user;

    HAL_GPIO_WritePin(GPIOA, (uint16_t)pin, (GPIO_PinState)state);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_stop(void* user)
{
    (void)user;

    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_start(void* user)
{
    (void)user;

    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_set_freq(void* user, uint32_t freq)
{
    (void)user;

    uint32_t prescaler;
    uint32_t period;
    if (frequency_to_prescaler_and_period(freq,
                                          80000000U,
                                          0U,
                                          1U << 16U,
                                          1U << 16U,
                                          &prescaler,
                                          &period)) {
        __HAL_TIM_DISABLE(&htim1);
        __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
        __HAL_TIM_SET_AUTORELOAD(&htim1, period);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, period / 2U);
        __HAL_TIM_ENABLE(&htim1);
    }

    return A4988_ERR_OK;
}

static step_motor_err_t step_motor_device_set_frequency(void* user, uint32_t frequency)
{
    a4988_set_frequency((a4988_t*)user, frequency);

    return STEP_MOTOR_ERR_OK;
}

static step_motor_err_t step_motor_device_set_direction(void* user,
                                                        step_motor_direction_t direction)
{
    a4988_set_direction((a4988_t*)user, (a4988_direction_t)direction);

    return STEP_MOTOR_ERR_OK;
}

static motor_driver_err_t motor_driver_motor_set_speed(void* user, float32_t speed)
{
    step_motor_set_speed(user, speed);

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_encoder_get_position(void* user, float32_t* position)
{
    *position = step_motor_get_position(user);

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_regulator_get_control(void* user,
                                                             float32_t error,
                                                             float32_t* control,
                                                             float32_t delta_time)
{
    *control = pid_regulator_get_sat_control(user, error, delta_time);

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_fault_get_current(void* user, float32_t* current)
{
    (void)user;

    *current = 0.5F;

    return MOTOR_DRIVER_ERR_OK;
}

static bool volatile has_delta_timer_elapsed = false;
static bool volatile has_pwm_finished = false;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2) {
        has_delta_timer_elapsed = true;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
        has_pwm_finished = true;
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    a4988_t a4988;
    a4988_initialize(&a4988,
                     &(a4988_config_t){.pin_dir = 1 << 10},
                     &(a4988_interface_t){.gpio_write_pin = a4988_gpio_write_pin,
                                          .pwm_start = a4988_pwm_start,
                                          .pwm_stop = a4988_pwm_stop,
                                          .pwm_set_freq = a4988_pwm_set_freq});

    volatile step_motor_t motor;
    step_motor_initialize(
        &motor,
        &(step_motor_config_t){.min_position = 0.0F,
                               .max_position = 359.0F,
                               .min_speed = 10.0F,
                               .max_speed = 500.0F,
                               .step_change = 0.9F},
        &(step_motor_interface_t){.device_user = &a4988,
                                  .device_set_frequency = step_motor_device_set_frequency,
                                  .device_set_direction = step_motor_device_set_direction},
        0.0F);

    pid_regulator_t regulator;
    pid_regulator_initialize(&regulator,
                             &(pid_regulator_config_t){.prop_gain = 10.0F,
                                                       .int_gain = 0.0F,
                                                       .dot_gain = 0.0F,
                                                       .min_control = 10.0F,
                                                       .max_control = 500.0F,
                                                       .sat_gain = 0.0F});

    motor_driver_t driver;
    motor_driver_initialize(
        &driver,
        &(motor_driver_config_t){.min_position = 0.0F,
                                 .max_position = 359.0F,
                                 .min_speed = 10.0F,
                                 .max_speed = 500.0F,
                                 .max_current = 2.0F},
        &(motor_driver_interface_t){.motor_user = &motor,
                                    .motor_set_speed = motor_driver_motor_set_speed,
                                    .encoder_user = &motor,
                                    .encoder_get_position = motor_driver_encoder_get_position,
                                    .regulator_user = &regulator,
                                    .regulator_get_control = motor_driver_regulator_get_control,
                                    .fault_get_current = motor_driver_fault_get_current});

    float32_t position = 10.0F;
    float32_t position_step = 1.0F;
    float32_t delta_time = 0.01F;

    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(&htim2);

    while (1) {
        if (has_delta_timer_elapsed) {
            motor_driver_set_position(&driver, position, delta_time);

            if (position > 360.0F || position < 0.0F) {
                position_step = -position_step;
            }
            position += position_step;

            has_delta_timer_elapsed = false;
        }

        if (has_pwm_finished) {
            step_motor_update_step_count(&motor);

            has_pwm_finished = false;
        }
    }
}

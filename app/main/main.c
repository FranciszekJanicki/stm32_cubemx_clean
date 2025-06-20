#include "main.h"
#include "a4988.h"
#include "a4988_config.h"
#include "gpio.h"
#include "pid.h"
#include "step_motor.h"
#include "step_motor_config.h"
#include "stm32l4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include <math.h>
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

static a4988_err_t a4988_gpio_write_pin(void*, uint32_t pin, bool state)
{
    HAL_GPIO_WritePin(GPIOA, (uint16_t)pin, (GPIO_PinState)state);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pulse_set_freq(void*, uint32_t freq)
{
    uint32_t prescaler;
    uint32_t period;

    frequency_to_prescaler_and_period(freq,
                                      80000000U,
                                      0U,
                                      1U << 16U,
                                      1U << 16U,
                                      &prescaler,
                                      &period);

    htim1.Instance->PSC = prescaler;
    htim1.Instance->ARR = period;
    htim1.Instance->CCR4 = period / 2U;

    printf("prescaler: %u, period: %u\n\r", prescaler, period);

    return A4988_ERR_OK;
}

static step_motor_err_t step_motor_driver_set_frequency(void* user, uint32_t frequency)
{
    a4988_set_frequency((a4988_t*)user, frequency);

    return STEP_MOTOR_ERR_OK;
}

static step_motor_err_t step_motor_driver_set_direction(void* user,
                                                        step_motor_direction_t direction)
{
    a4988_set_direction((a4988_t*)user, (a4988_direction_t)direction);

    return STEP_MOTOR_ERR_OK;
}

static bool volatile has_sampling_timer_elapsed = false;
static bool volatile has_pwm_pulse_finished = false;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2) {
        has_sampling_timer_elapsed = true;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
        has_pwm_pulse_finished = true;
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

    a4988_t a4988 = {};
    a4988_initialize(&a4988,
                     &(a4988_config_t){.pin_dir = 1 << 10},
                     &(a4988_interface_t){.gpio_write_pin = a4988_gpio_write_pin,
                                          .pulse_set_freq = a4988_pulse_set_freq});

    step_motor_t step_motor = {};
    step_motor_initialize(
        &step_motor,
        &(step_motor_config_t){.min_position = 0.0F,
                               .max_position = 360.0F,
                               .min_speed = 0.0F,
                               .max_speed = 1000.0F,
                               .step_change = 1.8F,
                               .stall_speed = 1.0F},
        &(step_motor_interface_t){.driver_user = &a4988,
                                  .driver_set_frequency = step_motor_driver_set_frequency,
                                  .driver_set_direction = step_motor_driver_set_direction});

    pid_t pid = {};
    pid_initialize(&pid,
                   &(pid_config_t){.p_gain = 1.0F, .i_gain = 0.0F, .d_gain = 0.0F, .sat = 100.0F});

    float32_t ref_position = 360.0F;
    float32_t sampling_time = 0.01F;

    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(&htim2);

    while (1) {
        if (has_sampling_timer_elapsed) {
            float32_t position = step_motor_get_position(&step_motor);
            float32_t error_position = ref_position - position;
            float32_t control_speed = pid_get_sat_u(&pid, error_position, sampling_time);
            step_motor_set_speed(&step_motor, control_speed);

            has_sampling_timer_elapsed = false;
        }

        if (has_pwm_pulse_finished) {
            step_motor_update_step_count(&step_motor);

            has_pwm_pulse_finished = false;
        }
    }
}

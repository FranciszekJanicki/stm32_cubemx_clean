#include "nvic.h"
#include "stm32l4xx_hal.h"

bool volatile has_delta_timer_elapsed = false;
bool volatile has_pwm_finished = false;

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
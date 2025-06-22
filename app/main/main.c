#include "main.h"
#include "gpio.h"
#include "joint_task.h"
#include "stm32l4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    joint_queue_initialize();
    joint_task_initialize();

    vTaskStartScheduler();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    BaseType_t task_woken = pdFALSE;

    if (htim->Instance == TIM2) {
        xTaskNotifyFromISR(joint_task_get_handle(),
                           JOINT_NOTIFY_DELTA_TIMER,
                           eSetBits,
                           &task_woken);
    }
    if (htim->Instance == TIM3) {
        HAL_IncTick();
    }

    portYIELD_FROM_ISR(task_woken);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    BaseType_t task_woken = pdFALSE;

    if (htim->Instance == TIM1) {
        xTaskNotifyFromISR(joint_task_get_handle(), JOINT_NOTIFY_PWM_PULSE, eSetBits, &task_woken);
    }

    portYIELD_FROM_ISR(task_woken);
}

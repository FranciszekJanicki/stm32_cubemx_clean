#include "main.h"
#include "gpio.h"
#include "motor_manager.h"
#include "stm32l4xx_hal.h"
#include "tim.h"
#include "usart.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    motor_manager_init();
}

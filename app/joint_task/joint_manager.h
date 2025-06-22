#ifndef JOINT_TASK_JOINT_MANAGER_H
#define JOINT_TASK_JOINT_MANAGER_H

#include "FreeRTOS.h"
#include "a4988.h"
#include "motor_driver.h"
#include "pid_regulator.h"
#include "queue.h"
#include "rotary_encoder.h"
#include "step_motor.h"
#include "task.h"
#include <stdint.h>
#include <stm32l4xx_hal.h>

typedef struct {
    float32_t position;
    float32_t delta_time;
    float32_t speed;
    bool is_running;

    a4988_t a4988;
    motor_driver_t driver;
    step_motor_t motor;
    rotary_encoder_t encoder;
    pid_regulator_t regulator;

    GPIO_TypeDef* dir_port;
    uint32_t dir_pin;
    TIM_HandleTypeDef* delta_timer;
    TIM_HandleTypeDef* pwm_timer;
    uint32_t pwm_channel;

    QueueHandle_t joint_queue;
} joint_manager_t;

typedef enum {
    JOINT_EVENT_TYPE_START,
    JOINT_EVENT_TYPE_STOP,
    JOINT_EVENT_TYPE_UPDATE,
} joint_event_type_t;

typedef union {
    struct {
        float32_t position;
        float32_t delta_time;
    } update;
} joint_event_payload_t;

typedef struct {
    joint_event_type_t type;
    joint_event_payload_t payload;
} joint_event_t;

typedef enum {
    JOINT_NOTIFY_DELTA_TIMER = (1 << 0),
    JOINT_NOTIFY_PWM_PULSE = (1 << 1),
    JOINT_NOTIFY_ALL = (JOINT_NOTIFY_DELTA_TIMER | JOINT_NOTIFY_PWM_PULSE),
} joint_notify_t;

typedef enum {
    JOINT_ERR_OK,
    JOINT_ERR_FAIL,
    JOINT_ERR_NOT_RUNNING,
    JOINT_ERR_ALREADY_RUNNING,
    JOINT_ERR_UNKNOWN_EVENT,
    JOINT_ERR_UNKNOWN_NOTIFY,
} joint_err_t;

joint_err_t joint_manager_initialize(joint_manager_t* manager);
joint_err_t joint_manager_process(joint_manager_t* manager);

#endif // JOINT_TASK_JOINT_MANAGER_H
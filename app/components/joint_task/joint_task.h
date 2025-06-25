#ifndef JOINT_TASK_JOINT_TASK_H
#define JOINT_TASK_JOINT_TASK_H

#include "joint_manager.h"

joint_err_t joint_task_initialize(joint_manager_t const* manager);
TaskHandle_t joint_task_get_handle(void);

joint_err_t joint_queue_initialize(void);
QueueHandle_t joint_queue_get_handle(void);

#endif // JOINT_TASK_JOINT_TASK_H
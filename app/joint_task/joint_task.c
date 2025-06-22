#include "joint_task.h"
#include "FreeRTOS.h"
#include "joint_manager.h"
#include "task.h"
#include "tim.h"
#include <assert.h>
#include <string.h>

static TaskHandle_t joint_task;
static StaticTask_t joint_task_buffer;
static StackType_t joint_task_stack[4096 / sizeof(StackType_t)];

static QueueHandle_t joint_queue;
static StaticQueue_t joint_queue_buffer;
static uint8_t joint_queue_storage[1U * sizeof(joint_event_t)];

static void joint_task_func(void* ctx)
{
    joint_manager_t manager;
    memcpy(&manager, ctx, sizeof(manager));

    manager.joint_queue = joint_queue;
    joint_manager_initialize(&manager);

    uint32_t notify = {};
    joint_event_t event = {};

    float position = 0.0F, step = 1.0F;

    while (1) {
        manager.position = position;
        joint_manager_process(&manager);

        if (position > 360.0F || position < 0.0F) {
            step = -step;
        }
        position += step;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

joint_err_t joint_task_initialize(joint_manager_t const* manager)
{
    joint_task = xTaskCreateStatic(joint_task_func,
                                   "joint_task",
                                   sizeof(joint_task_stack) / sizeof(StackType_t),
                                   manager,
                                   1U,
                                   joint_task_stack,
                                   &joint_task_buffer);

    return joint_task ? JOINT_ERR_OK : JOINT_ERR_FAIL;
}

TaskHandle_t joint_task_get_handle(void)
{
    return joint_task;
}

joint_err_t joint_queue_initialize(void)
{
    joint_queue =
        xQueueCreateStatic(1U, sizeof(joint_event_t), joint_queue_storage, &joint_queue_buffer);

    return joint_queue ? JOINT_ERR_OK : JOINT_ERR_FAIL;
}

QueueHandle_t joint_queue_get_handle(void)
{
    return joint_queue;
}
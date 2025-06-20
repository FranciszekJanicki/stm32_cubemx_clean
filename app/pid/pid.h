#ifndef PID_PID_H
#define PID_PID_H

#include "pid_config.h"

typedef struct {
    pid_state_t state;
    pid_config_t config;
} pid_t;

void pid_initialize(pid_t* pid, pid_config_t const* config);
void pid_deinitialize(pid_t* pid);

void pid_reset(pid_t* pid);

float32_t pid_get_u(pid_t* pid, float32_t e, float32_t dt);
float32_t pid_get_sat_u(pid_t* pid, float32_t e, float32_t dt);

#endif // PID_PID_H

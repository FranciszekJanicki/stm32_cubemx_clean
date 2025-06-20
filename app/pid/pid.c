#include "pid.h"
#include "pid_config.h"
#include <assert.h>
#include <string.h>

static inline float32_t pid_get_p_term(pid_t* pid, float32_t e)
{
    return pid->config.p_gain * e;
}

static inline float32_t pid_get_d_term(pid_t* pid, float32_t e, float32_t dt)
{
    float32_t dot_e = (e - pid->state.prev_e) / dt;

    if (pid->config.d_time > 0.0F) {
        float32_t alpha = dt / (pid->config.d_time + dt);
        pid->state.dot_e = alpha * dot_e + (1.0F - alpha) * pid->state.dot_e;
    } else {
        pid->state.dot_e = dot_e;
    }

    return pid->config.d_gain * pid->state.dot_e;
}

static inline float32_t pid_get_i_term(pid_t* pid, float32_t e, float32_t dt)
{
    pid->state.int_e += ((e + pid->state.prev_e) * dt / 2.0F);
    pid->state.int_sat_e += ((pid->state.sat_e + pid->state.prev_sat_e) * dt / 2.0F);

    return pid->config.i_gain * pid->state.int_e - pid->config.sat_gain * pid->state.int_sat_e;
}

static inline float32_t pid_get_clamped_u(pid_t* pid, float32_t u)
{
    if (u > pid->config.sat) {
        u = pid->config.sat;
    } else if (u < -pid->config.sat) {
        u = -pid->config.sat;
    }

    return u;
}

void pid_initialize(pid_t* pid, pid_config_t const* config)
{
    assert(pid && config);

    pid_reset(pid);
    pid->config = *config;
}

void pid_deinitialize(pid_t* pid)
{
    assert(pid);

    pid_reset(pid);
}

void pid_reset(pid_t* pid)
{
    assert(pid);

    memset(pid, 0, sizeof(*pid));
}

float32_t pid_get_u(pid_t* pid, float32_t e, float32_t dt)
{
    assert(pid && dt > 0.0F);

    float32_t p_term = pid_get_p_term(pid, e);
    float32_t i_term = pid_get_i_term(pid, e, dt);
    float32_t d_term = pid_get_d_term(pid, e, dt);

    pid->state.prev_e = e;

    return p_term + i_term + d_term;
}

float32_t pid_get_sat_u(pid_t* pid, float32_t e, float32_t dt)
{
    assert(pid && dt > 0.0F);

    float32_t u = pid_get_u(pid, e, dt);
    float32_t sat_u = pid_get_clamped_u(pid, u);

    pid->state.prev_sat_e = pid->state.sat_e;
    pid->state.sat_e = u - sat_u;

    return sat_u;
}

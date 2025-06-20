#ifndef PID_PID_CONFIG_H
#define PID_PID_CONFIG_H

typedef float float32_t;

typedef struct {
    float32_t prev_e;
    float32_t int_e;
    float32_t dot_e;
    float32_t sat_e;
    float32_t prev_sat_e;
    float32_t int_sat_e;
} pid_state_t;

typedef struct {
    float32_t p_gain;
    float32_t i_gain;
    float32_t d_gain;
    float32_t d_time;
    float32_t sat_gain;
    float32_t sat;
} pid_config_t;

#endif // PID_PID_CONFIG_H
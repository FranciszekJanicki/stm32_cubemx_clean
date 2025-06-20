#include "rotary_encoder.h"
#include <assert.h>
#include <string.h>

rotary_encoder_err_t rotary_encoder_initialize(rotary_encoder_t* encoder,
                                               rotary_encoder_config_t const* config,
                                               rotary_encoder_interface_t const* interface)
{
    assert(encoder);
}

rotary_encoder_err_t rotary_encoder_deinitialize(rotary_encoder_t* encoder)
{
    assert(encoder);
}

rotary_encoder_err_t rotary_encoder_get_position(rotary_encoder_t* encoder, float32_t* position)
{
    assert(encoder && position);
}

rotary_encoder_err_t rotary_encoder_get_speed(rotary_encoder_t* encoder,
                                              float32_t* speed,
                                              float32_t delta_time)
{
    assert(encoder && speed);
}

rotary_encoder_err_t rotary_encoder_get_acceleration(rotary_encoder_t* encoder,
                                                     float32_t* acceleration,
                                                     float32_t delta_time)
{
    assert(encoder && acceleration);
}
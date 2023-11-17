#ifndef __WS2812_ENCODER_HPP
#define __WS2812_ENCODER_HPP

#include <stdint.h>

#include "driver/rmt_encoder.h"

typedef struct {
    gpio_num_t pin;
    uint32_t resolution_hz;
} ws2812_encoder_config_t;

extern esp_err_t rmt_new_ws2812_encoder(const ws2812_encoder_config_t *config,
                                        rmt_channel_handle_t *txChannel,
                                        rmt_encoder_handle_t *ret_encoder);

#endif
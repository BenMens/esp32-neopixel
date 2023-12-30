#ifndef __WS2812_ENCODER_HPP
#define __WS2812_ENCODER_HPP

#include <stdint.h>

#include "driver/rmt_encoder.h"

typedef enum {
    WS2812B_ENCODING,
    WS2812F_ENCODING,
    SK68XXMINI_ENCODING,
} ws2812_encoding_t;

typedef struct {
    gpio_num_t pin;
    uint32_t resolution_hz;
    ws2812_encoding_t encoding_type;
} ws2812_encoder_config_t;

extern esp_err_t rmt_ws2812_encoder(const ws2812_encoder_config_t *config,
                                    rmt_channel_handle_t *txChannel,
                                    rmt_encoder_handle_t *ret_encoder);

#endif
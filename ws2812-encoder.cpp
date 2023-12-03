#include "ws2812-encoder.hpp"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_err.h"

static const char *TAG = "ws2812_encoder";

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *copy_encoder;
    rmt_encoder_t *bytes_encoder;
    rmt_symbol_word_t end_symbol;
    int state;
} ws2812_encoder_t;

IRAM_ATTR static size_t rmt_ws2812_encode(rmt_encoder_t *encoder,
                                          rmt_channel_handle_t channel,
                                          const void *primary_data,
                                          size_t data_size,
                                          rmt_encode_state_t *ret_state)
{
    ws2812_encoder_t *ws2812_encoder =
        __containerof(encoder, ws2812_encoder_t, base);

    size_t encodedSymbols = 0;
    int returnState = RMT_ENCODING_RESET;

    bool shouldYield = false;
    while (!shouldYield) {
        rmt_encode_state_t session_state = RMT_ENCODING_RESET;

        switch (ws2812_encoder->state) {
            case 0: {
                // send led data

                rmt_encoder_handle_t bytes_encoder =
                    ws2812_encoder->bytes_encoder;

                encodedSymbols +=
                    bytes_encoder->encode(bytes_encoder, channel, primary_data,
                                          data_size, &session_state);

                if (session_state & RMT_ENCODING_COMPLETE) {
                    ws2812_encoder->state = 1;
                }

            } break;

            case 1: {
                // send ending code

                rmt_encoder_handle_t copy_encoder =
                    ws2812_encoder->copy_encoder;

                encodedSymbols += copy_encoder->encode(
                    copy_encoder, channel, &ws2812_encoder->end_symbol,
                    sizeof(rmt_symbol_word_t), &session_state);

                if (session_state & RMT_ENCODING_COMPLETE) {
                    ws2812_encoder->state = 0;
                    returnState |= RMT_ENCODING_COMPLETE;
                    shouldYield = true;
                }

            } break;
        }

        if (session_state & RMT_ENCODING_MEM_FULL) {
            returnState |= RMT_ENCODING_MEM_FULL;
            shouldYield = true;
        }
    }

    *ret_state = (rmt_encode_state_t)returnState;

    return encodedSymbols;
}

static esp_err_t rmt_ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *ws2812_encoder =
        __containerof(encoder, ws2812_encoder_t, base);

    rmt_encoder_reset(ws2812_encoder->copy_encoder);
    rmt_encoder_reset(ws2812_encoder->bytes_encoder);
    ws2812_encoder->state = 0;

    return ESP_OK;
}

static esp_err_t rmt_ws2812_encoder_delete(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *ws2812_encoder =
        __containerof(encoder, ws2812_encoder_t, base);

    rmt_del_encoder(ws2812_encoder->copy_encoder);
    rmt_del_encoder(ws2812_encoder->bytes_encoder);
    free(ws2812_encoder);

    return ESP_OK;
}

esp_err_t rmt_ws2812_encoder(const ws2812_encoder_config_t *config,
                             rmt_channel_handle_t *txChannel,
                             rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_copy_encoder_config_t copy_encoder_config;
    rmt_bytes_encoder_config_t bytes_encoder_config;

    if (config->encoding_type == WS2812F_ENCODING) {
        bytes_encoder_config = {
            .bit0 = {{
                .duration0 = (uint16_t)(0.3 * config->resolution_hz / 1000000),
                .level0 = 1,
                .duration1 = (uint16_t)(0.9 * config->resolution_hz / 1000000),
                .level1 = 0,
            }},
            .bit1 = {{
                .duration0 = (uint16_t)(0.6 * config->resolution_hz / 1000000),
                .level0 = 1,
                .duration1 = (uint16_t)(0.6 * config->resolution_hz / 1000000),
                .level1 = 0,
            }},
            .flags =
                {
                    .msb_first = true,
                },
        };

    } else {
        bytes_encoder_config = {
            .bit0 = {{
                .duration0 = (uint16_t)(0.35 * config->resolution_hz / 1000000),
                .level0 = 1,
                .duration1 = (uint16_t)(0.8 * config->resolution_hz / 1000000),
                .level1 = 0,
            }},
            .bit1 = {{
                .duration0 = (uint16_t)(0.7 * config->resolution_hz / 1000000),
                .level0 = 1,
                .duration1 = (uint16_t)(0.6 * config->resolution_hz / 1000000),
                .level1 = 0,
            }},
            .flags =
                {
                    .msb_first = true,
                },
        };
    }

    rmt_tx_channel_config_t channelConfig = {
        .gpio_num = config->pin,
        .clk_src = RMT_CLK_SRC_APB,
        .resolution_hz = config->resolution_hz,
        .mem_block_symbols = 128,
        .trans_queue_depth = 1,
        .flags =
            {
                .invert_out = false,
                .with_dma = false,
                .io_loop_back = false,
                .io_od_mode = false,
            },
        .intr_priority = 0,
    };

    ws2812_encoder_t *ws2812_encoder = NULL;

    ESP_GOTO_ON_FALSE(config && ret_encoder && txChannel, ESP_ERR_INVALID_ARG,
                      err, TAG, "invalid argument");

    ESP_GOTO_ON_ERROR(rmt_new_tx_channel(&channelConfig, txChannel), err, TAG,
                      "error during channel creation");

    ESP_GOTO_ON_ERROR(rmt_enable(*txChannel), err, TAG,
                      "could not enable channel");

    ws2812_encoder = (ws2812_encoder_t *)calloc(1, sizeof(ws2812_encoder_t));
    ESP_GOTO_ON_FALSE(ws2812_encoder, ESP_ERR_NO_MEM, err, TAG,
                      "no mem for ir nec encoder");
    ws2812_encoder->base.encode = rmt_ws2812_encode;
    ws2812_encoder->base.del = rmt_ws2812_encoder_delete;
    ws2812_encoder->base.reset = rmt_ws2812_encoder_reset;

    ws2812_encoder->end_symbol = {{
        .duration0 = (uint16_t)(300 * config->resolution_hz / 1000000),
        .level0 = 0,
        .duration1 = 1,  // cannot be 0 to distinguish from end-marker
        .level1 = 0,
    }};

    rmt_new_copy_encoder(&copy_encoder_config, &ws2812_encoder->copy_encoder);

    rmt_new_bytes_encoder(&bytes_encoder_config,
                          &ws2812_encoder->bytes_encoder);

    rmt_encoder_reset(&ws2812_encoder->base);

    *ret_encoder = &ws2812_encoder->base;

    return ret;

err:
    if (ws2812_encoder) {
        if (ws2812_encoder->bytes_encoder) {
            rmt_del_encoder(ws2812_encoder->bytes_encoder);
        }
        if (ws2812_encoder->copy_encoder) {
            rmt_del_encoder(ws2812_encoder->copy_encoder);
        }
        free(ws2812_encoder);
    }

    return ret;
}
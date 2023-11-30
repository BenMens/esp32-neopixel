#include "neopixel.hpp"

#include "esp_log.h"
#include "math.h"

#define TAG "NEOPIXEL"

Pixels::Pixels(gpio_num_t pin, int pixelCount, StripType stripType,
               ColorOrder colorOrder, double gamma)
    : pixelCount(pixelCount), stripType(stripType), colorOrder(colorOrder)
{
    if (stripType == StripType::WS2812B || stripType == StripType::WS2812F) {
        colorChannelCount = 3;
        pixelData = new uint8_t[pixelCount * colorChannelCount];

        ws2812_encoder_config_t config;
        if (stripType == StripType::WS2812B) {
            config = {
                .pin = pin,
                .resolution_hz = 80 * 1000 * 1000,
                .encoding_type = WS2812B_ENCODING,
            };

        } else {
            config = {
                .pin = pin,
                .resolution_hz = 80 * 1000 * 1000,
                .encoding_type = WS2812F_ENCODING,
            };
        }

        rmt_ws2812_encoder(&config, &txChannel, &pixelEncoder);
    } else {
        ESP_LOGE(TAG, "strip type not supported");
        return;
    }

    setupPixels();
    setupGammaTable(gamma);
    write();
}

void Pixels::write()
{
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
        .flags =
            {
                .eot_level = 0,
            },
    };

    rmt_transmit(txChannel, pixelEncoder, pixelData,
                 pixelCount * colorChannelCount, &transmit_config);
}

void Pixels::waitTillWriteCompletes()
{
    rmt_tx_wait_all_done(txChannel, 2000);
}

void Pixels::clear()
{
    setupPixels();
}

void Pixels::setupGammaTable(double gamma)
{
    for (int i = 0; i <= 255; i++) {
        gammaTable[i] = round(pow((float)i / 255, gamma) * 255 + 0.49999);
    }
}

void Pixels::setupPixels()
{
    for (int i = 0; i < pixelCount; i++) {
        setupPixel(i);
    }
}

void Pixels::setupPixel(int index)
{
    int firstByteIndex = index * colorChannelCount;

    for (int i = 0; i < colorChannelCount; i++) {
        pixelData[firstByteIndex + i] = 0;
    }
}

void Pixels::setPixel(int index, Pixel pixel)
{
    setPixel(index, pixel.red, pixel.green, pixel.blue, pixel.white);
}

void Pixels::setPixel(int index, uint32_t color)
{
    setPixel(index, (color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff, 0);
}

void Pixels::setPixel(int index, uint8_t red, uint8_t green, uint8_t blue,
                      uint8_t white)
{
    if (index > pixelCount || index < 0) {
        ESP_LOGE(TAG, "index out of range");
        return;
    }

    uint8_t *pixel = pixelData + index * colorChannelCount;

    if (colorOrder == ColorOrder::GRB) {
        if (colorChannelCount > 0) {
            pixel[0] = gammaTable[green];
        }
        if (colorChannelCount > 1) {
            pixel[1] = gammaTable[red];
        }
        if (colorChannelCount > 2) {
            pixel[2] = gammaTable[blue];
        }
        if (colorChannelCount > 3) {
            pixel[3] = gammaTable[white];
        }

    } else if (colorOrder == ColorOrder::RGB) {
        if (colorChannelCount > 0) {
            pixel[0] = gammaTable[red];
        }
        if (colorChannelCount > 1) {
            pixel[1] = gammaTable[green];
        }
        if (colorChannelCount > 2) {
            pixel[2] = gammaTable[blue];
        }
        if (colorChannelCount > 3) {
            pixel[3] = gammaTable[white];
        }
    }
}

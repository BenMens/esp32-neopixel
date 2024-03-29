#ifndef _NEO_PIXEL_h
#define _NEO_PIXEL_h

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "ws2812-encoder.hpp"

struct Pixel {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t white;
};

class Pixels
{
   public:
    enum class StripType { WS2812F, WS2812B, SK68XXMINI };
    enum class ColorOrder { RGB, GRB };

    Pixels(gpio_num_t pin, int pixelCount, StripType stripType,
           ColorOrder colorOrder, double gamma);

    void setPixel(int index, Pixel pixel);
    void setPixel(int index, uint8_t red, uint8_t green, uint8_t blue,
                  uint8_t white);
    void setPixel(int index, uint32_t color);

    void write();
    void waitTillWriteCompletes();
    void clear();

   private:
    int pixelCount;
    int colorChannelCount;
    rmt_channel_handle_t txChannel;
    rmt_encoder_handle_t pixelEncoder;
    double gamma;
    StripType stripType;
    ColorOrder colorOrder;
    uint8_t* pixelData;

    void setupPixels();
    void setupPixel(int index);
    void setupGammaTable(double gamma);

    uint8_t gammaTable[255];
};

#endif
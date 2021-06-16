#ifndef _NEO_PIXEL_h
#define _NEO_PIXEL_h

#include "driver/gpio.h"
#include "driver/rmt.h"

struct Pixel {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t white;
};

class Pixels {
  public:
    enum class StripType {
      ws2812,
      ws6812
    };

    Pixels( gpio_num_t pin, int pixelCount, StripType stripType, 
            rmt_channel_t channel, double gamma);

    void setPixel(int index, Pixel pixel);
    void setPixel(int index, 
                  uint8_t red, 
                  uint8_t green, 
                  uint8_t blue, 
                  uint8_t white);
    void setPixel(int index, uint32_t color);
    Pixel getPixel(int index);

    void write();
    void clear();

  private:
    gpio_num_t pin;
    int pixelCount;
    int bitCount;
    int colorChannelCount;
    rmt_channel_t channel;
    double gamma;
    StripType stripType;
    rmt_item32_t* rmtItems;
    uint8_t* pixelData;

    int zeroBitHighTime = 0;
    int zeroBitLowTime = 0;
    int oneBitHighTime = 0;
    int oneBitLowTime = 0;

    void setupPixels();
    void setupPixel(int index);
    void setupPixelBit(int index);
    void setupGammaTable();
    void setupRmt();
    void setupTiming();

    uint8_t gammaTable[255];
};

#endif
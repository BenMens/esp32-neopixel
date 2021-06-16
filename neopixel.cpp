#include "neopixel.hpp"
#include "driver/rmt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "malloc.h"
#include "freertos/task.h"
#include <stdexcept>
#include "math.h"

#define TAG "NEOPIXEL"

Pixels::Pixels( gpio_num_t pin, int pixelCount, StripType stripType, 
                rmt_channel_t channel, double gamma)
    : pin(pin),
      pixelCount(pixelCount),
      channel(channel),
      gamma(gamma),
      stripType(stripType) {

  if (stripType == StripType::ws2812) {
    colorChannelCount = 3;
    bitCount = pixelCount * colorChannelCount * 8,
    rmtItems = new rmt_item32_t[bitCount + 1];

    pixelData = new uint8_t[pixelCount * colorChannelCount];
  } else {
    ESP_LOGE(TAG, "strip type not supported");
    return;
  }

  setupTiming();
  setupRmt();
  setupPixels();
  setupGammaTable();
  write();
}

void Pixels::write() {
  rmt_write_items(this->channel, this->rmtItems, this->bitCount + 1, true);
}

void Pixels::clear() {
  setupPixels();
}

void Pixels::setupTiming() {
  // based on 80Mhz clock freq
  if (stripType == StripType::ws2812) {
    oneBitHighTime = 56;
    oneBitLowTime = 48;
    zeroBitHighTime = 28;
    zeroBitLowTime = 64;

    // Make sure that the transmission is ended
    // with a pause to make sure that
    // two consecutive wites do not interfere    
    rmtItems[bitCount].duration0 = 4000;
    rmtItems[bitCount].level0 = 0;
    rmtItems[bitCount].duration1 = 0;
    rmtItems[bitCount].level1 = 0;
    
  } else {
    ESP_LOGE(TAG, "strip type not supported");
    return;
  }
}

void Pixels::setupGammaTable() {
  for (int i = 0; i <= 255; i++) {
    gammaTable[i] = round(pow((float)i / 255, gamma) * 255 + 0.49999);
  }
}

void Pixels::setupPixels() {
  for (int i = 0; i < pixelCount; i++) {
    this->setupPixel(i);
  }	
}

void Pixels::setupPixel(int index) {
  int firstByteIndex = index * colorChannelCount;

  for (int i = 0; i < colorChannelCount; i++) {
    this->pixelData[firstByteIndex + i] = 0;
  }

  for (
      int i = index * colorChannelCount * 8
      ; i < (index + 1) * colorChannelCount * 8
      ; i++) {

    setupPixelBit(i);
  }
}

void Pixels::setupPixelBit(int index) {
  this->rmtItems[index].level0 = 1;
  this->rmtItems[index].level1 = 0;
  this->rmtItems[index].duration0 = zeroBitHighTime;
  this->rmtItems[index].duration1 = zeroBitLowTime;
}

void Pixels::setupRmt() {
  rmt_config_t config;
  config.rmt_mode = RMT_MODE_TX;
  config.channel = channel;
  config.gpio_num = pin;
  config.mem_block_num = 1;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.clk_div = 1;

  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}

Pixel Pixels::getPixel(int index) {
  if (index > pixelCount || index < 0) {
    ESP_LOGE(TAG, "index out of range");
    return {.red = 0x00, .green = 0x00, .blue=0x00, .white=0x00};
  }

  int firstByteIndex = index * 4;

  Pixel pixel;
  pixel.red 		= this->pixelData[firstByteIndex + 0];
  pixel.green 	= this->pixelData[firstByteIndex + 1];
  pixel.blue 		= this->pixelData[firstByteIndex + 2];
  if (colorChannelCount > 3) {
    pixel.white 	= this->pixelData[firstByteIndex + 3];
  } else {
    pixel.white     = 0;
  }

  return pixel;
}

void Pixels::setPixel(int index, Pixel pixel) {
  setPixel(index, pixel.red, pixel.green, pixel.blue, pixel.white);
}

void Pixels::setPixel(int index, uint32_t color) {
  setPixel(index, (color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff, 0);
}

void Pixels::setPixel(int index, uint8_t red, uint8_t green, 
                      uint8_t blue, uint8_t white) {
  if (index > pixelCount || index < 0) {
    ESP_LOGE(TAG, "index out of range");
    return;
  }
  
  int firstByteIndex = index * colorChannelCount;

  this->pixelData[firstByteIndex + 0] = red;
  this->pixelData[firstByteIndex + 1] = green;
  this->pixelData[firstByteIndex + 2] = blue;
  this->pixelData[firstByteIndex + 3] = white;

  uint32_t widePixelData =  gammaTable[green] << 24 
                            | gammaTable[red] << 16 
                            | gammaTable[blue] << 8 
                            | gammaTable[white];

  uint32_t mask = 1 << 31;  
  int startBit = index * 8 * colorChannelCount;
  for (int i = startBit; i < startBit + 8 * colorChannelCount; i++) {
    if (widePixelData & mask) {
      this->rmtItems[i].duration0 = oneBitHighTime;
      this->rmtItems[i].duration1 = oneBitLowTime;
    } else {
      this->rmtItems[i].duration0 = zeroBitHighTime;
      this->rmtItems[i].duration1 = zeroBitLowTime;
    }
    
    mask >>= 1;
  }
}

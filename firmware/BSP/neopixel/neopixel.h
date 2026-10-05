/**
  ******************************************************************************
  * @file    neopixel.h
  * @brief   WS2812B NeoPixel RGB LED driver for STM32 HAL (PWM + DMA)
  ******************************************************************************
  */

#ifndef BSP_NEOPIXEL_H
#define BSP_NEOPIXEL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Adafruit NeoPixel Ring 12 LEDs (PID 1643) */
#define NUM_PIXELS   12

/**
  * @brief  Set RGB color for a specific pixel
  * @param  index: Pixel index (0 to NUM_PIXELS - 1)
  * @param  r: Red intensity (0..255)
  * @param  g: Green intensity (0..255)
  * @param  b: Blue intensity (0..255)
  */
void np_led_set_RGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

/**
  * @brief  Set RGB color for all pixels simultaneously
  * @param  r: Red intensity (0..255)
  * @param  g: Green intensity (0..255)
  * @param  b: Blue intensity (0..255)
  */
void np_led_set_all_RGB(uint8_t r, uint8_t g, uint8_t b);

/**
  * @brief  Trigger DMA transfer to render the colors on the NeoPixel ring
  */
void np_led_render(void);

/**
  * @brief  Clear all pixels (turn off) and render immediately
  */
void np_led_clear(void);

/**
  * @brief  Check if a DMA transfer is currently in progress
  * @retval true if busy, false if ready for next render
  */
bool np_led_is_busy(void);

#endif /* BSP_NEOPIXEL_H */

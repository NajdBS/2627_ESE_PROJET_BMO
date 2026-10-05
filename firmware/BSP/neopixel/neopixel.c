/**
  ******************************************************************************
  * @file    neopixel.c
  * @brief   WS2812B NeoPixel RGB LED driver implementation using TIM15 PWM + DMA
  ******************************************************************************
  */

#include "neopixel.h"
#include "tim.h"
#include "dma.h"

extern TIM_HandleTypeDef htim15;
extern DMA_HandleTypeDef hdma_tim15_ch1;

/* WS2812B Timing Constants @ 170 MHz (ARR = 212, T = 1.25 us)
 * High bit (1): ~0.8 us high -> 135
 * Low bit (0):  ~0.4 us high -> 67
 */
#define PWM_HI          (135)
#define PWM_LO          (67)

#define NUM_BPP         (3)  /* WS2812B: 3 bytes per pixel (GRB) */
#define NUM_BYTES       (NUM_BPP * NUM_PIXELS)

/* Double buffer for ping-pong DMA transfer: 2 LEDs * 24 bits = 48 bytes */
#define WR_BUF_LEN      (NUM_BPP * 8 * 2)

/* Internal frame buffer: GRB format */
static uint8_t rgb_arr[NUM_BYTES] = {0};

/* DMA ping-pong write buffer (word aligned for DMA) */
static uint8_t wr_buf[WR_BUF_LEN] __attribute__((aligned(4))) = {0};
static volatile uint8_t wr_buf_p = 0;

static inline uint8_t scale8(uint8_t x, uint8_t scale)
{
    return (uint8_t)(((uint16_t)x * scale) >> 8);
}

void np_led_set_RGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= NUM_PIXELS) {
        return;
    }

    /* WS2812B color order is Green, Red, Blue (GRB) */
    rgb_arr[3 * index + 0] = scale8(g, 0xB0); /* Green scaling for balance */
    rgb_arr[3 * index + 1] = r;
    rgb_arr[3 * index + 2] = scale8(b, 0xF0); /* Blue scaling */
}

void np_led_set_all_RGB(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t i = 0; i < NUM_PIXELS; i++) {
        np_led_set_RGB(i, r, g, b);
    }
}

bool np_led_is_busy(void)
{
    return (wr_buf_p != 0) || (hdma_tim15_ch1.State != HAL_DMA_STATE_READY);
}

void np_led_render(void)
{
    if (np_led_is_busy()) {
        /* Cancel ongoing transfer if still active */
        for (uint8_t i = 0; i < WR_BUF_LEN; i++) {
            wr_buf[i] = 0;
        }
        wr_buf_p = 0;
        HAL_TIM_PWM_Stop_DMA(&htim15, TIM_CHANNEL_1);
        return;
    }

    /* Pre-fill first two LEDs (LED 0 in 1st half, LED 1 in 2nd half) */
    for (uint8_t i = 0; i < 8; i++) {
        wr_buf[i     ] = PWM_LO << (((rgb_arr[0] << i) & 0x80) > 0);
        wr_buf[i +  8] = PWM_LO << (((rgb_arr[1] << i) & 0x80) > 0);
        wr_buf[i + 16] = PWM_LO << (((rgb_arr[2] << i) & 0x80) > 0);
        wr_buf[i + 24] = PWM_LO << (((rgb_arr[3] << i) & 0x80) > 0);
        wr_buf[i + 32] = PWM_LO << (((rgb_arr[4] << i) & 0x80) > 0);
        wr_buf[i + 40] = PWM_LO << (((rgb_arr[5] << i) & 0x80) > 0);
    }

    wr_buf_p = 2; /* Next LED index to load */
    HAL_TIM_PWM_Start_DMA(&htim15, TIM_CHANNEL_1, (uint32_t *)wr_buf, WR_BUF_LEN);
}

void np_led_clear(void)
{
    np_led_set_all_RGB(0, 0, 0);
    np_led_render();
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM15) {
        return;
    }

    if (wr_buf_p < NUM_PIXELS) {
        /* Fill first half of buffer (even LED) */
        for (uint8_t i = 0; i < 8; i++) {
            wr_buf[i     ] = PWM_LO << (((rgb_arr[3 * wr_buf_p    ] << i) & 0x80) > 0);
            wr_buf[i +  8] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 1] << i) & 0x80) > 0);
            wr_buf[i + 16] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 2] << i) & 0x80) > 0);
        }
        wr_buf_p++;
    } else if (wr_buf_p < NUM_PIXELS + 2) {
        /* First half reset latch (zero fill, >50 us) */
        for (uint8_t i = 0; i < WR_BUF_LEN / 2; i++) {
            wr_buf[i] = 0;
        }
        wr_buf_p++;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM15) {
        return;
    }

    if (wr_buf_p < NUM_PIXELS) {
        /* Fill second half of buffer (odd LED) */
        for (uint8_t i = 0; i < 8; i++) {
            wr_buf[i + 24] = PWM_LO << (((rgb_arr[3 * wr_buf_p    ] << i) & 0x80) > 0);
            wr_buf[i + 32] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 1] << i) & 0x80) > 0);
            wr_buf[i + 40] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 2] << i) & 0x80) > 0);
        }
        wr_buf_p++;
    } else if (wr_buf_p < NUM_PIXELS + 2) {
        /* Second half reset latch (zero fill) */
        for (uint8_t i = WR_BUF_LEN / 2; i < WR_BUF_LEN; i++) {
            wr_buf[i] = 0;
        }
        wr_buf_p++;
    } else {
        /* Frame transfer complete */
        wr_buf_p = 0;
        HAL_TIM_PWM_Stop_DMA(&htim15, TIM_CHANNEL_1);
    }
}

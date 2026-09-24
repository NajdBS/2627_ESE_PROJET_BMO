/**
  ******************************************************************************
  * @file           : ydlidar_x2.h
  * @brief          : YDLIDAR X2 driver using circular UART DMA reception.
  ******************************************************************************
  */

#ifndef BSP_YDLIDAR_X2_H_
#define BSP_YDLIDAR_X2_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stddef.h>

#define YDLIDAR_X2_BAUDRATE             115200
#define YDLIDAR_X2_DMA_BUF_SIZE         1024
#define YDLIDAR_X2_DEG_COUNT            360
#define YDLIDAR_X2_MAX_PACKET_SIZE      140

#define YDLIDAR_X2_HEADER_LSB           0xAA
#define YDLIDAR_X2_HEADER_MSB           0x55

typedef enum {
    YDLIDAR_X2_STATE_WAIT_PH = 0,         /* Waiting for packet header (0x55AA) */
    YDLIDAR_X2_STATE_READ_HEADER_INFO,    /* Reading header descriptor fields */
    YDLIDAR_X2_STATE_READ_SAMPLE_DATA     /* Reading distance samples */
} ydlidar_x2_parser_state_t;

typedef struct {
    UART_HandleTypeDef         *huart;
    uint8_t                    dma_buffer[YDLIDAR_X2_DMA_BUF_SIZE];
    uint16_t                   last_dma_head;
    
    /* Parser state machine */
    ydlidar_x2_parser_state_t  parser_state;
    uint8_t                    packet_buf[YDLIDAR_X2_MAX_PACKET_SIZE];
    uint16_t                   packet_idx;
    uint16_t                   expected_packet_len;
    uint8_t             last_byte;

    /* Point cloud data (360 degrees, 0 to 359) */
    uint16_t            distances[YDLIDAR_X2_DEG_COUNT]; /* Distance in mm (0 = invalid/blind zone) */
    
    /* Diagnostics and Telemetry */
    uint32_t            valid_packets_count;
    uint32_t            checksum_errors_count;
    uint32_t            laps_count;
    float               scan_frequency_hz;
    uint8_t             new_scan_ready;
} ydlidar_x2_t;

/**
  * @brief  Initializes the YDLIDAR X2 driver and starts circular DMA reception.
  * @param  lidar: Pointer to ydlidar_x2_t structure.
  * @param  huart: Pointer to UART handle (configured at 115200 bauds).
  */
void YDLIDAR_X2_Init(ydlidar_x2_t *lidar, UART_HandleTypeDef *huart);

/**
  * @brief  Non-blocking function to process new bytes from circular DMA buffer.
  *         Call this periodically in your main loop or dedicated RTOS task.
  * @param  lidar: Pointer to ydlidar_x2_t structure.
  */
void YDLIDAR_X2_Process(ydlidar_x2_t *lidar);

/**
  * @brief  Returns measured distance at a given angle (0 to 359 deg).
  * @param  lidar: Pointer to ydlidar_x2_t structure.
  * @param  angle_deg: Angle in degrees [0, 359].
  * @retval Distance in mm (0 if no detection / out of range).
  */
uint16_t YDLIDAR_X2_GetDistance(const ydlidar_x2_t *lidar, uint16_t angle_deg);

/**
  * @brief  Checks whether a new full 360 degree scan has completed.
  * @param  lidar: Pointer to ydlidar_x2_t structure.
  * @retval 1 if new scan is ready, 0 otherwise. Clears the flag automatically.
  */
uint8_t YDLIDAR_X2_IsNewScanReady(ydlidar_x2_t *lidar);

/**
  * @brief  Clears the distance table (fills with 0).
  * @param  lidar: Pointer to ydlidar_x2_t structure.
  */
void YDLIDAR_X2_ClearDistances(ydlidar_x2_t *lidar);

#ifdef __cplusplus
}
#endif

#endif /* BSP_YDLIDAR_X2_H_ */

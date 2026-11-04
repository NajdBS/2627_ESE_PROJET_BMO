/**
  ******************************************************************************
  * @file           : ydlidar_x2.c
  * @brief          : High-performance non-blocking driver for YDLIDAR X2 sensor.
  ******************************************************************************
  */

#include "ydlidar_x2.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static void YDLIDAR_X2_StartDMA(ydlidar_x2_t *lidar);
static void YDLIDAR_X2_DecodePacket(ydlidar_x2_t *lidar, const uint8_t *packet, uint16_t packet_len);

static void YDLIDAR_X2_StartDMA(ydlidar_x2_t *lidar)
{
    UART_HandleTypeDef *huart = lidar->huart;

    /* Clear sticky UART error flags before starting */
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    HAL_UART_Receive_DMA(huart, lidar->dma_buffer, YDLIDAR_X2_DMA_BUF_SIZE);

    /* Prevent HAL_UART_IRQHandler from aborting circular DMA on single-byte noise;
     * corrupted frames are filtered by the 16-bit XOR checksum instead.
     * Also mask unused DMA HT/TC interrupts since NDTR is polled at 100 Hz. */
    __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
    __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
    if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    }

    lidar->last_rx_ms = HAL_GetTick();
}

void YDLIDAR_X2_Init(ydlidar_x2_t *lidar, UART_HandleTypeDef *huart)
{
    if (lidar == NULL || huart == NULL) {
        return;
    }

    memset(lidar, 0, sizeof(ydlidar_x2_t));
    lidar->huart = huart;
    lidar->parser_state = YDLIDAR_X2_STATE_WAIT_PH;

    YDLIDAR_X2_StartDMA(lidar);
}

void YDLIDAR_X2_Restart(ydlidar_x2_t *lidar)
{
    if (lidar == NULL || lidar->huart == NULL) {
        return;
    }

    HAL_UART_AbortReceive(lidar->huart);

    /* Reset parser and clear stale point cloud (no ghost obstacles) */
    lidar->last_dma_head = 0;
    lidar->parser_state = YDLIDAR_X2_STATE_WAIT_PH;
    lidar->packet_idx = 0;
    lidar->last_byte = 0;
    memset(lidar->distances, 0, sizeof(lidar->distances));
    memset(lidar->last_lap_updated, 0, sizeof(lidar->last_lap_updated));

    YDLIDAR_X2_StartDMA(lidar);
    lidar->restart_count++;
}

void YDLIDAR_X2_Process(ydlidar_x2_t *lidar)
{
    if (lidar == NULL || lidar->huart == NULL || lidar->huart->hdmarx == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();

    /* Clear any transient UART error flags (e.g. ORE after debugger halt) so RX keeps flowing */
    __HAL_UART_CLEAR_FLAG(lidar->huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    /* Safety-net RX watchdog: restart DMA if sensor was unplugged or stalled > 500 ms */
    if (lidar->huart->RxState != HAL_UART_STATE_BUSY_RX ||
        (now - lidar->last_rx_ms) > YDLIDAR_X2_RX_TIMEOUT_MS) {
        YDLIDAR_X2_Restart(lidar);
        return;
    }

    /* Check current write position of circular DMA */
    uint16_t dma_cnt = __HAL_DMA_GET_COUNTER(lidar->huart->hdmarx);
    uint16_t head = YDLIDAR_X2_DMA_BUF_SIZE - dma_cnt;
    if (head >= YDLIDAR_X2_DMA_BUF_SIZE) {
        head = 0;
    }

    if (head == lidar->last_dma_head) {
        return; /* No new bytes received */
    }
    lidar->last_rx_ms = now;

    uint16_t bytes_to_read = 0;
    if (head > lidar->last_dma_head) {
        bytes_to_read = head - lidar->last_dma_head;
    } else {
        bytes_to_read = (YDLIDAR_X2_DMA_BUF_SIZE - lidar->last_dma_head) + head;
    }

    for (uint16_t b = 0; b < bytes_to_read; ++b) {
        uint8_t byte = lidar->dma_buffer[lidar->last_dma_head];
        lidar->last_dma_head = (lidar->last_dma_head + 1) % YDLIDAR_X2_DMA_BUF_SIZE;

        switch (lidar->parser_state) {
            case YDLIDAR_X2_STATE_WAIT_PH:
                if (lidar->last_byte == YDLIDAR_X2_HEADER_LSB && byte == YDLIDAR_X2_HEADER_MSB) {
                    lidar->packet_buf[0] = YDLIDAR_X2_HEADER_LSB;
                    lidar->packet_buf[1] = YDLIDAR_X2_HEADER_MSB;
                    lidar->packet_idx = 2;
                    lidar->parser_state = YDLIDAR_X2_STATE_READ_HEADER_INFO;
                }
                break;

            case YDLIDAR_X2_STATE_READ_HEADER_INFO:
                lidar->packet_buf[lidar->packet_idx++] = byte;
                /* Header has 10 bytes total: PH(2) + CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2) */
                if (lidar->packet_idx >= 10) {
                    uint8_t lsn = lidar->packet_buf[3];
                    lidar->expected_packet_len = 10 + ((uint16_t)lsn * 2);

                    if (lidar->expected_packet_len > YDLIDAR_X2_MAX_PACKET_SIZE) {
                        /* Packet size invalid or out of specification: reset */
                        lidar->parser_state = YDLIDAR_X2_STATE_WAIT_PH;
                        lidar->packet_idx = 0;
                        break;
                    }

                    if (lsn == 0) {
                        /* No samples, decode immediately */
                        YDLIDAR_X2_DecodePacket(lidar, lidar->packet_buf, lidar->expected_packet_len);
                        lidar->parser_state = YDLIDAR_X2_STATE_WAIT_PH;
                        lidar->packet_idx = 0;
                    } else {
                        lidar->parser_state = YDLIDAR_X2_STATE_READ_SAMPLE_DATA;
                    }
                }
                break;

            case YDLIDAR_X2_STATE_READ_SAMPLE_DATA:
                lidar->packet_buf[lidar->packet_idx++] = byte;
                if (lidar->packet_idx >= lidar->expected_packet_len) {
                    YDLIDAR_X2_DecodePacket(lidar, lidar->packet_buf, lidar->expected_packet_len);
                    lidar->parser_state = YDLIDAR_X2_STATE_WAIT_PH;
                    lidar->packet_idx = 0;
                }
                break;
        }

        lidar->last_byte = byte;
    }
}

static void YDLIDAR_X2_DecodePacket(ydlidar_x2_t *lidar, const uint8_t *packet, uint16_t packet_len)
{
    if (packet_len < 10) {
        return;
    }

    /* 1. Checksum Validation (16-bit XOR sum) */
    uint16_t calc_cs = 0;
    for (uint16_t i = 0; i < 8; i += 2) {
        calc_cs ^= (uint16_t)(packet[i] | (packet[i + 1] << 8));
    }
    for (uint16_t i = 10; i < packet_len; i += 2) {
        calc_cs ^= (uint16_t)(packet[i] | (packet[i + 1] << 8));
    }

    uint16_t recv_cs = (uint16_t)(packet[8] | (packet[9] << 8));
    if (calc_cs != recv_cs) {
        lidar->checksum_errors_count++;
        return;
    }

    lidar->valid_packets_count++;

    /* 2. Packet Metadata */
    uint8_t ct = packet[2];
    uint8_t lsn = packet[3];

    /* CT bit 0 indicates start of a 360-degree rotation (lap) */
    if (ct & 0x01) {
        lidar->laps_count++;
        lidar->new_scan_ready = 1;
        uint8_t freq_code = (ct >> 1);
        if (freq_code > 0) {
            lidar->scan_frequency_hz = (float)freq_code / 10.0f;
        }

        /* Aging mechanism: clear any angle not refreshed during the last 2 laps */
        uint16_t cur_lap = (uint16_t)lidar->laps_count;
        for (int deg = 0; deg < YDLIDAR_X2_DEG_COUNT; deg++) {
            if ((uint16_t)(cur_lap - lidar->last_lap_updated[deg]) > 1) {
                lidar->distances[deg] = 0;
            }
        }
    }

    if (lsn == 0) {
        return;
    }

    /* 3. Angles Extraction */
    uint16_t fsa_raw = (uint16_t)(packet[4] | (packet[5] << 8));
    uint16_t lsa_raw = (uint16_t)(packet[6] | (packet[7] << 8));

    float angle_fsa = ((float)(fsa_raw >> 1)) / 64.0f;
    float angle_lsa = ((float)(lsa_raw >> 1)) / 64.0f;

    if (angle_fsa >= 360.0f) angle_fsa -= 360.0f;
    if (angle_lsa >= 360.0f) angle_lsa -= 360.0f;

    /* 4. Angular Interpolation and Distance Extraction */
    float step_angle = 0.0f;
    if (lsn > 1) {
        float diff_angle = angle_lsa - angle_fsa;
        if (diff_angle < 0.0f) {
            diff_angle += 360.0f;
        }
        step_angle = diff_angle / (float)(lsn - 1);
    }

    for (uint8_t i = 0; i < lsn; ++i) {
        uint16_t sample_raw = (uint16_t)(packet[10 + (i * 2)] | (packet[10 + (i * 2) + 1] << 8));
        float distance_mm = (float)sample_raw / 4.0f;

        float cur_angle = angle_fsa + ((float)i * step_angle);

        if (distance_mm > 0.0f) {
            /* Optical geometric correction */
            float ang_correct = atanf(21.8f * (155.3f - distance_mm) / (155.3f * distance_mm)) * (180.0f / (float)M_PI);
            cur_angle += ang_correct;
        }
        cur_angle += (float)lidar->angle_offset_deg;

        while (cur_angle >= 360.0f) cur_angle -= 360.0f;
        while (cur_angle < 0.0f) cur_angle += 360.0f;

        int deg_index = (int)(cur_angle + 0.5f);
        if (deg_index >= YDLIDAR_X2_DEG_COUNT) {
            deg_index = 0;
        }

        /* If distance is 0 (no return/absorbed/too far), clear stale sample.
         * Otherwise, record the newly measured distance and mark refreshed lap. */
        if (distance_mm > 0.0f) {
            lidar->distances[deg_index] = (uint16_t)distance_mm;
            lidar->last_lap_updated[deg_index] = (uint16_t)lidar->laps_count;
        } else {
            lidar->distances[deg_index] = 0;
        }
    }
}

uint16_t YDLIDAR_X2_GetDistance(const ydlidar_x2_t *lidar, uint16_t angle_deg)
{
    if (lidar == NULL || angle_deg >= YDLIDAR_X2_DEG_COUNT) {
        return 0;
    }
    return lidar->distances[angle_deg];
}

uint8_t YDLIDAR_X2_IsNewScanReady(ydlidar_x2_t *lidar)
{
    if (lidar == NULL) {
        return 0;
    }
    uint8_t ready = lidar->new_scan_ready;
    lidar->new_scan_ready = 0;
    return ready;
}

void YDLIDAR_X2_ClearDistances(ydlidar_x2_t *lidar)
{
    if (lidar != NULL) {
        memset(lidar->distances, 0, sizeof(lidar->distances));
        memset(lidar->last_lap_updated, 0, sizeof(lidar->last_lap_updated));
    }
}

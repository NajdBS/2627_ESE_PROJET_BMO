/**
  ******************************************************************************
  * @file    ble.h
  * @brief   Bluetooth Low Energy (BLE) & UART command driver for BMO Robot
  ******************************************************************************
  */

#ifndef BLE_H
#define BLE_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>


#define BLE_RX_BUFFER_SIZE  64
#define BLE_CMD_MAX_LEN     64

/**
  * @brief High-level command parsed from BLE / UART stream
  */
typedef enum {
    BLE_CMD_NONE = 0,
    BLE_CMD_START,      /* "START" or "S" */
    BLE_CMD_STOP,       /* "STOP", "HALT", "IDLE", or "H" */
    BLE_CMD_RESET,      /* "RESET" or "RST" */
    BLE_CMD_MAP,        /* "MAP" or "M" (force persistent 2D LiDAR map view) */
    BLE_CMD_MANUAL_FWD, /* "FWD" or "F" */
    BLE_CMD_MANUAL_BWD, /* "BWD" or "B" */
    BLE_CMD_MANUAL_LFT, /* "LFT" or "L" */
    BLE_CMD_MANUAL_RGT, /* "RGT" or "R" */
    BLE_CMD_CUSTOM      /* Unrecognized command string */
} ble_command_type_t;

/**
  * @brief BLE / UART Command Device Handle
  */
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *en_port;
    uint16_t            en_pin;

    uint8_t             rx_byte;
    char                rx_line[BLE_RX_BUFFER_SIZE];
    uint8_t             rx_index;

    char                last_command[BLE_CMD_MAX_LEN];
    ble_command_type_t  last_command_type;
    volatile bool       command_ready;

    bool                enabled;
    bool                initialized;
} ble_t;

HAL_StatusTypeDef BLE_Init(ble_t *dev, UART_HandleTypeDef *huart, GPIO_TypeDef *en_port, uint16_t en_pin);
HAL_StatusTypeDef BLE_SetEnabled(ble_t *dev, bool enable);
bool BLE_HasCommand(ble_t *dev);
ble_command_type_t BLE_GetCommand(ble_t *dev, char *out_cmd, uint16_t max_len);
HAL_StatusTypeDef BLE_SendString(ble_t *dev, const char *str);
HAL_StatusTypeDef BLE_SendPrintf(ble_t *dev, const char *fmt, ...);
void BLE_RxCallback(ble_t *dev);
void BLE_ErrorCallback(ble_t *dev);

#endif /* BLE_H */

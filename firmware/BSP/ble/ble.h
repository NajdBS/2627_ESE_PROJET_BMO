/**
  ******************************************************************************
  * @file    ble.h
  * @brief   Bluetooth Low Energy (BLE) module driver for BMO Robot
  ******************************************************************************
  */

#ifndef BLE_H
#define BLE_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_RX_BUFFER_SIZE  64
#define BLE_CMD_MAX_LEN     64

/**
  * @brief High-level command parsed from BLE stream
  */
typedef enum {
    BLE_CMD_NONE = 0,
    BLE_CMD_START,      /* "START" or 'S' */
    BLE_CMD_STOP,       /* "STOP" or 'H' (Halt) */
    BLE_CMD_RESET,      /* "RESET" or 'R' */
    BLE_CMD_MANUAL_FWD, /* 'F' */
    BLE_CMD_MANUAL_BWD, /* 'B' */
    BLE_CMD_MANUAL_LFT, /* 'L' */
    BLE_CMD_MANUAL_RGT, /* 'R' */
    BLE_CMD_CUSTOM      /* Unrecognized command string */
} ble_command_type_t;

/**
  * @brief BLE Device Structure Handle
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

/**
  * @brief  Initialize the BLE module interface.
  * @param  dev: Pointer to BLE device handle.
  * @param  huart: Pointer to UART handle (USART3).
  * @param  en_port: GPIO port for BLE_EN pin.
  * @param  en_pin: GPIO pin for BLE_EN pin.
  * @retval HAL_OK on success.
  */
HAL_StatusTypeDef BLE_Init(ble_t *dev, UART_HandleTypeDef *huart, GPIO_TypeDef *en_port, uint16_t en_pin);

/**
  * @brief  Set BLE hardware enable state (via BLE_EN pin).
  * @param  dev: Pointer to BLE device handle.
  * @param  enable: true to enable, false to disable.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BLE_SetEnabled(ble_t *dev, bool enable);

/**
  * @brief  Check if a full command line has been received over BLE.
  * @param  dev: Pointer to BLE device handle.
  * @retval true if a command is pending.
  */
bool BLE_HasCommand(ble_t *dev);

/**
  * @brief  Retrieve the last received command and clear the pending flag.
  * @param  dev: Pointer to BLE device handle.
  * @param  out_cmd: Buffer to receive command string (null-terminated).
  * @param  max_len: Size of output buffer.
  * @retval Command type enum.
  */
ble_command_type_t BLE_GetCommand(ble_t *dev, char *out_cmd, uint16_t max_len);

/**
  * @brief  Send a null-terminated string over BLE.
  * @param  dev: Pointer to BLE device handle.
  * @param  str: Null-terminated string to transmit.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BLE_SendString(ble_t *dev, const char *str);

/**
  * @brief  Send formatted telemetry data string over BLE.
  * @param  dev: Pointer to BLE device handle.
  * @param  fmt: Printf-style format string.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BLE_SendPrintf(ble_t *dev, const char *fmt, ...);

/**
  * @brief  UART RX Complete interrupt callback handler.
  *         Must be called from HAL_UART_RxCpltCallback.
  * @param  dev: Pointer to BLE device handle.
  */
void BLE_RxCallback(ble_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* BLE_H */

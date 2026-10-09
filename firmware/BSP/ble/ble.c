/**
  ******************************************************************************
  * @file    ble.c
  * @brief   Implementation of BLE driver for BMO Robot
  ******************************************************************************
  */

#include "ble.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define BLE_TX_TIMEOUT_MS  100

static ble_command_type_t BLE_ClassifyCommand(const char *cmd)
{
    if (!cmd || strlen(cmd) == 0) {
        return BLE_CMD_NONE;
    }

    if (strcmp(cmd, "START") == 0 || strcmp(cmd, "S") == 0) {
        return BLE_CMD_START;
    }
    if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "H") == 0 || strcmp(cmd, "HALT") == 0) {
        return BLE_CMD_STOP;
    }
    if (strcmp(cmd, "RESET") == 0 || strcmp(cmd, "R") == 0) {
        return BLE_CMD_RESET;
    }
    if (strcmp(cmd, "F") == 0 || strcmp(cmd, "FWD") == 0) {
        return BLE_CMD_MANUAL_FWD;
    }
    if (strcmp(cmd, "B") == 0 || strcmp(cmd, "BWD") == 0) {
        return BLE_CMD_MANUAL_BWD;
    }
    if (strcmp(cmd, "L") == 0 || strcmp(cmd, "LFT") == 0) {
        return BLE_CMD_MANUAL_LFT;
    }
    if (strcmp(cmd, "R") == 0 || strcmp(cmd, "RGT") == 0) {
        return BLE_CMD_MANUAL_RGT;
    }

    return BLE_CMD_CUSTOM;
}

HAL_StatusTypeDef BLE_Init(ble_t *dev, UART_HandleTypeDef *huart, GPIO_TypeDef *en_port, uint16_t en_pin)
{
    if (!dev || !huart) {
        return HAL_ERROR;
    }

    memset(dev, 0, sizeof(ble_t));
    dev->huart   = huart;
    dev->en_port = en_port;
    dev->en_pin  = en_pin;

    /* Assert enable pin high by default */
    BLE_SetEnabled(dev, true);

    dev->initialized = true;

    /* Start non-blocking 1-byte reception in interrupt mode */
    return HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

HAL_StatusTypeDef BLE_SetEnabled(ble_t *dev, bool enable)
{
    if (!dev) {
        return HAL_ERROR;
    }

    dev->enabled = enable;
    if (dev->en_port) {
        HAL_GPIO_WritePin(dev->en_port, dev->en_pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    return HAL_OK;
}

bool BLE_HasCommand(ble_t *dev)
{
    return (dev != NULL) && dev->command_ready;
}

ble_command_type_t BLE_GetCommand(ble_t *dev, char *out_cmd, uint16_t max_len)
{
    if (!dev || !dev->command_ready) {
        return BLE_CMD_NONE;
    }

    if (out_cmd && max_len > 0) {
        strncpy(out_cmd, dev->last_command, max_len - 1);
        out_cmd[max_len - 1] = '\0';
    }

    dev->command_ready = false;
    return dev->last_command_type;
}

HAL_StatusTypeDef BLE_SendString(ble_t *dev, const char *str)
{
    if (!dev || !dev->huart || !str) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(dev->huart, (const uint8_t *)str, (uint16_t)strlen(str), BLE_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef BLE_SendPrintf(ble_t *dev, const char *fmt, ...)
{
    if (!dev || !dev->huart || !fmt) {
        return HAL_ERROR;
    }

    char buffer[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len <= 0) {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(dev->huart, (const uint8_t *)buffer, (uint16_t)len, BLE_TX_TIMEOUT_MS);
}

void BLE_RxCallback(ble_t *dev)
{
    if (!dev || !dev->huart) {
        return;
    }

    uint8_t byte = dev->rx_byte;

    if (byte == '\r' || byte == '\n') {
        if (dev->rx_index > 0) {
            dev->rx_line[dev->rx_index] = '\0';
            strncpy(dev->last_command, dev->rx_line, sizeof(dev->last_command) - 1);
            dev->last_command[sizeof(dev->last_command) - 1] = '\0';
            dev->last_command_type = BLE_ClassifyCommand(dev->last_command);
            dev->command_ready = true;
            dev->rx_index = 0;
        }
    } else if (dev->rx_index < (sizeof(dev->rx_line) - 1)) {
        dev->rx_line[dev->rx_index++] = (char)byte;
    }

    /* Re-arm interrupt for next byte */
    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

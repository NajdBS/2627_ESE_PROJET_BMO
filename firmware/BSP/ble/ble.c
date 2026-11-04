/**
  ******************************************************************************
  * @file    ble.c
  * @brief   Implementation of BLE & UART command parser for BMO Robot
  ******************************************************************************
  */

#include "ble.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define BLE_TX_TIMEOUT_MS  100

static ble_command_type_t BLE_ClassifyCommand(const char *cmd)
{
    if (!cmd) {
        return BLE_CMD_NONE;
    }

    /* Trim leading spaces and convert to uppercase for case-insensitive matching */
    while (*cmd == ' ' || *cmd == '\t') {
        cmd++;
    }

    char norm[BLE_CMD_MAX_LEN];
    size_t j = 0;
    for (size_t i = 0; cmd[i] != '\0' && j < (sizeof(norm) - 1); i++) {
        char c = cmd[i];
        if (c >= 'a' && c <= 'z') {
            c = (char)(c - ('a' - 'A'));
        }
        norm[j++] = c;
    }
    while (j > 0 && (norm[j - 1] == ' ' || norm[j - 1] == '\t')) {
        j--;
    }
    norm[j] = '\0';

    if (j == 0) {
        return BLE_CMD_NONE;
    }

    if (strcmp(norm, "START") == 0 || strcmp(norm, "S") == 0) {
        return BLE_CMD_START;
    }
    if (strcmp(norm, "STOP") == 0 || strcmp(norm, "H") == 0 ||
        strcmp(norm, "HALT") == 0 || strcmp(norm, "IDLE") == 0) {
        return BLE_CMD_STOP;
    }
    if (strcmp(norm, "RESET") == 0 || strcmp(norm, "RST") == 0) {
        return BLE_CMD_RESET;
    }
    if (strcmp(norm, "MAP") == 0 || strcmp(norm, "M") == 0) {
        return BLE_CMD_MAP;
    }
    if (strcmp(norm, "F") == 0 || strcmp(norm, "FWD") == 0) {
        return BLE_CMD_MANUAL_FWD;
    }
    if (strcmp(norm, "B") == 0 || strcmp(norm, "BWD") == 0) {
        return BLE_CMD_MANUAL_BWD;
    }
    if (strcmp(norm, "L") == 0 || strcmp(norm, "LFT") == 0) {
        return BLE_CMD_MANUAL_LFT;
    }
    if (strcmp(norm, "R") == 0 || strcmp(norm, "RGT") == 0) {
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

    /* Assert enable pin high by default (no-op if en_port is NULL) */
    BLE_SetEnabled(dev, true);

    dev->initialized = true;

    /* Clear sticky error flags and start non-blocking 1-byte interrupt RX */
    __HAL_UART_CLEAR_FLAG(dev->huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
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
    } else if (byte == 0x08 || byte == 0x7F) {
        /* Handle interactive terminal Backspace / Delete */
        if (dev->rx_index > 0) {
            dev->rx_index--;
        }
    } else if (dev->rx_index < (sizeof(dev->rx_line) - 1)) {
        dev->rx_line[dev->rx_index++] = (char)byte;
    }

    /* Re-arm interrupt for next byte */
    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

void BLE_ErrorCallback(ble_t *dev)
{
    if (!dev || !dev->huart) {
        return;
    }

    /* Clear sticky UART error flags (ORE/FE/NE) and re-arm 1-byte interrupt RX */
    __HAL_UART_CLEAR_FLAG(dev->huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    dev->rx_index = 0;
    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

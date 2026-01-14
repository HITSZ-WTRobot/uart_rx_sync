/**
 * @file    uart_rx_sync.h
 * @author  syhanjin
 * @date    2026-01-14
 * @brief   带帧头同步功能的串口接收库
 */
#ifndef UART_RX_SYNC_H
#define UART_RX_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef UART_RX_SYNC_MAX_HDR
#    define UART_RX_SYNC_MAX_HDR (4)
#endif

/**
 * 数据解析回调
 * @note data 不包括帧头
 * @return 解析是否成功
 */
typedef bool (*UartRxSync_DecodeData_Callback)(void* user, const uint8_t* data);

typedef enum
{
    UART_RX_SYNC_WAIT_HEAD = 0U,
    UART_RX_SYNC_RECEIVING,
    UART_RX_SYNC_DMA_ACTIVE,
} UartRxSync_SyncState_t;

typedef struct
{
    UART_HandleTypeDef* huart;

    UartRxSync_SyncState_t sync_state;
    uint8_t                hdr[UART_RX_SYNC_MAX_HDR];
    size_t                 hdr_len;
    size_t                 hdr_idx;
    size_t                 frame_len;
    uint8_t*               buffer;

    void* user;

    UartRxSync_DecodeData_Callback decode;

#ifdef DEBUG
    uint32_t hdr_match_cnt;
    uint32_t hdr_error_cnt;
    uint32_t data_received_cnt;
    uint32_t decode_success_cnt;
    uint32_t decode_fail_cnt;
    uint32_t rx_error_event_cnt;
#endif
} UartRxSync_t;

typedef struct
{
    UART_HandleTypeDef* huart;
    uint8_t*            buffer;

    struct
    {
        size_t         len;
        const uint8_t* content;
    } header;

    size_t frame_len;

    void* user;

    UartRxSync_DecodeData_Callback decode_data_callback;
} UartRxSync_Config_t;

void UartRxSync_Init(UartRxSync_t* sync, const UartRxSync_Config_t* config);
void UartRxSync_RxErrorHandler(UartRxSync_t* sync);
void UartRxSync_RxCallback(UartRxSync_t* sync);

static bool UartRxSync_isConnected(UartRxSync_t* sync)
{
    return sync->sync_state == UART_RX_SYNC_DMA_ACTIVE;
}

#ifdef __cplusplus
}
#endif

#endif // UART_RX_SYNC_H

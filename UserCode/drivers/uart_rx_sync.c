/**
 * @file    uart_rx_sync.c
 * @author  syhanjin
 * @date    2026-01-14
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "uart_rx_sync.h"

#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef DEBUG
#    define HDR_MATCH(__SYNC__)      ((__SYNC__)->hdr_match_cnt++)
#    define HDR_ERROR(__SYNC__)      ((__SYNC__)->hdr_error_cnt++)
#    define RECEIVED(__SYNC__)       ((__SYNC__)->data_received_cnt++)
#    define DECODE_SUCCESS(__SYNC__) ((__SYNC__)->decode_success_cnt++)
#    define DECODE_FAILED(__SYNC__)  ((__SYNC__)->decode_fail_cnt++)
#    define RX_ERROR_EVENT(__SYNC__) ((__SYNC__)->rx_error_event_cnt++)
#else
#    define HDR_MATCH(__SYNC__)      ((void*) (__SYNC__))
#    define HDR_ERROR(__SYNC__)      ((void*) (__SYNC__))
#    define RECEIVED(__SYNC__)       ((void*) (__SYNC__))
#    define DECODE_SUCCESS(__SYNC__) ((void*) (__SYNC__))
#    define DECODE_FAILED(__SYNC__)  ((void*) (__SYNC__))
#    define RX_ERROR_EVENT(__SYNC__) ((void*) (__SYNC__))
#endif

void UartRxSync_Init(UartRxSync_t* sync, const UartRxSync_Config_t* config)
{
    assert_param(config->huart != NULL);
    assert_param(config->header.len > 0);
    assert_param(config->frame_len > config->header.len);
    assert_param(config->header.len <= UART_RX_SYNC_MAX_HDR);
    assert_param(config->buffer != NULL);
    assert_param(config->user != NULL);
    assert_param(config->decode_data_callback != NULL);
    // 必须开启 RX DMA 循环模式
    assert_param(config->huart->hdmarx != NULL);
    assert_param(config->huart->hdmarx->Init.Mode == DMA_CIRCULAR);

    sync->huart = config->huart;

    sync->hdr_len = config->header.len;
    memcpy(sync->hdr, config->header.content, config->header.len);

    sync->frame_len = config->frame_len;

    sync->buffer = config->buffer;

    sync->user   = config->user;
    sync->decode = config->decode_data_callback;

    sync->hdr_idx    = 0;
    sync->sync_state = UART_RX_SYNC_WAIT_HEAD;

    HAL_UART_Receive_IT(sync->huart, sync->buffer, 1);
}
/**
 * 从环形缓冲区检查 header
 * @param hdr * @param buffer * @param idx * @param len
 * @return
 */
static bool check_header(const uint8_t* hdr,
                         const uint8_t* buffer,
                         const size_t   idx,
                         const size_t   len)
{
    // const uint8_t* ptr = buffer + idx;
    // for (size_t i = idx; i < len; i++)
    //     if (*ptr++ != *hdr++)
    //         return false;
    // ptr = buffer;
    // for (size_t i = 0; i < idx; i++)
    //     if (*ptr++ != *hdr++)
    //         return false;
    // return true;
    for (const uint8_t *ptr = buffer + idx, *end = buffer + len; ptr != end; ptr++)
        if (*ptr != *hdr++)
            return false;
    for (const uint8_t *ptr = buffer, *end = buffer + idx; ptr != end; ptr++)
        if (*ptr != *hdr++)
            return false;
    return true;
}

static void decode(UartRxSync_t* sync)
{
    if (!sync->decode(sync->user, sync->buffer + sync->hdr_len))
    {
        DECODE_FAILED(sync);
        // 此处无须处理，由用户自行丢弃该帧即可
    }
    else
    {
        DECODE_SUCCESS(sync);
    }
}

/**
 * 接收错误事件处理函数
 * @param sync
 */
void UartRxSync_RxErrorHandler(UartRxSync_t* sync)
{
    if (sync->huart->ErrorCode == HAL_UART_ERROR_NONE)
    {
        // not a real uart error
        return;
    }
    RX_ERROR_EVENT(sync);

    // clear error flags
    __HAL_UART_CLEAR_PEFLAG(sync->huart);
    __HAL_UART_CLEAR_FEFLAG(sync->huart);
    __HAL_UART_CLEAR_NEFLAG(sync->huart);
    __HAL_UART_CLEAR_OREFLAG(sync->huart);

    // restart receive
    HAL_UART_AbortReceive(sync->huart);
    if (sync->sync_state != UART_RX_SYNC_WAIT_HEAD)
    {
        sync->sync_state = UART_RX_SYNC_WAIT_HEAD;
    }
    HAL_UART_Receive_IT(sync->huart, sync->buffer, 1);
    sync->hdr_idx = 0;
}

/**
 * 接收完成回调函数
 * @param sync
 */
void UartRxSync_RxCallback(UartRxSync_t* sync)
{
    if (sync->sync_state == UART_RX_SYNC_DMA_ACTIVE)
    {
        RECEIVED(sync);
        if (!check_header(sync->hdr, sync->buffer, 0, sync->hdr_len))
        {
            // 帧头错误，重新匹配
            HDR_ERROR(sync);
            HAL_UART_AbortReceive(sync->huart);
            sync->sync_state = UART_RX_SYNC_WAIT_HEAD;
            HAL_UART_Receive_IT(sync->huart, sync->buffer, 1);
            sync->hdr_idx = 0;
            return;
        }
        decode(sync);
    }
    else if (sync->sync_state == UART_RX_SYNC_WAIT_HEAD)
    {
        // 匹配到帧头最后一位就往前进行一次 check
        size_t idx_next = sync->hdr_idx + 1;
        if (idx_next == sync->hdr_len)
            idx_next = 0;
        if (sync->buffer[sync->hdr_idx] == sync->hdr[sync->hdr_len - 1])
        {
            if (check_header(sync->hdr, sync->buffer, idx_next, sync->hdr_len))
            {
                HDR_MATCH(sync);
                // 使用 DMA 接收完剩下的内容
                HAL_UART_Receive_DMA(sync->huart,
                                     sync->buffer + sync->hdr_len,
                                     sync->frame_len - sync->hdr_len);
                sync->sync_state = UART_RX_SYNC_RECEIVING;
                return;
            }
        }
        // 继续接收下一位
        HAL_UART_Receive_IT(sync->huart, sync->buffer + idx_next, 1);
        sync->hdr_idx = idx_next;
    }
    else if (sync->sync_state == UART_RX_SYNC_RECEIVING)
    {
        RECEIVED(sync);
        HAL_UART_AbortReceive(sync->huart);
        /**
         * 由于 decode 抛弃了 head，所以这里可以先开始接收
         * 只要保证 decode 时间 < 1 / bitrate * 10 * header_len 即可
         * 对于 115200 bitrate，1 byte 需要约 86us
         * 对于 2M bitrate, 1 byte 需要约 5us
         * 24 字节的查表法 CRC8 需要约 1.4us，逐位计算需要约 2.3us
         *
         * 其实建议先解算再继续接收( 但是我是犟种
         */
        HAL_UART_Receive_DMA(sync->huart, sync->buffer, sync->frame_len);
        sync->sync_state = UART_RX_SYNC_DMA_ACTIVE;
        decode(sync);
    }
}

#ifdef __cplusplus
}
#endif
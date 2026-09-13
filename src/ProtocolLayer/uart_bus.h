/**
 * @file uart_bus.h
 * @brief usart1 串口总线：统一持有“唯一”的异步回调，并管理 DMA 收发
 *
 * 背景：Zephyr 的异步 UART API 每个实例只有一个事件回调槽位
 * （uart_callback_set() 存的 data->async_cb），RX/TX 的所有事件都从这
 * 一个函数进来，用 evt->type 区分。所以不要在别的模块里再调用
 * uart_callback_set() —— 那是覆盖而不是叠加，会让本模块收不到事件。
 *
 * 使用方式：
 *   - 接收方：Uart_Bus_Set_Rx_Callback() 注册解析函数；
 *   - 发送方：Uart_Bus_Send() 提交一整帧；
 *   - 初始化：Uart_Bus_Init()，重复调用安全（只初始化一次）。
 */
#ifndef __UART_BUS_H
#define __UART_BUS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 发送缓冲大小。裁判系统 UI 最长帧为 5+2+111+2 = 120 字节 */
#define UART_BUS_TX_BUF_SIZE 128

/* 收到的数据块回调。在 UART 事件回调（中断/回调上下文）里被调用，
 * 实现里不要阻塞、不要太慢。 */
typedef void (*uart_bus_rx_cb_t)(const uint8_t *data, uint32_t len);

/**
 * @brief 注册“收到数据”的回调（owner 分发里的 RX 分支）
 *
 * @param cb 回调函数，传 NULL 表示不关心接收
 */
void Uart_Bus_Set_Rx_Callback(uart_bus_rx_cb_t cb);

/**
 * @brief 初始化 usart1 的异步(DMA)收发：注册唯一回调 + 启动 DMA 接收
 *
 * 内部先 uart_callback_set() 再 uart_rx_enable()，重复调用安全。
 *
 * @return 0 成功，负值为 errno
 */
int Uart_Bus_Init(void);

/**
 * @brief 异步(DMA)发送一整帧
 *
 * 内部有一块专属发送缓冲：先把 buf 拷进去再交给 DMA，所以调用者返回后
 * 可以立刻改写自己的 buf；上一帧没发完时也不会去碰那块内存。
 *
 * 只允许在任务上下文调用（不要放在中断里）。
 *
 * @param buf 待发送数据（函数内会拷贝）
 * @param len 长度，不得超过 UART_BUS_TX_BUF_SIZE
 * @return 0=已提交(HAL_OK)，2=忙(HAL_BUSY，下一周期重试)，1=错误(HAL_ERROR)
 */
uint8_t Uart_Bus_Send(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __UART_BUS_H */

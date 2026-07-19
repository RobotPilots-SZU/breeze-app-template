#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <stdint.h>

// 喵板 STM32H723
#include <zephyr/linker/section_tags.h>   // 提供 __nocache


/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_vofa_uart)
static const struct device *uart = DEVICE_DT_GET(UART_DEVICE_NODE);

LOG_MODULE_REGISTER(vofa_uart, LOG_LEVEL_DBG);


/* ------------------------------ 发送内存池和队列 -------------------------------------- */

// C板 STM32F40
// /* 最多排队 8 个数据包，每个包最大 32 字节（根据需要调整） */
// NET_BUF_POOL_DEFINE(tx_pool, 32, 64, 0, NULL);

// 喵板 STM32H723
/* ---------- 不可缓存内存池 ---------- */
#define TX_POOL_COUNT 32
#define TX_POOL_SIZE  64
#define TX_USER_DATA  0

/* ---------- 手动模拟 NET_BUF_POOL_FIXED_DEFINE ---------- */

/* 1. 控制块数组（普通内存） */
struct _net_buf_tx_pool {
    uint8_t b[sizeof(struct net_buf)];
    uint8_t ud[TX_USER_DATA];
} __net_buf_align;

static struct _net_buf_tx_pool _net_buf_tx_pool[TX_POOL_COUNT] __noinit;

/* 2. 数据数组（不可缓存内存） */
static uint8_t __nocache __aligned(4) net_buf_data_tx_pool[TX_POOL_COUNT][TX_POOL_SIZE] __net_buf_align;

/* 3. 固定池描述 */
static const struct net_buf_pool_fixed net_buf_fixed_tx_pool = {
    .data_pool = (uint8_t *)net_buf_data_tx_pool,
};

/* 4. 分配器 */
static const struct net_buf_data_alloc net_buf_fixed_alloc_tx_pool = {
    .cb = &net_buf_fixed_cb,
    .alloc_data = (void *)&net_buf_fixed_tx_pool,
    .max_alloc_size = TX_POOL_SIZE,
};

/* 5. 注册池 */
static STRUCT_SECTION_ITERABLE(net_buf_pool, tx_pool) =
    NET_BUF_POOL_INITIALIZER(tx_pool, &net_buf_fixed_alloc_tx_pool,
                             _net_buf_tx_pool, TX_POOL_COUNT, TX_USER_DATA, NULL);
/* -------------------------------------------------------------------------------------- */


/* 发送队列，存放排队等待的数据包 */
static K_FIFO_DEFINE(tx_fifo);

/* 当前正在通过 DMA 发送的数据包 */
static struct net_buf *tx_pending;

/* ---------- 异步发送完成回调 ---------- */
static void uart_callback(const struct device *dev, struct uart_event *evt,
                          void *user_data)
{   
    switch (evt->type) 
    {
        case UART_TX_DONE:
            LOG_DBG("TX complete %p", tx_pending);

            /* 释放已完成发送的包 */
            net_buf_unref(tx_pending);
            tx_pending = NULL;

            /* 检查队列中是否有下一个包，有则立即启动发送 */
            {
                struct net_buf *buf = static_cast<struct net_buf *>(k_fifo_get(&tx_fifo, K_NO_WAIT));
                if (buf != NULL) {
                    int key = irq_lock();
                    int ret = uart_tx(dev, buf->data, buf->len, SYS_FOREVER_US);
                    if (ret == 0) {
                        tx_pending = buf;
                    }
                    irq_unlock(key);

                    if (ret != 0) {
                        net_buf_unref(buf);   /* 发送失败，丢弃 */
                    }
                }
            }
            break;

        case UART_TX_ABORTED:
            LOG_DBG("TX aborted %p", tx_pending);
            net_buf_unref(tx_pending);
            tx_pending = NULL;
            break;

        default:
            break;
    }
}

/* ---------- VOFA+ 发送接口（非阻塞） ---------- */
void vofa_send_floats(const struct device *dev, const float *data, size_t count)
{
    /* 从内存池申请一个数据包 */
    struct net_buf *buf = net_buf_alloc(&tx_pool, K_NO_WAIT);
    if (!buf) {
        return;   /* 池子满了，直接丢弃，保证实时性 */
    }

    /* 将浮点数据填入包中 */
    net_buf_add_mem(buf, data, count * sizeof(float));

    /* 添加 VOFA+ JustFloat 帧尾（+Inf） */
    static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    net_buf_add_mem(buf, tail, sizeof(tail));

    /*
     * 用中断锁保护 tx_pending 赋值，防止以下竞态条件：
     *
     *   uart_tx() 返回 0 后 DMA 极快完成 → 回调触发
     *   → 回调释放旧的 tx_pending → tx_pending = NULL
     *   → 主线程才执行 tx_pending = buf（此时已无人能释放它）
     *
     * 锁内 uart_tx() + tx_pending 赋值是原子的，回调无法插入。
     */
    int key = irq_lock();

    int ret = uart_tx(dev, buf->data, buf->len, SYS_FOREVER_US);
    if (ret == 0) {
        tx_pending = buf;          /* 在锁内赋值，回调无法插队 */
    }

    irq_unlock(key);

    if (ret == 0)
    {
        LOG_DBG("TX started %p", buf);
    }
    else if (ret == -EBUSY)
    {
        LOG_DBG("UART busy, queuing %p", buf);
        k_fifo_put(&tx_fifo, buf); /* DMA 忙，放入队列等待回调自动发送 */
    }
    else {
        LOG_ERR("UART TX failed with error: %d", ret);
        net_buf_unref(buf);        /* 其他错误，丢弃该包 */
    }
}

/* ---------- 初始化串口 ---------- */
static int uart_init(void)
{
    if (!device_is_ready(uart)) 
    {
        LOG_ERR("UART device not ready %p", uart);
        return -ENODEV;
    }

    /* 注册异步回调 */
    uart_callback_set(uart, uart_callback, NULL);

    return 0;
}


int main(void)
{
    if (uart_init() < 0) 
    {
        return -1;
    }
    LOG_INF("VOFA+ UART sender started\n");
    
    int i = 0;  // 示例数据

    while (1) 
    {
        // 打包一帧 JustFloat 数据：3 个 float（时间戳、数据1、数据2）
        // 一个数据包最多放 15 个数据
        float vofa_data[] = {
            (float)k_uptime_get_32() / 1000.0f, // 时间戳（秒）
            (float)i,                           // 示例值
            (float)(2 * i)                      // 示例值的两倍
        };

        vofa_send_floats(uart, vofa_data, sizeof(vofa_data) / sizeof(float));

        i++;    // 更新示例数据

        k_sleep(K_MSEC(1));
    }

    return 0;
}

/**
 * @file uart_bus.c
 * @brief usart1 串口总线：唯一异步回调 + DMA 收发（RX 双缓冲 / TX 单缓冲）
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include "string.h"
#include "uart_bus.h"

LOG_MODULE_REGISTER(uart_bus, LOG_LEVEL_INF);

#define USART1_NODE DT_NODELABEL(usart1)
static const struct device *const bus_uart = DEVICE_DT_GET(USART1_NODE);

/* ==================== 接收：DMA 双缓冲 ==================== */
/* 本板 DCACHE 关闭，普通 .bss 就能给 DMA 用（stm32_buf_in_nocache() 恒为 true） */
#define RX_DMA_BUF_SIZE 256
#define RX_DMA_BUF_NUM  2
/* 空闲多久上报一次 UART_RX_RDY（微秒）；0 会退化成靠 IDLE 中断上报 */
#define RX_INACTIVITY_TIMEOUT_US 1000

static uint8_t rx_dma_buf[RX_DMA_BUF_NUM][RX_DMA_BUF_SIZE];
/* 还没交给驱动的那一块，在 UART_RX_BUF_REQUEST 里交出去 */
static uint8_t *rx_free_dma_buf = rx_dma_buf[1];

/* ==================== 发送：单缓冲 + 忙标志 ==================== */
/* 驱动同一时间只允许一笔异步发送，所以“一块缓冲 + 忙标志”就够：
 * 忙的时候绝对不写这块内存，避免 DMA 搬运途中被改写。 */
#define TX_BUSY_TIMEOUT_MS 100
static uint8_t tx_buf[UART_BUS_TX_BUF_SIZE];
static volatile bool tx_busy;
static volatile uint32_t tx_submit_ms;

/* 收到数据后交给谁（judge）；只有一个接收方 */
static uart_bus_rx_cb_t rx_callback;

static bool initialized;

static void Uart_Bus_Rx_Restart_Work(struct k_work *work);
static K_WORK_DEFINE(uart_bus_rx_restart_work, Uart_Bus_Rx_Restart_Work);

/**
 * @brief 唯一的 UART 事件回调（一个实例只有一个回调）
 *
 * 在中断/回调上下文运行：这里只做“分发 + 换缓冲”，重活交给 rx_callback。
 */
static void Uart_Bus_Callback(const struct device *dev,
		struct uart_event *evt, void *user_data)
{
	(void)user_data;

	switch (evt->type)
	{
	case UART_RX_RDY:
		/* 收到一段数据：分发给使用者（judge 解析） */
		if (rx_callback != NULL)
		{
			rx_callback(evt->data.rx.buf + evt->data.rx.offset,
				    evt->data.rx.len);
		}
		break;

	case UART_RX_BUF_REQUEST:
		/* 驱动要下一块缓冲，保证接收不断流 */
		if (rx_free_dma_buf != NULL &&
		    uart_rx_buf_rsp(dev, rx_free_dma_buf, RX_DMA_BUF_SIZE) == 0)
		{
			rx_free_dma_buf = NULL;
		}
		break;

	case UART_RX_BUF_RELEASED:
		/* 驱动交还这一块，可以再用 */
		rx_free_dma_buf = evt->data.rx_buf.buf;
		break;

	case UART_RX_DISABLED:
		/* 接收停了（没有空闲缓冲 / 出错）：到线程上下文里重启 */
		(void)k_work_submit(&uart_bus_rx_restart_work);
		break;

	case UART_TX_DONE:
	case UART_TX_ABORTED:
		/* 发送结束，tx_buf 可以再次使用 */
		tx_busy = false;
		break;

	default:
		break;
	}
}

static void Uart_Bus_Rx_Restart_Work(struct k_work *work)
{
	(void)work;

	rx_free_dma_buf = rx_dma_buf[1];

	int ret = uart_rx_enable(bus_uart, rx_dma_buf[0], RX_DMA_BUF_SIZE,
				 RX_INACTIVITY_TIMEOUT_US);
	if (ret != 0)
	{
		LOG_ERR("uart_rx_enable failed: %d", ret);
	}
}

void Uart_Bus_Set_Rx_Callback(uart_bus_rx_cb_t cb)
{
	rx_callback = cb;
}

uint8_t Uart_Bus_Send(const uint8_t *buf, uint16_t len)
{
	if (buf == NULL || len == 0 || len > UART_BUS_TX_BUF_SIZE)
	{
		LOG_ERR("bad tx len %u", (unsigned int)len);
		return 1; /* HAL_ERROR */
	}

	if (tx_busy)
	{
		/* 上一笔还在搬：不能动 tx_buf，让调用者下周期重试 */
		if (((uint32_t)k_uptime_get_32() - tx_submit_ms) < TX_BUSY_TIMEOUT_MS)
		{
			return 2; /* HAL_BUSY */
		}

		/* 兜底：完成事件丢了（正常不会发生），强制放行 */
		LOG_WRN("tx done event missing, force free tx buffer");
		tx_busy = false;
	}

	memcpy(tx_buf, buf, len);

	int ret = uart_tx(bus_uart, tx_buf, len, 0);
	if (ret == 0)
	{
        tx_busy = true;
		tx_submit_ms = (uint32_t)k_uptime_get_32();
		return 0; /* HAL_OK：DMA 已启动，不代表已经发完 */
	}

	if (ret == -EBUSY)
	{
		/* 驱动手里还有一笔，说明 tx_buf 仍被占用 */
        tx_busy = true;
		tx_submit_ms = (uint32_t)k_uptime_get_32();
		return 2; /* HAL_BUSY */
	}

	LOG_ERR("uart_tx failed: %d", ret);
	return 1; /* HAL_ERROR */
}

int Uart_Bus_Init(void)
{
	if (initialized)
	{
		return 0;
	}

	if (!device_is_ready(bus_uart))
	{
		LOG_ERR("usart1 not ready");
		return -ENODEV;
	}

	/* 异步 API：注册“唯一”的事件回调。
	 * CONFIG_UART_EXCLUSIVE_API_CALLBACKS 会顺手清掉中断驱动 API 的回调，
	 * 所以本工程其它地方不要再调用 uart_irq_callback_user_data_set()。 */
	int ret = uart_callback_set(bus_uart, Uart_Bus_Callback, NULL);
	if (ret != 0)
	{
		return ret;
	}

	rx_free_dma_buf = rx_dma_buf[1];
	ret = uart_rx_enable(bus_uart, rx_dma_buf[0], RX_DMA_BUF_SIZE,
			     RX_INACTIVITY_TIMEOUT_US);
	if (ret != 0)
	{
		LOG_ERR("uart_rx_enable failed: %d", ret);
		return ret;
	}

	initialized = true;
	return 0;
}

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(judge, LOG_LEVEL_INF);

#include "judge_protocol.h"
#include "string.h"
#include "crc.h"
#include "judge.h"

/* 通过设备树节点获取 USART1 */
#define USART1_NODE DT_NODELABEL(usart1) 
static const struct device *const usart1 = DEVICE_DT_GET(USART1_NODE);


judge_frame_header_t judge_frame_header;
drv_judge_info_t drv_judge_info = {
	.frame_header = &judge_frame_header,
};

#define RX_BUF_SIZE 256
static uint8_t rx_buf[RX_BUF_SIZE];
static uint16_t rx_len = 0;
void USART1_rxDataHandler(const struct device *dev, void *user_data)
{	
	uart_irq_update(dev);
	uint8_t byte;

	while (uart_irq_rx_ready(dev) && uart_fifo_read(dev, &byte, 1) > 0)
	{
		//将数据填入
		if (rx_len < RX_BUF_SIZE)
		{
			rx_buf[rx_len++] = byte;
		}
		else
		{
			rx_len = 0;//数据满了置零
		}
		while (rx_len >= 5) // SOF(1)+Data Length(4)
		{
			uint16_t i;
			for (i = 0; i < rx_len && rx_buf[i] != 0xA5; i++);//判断首帧
			if (i == rx_len)
			{
				rx_len = 0;
				break; // 没有首帧跳出
			}
			//得到首帧index
			if (i)
			{
				memmove(rx_buf, rx_buf + i, rx_len - i);//移动数据覆盖首帧前的无用数据
				rx_len -= i;
				continue;
			}

			drv_judge_info.frame_header->SOF = rx_buf[0];
			memcpy(&drv_judge_info.frame_header->data_length, rx_buf + 1, 4);
			uint16_t frame_length = 5 + 2 + drv_judge_info.frame_header->data_length + 2;
			if (rx_len < frame_length)//等待完整帧
			{
				break;
			}
			// 先CRC8
			if (Verify_CRC8_Check_Sum(rx_buf, 5) && Verify_CRC16_Check_Sum(rx_buf, frame_length))
			{
				memcpy(&drv_judge_info.cmd_id, rx_buf + 5, 2);
				// LOG_INF("drv_judge_info.cmd_id%d", drv_judge_info.cmd_id);
				Judge_Data_Update(drv_judge_info.cmd_id, rx_buf + 7);
				memcpy(&drv_judge_info.frame_tail, rx_buf + 5 + 2 + drv_judge_info.frame_header->data_length, 2);
				//消费完整帧
				memmove(rx_buf, rx_buf + frame_length, rx_len - frame_length);
				rx_len -= frame_length;
			}
			else
			{
				// CRC 失败：往后找下一个首帧，一次搬到位（不再逐字节搬移）
				for (i = 1; i < rx_len && rx_buf[i] != 0xA5; i++);
				if (i == rx_len)
				{
					rx_len = 0;
					break;
				}
				memmove(rx_buf, rx_buf + i, rx_len - i);
				rx_len -= i;
			}
		}
	}
}

int Judge_Init(Judge_t *judge)
{
	judge->status->offline_cnt = judge->status->offline_cnt_max;
	judge->status->status = DEV_OFFLINE;

	judge->rx = Judge_Data_Update;

	judge->heartbeat = Judge_Heart_Beat;

	/* 检查设备是否 ready */
	if (!device_is_ready(usart1))
	{
		LOG_ERR("UART1 device not ready");
		return -1;
	}
	/* 注册中断回调 */
	int ret = uart_irq_callback_user_data_set(usart1, USART1_rxDataHandler, NULL);
	if ( ret != 0)
	{
		return ret;
	}
	

	/* 使能接收中断 */
	uart_irq_rx_enable(usart1);

	return 0;
}



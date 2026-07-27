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

void judge_receive(uint8_t *rxBuf)
{
	
	uint16_t frame_length;
	if( rxBuf == NULL )
	{
		return;
	}
	drv_judge_info.frame_header->SOF = rxBuf[0];
	if(drv_judge_info.frame_header->SOF == 0xA5)
	{
		memcpy(&drv_judge_info.frame_header->data_length, rxBuf + 1, 4);
		if(Verify_CRC8_Check_Sum(rxBuf, 5) == 1)
		{
			frame_length = 5 + 2 + drv_judge_info.frame_header->data_length + 2;
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == 1)
			{
				memcpy(&drv_judge_info.cmd_id, rxBuf + 5, 2);
				Judge_Data_Update(drv_judge_info.cmd_id, rxBuf + 7);
				
			}
			memcpy(&drv_judge_info.frame_tail, rxBuf + 5 + 2 + drv_judge_info.frame_header->data_length, 2);
			
			/* 如果一个数据包出现了多帧数据就再次读取 */
			if(rxBuf[frame_length] == 0xA5)
			{
				judge_receive( &rxBuf[frame_length] );
			}
		}
	}


}

int Judge_Init(void)
{
	/* 检查设备是否 ready */
	if (!device_is_ready(usart1))
	{
		LOG_ERR("UART1 device not ready");
		return;
	}

	/* 注册中断回调 */
	uart_irq_callback_user_data_set(usart1, uart1_irq_callback, NULL);

	/* 使能接收中断 */
	uart_irq_rx_enable(usart1);

	judge->status->offline_cnt = judge->status->offline_cnt_max;
	judge->status->status = DEV_OFFLINE;

	judge->rx = Judge_Data_Update;

	judge->heartbeat = Judge_Heart_Beat;

}


void USART1_rxDataHandler(uint8_t *rxBuf)
{	
	judge_receive(rxBuf);
}

#include "board_protocol.h"
// TODO: Judge模块尚未迁移，待移植后取消注释
// #include "judge.h"
#include <string.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <drivers/can_tx_manager.h>
#include <drivers/can_rx_manager.h>
#include "rp_math.h"


Board_Tx_Pkt_t    board_tx_pkt;
Board_Rx_Meg_t    board_rx_meg;

/* can_tx_manager_send 的 NULL 回调会导致 USAGE FAULT，加个空回调代替 */
static void board_tx_cb(const struct device *dev, int error, void *user_data)
{
	(void)dev;
	(void)error;
	(void)user_data;
}

Board_Status_t board_status = 
{
	.offline_cnt_max = BOARD_OFFLINE_CNT_MAX,
};

Board_t board = 
{
	.tx_pkt = &board_tx_pkt,
	.rx_meg = &board_rx_meg,
	.status = &board_status,
	
	.init = Board_Init,
};

static const struct device *board_tx_mgr = NULL;
static const struct device *board_rx_mgr = NULL;

uint8_t  pkt_01[8];
uint8_t  pkt_02[8];
uint8_t  pkt_03[8];
uint8_t  pkt_04[8];

/* CAN TX fill_buffer 回调 */
static int board_fill_pkt_01(struct can_frame *frame, void *user_data)
{
	memcpy(frame->data, pkt_01, 8);
	return 0;
}

static int board_fill_pkt_02(struct can_frame *frame, void *user_data)
{
	memcpy(frame->data, pkt_02, 8);
	return 0;
}

static int board_fill_pkt_03(struct can_frame *frame, void *user_data)
{
	memcpy(frame->data, pkt_03, 8);
	return 0;
}

static int board_fill_pkt_04(struct can_frame *frame, void *user_data)
{
	memcpy(frame->data, pkt_04, 8);
	return 0;
}

/* CAN RX handler */
static void board_rx_handler_01(const struct can_frame *frame, void *user_data)
{
	Board_Rx_Meg_01(&board, (uint8_t *)frame->data);
}

static void board_rx_handler_02(const struct can_frame *frame, void *user_data)
{
	Board_Rx_Meg_02(&board, (uint8_t *)frame->data);
}


int Board_Init(Board_t* board)
{
	board_tx_mgr = device_get_binding("can_tx_mgr2");
	board_rx_mgr = device_get_binding("can_rx_mgr2");

	if (board_tx_mgr == NULL || board_rx_mgr == NULL) {
		return -1;
	}

	board->status->offline_cnt = board->status->offline_cnt_max;
	board->status->status = DEV_OFFLINE;
	
	board->tx_01 = Board_Tx_Pkt_01;
	board->tx_02 = Board_Tx_Pkt_02;
	board->tx_03 = Board_Tx_Pkt_03;
	board->tx_04 = Board_Tx_Pkt_04;
	
	board->rx_01 = Board_Rx_Meg_01;
	board->rx_02 = Board_Rx_Meg_02;
	
	board->heartbeat = Board_Heart_Beat;

	/* 注册 CAN 发送，由 tx_xx 手动调用发送 */
	can_tx_manager_register(board_tx_mgr, ID_PKT_01, 0, 8, 0, 0, board_fill_pkt_01, NULL);
	can_tx_manager_register(board_tx_mgr, ID_PKT_02, 0, 8, 0, 0, board_fill_pkt_02, NULL);
	can_tx_manager_register(board_tx_mgr, ID_PKT_03, 0, 8, 0, 0, board_fill_pkt_03, NULL);
	can_tx_manager_register(board_tx_mgr, ID_PKT_04, 0, 8, 0, 0, board_fill_pkt_04, NULL);

	/* 注册 CAN 接收 */
	struct can_filter filter_01 = { .id = ID_MEG_01, .mask = CAN_STD_ID_MASK, .flags = 0 };
	can_rx_manager_register(board_rx_mgr, &filter_01, board_rx_handler_01, NULL);
	struct can_filter filter_02 = { .id = ID_MEG_02, .mask = CAN_STD_ID_MASK, .flags = 0 };
	can_rx_manager_register(board_rx_mgr, &filter_02, board_rx_handler_02, NULL);

	return 0;
}


void Board_Heart_Beat(Board_t* board)
{
	board->status->offline_cnt ++;
	
	if(board->status->offline_cnt >= board->status->offline_cnt_max)
	{
		board->status->offline_cnt = board->status->offline_cnt_max;
		board->status->status = DEV_OFFLINE;
	}
	else
	{
		board->status->status = DEV_ONLINE;
	}
}



void Board_Tx_Pkt_01(Board_t* board)
{
	memset(pkt_01, 0, 8);
	
	pkt_01[0] |= (board->tx_pkt->car_pkt.car_state & 0x03) << 0;
	pkt_01[0] |= (board->tx_pkt->car_pkt.gimbal_mode & 0x01) << 2;
	pkt_01[0] |= (board->tx_pkt->car_pkt.vision_mode & 0x07) << 3;
	pkt_01[0] |= (board->tx_pkt->car_pkt.game_start & 0x01) << 6;
	pkt_01[0] |= (board->tx_pkt->car_pkt.my_color & 0x01) << 7;
	
	uint16_t t1,t2;
	
	t1 = float_to_uint(board->tx_pkt->car_pkt.v_x,-8000.f,8000.f,16);
	t2 = float_to_uint(board->tx_pkt->car_pkt.v_y,-8000.f,8000.f,16);
	
	pkt_01[1] = t1>>8;
	pkt_01[2] = t1;
	pkt_01[3] = t2>>8;
	pkt_01[4] = t2;

									 
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.launch_state & 0x01) << 0;
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.shoot_mode & 0x01) << 1;
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.shoot_level & 0x01) << 2;
	pkt_01[5] |= (board->tx_pkt->gimbal_target_pkt.is_hole & 0x01) << 3;
	
	if (board_tx_mgr) {
		can_tx_manager_send(board_tx_mgr, K_MSEC(10), board_tx_cb, ID_PKT_01, NULL);
	}
}

void Board_Tx_Pkt_02(Board_t* board)
{
	uint16_t t1,t2,t3,t4;
	
	t1 = float_to_uint(board->tx_pkt->gimbal_target_pkt.pitch_imu_tar,-360.f,360.f,16); //pitch陀螺仪模式目标角度
	t2 = float_to_uint(board->tx_pkt->gimbal_target_pkt.yaw_imu_tar,-360.f,360.f,16);
	t3 = float_to_uint(board->tx_pkt->gimbal_target_pkt.pitch_mec_tar,-4.f,4.f,16); //pitch机械模式目标角度
	t4 = float_to_uint(board->tx_pkt->gimbal_target_pkt.yaw_mec_tar,-4.f,4.f,16);   //ywa轴发射后角度偏移
	  
	pkt_02[0] = t1>>8;
	pkt_02[1] = t1;
	pkt_02[2] = t2>>8;
	pkt_02[3] = t2;
	pkt_02[4] = t3>>8;
	pkt_02[5] = t3;
	pkt_02[6] = t4>>8;
	pkt_02[7] = t4;
	
	if (board_tx_mgr) {
		can_tx_manager_send(board_tx_mgr, K_MSEC(10), board_tx_cb, ID_PKT_02, NULL);
	}
}

void Board_Tx_Pkt_03(Board_t* board)
{
	uint16_t t1,t2;
	
	t1 = float_to_uint(board->tx_pkt->judge_shoot_pkt.shoot_speed,-50.f,50.f,16);
	t2 = float_to_uint(board->tx_pkt->judge_shoot_pkt.shoot_freq,-50.f,50.f,16);
	
	// TODO: Judge模块尚未迁移，待移植后取消注释
	// board->tx_pkt->judge_shoot_pkt.shoot_heat_err = judge.pkt->shooter_barrel_heat_limit - judge.pkt->shooter_17mm_1_barrel_heat;
	// board->tx_pkt->judge_shoot_pkt.allowance_max = judge.pkt->projectile_allowance_17mm;

	
	pkt_03[0] = t1>>8;
	pkt_03[1] = t1;
	pkt_03[2] = t2>>8;
	pkt_03[3] = t2;
	pkt_03[4] = board->tx_pkt->judge_shoot_pkt.shoot_heat_err>>8;
	pkt_03[5] = board->tx_pkt->judge_shoot_pkt.shoot_heat_err;
	pkt_03[6] = board->tx_pkt->judge_shoot_pkt.allowance_max>>8;
	pkt_03[7] = board->tx_pkt->judge_shoot_pkt.allowance_max;
	
	if (board_tx_mgr) {
		can_tx_manager_send(board_tx_mgr, K_MSEC(10), board_tx_cb, ID_PKT_03, NULL);
	}
}


void Board_Tx_Pkt_04(Board_t* board)
{
	for(uint8_t i = 0;i<8;i++)
	{
	  pkt_04[i] = board->tx_pkt->blood_pkt.blood[i];
	}

	if (board_tx_mgr) {
		can_tx_manager_send(board_tx_mgr, K_MSEC(10), board_tx_cb, ID_PKT_04, NULL);
	}
}


void Board_Rx_Meg_01(Board_t* board,uint8_t* rxbuf)
{
	board->rx_meg->state_meg.yaw_motor_state= (rxbuf[0] >> 0) & 0x01;
	board->rx_meg->state_meg.pitch_motor_state= (rxbuf[0] >> 1) & 0x01;
	board->rx_meg->state_meg.height_motor_state= (rxbuf[0] >> 2) & 0x01;
	board->rx_meg->state_meg.r_fric_state= (rxbuf[0] >> 3) & 0x01;
	board->rx_meg->state_meg.l_fric_state= (rxbuf[0] >> 4) & 0x01;
	board->rx_meg->state_meg.dial_motor_state= (rxbuf[0] >> 5) & 0x01;
	board->rx_meg->state_meg.vision_state= (rxbuf[0] >> 6) & 0x01;
	board->rx_meg->state_meg.is_down= (rxbuf[0] >> 7) & 0x01;
	
	uint16_t t1 = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];  
    uint16_t t2 = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];

    board->rx_meg->vision_meg.vision_yaw_tar = uint_to_float(t1, -360.0f, 360.0f,16);
    board->rx_meg->vision_meg.vision_pitch_tar = uint_to_float(t2, -360.0f, 360.0f,16);
	board->rx_meg->vision_meg.is_find_target = (rxbuf[6] >> 0) & 0x01;
	
	board->status->offline_cnt = 0;
}


void Board_Rx_Meg_02(Board_t* board,uint8_t* rxbuf)
{
  uint16_t t1 = ((uint16_t)rxbuf[0] << 8) | rxbuf[1];  
  uint16_t t2 = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];
  uint16_t t3 = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];
  uint16_t t4 = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];
    
  board->rx_meg->gimbal_meg.yaw_mec     = uint_to_float(t1, -4.f, 4.f,16);
  board->rx_meg->gimbal_meg.pitch_mec   = uint_to_float(t2, -4.f, 4.f,16);
  board->rx_meg->gimbal_meg.yaw_imu     = uint_to_float(t3, -360.0f, 360.0f,16);
  board->rx_meg->gimbal_meg.pitch_imu   = uint_to_float(t4, -360.0f, 360.0f,16);

  board->status->offline_cnt = 0;
}

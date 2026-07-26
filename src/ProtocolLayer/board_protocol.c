#include "board_protocol.h"


Board_Tx_Info_t Board_Tx_Info;
Board_Rx_Info_t Board_Rx_Info;
Board_HeartBeat_t Board_HeartBeat = {.offline_cnt_pack_1 = 0,\
    .offline_cnt_pack_2 = 0, .offline_cnt_pack_3 = 0, .offline_cnt_pack_4 = 0, .offline_cnt_max = 100};

static const struct device *board_tx_mgr = NULL;
static const struct device *board_rx_mgr = NULL;

/*------------------------------------buf start---------------------------------------*/
uint8_t pkt_01_rx[2];
uint8_t pkt_02_rx[8];
uint8_t pkt_03_rx[8];
uint8_t pkt_04_rx[3];

uint8_t board_tx_buf1[8];
uint8_t board_tx_buf2[8];

/*-------------------------------------buf end--------------------------------*/

static int board_fill_tx1(struct can_frame *frame, void *user_data)
{
    memcpy(frame->data, board_tx_buf1, 8);
    return 0;
}

static int board_fill_tx2(struct can_frame *frame, void *user_data)
{
    memcpy(frame->data, board_tx_buf2, 8);
    return 0;
}

static void board_rx_handler_01(const struct can_frame *frame, void *user_data)
{
    Board_Rx_01((uint8_t *)frame->data);
}

static void board_rx_handler_02(const struct can_frame *frame, void *user_data)
{
    Board_Rx_02((uint8_t *)frame->data);
}

static void board_rx_handler_03(const struct can_frame *frame, void *user_data)
{
    Board_Rx_03((uint8_t *)frame->data);
}

static void board_rx_handler_04(const struct can_frame *frame, void *user_data)
{
    Board_Rx_04((uint8_t *)frame->data);
}

int Board_Init(void)
{
    board_tx_mgr = device_get_binding("can_tx_mgr2");
    board_rx_mgr = device_get_binding("can_rx_mgr2");

    if (board_tx_mgr == NULL || board_rx_mgr == NULL) {
        return -1;
    }

    can_tx_manager_register(board_tx_mgr, ID_Board_Tx1, 0, 8, 0, 0, board_fill_tx1, NULL);
    can_tx_manager_register(board_tx_mgr, ID_Board_Tx2, 0, 8, 0, 0, board_fill_tx2, NULL);

    struct can_filter filter_01 = { .id = ID_Board_Rx1, .mask = CAN_STD_ID_MASK, .flags = 0 };
    can_rx_manager_register(board_rx_mgr, &filter_01, board_rx_handler_01, NULL);

    struct can_filter filter_02 = { .id = ID_Board_Rx2, .mask = CAN_STD_ID_MASK, .flags = 0 };
    can_rx_manager_register(board_rx_mgr, &filter_02, board_rx_handler_02, NULL);

    struct can_filter filter_03 = { .id = ID_Board_Rx3, .mask = CAN_STD_ID_MASK, .flags = 0 };
    can_rx_manager_register(board_rx_mgr, &filter_03, board_rx_handler_03, NULL);

    struct can_filter filter_04 = { .id = ID_Board_Rx4, .mask = CAN_STD_ID_MASK, .flags = 0 };
    can_rx_manager_register(board_rx_mgr, &filter_04, board_rx_handler_04, NULL);

    return 0;
}

static void Board_Tx_Meg_02(uint8_t *txbuf)
{
    // yaw_mec
    uint16_t t1 = float_to_uint(Board_Tx_Info.gimbal_meg.yaw_mec, -4.0f, 4.0f, 16);
    txbuf[0] = (t1 >> 8) & 0xFF;
    txbuf[1] = t1 & 0xFF;

    // pitch_mec
    uint16_t t2 = float_to_uint(Board_Tx_Info.gimbal_meg.pitch_mec, -4.0f, 4.0f, 16);
    txbuf[2] = (t2 >> 8) & 0xFF;
    txbuf[3] = t2 & 0xFF;

    // yaw_imu
    uint16_t t3 = float_to_uint(Board_Tx_Info.gimbal_meg.yaw_imu, -360.0f, 360.0f, 16);
    txbuf[4] = (t3 >> 8) & 0xFF;
    txbuf[5] = t3 & 0xFF;

    // pitch_imu
    uint16_t t4 = float_to_uint(Board_Tx_Info.gimbal_meg.pitch_imu, -360.0f, 360.0f, 16);
    txbuf[6] = (t4 >> 8) & 0xFF;
    txbuf[7] = t4 & 0xFF;

    if (board_tx_mgr) {
        can_tx_manager_send(board_tx_mgr, K_MSEC(10), NULL, ID_Board_Tx2, NULL);
    }
}

static void Board_Tx_Meg_01(uint8_t *txbuf)
{
    // 7个电机/加上升降状态 打包到 txbuf[0]
    memcpy(&txbuf[0], &Board_Tx_Info.state_meg, 1);
	txbuf[1] = Board_Tx_Info.state_meg.lift_state;
    // 视觉 yaw 目标
    uint16_t t1 = float_to_uint(Board_Tx_Info.vision_meg.vision_yaw_tar, -360.0f, 360.0f, 16);
    txbuf[2] = (t1 >> 8) & 0xFF;
    txbuf[3] = t1 & 0xFF;

    // 视觉 pitch 目标
    uint16_t t2 = float_to_uint(Board_Tx_Info.vision_meg.vision_pitch_tar, -360.0f, 360.0f, 16);
    txbuf[4] = (t2 >> 8) & 0xFF;
    txbuf[5] = t2 & 0xFF;

    txbuf[6] = Board_Tx_Info.vision_meg.is_find_target ? 1 : 0;

    if (board_tx_mgr) {
        can_tx_manager_send(board_tx_mgr, K_MSEC(10), NULL, ID_Board_Tx1, NULL);
    }
}

// 接收 PKT01：解析状态 + 发射指令
static void Board_Rx_Pkt_01(uint8_t *rxbuf)
{
    // 第0字节 + 第1字节 → 直接 memcpy 进位域结构体（2字节）
    memcpy(&Board_Rx_Info.state_pkt, rxbuf, 2);
	memcpy(&Board_Rx_Info.shoot_pkt, &rxbuf[5], 1);
}

// 接收 PKT02：解析云台目标角度
void Board_Rx_Pkt_02(uint8_t *rxbuf)
{
    uint16_t t1, t2, t3, t4;

    t1 = ((uint16_t)rxbuf[0] << 8) | rxbuf[1];
    t2 = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];
    t3 = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];
    t4 = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];

    Board_Rx_Info.gimbal_target_pkt.pitch_imu_tar = uint_to_float(t1, -360.0f, 360.0f, 16);
    Board_Rx_Info.gimbal_target_pkt.yaw_imu_tar = uint_to_float(t2, -360.0f, 360.0f, 16);
    Board_Rx_Info.gimbal_target_pkt.pitch_mec_tar = uint_to_float(t3, -4.0f, 4.0f, 16);
    Board_Rx_Info.gimbal_target_pkt.yaw_mec_tar = uint_to_float(t4, -4.0f, 4.0f, 16);
}

// 接收 PKT03：解析裁判系统热量
void Board_Rx_Pkt_03(uint8_t *rxbuf)
{
    uint16_t t1, t2;

    t1 = ((uint16_t)rxbuf[0] << 8) | rxbuf[1];
    t2 = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];

    Board_Rx_Info.judge_shoot_pkt.shoot_speed = uint_to_float(t1, -50.0f, 50.0f, 16);
    Board_Rx_Info.judge_shoot_pkt.shoot_freq = uint_to_float(t2, -50.0f, 50.0f, 16);
    Board_Rx_Info.judge_shoot_pkt.shoot_heat_err = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];
    Board_Rx_Info.judge_shoot_pkt.allowance_max = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];
}

// 接收 PKT04：解析血量信息（直接 memcpy）
void Board_Rx_Pkt_04(uint8_t *rxbuf)
{
    memcpy(&Board_Rx_Info.blood_pkt, rxbuf, 8);
}

void Board_Tx_Update(Board_Tx_Info_t *Board_Tx_Info)
{
    // 云台信息 (Board_Gimbal_Meg_t)
    Board_Tx_Info->gimbal_meg.yaw_imu = imu_get_yaw();
    Board_Tx_Info->gimbal_meg.pitch_imu = imu_get_pitch();
    //TO DO 移植gimbal
    // Board_Tx_Info->gimbal_meg.pitch_mec = Gimbal.base_info.pitch_motor_angle;
    // Board_Tx_Info->gimbal_meg.yaw_mec = Gimbal.base_info.yaw_motor_angle;
    // Board_Tx_Info->state_meg.lift_state = Gimbal.Lift.lift_state;

    //TO DO 移植vision
    // 视觉目标 (Board_Vision_Meg_t)
    // Board_Tx_Info->vision_meg.vision_yaw_tar = vision.VtoE->yaw;
    // Board_Tx_Info->vision_meg.vision_pitch_tar = vision.VtoE->pitch;
    // Board_Tx_Info->vision_meg.is_find_target = vision.VtoE->is_find_target;
   
    //TO DO 移植dev_motor
    // 电机状态 (1字节 位域结构体)
    // Board_Tx_Info->state_meg.pitch_motor_state = (dm_motor[PITCH].state->status == DEV_ONLINE) ? 1 : 0;
    // Board_Tx_Info->state_meg.yaw_motor_state = (dm_motor[YAW].state->status == DEV_ONLINE) ? 1 : 0;
    // Board_Tx_Info->state_meg.lift_motor_state = (rm_motor[LIFT].state->status == DEV_ONLINE) ? 1 : 0;
    // Board_Tx_Info->state_meg.l_fric_state = (rm_motor[L_Fric].state->status == DEV_ONLINE) ? 1 : 0;
    // Board_Tx_Info->state_meg.r_fric_state = (rm_motor[R_Fric].state->status == DEV_ONLINE) ? 1 : 0;
    // Board_Tx_Info->state_meg.dial_motor_state = (dail_motor.KT_motor_info .state_info.work_state == M_ONLINE) ? 1 : 0;
    
    // if(Gimbal.Lift.lift_state == LIFT_DOWN)
    //     Board_Tx_Info->state_meg.lift_state = 0;
    // else if(Gimbal.Lift.lift_state == LIFT_UP)
    //     Board_Tx_Info->state_meg.lift_state = 2;
	// else
	// 	Board_Tx_Info->state_meg.lift_state = 1;
    // Board_Tx_Info->state_meg.vision_state = (vision.status->rx_state == DEV_ONLINE) ? 1 : 0;

}

void Board_Rx_01(uint8_t *rxbuf)
{
    Board_Rx_Pkt_01(rxbuf);
    Board_HeartBeat.offline_cnt_pack_1 = 0;
}

void Board_Rx_02(uint8_t *rxbuf)
{
    Board_Rx_Pkt_02(rxbuf);
    Board_HeartBeat.offline_cnt_pack_2 = 0;
}

void Board_Rx_03(uint8_t *rxbuf)
{
    Board_Rx_Pkt_03(rxbuf);
    Board_HeartBeat.offline_cnt_pack_3 = 0;
}

void Board_Rx_04(uint8_t *rxbuf)
{
    Board_Rx_Pkt_04(rxbuf);
    Board_HeartBeat.offline_cnt_pack_4 = 0;
}

void Send_To_Down_Board(void)
{
    Board_Tx_Update(&Board_Tx_Info);
    Board_Tx_Meg_01(board_tx_buf1);
    Board_Tx_Meg_02(board_tx_buf2);
}

int offline_count=0;
int err_count_1 = 0,err_count_2 = 0;
void C_Board_Communicate_HeartBeat(void)
{
    Board_HeartBeat.offline_cnt_pack_1++;
    Board_HeartBeat.offline_cnt_pack_2++;
    Board_HeartBeat.offline_cnt_pack_3++;

    if (Board_HeartBeat.offline_cnt_pack_1 > Board_HeartBeat.offline_cnt_max ||
        Board_HeartBeat.offline_cnt_pack_2 > Board_HeartBeat.offline_cnt_max ||
        Board_HeartBeat.offline_cnt_pack_3 > Board_HeartBeat.offline_cnt_max)
    {
        Board_HeartBeat.status = DEV_OFFLINE;
        offline_count ++;
    }
    if(Board_HeartBeat.offline_cnt_pack_1 > Board_HeartBeat.offline_cnt_max)
        Board_HeartBeat.offline_cnt_pack_1 = Board_HeartBeat.offline_cnt_max;
    if(Board_HeartBeat.offline_cnt_pack_2 > Board_HeartBeat.offline_cnt_max)
        Board_HeartBeat.offline_cnt_pack_2 = Board_HeartBeat.offline_cnt_max;
    if(Board_HeartBeat.offline_cnt_pack_3 > Board_HeartBeat.offline_cnt_max)
        Board_HeartBeat.offline_cnt_pack_3 = Board_HeartBeat.offline_cnt_max;
    if (Board_HeartBeat.status == DEV_OFFLINE)
    {
        Board_HeartBeat.status = DEV_ONLINE;
    }
}

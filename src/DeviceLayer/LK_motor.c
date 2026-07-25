// #include "KT_motor.h"

// /* Private functions ---------------------------------------------------------*/
// #define OFFLINE_LINE_CNT_MAX 100
// #define KT_TX_IQ_CONTROL_MAX 2000		   //-2048~2048   ← 被 W_iqControl 使用（外部调用了）


// void KT_motor_class_heartbeat(KT_motor_t *motor);

// void get_kt_motor_info(KT_motor_t *motor, uint8_t *rxBuf);

// static void KT_Encoder_Sum_Cal(KT_motor_t *motor);

// void tx_kt_motor_W_command(KT_motor_t *motor, uint8_t command);

// void write_kt_motor_iqControl_param(KT_motor_t *motor, int16_t iqControl);

// /**

//  *	@brief	电机心跳，如果失联，下一次收到CAN数据时offline_cnt会被get_info清零

//  *	        注：selfprotect_cnt/flag逻辑虽在旧代码中实现，但上层从未读取，已注释

//  */

// void KT_motor_class_heartbeat(KT_motor_t *motor)

// {

// 	static int16_t current_last;

// 	if (motor == NULL)
// 		return;

// 	KT_motor_state_info_t *state_info = &motor->KT_motor_info.state_info;
//     state_info->work_state = get_motor_heartbeat_status(motor->lk_motor) ? M_ONLINE : M_OFFLINE;

// 	state_info->offline_cnt++;

// 	// 发过来的电流一直相同判断为进入电流保护（上层未读取selfprotect_flag，迁移后可删）
// 	// if (motor->KT_motor_info.rx_info.current == current_last)
// 	// {
// 	//     state_info->selfprotect_cnt++;
// 	// }
// 	// current_last = motor->KT_motor_info.rx_info.current;

// 	if (state_info->offline_cnt > state_info->offline_cnt_max)
// 	{
// 		state_info->offline_cnt = state_info->offline_cnt_max;
// 		state_info->work_state = M_OFFLINE;
// 	}
// 	else
//     {
// 		if (state_info->work_state == M_OFFLINE)
// 			state_info->work_state = M_ONLINE;
// 	}
// }


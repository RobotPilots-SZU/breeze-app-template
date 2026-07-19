#include "gimbal.h"
// #include "vision.h"
// #include "infantry.h"
// #include "board_protocol.h"
#include "rp_math.h"
#include "imu_wrapper.h"
#include <drivers/remote.h>

static void Gimbal_Init(Gimbal_t* gimbal);
static void Gimbal_Data_Update(Gimbal_t* gimbal);
static void Gimbal_Status_Update(Gimbal_t* gimbal);
static void Gimbal_Slave_Update(Gimbal_t* gimbal);
static void Gimbal_Boss_Update(Gimbal_t* gimbal);
static void Gimbal_Offline_Update(Gimbal_t* gimbal);
static void Gimbal_Offline_Process(Gimbal_t* gimbal);
static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal);
static void Gimbal_Work(Gimbal_t* gimbal);

// Zephyr 遥控器数据指针，在 Gimbal_Init 中初始化
static rc_sensor_info_t *rc_info = NULL;


Gimbal_t gimbal = {
  .mode = G_SLEEP,
  .gimbal_reset_flag= false,
		
	.config = {
	  .yaw_zero[FRONT] = YAW_MEC_ZERO_ANGLE,
	  .yaw_zero[BEHIND] = 2.48340917f,
	  .rc_yaw_imu_step = 0.2f,
	  .rc_yaw_mec_step = 0.02f,
	  .rc_pitch_mec_step = 0.002f,
	  .rc_pitch_imu_step = 0.002f,
	  .key_yaw_mec_step = 0.02f,
	  .key_pitch_mec_step = 0.02f,
	  .key_yaw_imu_step = 0.02f,
	  .key_pitch_imu_step = 0.02f,
	  
	},
	
	.init = Gimbal_Init,
};


float test_max,test_min;
static void Gimbal_Init(Gimbal_t* gimbal)
{
    /********************这里引入遥控器驱动后删除************************/
	const struct device *remote_dev = DEVICE_DT_GET(DT_ALIAS(remote0));
	rc_sensor_t *rc = remote_get_sensor(remote_dev);
	if (rc != NULL) {
		rc_info = rc->info;
	}
	
	gimbal->work = Gimbal_Work;
}

/**
 * @brief  云台状态模式更新
 * @note   
 */
static void Gimbal_Status_Update(Gimbal_t* gimbal)
{
	// TODO: infantry模块尚未迁移，待移植后取消注释
	/*
	test_max = PITCH_IMU_MAX_ANGLE;
	test_min = PITCH_IMU_MIN_ANGLE;
	switch (infantry.mode)
	{
	  case I_SLEEP:
			gimbal->mode = G_SLEEP;
			break;
		
		case I_INIT:
			gimbal->mode = G_INIT;
			break;
		
		case I_MEC:
			gimbal->mode = G_SLAVE;
			break;
		
		case I_HOLE:
			 if(infantry.flag.hole_flag == true && infantry.flag.chassis_reset.value == true)
			{
				gimbal->mode = G_BOSS;
			}
			else if(infantry.flag.hole_flag == true && infantry.flag.chassis_reset.value == false)
			{
				gimbal->mode = G_SLAVE;
			}
			else if(infantry.flag.hole_flag == false && board.rx_meg->state_meg.is_down == false)
			{
				gimbal->mode = G_BOSS;
				infantry.mode = I_IMU;
			}
			else if(infantry.flag.hole_flag == false && board.rx_meg->state_meg.is_down == true)
			{
				gimbal->mode = G_SLAVE;
			}
			break;
			
		case I_IMU:
		case I_TURN:
			gimbal->mode = G_BOSS;
			break;
		
		default:
			break;
	}
	*/
}

/**
 * @brief  云台数据更新
 * @note   来自板间
 */
static void Gimbal_Data_Update(Gimbal_t* gimbal)
{
	// TODO: board_protocol尚未迁移，待移植后取消注释
	// gimbal->info.yaw_mec = board.rx_meg->gimbal_meg.yaw_mec;

	// gimbal->info.yaw_imu = imu_sensor.info->base_info.yaw;
	//gimbal->info.yaw_imu = imu_get_yaw();
	
	// TODO: board_protocol尚未迁移，待移植后取消注释
	// gimbal->info.pitch_mec = board.rx_meg->gimbal_meg.pitch_mec;
	// gimbal->info.pitch_imu = board.rx_meg->gimbal_meg.pitch_imu;

	// gimbal->info.pitch_imu = imu_sensor.info->base_info.pitch;
	//gimbal->info.pitch_imu = imu_get_pitch();

	// TODO: board_protocol尚未迁移
	// gimbal->info.yaw_mec_err_raw = motor_half_cycle(gimbal->info.yaw_mec - YAW_MEC_ZERO_ANGLE,2*PI);
	// gimbal->info.pitch_mec_err_raw = motor_half_cycle(gimbal->info.pitch_mec - PITCH_MEC_ZERO_ANGLE,2*PI);
};

static uint16_t reset_tick = 0;
/**
 * @brief  云台初始化工作过程
 * @note   机械模式归位中值，头抬升到位
 */
static void Gimbal_Init_Process(Gimbal_t* gimbal)
{
	gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
	gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	
	reset_tick ++;
	
	if(abs(gimbal->info.yaw_mec_err_raw) <= 5.f/180.f*PI && abs(gimbal->info.pitch_mec_err_raw) <= 5.f/180.f*PI)
	{
		gimbal->gimbal_reset_flag = true;
//		board.tx_pkt->gimbal_target_pkt.is_hole = true;
		reset_tick = 0;
	}
//	else if(board.rx_meg->state_meg.is_down == 1)
//	{
////		gimbal->gimbal_reset_flag = true;
////		reset_tick = 0;
//	}
	else if(reset_tick >= 4000)
	{
		gimbal->gimbal_reset_flag = true;
		reset_tick = 0;
	}
}


static void Gimbal_Direct_Update(Gimbal_t* gimbal)
{
	if(abs(gimbal->info.yaw_mec_err_raw) <= PI/4)
	{
		
	}
	else if(abs(gimbal->info.yaw_mec_err_raw) >= 3*PI/4)
	{
		
	}
	else if(gimbal->info.yaw_mec_err_raw > PI/4 && gimbal->info.yaw_mec_err_raw < 3*PI/4)
	{
		
	}
	else if(gimbal->info.yaw_mec_err_raw < -PI/4 && gimbal->info.yaw_mec_err_raw > -3*PI/4)
	{
		
	}
	
}



static void  Gimbal_Slave_Update(Gimbal_t* gimbal)
{
	// TODO: infantry模块尚未迁移，待移植后取消注释
	/*
	if(gimbal->gimbal_reset_flag == false || gimbal->mode == G_INIT)
	{
	  gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
		gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	
	}
	else{
		if(gimbal->info.yaw_mec_err_raw <= PI/2)
		{
			gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[FRONT];
		}
		else{
		  gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[BEHIND];
		
		}
		
		if(infantry.mode == I_HOLE && board.tx_pkt->gimbal_target_pkt.is_hole == 1)
		{
			gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
		  gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
		}
		else{
		  if(infantry.ctrl == RC_CTRL)
		  {
			  gimbal->target.pitch_mec_tar += rc_sensor.info->ch1/660.f * gimbal->config.rc_pitch_mec_step;
		  }
		  else if(infantry.ctrl == KEY_CTRL)
		  {
			  gimbal->target.pitch_mec_tar += rc_sensor.info->mouse_y * gimbal->config.key_pitch_mec_step;
		  }
		
		}
		
		gimbal->target.pitch_mec_tar = motor_half_cycle(gimbal->target.pitch_mec_tar,2*PI);
		
		  gimbal->target.pitch_mec_tar = constrain(gimbal->target.pitch_mec_tar,PITCH_MEC_MIN_ANGLE,PITCH_MEC_MAX_ANGLE);
		
	  gimbal->target.yaw_imu_tar = gimbal->info.yaw_imu;
		gimbal->target.pitch_imu_tar = gimbal->info.pitch_imu;
	}
//	gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_mec - gimbal->target.yaw_mec_tar,2*PI);
	*/
 
}


static void  Gimbal_Boss_Update(Gimbal_t* gimbal)
{
	// TODO: vision / infantry / board_protocol 尚未迁移，待移植后取消注释
	/*
	//视觉模式上板直接用视觉包目标值，下板需要实时更新目标值防止退出视觉时目标值衔接错误导致头动
	if(vision.mode != V_NORMAL && board.rx_meg->state_meg.vision_state == true && board.rx_meg->vision_meg.is_find_target == true)  
	{
		gimbal->target.yaw_imu_tar = board.rx_meg->vision_meg.vision_yaw_tar;
	  gimbal->target.pitch_imu_tar = board.rx_meg->vision_meg.vision_pitch_tar;
	}

	else
	{
	  if(infantry.ctrl == RC_CTRL)
    {
		  gimbal->target.yaw_imu_tar -= rc_sensor.info->ch0/660.f * gimbal->config.rc_yaw_imu_step;
		  gimbal->target.pitch_imu_tar += rc_sensor.info->ch1/660.f * gimbal->config.rc_pitch_imu_step;
			
	  }
	  else if(infantry.ctrl == KEY_CTRL)
	  {
		  gimbal->target.yaw_imu_tar -= rc_sensor.info->mouse_x * gimbal->config.key_yaw_imu_step;
		  gimbal->target.pitch_imu_tar += rc_sensor.info->mouse_y * gimbal->config.key_pitch_imu_step;
			
	  }

	}
	*/
	
	gimbal->target.yaw_imu_tar = motor_half_cycle(gimbal->target.yaw_imu_tar,360.f);
	gimbal->target.pitch_imu_tar = constrain(gimbal->target.pitch_imu_tar,PITCH_IMU_MIN_ANGLE,PITCH_IMU_MAX_ANGLE);
	
	// TODO: infantry模块尚未迁移，待移植后取消注释
	/*
	if(infantry.flag.chassis_reset.value == true)
	{
		gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[FRONT];
	}
	else{
		if(abs(gimbal->info.yaw_mec_err_raw) <= PI/2)
		{
			gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[FRONT];
		}
		else{
			gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[BEHIND];
		}
	}
	*/
	
	gimbal->target.pitch_mec_tar = gimbal->info.pitch_mec;
}



static void Gimbal_Offline_Update(Gimbal_t* gimbal)
{
	
}

/**
 * @brief  云台失联处理
 * @note   
 */
static void Gimbal_Offline_Process(Gimbal_t* gimbal)
{
	gimbal->gimbal_reset_flag = false;
	reset_tick = 0;
	
	//机械目标值回正
	gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
	gimbal->target.yaw_imu_tar = gimbal->info.yaw_imu;
	gimbal->target.pitch_imu_tar = 0;

}

/**
 * @brief  云台命令发送
 * @note   更新到板间
 */
static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal)
{
	// TODO: board_protocol / infantry 尚未迁移，待移植后取消注释
	/*
  board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = gimbal->target.yaw_mec_tar;
	board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = gimbal->target.yaw_imu_tar;
	board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = gimbal->target.pitch_mec_tar;
	board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = gimbal->target.pitch_imu_tar;
	
	if(infantry.flag.U_turn_flag.value == true || infantry.flag.R_turn_flag.value == true || infantry.flag.L_turn_flag.value == true)
	{
		gimbal->info.yaw_mec_err_act = 0;
	}
	else{
		gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_mec - gimbal->target.yaw_mec_tar,2*PI);
	}
	
	if(gimbal->mode == G_SLEEP || gimbal->mode == G_INIT || gimbal->mode == G_SLAVE)
	{
		board.tx_pkt->car_pkt.gimbal_mode = 0;
	}
	else{
	  board.tx_pkt->car_pkt.gimbal_mode = 1;
	}
	
	if(infantry.mode == I_INIT){
	}
	else if(infantry.flag.hole_flag == false)
	{
		board.tx_pkt->gimbal_target_pkt.is_hole = false;
	}
	else if(infantry.flag.chassis_reset.value == true)
	{
		board.tx_pkt->gimbal_target_pkt.is_hole = false;
	}	
	else{
	   board.tx_pkt->gimbal_target_pkt.is_hole = true; 
	} 
	*/
}


static void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Data_Update(gimbal);
	Gimbal_Offline_Update(gimbal);
	Gimbal_Status_Update(gimbal);
	switch (gimbal->mode)
	{
		case G_SLEEP:
			Gimbal_Offline_Process(gimbal);
			break;
		
		case G_INIT:
			Gimbal_Init_Process(gimbal);
		  break;

		case G_SLAVE:
			Gimbal_Slave_Update(gimbal);
		  break;
		
		case G_BOSS:
			Gimbal_Boss_Update(gimbal);
			break;
	
	  default:
			break;
	}
	
	Gimbal_Cmd_Transmit(gimbal);

}

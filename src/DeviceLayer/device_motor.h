#pragma once

#include "RM_motor.h"
#include "DM_Motor.h"

#ifdef CONFIG_CAN_TX_MANAGER
#include <drivers/can_tx_manager.h>
#endif
#ifdef CONFIG_CAN_RX_MANAGER
#include <drivers/can_rx_manager.h>
#endif


#define MOTOR_3508_CURRENT_MAX 10000

#define FRIC_R_NODE DT_NODELABEL(fric_r)
#define FRIC_L_NODE DT_NODELABEL(fric_l)
#define GIMBAL_YAW_NODE DT_NODELABEL(gimbal_yaw)
#define GIMBAL_PITCH_NODE DT_NODELABEL(gimbal_pitch)

#ifdef CONFIG_CAN_RX_MANAGER
#define RX_MANAGER1_NODE DT_NODELABEL(can_rx_mgr1)
#define RX_MANAGER2_NODE DT_NODELABEL(can_rx_mgr2)
#endif

typedef enum
{
    FRIC_L = 0,
    FRIC_R,
    FRIC_CNT,
} Fric_List_e;

typedef enum
{
    YAW = 0,
    PITCH,
    GIMBAL_CNT,
}Gimbal_List_e;

#ifdef __cplusplus
extern "C"
{
#endif
    extern Motor_RM_t fric_motor[FRIC_CNT];
    extern Motor_DM_t gimbal_motor[GIMBAL_CNT];

    int Motor_Init(void);
    void Motor_Heartbeat(void);
#ifdef __cplusplus
}
#endif
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

#define CHASSIS_LF_NODE DT_NODELABEL(chassis_lf)
#define CHASSIS_LB_NODE DT_NODELABEL(chassis_lb)
#define CHASSIS_RF_NODE DT_NODELABEL(chassis_rf)
#define CHASSIS_RB_NODE DT_NODELABEL(chassis_rb)
#define GIMBAL_YAW_NODE DT_NODELABEL(gimbal_yaw)

// #define DIAL_MOTOR_NODE DT_NODELABEL(lk_motor1)

#ifdef CONFIG_CAN_RX_MANAGER
#define RX_MANAGER_NODE DT_NODELABEL(can_rx_mgr1)
#endif

typedef enum
{
    WHEEL_LF = 0,
    WHEEL_LB,
    WHEEL_RF,
    WHEEL_RB,
    WHEEL_CNT,
} Wheel_List_e;

typedef enum
{
    YAW = 0,
    GIMBAL_CNT,
}
Gimbal_List_e;

#ifdef __cplusplus
extern "C"
{
#endif
    extern Motor_RM_t wheel_motor[WHEEL_CNT];
    extern Motor_DM_t gimbal_motor[GIMBAL_CNT];

    int Motor_Init(void);
    void Motor_Heartbeat(void);
#ifdef __cplusplus
}
#endif
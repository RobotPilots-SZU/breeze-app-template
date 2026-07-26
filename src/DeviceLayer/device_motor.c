#include "device_motor.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

pid_ctrl_t rm_speed_pid[RM_CNT] = {
    [FRIC_L] = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integral_max = 7000.0f,
        .out_max = 10000.0f,

    },
    [FRIC_R] = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integral_max = 7000.0f,
        .out_max = 10000.0f,
    },
    [LIFT] = {
        .kp = 8,
        .ki = 0,
        .kd = 0,
        .integral_max = 0.0f,
        .out_max = 300.0f,
    },

};
pid_ctrl_t rm_angle_inn_pid[RM_CNT] = {
    [LIFT] = {
        .kp = 40.0f,
        .ki = 0,
        .kd = 0,
        .integral_max = 0.0f,
        .out_max = 1000.0f,
    },
};

pid_ctrl_t rm_angle_out_pid[RM_CNT] = {
    [LIFT] = {
        .kp = 8.0f,
        .ki = 0,
        .kd = 0,
        .integral_max = 0.0f,
        .out_max = 1000.0f,
    },
};

Motor_RM_Rx_Info_t rm_rx_info[RM_CNT];
Motor_RM_Tx_Info_t rm_tx_info[RM_CNT];
Motor_RM_State_t rm_state[RM_CNT];
Motor_RM_Ctrl_Info_t rm_ctrl_info[RM_CNT] = {
    [FRIC_L] = {
        .angle_ctrl_inner = &rm_angle_inn_pid[FRIC_L],
        .angle_ctrl_outer = &rm_angle_out_pid[FRIC_L],
        .speed_ctrl = &rm_speed_pid[FRIC_L],
    },
    [FRIC_R] = {
        .angle_ctrl_inner = &rm_angle_inn_pid[FRIC_R],
        .angle_ctrl_outer = &rm_angle_out_pid[FRIC_R],
        .speed_ctrl = &rm_speed_pid[FRIC_R],
    },
    [LIFT] = {
        .angle_ctrl_inner = &rm_angle_inn_pid[LIFT],
        .angle_ctrl_outer = &rm_angle_out_pid[LIFT],
        .speed_ctrl = &rm_speed_pid[LIFT],
    },
};

Motor_RM_t rm_motor[RM_CNT] = {
    [FRIC_L] = {
        .rx_info = &rm_rx_info[FRIC_L],
        .tx_info = &rm_tx_info[FRIC_L],
        .state = &rm_state[FRIC_L],
        .ctrl = &rm_ctrl_info[FRIC_L],
        .motor = DEVICE_DT_GET(FRIC_L_NODE),
        .single_init = RM_Motor_Init,
        .type = _3508_Single,
    },
    [FRIC_R] = {
        .rx_info = &rm_rx_info[FRIC_R],
        .tx_info = &rm_tx_info[FRIC_R],
        .state = &rm_state[FRIC_R],
        .ctrl = &rm_ctrl_info[FRIC_R],
        .motor = DEVICE_DT_GET(FRIC_R_NODE),
        .single_init = RM_Motor_Init,
        .type = _3508_Single,
    },
    [LIFT] = {
        .rx_info = &rm_rx_info[LIFT],
        .tx_info = &rm_tx_info[LIFT],
        .state = &rm_state[LIFT],
        .ctrl = &rm_ctrl_info[LIFT],
        .motor = DEVICE_DT_GET(LIFT_NODE),
        .single_init = RM_Motor_Init,
        .type = _2006_Reduction,
    },
};

//DM_MOTOR-----------------------------------------------------------
//pidInfo被放在了gimbal里
Motor_DM_Ctrl_Info_t Yaw_Ctrl;
Motor_DM_Rx_Info_t Yaw_Rx_Info;
Motor_DM_Tx_Info_t Yaw_Tx_Info;
Motor_DM_State_t Yaw_State;

Motor_DM_Rx_Info_t Pitch_Rx_Info;
Motor_DM_Tx_Info_t Pitch_Tx_Info;
Motor_DM_State_t Pitch_State;
Motor_DM_Ctrl_Info_t Pitch_Ctrl;

Motor_DM_t gimbal_motor[GIMBAL_CNT] =
{
    [YAW] =
    {
        .rx_info = &Yaw_Rx_Info,
        .tx_info = &Yaw_Tx_Info,
        .state = &Yaw_State,
        .ctrl = &Yaw_Ctrl,
        .motor = DEVICE_DT_GET(GIMBAL_YAW_NODE),
        .single_init = &DM_Single_Motor_Init,
        .type = dm_4310,
    },

    [PITCH] =
    {
        .rx_info = &Pitch_Rx_Info,
        .tx_info = &Pitch_Tx_Info,
        .state = &Pitch_State,
        .ctrl = &Pitch_Ctrl,
        .motor = DEVICE_DT_GET(GIMBAL_PITCH_NODE),
        .single_init = &DM_Single_Motor_Init,
        .type = dm_4310,
    },
};

//MOTOR_INIT----------------------------------------------------
int Motor_Init()
{
#ifdef CONFIG_CAN_RX_MANAGER
    const struct device *rx_mgr1 = DEVICE_DT_GET(RX_MANAGER1_NODE);
    const struct device *rx_mgr2 = DEVICE_DT_GET(RX_MANAGER2_NODE);
#endif

    for (int i = 0; i < RM_CNT; i++)
    {
        if (!rm_motor[i].motor)
        {
            LOG_ERR("FRIC Motor %d not found!", i);
            return -ENODEV;
        }
        if (!device_is_ready(rm_motor[i].motor))
        {
            LOG_ERR("FRIC Motor %d is not ready!", i);
            return -ENODEV;
        }
        rm_motor[i].single_init(&rm_motor[i]);
    }

    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        if (!gimbal_motor[i].motor)
        {
            LOG_ERR("Motor %d not found!", i);
            return -ENODEV;
        }
        if (!device_is_ready(gimbal_motor[i].motor))
        {
            LOG_ERR("Motor %d is not ready!", i);
            return -ENODEV;
        }
        gimbal_motor[i].single_init(&gimbal_motor[i]);
    }

#ifdef CONFIG_CAN_RX_MANAGER
    if (!rx_mgr1)
    {
        LOG_ERR("CAN RX manager not found");
        return -ENODEV;
    }
    if (!device_is_ready(rx_mgr1))
    {
        LOG_ERR("CAN RX manager not ready: %s", rx_mgr1->name);
        return -ENODEV;
    }
    if (!rx_mgr2)
    {
        LOG_ERR("CAN RX manager not found");
        return -ENODEV;
    }
    if (!device_is_ready(rx_mgr2))
    {
        LOG_ERR("CAN RX manager not ready: %s", rx_mgr2->name);
        return -ENODEV;
    }
#endif



    for (int i = 0; i < RM_CNT; i++)
    {
        if (register_motor(rm_motor[i].motor) < 0) // 这是通用的多电机注册函数
        {
            LOG_ERR("Failed to register motor %d", i);
            return -ENODEV;
        }
    }
    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        if (register_motor(gimbal_motor[i].motor) < 0) // 这是通用的多电机注册函数
        {
            LOG_ERR("Failed to register motor %d", i);
            return -ENODEV;
        }
    }
    // if(register_motor(dail_motor.lk_motor) < 0)
    // {
    //     LOG_ERR("Failed to register dial motor");
    //     return -ENODEV;
    // }
    

    for (int i = 0; i < RM_CNT; i++)
    {
        rm_motor[i].single_sleep(&rm_motor[i]); // 内部使用通用的多电机扭矩控制函数
    }

    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        gimbal_motor[i].single_sleep(&gimbal_motor[i]);
    }
    // motor_disable(dail_motor.lk_motor);  // 上电卸力
    

    // dail_motor.init(&dail_motor);
    

    return 0;
}

void Motor_Heartbeat()
{
    for (int i = 0; i < RM_CNT; i++)
    {
        rm_motor[i].single_heart_beat(&rm_motor[i]);
    }

    for (int i = 0; i < GIMBAL_CNT; i++)
    {
        gimbal_motor[i].single_heart_beat(&gimbal_motor[i]);
    }
}
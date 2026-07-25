#include "device_motor.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

//RM_motor
pid_ctrl_t wheel_speed_pid[WHEEL_CNT] = {
    [WHEEL_LF] = {
        .kp = 1,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 5.4,

    },
    [WHEEL_LB] = {
        .kp = 1,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 5.4,

    },
    [WHEEL_RF] = {
        .kp = 1,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 5.4,

    },
    [WHEEL_RB] = {
        .kp = 1,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 5.4,

    },

};

pid_ctrl_t wheel_angle_inn_pid[WHEEL_CNT] = {
    [WHEEL_LF] = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 0,

    },
    [WHEEL_LB] = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 0,

    },
    [WHEEL_RF] = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 0,

    },
    [WHEEL_RB] = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 0,

    },

};

pid_ctrl_t wheel_angle_out_pid[WHEEL_CNT] = {
    [WHEEL_LF] = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 3.8,

    },
    [WHEEL_LB] = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 3.8,

    },
    [WHEEL_RF] = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 3.8,

    },
    [WHEEL_RB] = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integral_max = 0,
        .out_max = 3.8,

    },

};

Motor_RM_Rx_Info_t wheel_rx_info[WHEEL_CNT];
Motor_RM_Tx_Info_t wheel_tx_info[WHEEL_CNT];
Motor_RM_State_t wheel_state[WHEEL_CNT];
Motor_RM_Ctrl_Info_t wheel_ctrl_info[WHEEL_CNT] = {
    [WHEEL_LF] = {
        .angle_ctrl_inner = &wheel_angle_inn_pid[WHEEL_LF],
        .angle_ctrl_outer = &wheel_angle_out_pid[WHEEL_LF],
        .speed_ctrl = &wheel_speed_pid[WHEEL_LF],
    },
    [WHEEL_LB] = {
        .angle_ctrl_inner = &wheel_angle_inn_pid[WHEEL_LB],
        .angle_ctrl_outer = &wheel_angle_out_pid[WHEEL_LB],
        .speed_ctrl = &wheel_speed_pid[WHEEL_LB],
    },
    [WHEEL_RF] = {
        .angle_ctrl_inner = &wheel_angle_inn_pid[WHEEL_RF],
        .angle_ctrl_outer = &wheel_angle_out_pid[WHEEL_RF],
        .speed_ctrl = &wheel_speed_pid[WHEEL_RF],
    },
    [WHEEL_RB] = {
        .angle_ctrl_inner = &wheel_angle_inn_pid[WHEEL_RB],
        .angle_ctrl_outer = &wheel_angle_out_pid[WHEEL_RB],
        .speed_ctrl = &wheel_speed_pid[WHEEL_RB],
    },

};

Motor_RM_t wheel_motor[WHEEL_CNT] = {
    [WHEEL_LF] = {
        .rx_info = &wheel_rx_info[WHEEL_LF],
        .tx_info = &wheel_tx_info[WHEEL_LF],
        .state   = &wheel_state[WHEEL_LF],
        .ctrl = &wheel_ctrl_info[WHEEL_LF],
        .motor = DEVICE_DT_GET(CHASSIS_LF_NODE),
        .single_init = RM_Motor_Init,
    },
    [WHEEL_LB] = {
        .rx_info = &wheel_rx_info[WHEEL_LB],
        .tx_info = &wheel_tx_info[WHEEL_LB],
        .state   = &wheel_state[WHEEL_LB],
        .ctrl = &wheel_ctrl_info[WHEEL_LB],
        .motor = DEVICE_DT_GET(CHASSIS_LB_NODE),
        .single_init = RM_Motor_Init,
    },
    [WHEEL_RF] = {
        .rx_info = &wheel_rx_info[WHEEL_RF],
        .tx_info = &wheel_tx_info[WHEEL_RF],
        .state   = &wheel_state[WHEEL_RF],
        .ctrl = &wheel_ctrl_info[WHEEL_RF],
        .motor = DEVICE_DT_GET(CHASSIS_RF_NODE),
        .single_init = RM_Motor_Init,
    },
    [WHEEL_RB] = {
        .rx_info = &wheel_rx_info[WHEEL_RB],
        .tx_info = &wheel_tx_info[WHEEL_RB],
        .state   = &wheel_state[WHEEL_RB],
        .ctrl = &wheel_ctrl_info[WHEEL_RB],
        .motor = DEVICE_DT_GET(CHASSIS_RB_NODE),
        .single_init = RM_Motor_Init,
    },

};

//LK_motor
/*LK4005 START*/
// #ifdef DIAL_PID

// pid_ctrl_t dail_speed =
//     {
//         .kp = 0.15f,
//         .ki = 0.0f,
//         .kd = 0.f,
//         .integral_max = 500.f,
//         .out_max = 1000.f,
// };
// pid_ctrl_t dail_position_out =
//     {
//         .kp = 0.10,
//         .ki = 0.f,
//         .kd = 0.f,
//         .integral_max = 0.f,
//         .out_max = 1000000.f,
// };
// pid_ctrl_t dail_position_inner =
//     {
//         .kp = 0.06f,
//         .ki = 0.f,
//         .kd = 0.f,
//         .integral_max = 0.f,
//         .out_max = 1000.f,
// };

// #else

// pid_ctrl_t dail_speed =
//     {
//         .kp = 0.15f,
//         .ki = 0.2f,
//         .kd = 0.f,
//         .integral_max = 500.f,
//         .out_max = 1000.f,
// };
// pid_ctrl_t dail_position_out =
//     {
//         .kp = 0.20f,
//         .ki = 0.f,
//         .kd = 0.f,
//         .integral_max = 0.f,
//         .out_max = 1000000.f,
// };
// pid_ctrl_t dail_position_inner =
//     {
//         .kp = 0.1f,
//         .ki = 0.f,
//         .kd = 0.f,
//         .integral_max = 0.f,
//         .out_max = 1000.f,
// };
// #endif

// dail_pid_info_t dail_pid = {
//     .speed = &dail_speed,
//     .position_inner = &dail_position_inner,
//     .position_outer = &dail_position_out,
// };

// KT_motor_t dail_motor = {
//     .lk_motor = DEVICE_DT_GET(DIAL_MOTOR_NODE),
//     .KT_motor_info = {
//         .id = {
//             .tx_id = ID_DIAL,
//             .rx_id = ID_DIAL,
//             .drive_type = M_CAN1,
//             .motor_type = KT4005,
//         },
//         .tx_info = {
//             .angle_single_Control = 0,
//             .angle_single_Control_maxSpeed = 0,
//             .angle_single_Control_spinDirection = 0,
//             .angle_add_Control = 0,
//             .angle_add_Control_maxSpeed = 0,
//             .angle_sum_Control = 0,
//             .angle_sum_Control_maxSpeed = 0,
//             .iqControl = 0,
//             .speedControl = 0,
//         },
//     },
//     .init = KT_motor_class_init,
// };


int Motor_Init()
{
#ifdef CONFIG_CAN_RX_MANAGER
    const struct device *rx_mgr = DEVICE_DT_GET(RX_MANAGER_NODE);
#endif

    for (int i = 0; i < WHEEL_CNT; i++)
    {
        if (!wheel_motor[i].motor)
        {
            LOG_ERR("Motor %d not found!", i);
            return -ENODEV;
        }
        if (!device_is_ready(wheel_motor[i].motor))
        {
            LOG_ERR("Motor %d is not ready!", i);
            return -ENODEV;
        }
        wheel_motor[i].single_init(&wheel_motor[i]);
    }

    // if (!dail_motor.lk_motor) {
    //     LOG_ERR("Dial motor device not found!");
    //     return -ENODEV;
    // }
    // if (!device_is_ready(dail_motor.lk_motor)) {
    //     LOG_ERR("Dial motor not ready!");
    //     return -ENODEV;
    // }

    

#ifdef CONFIG_CAN_RX_MANAGER
    if (!rx_mgr)
    {
        LOG_ERR("CAN RX manager not found");
        return -ENODEV;
    }
    if (!device_is_ready(rx_mgr))
    {
        LOG_ERR("CAN RX manager not ready: %s", rx_mgr->name);
        return -ENODEV;
    }
#endif



    for (int i = 0; i < WHEEL_CNT; i++)
    {
        if (register_motor(wheel_motor[i].motor) < 0) // 这是通用的多电机注册函数
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
    

    for (int i = 0; i < WHEEL_CNT; i++)
    {
        wheel_motor[i].single_sleep(&wheel_motor[i]); // 内部使用通用的多电机扭矩控制函数
    }
    // motor_disable(dail_motor.lk_motor);  // 上电卸力
    

    // dail_motor.init(&dail_motor);
    

    return 0;
}

void Motor_Heartbeat()
{
    for (int i = 0; i < WHEEL_CNT; i++)
    {
        wheel_motor[i].single_heart_beat(&wheel_motor[i]);
    }
}
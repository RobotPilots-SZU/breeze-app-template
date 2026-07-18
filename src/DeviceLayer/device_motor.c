#include "device_motor.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

<<<<<<< HEAD
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
        .ctrl = &wheel_ctrl_info[WHEEL_LF],
        .motor = DEVICE_DT_GET(CHASSIS_LF_NODE),
        .single_init = RM_Motor_Init,
    },
    [WHEEL_LB] = {
        .rx_info = &wheel_rx_info[WHEEL_LB],
        .tx_info = &wheel_tx_info[WHEEL_LB],
        .ctrl = &wheel_ctrl_info[WHEEL_LB],
        .motor = DEVICE_DT_GET(CHASSIS_LB_NODE),
        .single_init = RM_Motor_Init,
    },
    [WHEEL_RF] = {
        .rx_info = &wheel_rx_info[WHEEL_RF],
        .tx_info = &wheel_tx_info[WHEEL_RF],
        .ctrl = &wheel_ctrl_info[WHEEL_RF],
        .motor = DEVICE_DT_GET(CHASSIS_RF_NODE),
        .single_init = RM_Motor_Init,
    },
    [WHEEL_RB] = {
        .rx_info = &wheel_rx_info[WHEEL_RB],
        .tx_info = &wheel_tx_info[WHEEL_RB],
        .ctrl = &wheel_ctrl_info[WHEEL_RB],
        .motor = DEVICE_DT_GET(CHASSIS_RB_NODE),
        .single_init = RM_Motor_Init,
    },

};

int Motor_Init()
{
=======
#if defined(__cplusplus)
extern "C" {
#endif

int Motor_Init()
{
    const struct device *motor_lf = DEVICE_DT_GET(CHASSIS_LF_NODE);
    const struct device *motor_rf = DEVICE_DT_GET(CHASSIS_RF_NODE);
    const struct device *motor_lb = DEVICE_DT_GET(CHASSIS_LB_NODE);
    const struct device *motor_rb = DEVICE_DT_GET(CHASSIS_RB_NODE);
>>>>>>> origin/fhw_test
#ifdef CONFIG_CAN_RX_MANAGER
    const struct device *rx_mgr = DEVICE_DT_GET(RX_MANAGER_NODE);
#endif

<<<<<<< HEAD
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

    
=======
    if (!motor_lf)
    {
        LOG_ERR("motor LF not found");
        return -ENODEV;
    }
    if (!motor_rf)
    {
        LOG_ERR("motor RF not found");
        return -ENODEV;
    }
    if (!motor_lb)
    {
        LOG_ERR("motor LB not found");
        return -ENODEV;
    }
    if (!motor_rb)
    {
        LOG_ERR("motor RB not found");
        return -ENODEV;
    }

    if (!device_is_ready(motor_lf))
    {
        LOG_ERR("motor LF not ready: %s", motor_lf->name);
        return -ENODEV;
    }
    if (!device_is_ready(motor_rf))
    {
        LOG_ERR("motor RF not ready: %s", motor_rf->name);
        return -ENODEV;
    }
    if (!device_is_ready(motor_lb))
    {
        LOG_ERR("motor LB not ready: %s", motor_lb->name);
        return -ENODEV;
    }
    if (!device_is_ready(motor_rb))
    {
        LOG_ERR("motor RB not ready: %s", motor_rb->name);
        return -ENODEV;
    }
>>>>>>> origin/fhw_test

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

<<<<<<< HEAD


    for (int i = 0; i < WHEEL_CNT; i++)
    {
        register_motor(wheel_motor[i].motor);// 这是通用的多电机注册函数
    }

    for (int i = 0; i < WHEEL_CNT; i++)
    {
        wheel_motor[i].single_sleep(); // 内部使用通用的多电机扭矩控制函数
    }

    return 0;
}
=======
    register_motor(motor_lf);
    register_motor(motor_rf);
    register_motor(motor_lb);
    register_motor(motor_rb);

    motor_torque_control(motor_lf, 0); // 这是通用的多电机扭矩控制函数
    motor_torque_control(motor_rf, 0);
    motor_torque_control(motor_lb, 0);
    motor_torque_control(motor_rb, 0);

    return 0;
}

#if defined(__cplusplus)
}
#endif
>>>>>>> origin/fhw_test

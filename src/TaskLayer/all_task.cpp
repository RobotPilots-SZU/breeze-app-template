#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "conf_task.hpp"
#include "device.hpp"				// 设备
#include "board_protocol.h"

LOG_MODULE_REGISTER(all_task, LOG_LEVEL_INF);

extern "C" void StartSystemUpdateTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("System Task started");
    while (true)
    {
        Send_To_Down_Board();
        k_sleep(K_MSEC(1));
    }
}

extern "C" void StartUpdateTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Update Task started");
    while (true)
    {
        breeze::Imu_Process();
        k_sleep(K_MSEC(1));
    }
}

extern "C" void StartHeartbeatTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Heartbeat Task started");
    while (true)
    {
        Motor_Heartbeat();
        C_Board_Communicate_HeartBeat();
        
        /* 喂狗 */
        IWDG_Feed(iwdg_dev, IWDG_Channel_ID);
        
        k_sleep(K_MSEC(100));
    }
}

// VOFA+ JustFloat模式 帧尾
static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

extern "C" void StartMonitorTask(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Monitor Task started");
    while (true)
    {

        
		// 发送数据
        float vofa_data[] = {
            (float)k_uptime_get_32() / 1000.0f,
            // (float)wheel_motor[0].ctrl->speed_ctrl->target,
            // (float)wheel_motor[0].ctrl->speed_ctrl->err,
            // (float)wheel_motor[0].tx_info->torque,
            // (float)wheel_motor[1].ctrl->speed_ctrl->target,
            // (float)wheel_motor[1].ctrl->speed_ctrl->err,
            // (float)wheel_motor[1].tx_info->torque,
        };
        SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_data, sizeof(vofa_data));
        SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_tail, sizeof(vofa_tail));

        k_sleep(K_MSEC(4));
    }
}

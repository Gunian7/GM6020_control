/**
 * @file gm6020_task.c
 * @brief GM6020 电机控制任务实现（FreeRTOS）
 * 
 *   1. osDelay(1) 等待 1ms
 *   2. 根据控制模式选择控制算法
 *   3. 发送 CAN 控制帧
 * 
 * CAN 接收中断处理反馈数据。
 */
#include "gm6020_task.h"
#include "cmsis_os.h"

/* 引用 main.c 中定义的 CAN 句柄 */
extern CAN_HandleTypeDef hcan1;

/* 全局电机控制数组 - 用户在 main.c 中定义 */
gm6020_control_t g_gm6020_motors[GM6020_MAX_MOTORS];

/**
 * @brief CAN 接收中断回调
 * 
 * 在 HAL_CAN_RxFifo0MsgPendingCallback 中处理 GM6020 反馈数据。
 * 需要确保在 CubeMX 中使能了 CAN1 的接收中断。
 * 
 * 开发板 C 型 CAN1 引脚: PD0(CAN1_RX), PD1(CAN1_TX)
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint8_t i;
    
    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            return;
        }
        
        /* 遍历所有电机，找到匹配的反馈 ID */
        for (i = 0; i < GM6020_MAX_MOTORS; i++)
        {
            if (rx_header.StdId == g_gm6020_motors[i].feedback_id)
            {
                GM6020_FeedbackProcess(&g_gm6020_motors[i], &rx_header, rx_data);
                break;
            }
        }
    }
}

/**
 * @brief GM6020 电机控制主任务
 * 
 * 在 FreeRTOS 中以 1ms 周期循环运行。
 * 根据每个电机配置的控制模式，执行对应的控制算法。
 * 
 * 使用示例：
 * 
 *   // 在 main() 中创建任务:
 *   osThreadDef(gm6020_task, GM6020_ControlTask, osPriorityNormal, 0, 512);
 *   osThreadCreate(osThread(gm6020_task), NULL);
 * 
 *   或者直接在 FreeRTOS 中:
 *   xTaskCreate(GM6020_ControlTask, "gm6020_task", 512, NULL, 2, NULL);
 */
void GM6020_ControlTask(void *argument)
{
    uint8_t i;
    
    /* 初始化所有电机（示例：初始化4个，ID 1~4） */
    GM6020_InitAll(g_gm6020_motors, GM6020_MAX_MOTORS);
    
    /* 等待 CAN 初始化完成 */
    osDelay(100);

    /* 注意：CAN 启动已在 main.c 中完成，此处不再重复调用 HAL_CAN_Start */

    /* === 初始状态：所有电机不转，等待 Ozone 设置目标 === */
    /* 用 Ozone 修改变量的示例：
     *   电机1位置模式: g_gm6020_motors[0].target_angle_deg = 90.0;
     *                   g_gm6020_motors[0].control_mode = 2;
     *   电机1速度模式: g_gm6020_motors[0].target_speed_rpm = 50.0f;
     *                   g_gm6020_motors[0].control_mode = 1;
     *   查看当前角度:   g_gm6020_motors[0].feedback.total_angle_deg
     *   查看当前转速:   g_gm6020_motors[0].feedback.speed_rpm
     */
    
    /* === 主循环 === */
    for (;;)
    {
        /* 1ms 控制周期 */
        osDelay(GM6020_CONTROL_CYCLE_MS);
        
        /* 对每个电机执行控制算法 */
        for (i = 0; i < GM6020_MAX_MOTORS; i++)
        {
            switch (g_gm6020_motors[i].control_mode)
            {
                case 0:  /* 电压开环 */
                    GM6020_VoltageControl(&g_gm6020_motors[i]);
                    break;
                    
                case 1:  /* 速度环 */
                    GM6020_SpeedControl(&g_gm6020_motors[i]);
                    break;
                    
                case 2:  /* 位置环（串级：角度环+速度环） */
                    GM6020_PositionControl(&g_gm6020_motors[i]);
                    break;
                    
                default:
                    break;
            }
        }
        
        /* 通过 CAN 发送控制指令（电压模式） */
        GM6020_SendVoltageCmd(g_gm6020_motors, GM6020_MAX_MOTORS);
    }
}

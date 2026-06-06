/**
 * @file gm6020.c
 * @brief GM6020 电机驱动实现
 * 
 * 实现速度环、位置环（串级PID）控制。
 * 基于 RoboMaster 开发板 C 型教程中 PID 框架和第19章串级PID控制思想。
 */
#include "gm6020.h"
#include "string.h"

/* 引用 main.c 中定义的 CAN 句柄 */
extern CAN_HandleTypeDef hcan1;

/* ======================== 初始化 ======================== */

/**
 * @brief 初始化单个 GM6020 电机控制结构体
 * @param motor    电机控制结构体指针
 * @param motor_id 拨码开关设置的电机 ID (1~7)
 */
void GM6020_Init(gm6020_control_t *motor, uint8_t motor_id)
{
    if (motor == NULL) return;
    if (motor_id < 1 || motor_id > 7) return;
    
    /* 清空结构体 */
    memset(motor, 0, sizeof(gm6020_control_t));
    
    /* 设置 ID */
    motor->motor_id    = motor_id;
    motor->feedback_id = 0x204 + motor_id;  /* 反馈报文 ID */
    
    /* 默认模式：速度环 */
    motor->control_mode = 1;
    
    /* ---------- 速度环 PID 初始化 ---------- */
    /* 
     * 速度环 PID 参数（需根据实际调试调整）:
     * 对于 GM6020, 速度范围 0~320rpm
     * 输出电压范围 -25000~25000
     * 建议从较小的 Kp 开始调试
     */
    {
        fp32 speed_pid_param[3] = {15.0f, 0.5f, 0.0f};  /* Kp, Ki, Kd */
        PID_Init(&motor->speed_pid, PID_POSITION,
                 speed_pid_param,
                 25000.0f,   /* max_out: 对应 GM6020 电压最大输出 */
                 5000.0f);   /* max_iout */
    }
    
    /* ---------- 角度环 PID 初始化 ---------- */
    /*
     * 角度环 PID 参数（需根据实际调试调整）:
     * 角度范围 0~360°
     * 外环输出是内环的速度设定值，范围 0~320rpm
     */
    {
        fp32 angle_pid_param[3] = {8.0f, 0.0f, 0.0f};   /* Kp, Ki, Kd */
        PID_Init(&motor->angle_pid, PID_POSITION,
                 angle_pid_param,
                 300.0f,    /* max_out: 最大输出转速 (rpm) */
                 100.0f);   /* max_iout */
    }
}

/**
 * @brief 批量初始化多个电机
 */
void GM6020_InitAll(gm6020_control_t *motors, uint8_t count)
{
    uint8_t i;
    for (i = 0; i < count; i++)
    {
        GM6020_Init(&motors[i], i + 1);
    }
}

/* ======================== 反馈处理 ======================== */

/**
 * @brief 处理 CAN 接收到的 GM6020 反馈报文
 * 
 * 反馈报文格式（来自 GM6020 文档）:
 *   ID = 0x204 + 拨码ID
 *   DATA[0:1] = 机械角度 (0~8191, 14位)
 *   DATA[2:3] = 转速 (rpm, int16)
 *   DATA[4:5] = 实际转矩电流 (int16)
 *   DATA[6]   = 电机温度 (℃)
 *   DATA[7]   = 保留
 * 
 * @param motor     电机控制结构体指针
 * @param rx_header CAN 接收头
 * @param rx_data   接收数据 (8字节)
 */
void GM6020_FeedbackProcess(gm6020_control_t *motor, CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    if (motor == NULL || rx_data == NULL) return;
    
    /* 校验 ID */
    if (rx_header->StdId != motor->feedback_id) return;
    
    gm6020_feedback_t *fb = &motor->feedback;
    
    /* 解析原始数据 */
    fb->angle_raw   = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
    fb->speed_rpm   = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    fb->current_raw = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    fb->temperature = rx_data[6];
    
    /* 角度转换: 原始值 0~8191 → 0~360° */
    fb->angle_deg = (fp32)fb->angle_raw * 360.0f / GM6020_ANGLE_MAX;
    fb->angle_rad = fb->angle_deg * 3.14159265f / 180.0f;
    
    /* 电流转换: 原始值 → A (仅在电流模式下有效) */
    fb->current_A = (fp32)fb->current_raw * 3.0f / GM6020_CURRENT_MAX;
    
    /* ---------- 角度解缠绕（多圈累计） ---------- */
    /* 
     * GM6020 反馈角度是 0~8191 (对应 0~360°) 的循环值。
     * 要追踪多圈位置，需要检测过零翻转。
     */
    int16_t diff = (int16_t)(fb->angle_raw - fb->last_angle_raw);
    
    if (diff > GM6020_ANGLE_MAX / 2)
    {
        /* 正向过零: 角度从大跳小 */
        motor->feedback.round_count--;
    }
    else if (diff < -(GM6020_ANGLE_MAX / 2))
    {
        /* 反向过零: 角度从小跳大 */
        motor->feedback.round_count++;
    }
    
    motor->feedback.last_angle_raw = fb->angle_raw;
    
    /* 计算累积角度 */
    motor->feedback.total_angle_deg = (fp32)motor->feedback.round_count * 360.0f + fb->angle_deg;
}

/* ======================== 控制算法 ======================== */

/**
 * @brief 速度环控制（单环）
 * 
 * 输入：motor->target_speed_rpm（期望转速）
 * 反馈：motor->feedback.speed_rpm（实际转速）
 * 输出：motor->current_set（电压控制量, 发送给电机）
 * 
 * 控制链路:
 *   目标转速 → [速度PID] → 电压值 → CAN发送到电机
 */
void GM6020_SpeedControl(gm6020_control_t *motor)
{
    if (motor == NULL) return;
    
    fp32 speed_ref  = (fp32)motor->feedback.speed_rpm;
    fp32 speed_set  = motor->target_speed_rpm;
    
    /* 速度 PID 计算：输出为电压控制量 (-25000~25000) */
    fp32 out = PID_Calc(&motor->speed_pid, speed_ref, speed_set);
    
    motor->current_set = (int16_t)out;
}

/**
 * @brief 位置环控制（串级 PID: 角度环 + 速度环）
 * 
 * 这是云台/关节位置控制的典型串级结构：
 * 
 *   目标角度 → [角度PID] → 目标速度 → [速度PID] → 电压值 → CAN
 *                  ↑                        ↑
 *             角度反馈                  速度反馈
 * 
 * 外环（角度环）:
 *   输入：期望角度 motor->target_angle_deg
 *   反馈：电机当前角度 motor->feedback.total_angle_deg（解缠绕后的累计角度）
 *   输出：期望角速度，作为内环的设定值
 * 
 * 内环（速度环）:
 *   输入：角度环输出的期望角速度
 *   反馈：电机当前转速 motor->feedback.speed_rpm
 *   输出：电压控制量，通过 CAN 发送
 */
void GM6020_PositionControl(gm6020_control_t *motor)
{
    if (motor == NULL) return;
    
    /* ---- 外环：角度环 ---- */
    /* 角度环输入：期望角度 vs 实际累积角度 */
    /* 注意：如果控制范围在单圈内，可以用 angle_deg；
     *       如果需要多圈定位，使用 total_angle_deg */
    fp32 angle_ref = motor->feedback.total_angle_deg;  /* 实际位置（度） */
    fp32 angle_set = motor->target_angle_deg;          /* 目标位置（度） */
    
    /* 角度环输出 = 期望角速度 (rpm) */
    fp32 speed_set = PID_Calc(&motor->angle_pid, angle_ref, angle_set);
    
    /* 保存用于调试 */
    motor->speed_set = speed_set;
    
    /* ---- 内环：速度环 ---- */
    /* 速度环输入：角度环输出的角速度设定值 vs 实际转速 */
    fp32 speed_ref = (fp32)motor->feedback.speed_rpm;  /* 实际转速（rpm） */
    
    /* 速度环输出 = 电压控制量 */
    fp32 out = PID_Calc(&motor->speed_pid, speed_ref, speed_set);
    
    motor->current_set = (int16_t)out;
}

/**
 * @brief 电压开环控制
 * 
 * 直接设置电压值，无 PID 反馈。
 */
void GM6020_VoltageControl(gm6020_control_t *motor)
{
    if (motor == NULL) return;
    motor->current_set = motor->target_voltage;
}

/* ======================== CAN 发送 ======================== */

/**
 * @brief 发送 GM6020 电压控制指令（电压模式）
 * 
 * 根据 GM6020 文档:
 *   ID 1~4 → 0x1FF, ID 5~7 → 0x2FF
 *   每电机 2 字节 int16, 范围 -25000~25000
 * 
 * @param motors 电机数组
 * @param count  电机数量
 */
void GM6020_SendVoltageCmd(gm6020_control_t *motors, uint8_t count)
{
    if (motors == NULL || count == 0) return;
    
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t send_mail_box;
    uint8_t i;
    
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0x08;
    tx_header.IDE = CAN_ID_STD;
    tx_header.TransmitGlobalTime = DISABLE;
    
    /* ---- Group 1: ID 1~4, 标识符 0x1FF ---- */
    memset(tx_data, 0, 8);
    for (i = 0; i < count && i < 4; i++)
    {
        if (motors[i].motor_id >= 1 && motors[i].motor_id <= 4)
        {
            tx_data[i * 2]     = (uint8_t)(motors[i].current_set >> 8);
            tx_data[i * 2 + 1] = (uint8_t)(motors[i].current_set);
        }
    }
    tx_header.StdId = CAN_GM6020_VOLTAGE_ID_GROUP1;
    HAL_CAN_AddTxMessage(&GM6020_CAN_HANDLE, &tx_header, tx_data, &send_mail_box);
    
    /* ---- Group 2: ID 5~7, 标识符 0x2FF ---- */
    if (count > 4)
    {
        memset(tx_data, 0, 8);
        for (i = 4; i < count && i < 7; i++)
        {
            if (motors[i].motor_id >= 5 && motors[i].motor_id <= 7)
            {
                uint8_t idx = i - 4;
                tx_data[idx * 2]     = (uint8_t)(motors[i].current_set >> 8);
                tx_data[idx * 2 + 1] = (uint8_t)(motors[i].current_set);
            }
        }
        tx_header.StdId = CAN_GM6020_VOLTAGE_ID_GROUP2;
        HAL_CAN_AddTxMessage(&GM6020_CAN_HANDLE, &tx_header, tx_data, &send_mail_box);
    }
}

/**
 * @brief 发送 GM6020 电流控制指令（电流模式）
 * 
 *   ID 1~4 → 0x1FE, ID 5~7 → 0x2FE
 *   每电机 2 字节 int16, 范围 -16384~16384 (对应 -3A~3A)
 */
void GM6020_SendCurrentCmd(gm6020_control_t *motors, uint8_t count)
{
    if (motors == NULL || count == 0) return;
    
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t send_mail_box;
    uint8_t i;
    
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0x08;
    tx_header.IDE = CAN_ID_STD;
    tx_header.TransmitGlobalTime = DISABLE;
    
    /* ---- Group 1: ID 1~4, 标识符 0x1FE ---- */
    memset(tx_data, 0, 8);
    for (i = 0; i < count && i < 4; i++)
    {
        if (motors[i].motor_id >= 1 && motors[i].motor_id <= 4)
        {
            tx_data[i * 2]     = (uint8_t)(motors[i].current_set >> 8);
            tx_data[i * 2 + 1] = (uint8_t)(motors[i].current_set);
        }
    }
    tx_header.StdId = CAN_GM6020_CURRENT_ID_GROUP1;
    HAL_CAN_AddTxMessage(&GM6020_CAN_HANDLE, &tx_header, tx_data, &send_mail_box);
    
    /* ---- Group 2: ID 5~7, 标识符 0x2FE ---- */
    if (count > 4)
    {
        memset(tx_data, 0, 8);
        for (i = 4; i < count && i < 7; i++)
        {
            if (motors[i].motor_id >= 5 && motors[i].motor_id <= 7)
            {
                uint8_t idx = i - 4;
                tx_data[idx * 2]     = (uint8_t)(motors[i].current_set >> 8);
                tx_data[idx * 2 + 1] = (uint8_t)(motors[i].current_set);
            }
        }
        tx_header.StdId = CAN_GM6020_CURRENT_ID_GROUP2;
        HAL_CAN_AddTxMessage(&GM6020_CAN_HANDLE, &tx_header, tx_data, &send_mail_box);
    }
}

/* ======================== 辅助函数 ======================== */

/**
 * @brief 设置速度环目标转速
 */
void GM6020_SetTargetSpeed(gm6020_control_t *motor, fp32 speed_rpm)
{
    if (motor == NULL) return;
    motor->control_mode = 1;  /* 切换到速度模式 */
    motor->target_speed_rpm = speed_rpm;
}

/**
 * @brief 设置位置环目标角度
 */
void GM6020_SetTargetAngle(gm6020_control_t *motor, fp32 angle_deg)
{
    if (motor == NULL) return;
    motor->control_mode = 2;  /* 切换到位置模式（串级） */
    motor->target_angle_deg = angle_deg;
}

/**
 * @brief 停止电机
 */
void GM6020_Stop(gm6020_control_t *motor)
{
    if (motor == NULL) return;
    motor->target_speed_rpm = 0;
    motor->target_angle_deg = 0;
    motor->target_voltage   = 0;
    motor->current_set      = 0;
    
    /* 清除 PID 累积 */
    PID_Clear(&motor->speed_pid);
    PID_Clear(&motor->angle_pid);
}

/**
 * @brief 设置速度环 PID 参数
 */
void GM6020_SetSpeedPID(gm6020_control_t *motor, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out)
{
    if (motor == NULL) return;
    fp32 param[3] = {Kp, Ki, Kd};
    PID_Init(&motor->speed_pid, motor->speed_pid.mode, param, max_out, motor->speed_pid.max_iout);
}

/**
 * @brief 设置角度环 PID 参数
 */
void GM6020_SetAnglePID(gm6020_control_t *motor, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out)
{
    if (motor == NULL) return;
    fp32 param[3] = {Kp, Ki, Kd};
    PID_Init(&motor->angle_pid, motor->angle_pid.mode, param, max_out, motor->angle_pid.max_iout);
}

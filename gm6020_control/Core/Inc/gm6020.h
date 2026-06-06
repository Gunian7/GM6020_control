/**
 * @file gm6020.h
 * @brief GM6020 电机驱动头文件
 * 
 * 基于 RoboMaster 开发板 C 型（STM32F407IG）
 * 参考文档：
 *   - RoboMaster GM6020直流无刷电机使用说明 V1.4
 *   - RoboMaster 开发板 C 型用户手册 V1.0
 *   - RoboMaster 开发板 C 型嵌入式软件教程 V1.0
 */

#ifndef GM6020_H
#define GM6020_H

#include "main.h"
#include "pid.h"

/* ======================== CAN 通信协议 ======================== */
/*
 * GM6020 CAN 协议（波特率 1Mbps）：
 *
 * 1. 电压控制模式（出厂默认）
 *    控制报文（标准帧）:
 *      0x1FF → 电机 ID 1~4
 *      0x2FF → 电机 ID 5~7
 *    数据域: 每2字节 = int16 电压值
 *            ID1: DATA[0:1], ID2: DATA[2:3], ID3: DATA[4:5], ID4: DATA[6:7]
 *            电压范围: -25000 ~ 0 ~ 25000
 *
 * 2. 电流控制模式（需在 RoboMaster Assistant 中开启）
 *    控制报文（标准帧）:
 *      0x1FE → 电机 ID 1~4
 *      0x2FE → 电机 ID 5~7
 *    数据域: 同电压模式
 *            电流范围: -16384 ~ 0 ~ 16384 对应 -3A ~ 0 ~ 3A
 *
 * 3. 电机反馈报文（1KHz 自动发送）
 *    标识符: 0x204 + 拨码ID
 *    数据域:
 *      DATA[0:1] = 机械角度 (0~8191，14位)
 *      DATA[2:3] = 转速 (rpm)
 *      DATA[4:5] = 实际转矩电流
 *      DATA[6]   = 电机温度
 *      DATA[7]   = 保留
 */

/* GM6020 拨码与 CAN ID 映射 */
#define GM6020_ID_INVALID   0
#define GM6020_ID_1         1   /* 拨码 001 → 反馈ID 0x205, 控制组 0x1FF */
#define GM6020_ID_2         2   /* 拨码 010 → 反馈ID 0x206, 控制组 0x1FF */
#define GM6020_ID_3         3   /* 拨码 011 → 反馈ID 0x207, 控制组 0x1FF */
#define GM6020_ID_4         4   /* 拨码 100 → 反馈ID 0x208, 控制组 0x1FF */
#define GM6020_ID_5         5   /* 拨码 101 → 反馈ID 0x209, 控制组 0x2FF */
#define GM6020_ID_6         6   /* 拨码 110 → 反馈ID 0x20A, 控制组 0x2FF */
#define GM6020_ID_7         7   /* 拨码 111 → 反馈ID 0x20B, 控制组 0x2FF */

/* 控制报文标识符 */
#define CAN_GM6020_VOLTAGE_ID_GROUP1   0x1FF  /* 电压模式, 电机 ID 1~4 */
#define CAN_GM6020_VOLTAGE_ID_GROUP2   0x2FF  /* 电压模式, 电机 ID 5~7 */
#define CAN_GM6020_CURRENT_ID_GROUP1   0x1FE  /* 电流模式, 电机 ID 1~4 */
#define CAN_GM6020_CURRENT_ID_GROUP2   0x2FE  /* 电流模式, 电机 ID 5~7 */

/* 电压/电流控制量范围 */
#define GM6020_VOLTAGE_MAX     25000
#define GM6020_CURRENT_MAX     16384   /* 对应 3A */

/* 反馈报文中的角度范围 */
#define GM6020_ANGLE_MAX       8191

/* 电机特征参数 */
#define GM6020_TORQUE_CONSTANT 0.741f   /* 转矩常数: 741 mN·m/A */
#define GM6020_SPEED_CONSTANT  13.33f   /* 转速常数: 13.33 rpm/V */
#define GM6020_MAX_SPEED       320.0f   /* 最大空载转速: 320 rpm */
#define GM6020_RATED_TORQUE    1.2f     /* 额定扭矩: 1.2 N·m */

/* ======================== 数据结构 ======================== */

/* 电机反馈数据结构 */
typedef struct
{
    uint16_t angle_raw;      /* 原始机械角度 (0~8191) */
    fp32     angle_deg;      /* 角度 (度) */
    fp32     angle_rad;      /* 角度 (弧度) */
    int16_t  speed_rpm;      /* 转速 (rpm) */
    int16_t  current_raw;    /* 原始电流值 */
    fp32     current_A;      /* 电流 (A) */
    uint8_t  temperature;    /* 电机温度 (℃) */
    
    /* 累积角度（跨周期追踪用） */
    fp32     total_angle_deg;   /* 累积机械角度 (度), 可正负 */
    int32_t  round_count;       /* 旋转圈数 */
    uint16_t last_angle_raw;    /* 上一次的角度原始值, 用于解缠绕 */
} gm6020_feedback_t;

/* GM6020 电机控制结构体（速度环 + 位置环串级PID） */
typedef struct
{
    /* 电机标识 */
    uint8_t motor_id;          /* 拨码开关 ID (1~7) */
    uint32_t feedback_id;      /* 反馈报文 ID = 0x204 + motor_id */
    
    /* 反馈数据 */
    gm6020_feedback_t feedback;
    
    /* 控制模式 */
    uint8_t control_mode;      /* 0=电压开环, 1=速度环, 2=位置环(串级) */
    
    /* 控制目标值 */
    fp32 target_speed_rpm;     /* 速度环目标 (rpm) */
    fp32 target_angle_deg;     /* 位置环目标 (度) */
    int16_t target_voltage;    /* 开环电压目标 (-25000~25000) */
    
    /* ============ 串级 PID ============ */
    /* 外环：角度环 → 输出期望角速度 (作为速度环的set) */
    pid_type_def angle_pid;
    
    /* 内环：速度环 → 输出电压控制量 */
    pid_type_def speed_pid;
    
    /* PID 控制中间值 */
    fp32 speed_set;            /* 角度环计算出的速度设定值 */
    int16_t current_set;       /* 最终发送的电流/电压值 */
    
} gm6020_control_t;

/* ======================== CAN 接口 ======================== */

/* 根据文档教程，CAN1 用于控制电机 (PD0=CAN1_RX, PD1=CAN1_TX)
 * 开发板 C 型上的 2-pin CAN 接口就是 CAN1
 */
#define GM6020_CAN_HANDLE     hcan1

/* ======================== 函数声明 ======================== */

/* 初始化 */
void GM6020_Init(gm6020_control_t *motor, uint8_t motor_id);
void GM6020_InitAll(gm6020_control_t *motors, uint8_t count);

/* 反馈处理（在 HAL_CAN_RxFifo0MsgPendingCallback 中调用） */
void GM6020_FeedbackProcess(gm6020_control_t *motor, CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

/* 控制量计算 */
void GM6020_SpeedControl(gm6020_control_t *motor);       /* 速度环 */
void GM6020_PositionControl(gm6020_control_t *motor);    /* 位置环（串级: 角度环+速度环） */
void GM6020_VoltageControl(gm6020_control_t *motor);     /* 电压开环 */

/* CAN 发送 */
void GM6020_SendVoltageCmd(gm6020_control_t *motors, uint8_t count);  /* 电压模式发送 */
void GM6020_SendCurrentCmd(gm6020_control_t *motors, uint8_t count);  /* 电流模式发送 */

/* 辅助函数 */
void GM6020_SetTargetSpeed(gm6020_control_t *motor, fp32 speed_rpm);
void GM6020_SetTargetAngle(gm6020_control_t *motor, fp32 angle_deg);
void GM6020_Stop(gm6020_control_t *motor);

/* PID 参数调节 */
void GM6020_SetSpeedPID(gm6020_control_t *motor, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out);
void GM6020_SetAnglePID(gm6020_control_t *motor, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out);

#endif /* GM6020_H */

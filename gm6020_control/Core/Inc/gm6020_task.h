/**
 * @file gm6020_task.h
 * @brief GM6020 电机控制任务头文件
 * 
 * 基于 FreeRTOS 的任务框架，参考开发板 C 型教程第19章云台控制任务。
 */
#ifndef GM6020_TASK_H
#define GM6020_TASK_H

#include "gm6020.h"

/* 控制频率 */
#define GM6020_CONTROL_FREQ      1000  /* 1KHz, 与GM6020反馈频率一致 */
#define GM6020_CONTROL_CYCLE_MS  1     /* 控制周期 1ms */

/* 最大支持的电机数量 */
#define GM6020_MAX_MOTORS        4

/* 外部声明 - 用户可在 main.c 中定义 */
extern gm6020_control_t g_gm6020_motors[GM6020_MAX_MOTORS];

/* 任务函数声明 */
void GM6020_ControlTask(void *argument);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif /* GM6020_TASK_H */

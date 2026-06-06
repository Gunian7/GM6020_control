/**
 * @file pid.h
 * @brief PID 控制器头文件（基于RoboMaster开发板C型教程）
 */
#ifndef PID_H
#define PID_H

#include "main.h"

/* fp32 类型定义（RoboMaster 例程中使用的浮点类型） */
typedef float fp32;
#define PID_POSITION 0  /* 位置式 PID */
#define PID_DELTA    1  /* 增量式 PID */

/* PID 结构体定义 */
typedef struct
{
    uint8_t mode;              /* PID 模式 */
    
    fp32 Kp;                   /* 比例系数 */
    fp32 Ki;                   /* 积分系数 */
    fp32 Kd;                   /* 微分系数 */
    
    fp32 max_out;              /* 最大输出 */
    fp32 max_iout;             /* 最大积分输出 */
    
    fp32 set;                  /* 设定值 */
    fp32 fdb;                  /* 反馈值 */
    
    fp32 Pout;                 /* 比例项输出 */
    fp32 Iout;                 /* 积分项输出 */
    fp32 Dout;                 /* 微分项输出 */
    fp32 out;                  /* PID 总输出 */
    
    fp32 error[3];             /* 误差: error[0]=当前, error[1]=前一次, error[2]=前两次 */
    fp32 Dbuf[3];              /* 微分缓冲 */
} pid_type_def;

/* 限幅宏 */
#define LimitMax(val, max) \
    do { \
        if ((val) > (max))      (val) = (max); \
        else if ((val) < -(max)) (val) = -(max); \
    } while(0)

/* 函数声明 */
void PID_Init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 PID_Calc(pid_type_def *pid, fp32 ref, fp32 set);
void PID_Clear(pid_type_def *pid);

#endif /* PID_H */

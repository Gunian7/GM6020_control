/**
 * @file pid.c
 * @brief PID 控制器实现（基于RoboMaster开发板C型教程第16章）
 */
#include "pid.h"

/**
 * @brief 初始化 PID 控制器
 * @param pid     PID 结构体指针
 * @param mode    PID 模式（PID_POSITION 或 PID_DELTA）
 * @param PID     PID 参数数组 [Kp, Ki, Kd]
 * @param max_out 最大输出限幅
 * @param max_iout 积分项最大限幅
 */
void PID_Init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    
    pid->mode     = mode;
    pid->Kp       = PID[0];
    pid->Ki       = PID[1];
    pid->Kd       = PID[2];
    pid->max_out  = max_out;
    pid->max_iout = max_iout;
    
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief PID 计算
 * @param pid PID 结构体指针
 * @param ref 当前反馈值
 * @param set 期望设定值
 * @return PID 输出值
 *
 * 支持位置式 PID 和增量式 PID：
 *   位置式: u(k) = Kp*e(k) + Ki*Σe(i) + Kd*[e(k)-e(k-1)]
 *   增量式: Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
 */
fp32 PID_Calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    
    /* 更新误差缓冲区 */
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    
    if (pid->mode == PID_POSITION)
    {
        /* 位置式 PID */
        pid->Pout = pid->Kp * pid->error[0];
        
        pid->Iout += pid->Ki * pid->error[0];
        LimitMax(pid->Iout, pid->max_iout);
        
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        /* 增量式 PID */
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        
        pid->Iout = pid->Ki * pid->error[0];
        
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    
    return pid->out;
}

/**
 * @brief 清除 PID 数据
 */
void PID_Clear(pid_type_def *pid)
{
    if (pid == NULL) return;
    
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @file gm6020_test.c
 * @brief GM6020 阶跃响应测试程序
 * 
 * 用于验证位置环（串级PID）的性能。
 * 
 * 测试方法：
 *   1. 烧录程序，打开串口助手（115200-8-N-1）
 *   2. 电机先归零，然后每5秒跳变一个目标角度
 *   3. 串口打印"时间(ms)  目标角度  实际角度  实际转速"数据
 *   4. 将数据导入 Excel / Python 画出响应曲线
 *   5. 从曲线读取：超调量、上升时间、峰值时间、稳态误差
 * 
 * 测试序列：
 *   0-5s:  归零     (目标0°)
 *   5-10s: 阶跃90°  (目标90°)
 *   10-15s:保持     (目标90°)
 *   15-20s:阶跃-90° (目标-90°)
 *   20-25s:保持     (目标-90°)
 *   25-30s:归零     (目标0°)
 *   30-35s:阶跃180° (目标180°)
 *   35-40s:保持     (目标180°)
 */
#include "gm6020_task.h"
#include "stdio.h"
#include "string.h"

/* ======================== 测试参数 ======================== */

/* 测试阶段定义 */
typedef enum {
    TEST_INIT        = 0,   /* 初始化归零 */
    TEST_STEP_POS90  = 1,   /* 阶跃 +90° */
    TEST_HOLD_POS90  = 2,   /* 保持 +90° */
    TEST_STEP_NEG90  = 3,   /* 阶跃 -90° */
    TEST_HOLD_NEG90  = 4,   /* 保持 -90° */
    TEST_STEP_ZERO   = 5,   /* 归零 */
    TEST_STEP_POS180 = 6,   /* 阶跃 +180° */
    TEST_HOLD_POS180 = 7,   /* 保持 +180° */
    TEST_DONE        = 8,   /* 测试完成 */
} test_phase_t;

/* 每个阶段的持续时间 (ms) */
#define PHASE_DURATION_MS   5000

/* 串口打印缓冲区 */
static char uart_buf[128];

/* ======================== 串口打印 ======================== */

/**
 * @brief 通过 UART 打印字符串（需根据你的工程修改 UART 句柄）
 * 
 * 开发板 C 型默认 UART 接口：
 *   UART1 (4-pin接口): PA9(TX), PB7(RX)  — 外壳丝印"UART2"
 *   UART2 (3-pin接口): PG14(TX), PG9(RX) — 外壳丝印"UART1"
 * 
 * 如果使用不同串口，修改 huart1 为对应的句柄。
 */
static void uart_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(uart_buf, sizeof(uart_buf), fmt, args);
    va_end(args);
    
    if (len > 0)
    {
        /* 默认使用 UART1（PA9_TX, PB7_RX） */
        HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, len, 100);
    }
}

/**
 * @brief 打印 CSV 格式的数据头
 */
static void print_csv_header(void)
{
    uart_printf("time_ms,target_angle,actual_angle,speed_rpm,phase\r\n");
}

/**
 * @brief 打印一行数据 (CSV 格式)
 */
static void print_data_line(uint32_t time_ms, fp32 target, fp32 actual, int16_t speed, uint8_t phase)
{
    uart_printf("%lu,%.2f,%.2f,%d,%d\r\n",
                (unsigned long)time_ms, target, actual, speed, phase);
}

/* ======================== 测试控制 ======================== */

/**
 * @brief 执行测试序列
 * 
 * 在每个阶段开始时设置新的目标角度，然后持续记录响应数据。
 */
void GM6020_RunStepTest(void)
{
    test_phase_t current_phase = TEST_INIT;
    uint32_t phase_start_time = 0;
    uint32_t current_time = 0;
    uint8_t phase_just_changed = 1;
    
    /* 打印 CSV 头 */
    print_csv_header();
    
    uart_printf("# GM6020 Step Response Test Started\r\n");
    uart_printf("# Motor ID: %d\r\n", g_gm6020_motors[0].motor_id);
    uart_printf("# Angle PID: Kp=%.1f Ki=%.1f Kd=%.1f\r\n",
                g_gm6020_motors[0].angle_pid.Kp,
                g_gm6020_motors[0].angle_pid.Ki,
                g_gm6020_motors[0].angle_pid.Kd);
    uart_printf("# Speed PID: Kp=%.1f Ki=%.1f Kd=%.1f\r\n",
                g_gm6020_motors[0].speed_pid.Kp,
                g_gm6020_motors[0].speed_pid.Ki,
                g_gm6020_motors[0].speed_pid.Kd);
    
    for (;;)
    {
        /* 1ms 控制周期（与反馈同步） */
        osDelay(1);
        current_time += 1;
        
        /* ---- 阶段切换逻辑 ---- */
        if (current_time - phase_start_time >= PHASE_DURATION_MS)
        {
            phase_start_time = current_time;
            phase_just_changed = 1;
            
            switch (current_phase)
            {
                case TEST_INIT:
                    current_phase = TEST_STEP_POS90;
                    break;
                case TEST_STEP_POS90:
                    current_phase = TEST_HOLD_POS90;
                    break;
                case TEST_HOLD_POS90:
                    current_phase = TEST_STEP_NEG90;
                    break;
                case TEST_STEP_NEG90:
                    current_phase = TEST_HOLD_NEG90;
                    break;
                case TEST_HOLD_NEG90:
                    current_phase = TEST_STEP_ZERO;
                    break;
                case TEST_STEP_ZERO:
                    current_phase = TEST_STEP_POS180;
                    break;
                case TEST_STEP_POS180:
                    current_phase = TEST_HOLD_POS180;
                    break;
                case TEST_HOLD_POS180:
                    current_phase = TEST_DONE;
                    break;
                default:
                    break;
            }
        }
        
        /* ---- 阶段开始时设置目标 ---- */
        if (phase_just_changed)
        {
            phase_just_changed = 0;
            
            switch (current_phase)
            {
                case TEST_INIT:
                    uart_printf("# [PHASE] INIT: target = 0 deg\r\n");
                    GM6020_SetTargetAngle(&g_gm6020_motors[0], 0.0f);
                    break;
                case TEST_STEP_POS90:
                    uart_printf("# [PHASE] STEP +90 deg\r\n");
                    GM6020_SetTargetAngle(&g_gm6020_motors[0], 90.0f);
                    break;
                case TEST_HOLD_POS90:
                    uart_printf("# [PHASE] HOLD +90 deg\r\n");
                    /* 目标不变，观察稳态 */
                    break;
                case TEST_STEP_NEG90:
                    uart_printf("# [PHASE] STEP -90 deg (target from +90 to -90)\r\n");
                    GM6020_SetTargetAngle(&g_gm6020_motors[0], -90.0f);
                    break;
                case TEST_HOLD_NEG90:
                    uart_printf("# [PHASE] HOLD -90 deg\r\n");
                    break;
                case TEST_STEP_ZERO:
                    uart_printf("# [PHASE] STEP back to 0 deg\r\n");
                    GM6020_SetTargetAngle(&g_gm6020_motors[0], 0.0f);
                    break;
                case TEST_STEP_POS180:
                    uart_printf("# [PHASE] STEP +180 deg\r\n");
                    GM6020_SetTargetAngle(&g_gm6020_motors[0], 180.0f);
                    break;
                case TEST_HOLD_POS180:
                    uart_printf("# [PHASE] HOLD +180 deg\r\n");
                    break;
                case TEST_DONE:
                    uart_printf("# [PHASE] TEST COMPLETED\r\n");
                    break;
                default:
                    break;
            }
        }
        
        /* ---- 执行控制算法 ---- */
        GM6020_PositionControl(&g_gm6020_motors[0]);
        
        /* ---- 发送 CAN 指令 ---- */
        GM6020_SendVoltageCmd(g_gm6020_motors, 1);
        
        /* ---- 记录数据（每 5ms 打印一次，减少串口负载） ---- */
        if (current_time % 5 == 0)
        {
            gm6020_control_t *m = &g_gm6020_motors[0];
            print_data_line(
                current_time,
                m->target_angle_deg,
                m->feedback.total_angle_deg,
                m->feedback.speed_rpm,
                (uint8_t)current_phase
            );
        }
        
        /* 测试完成则停止 */
        if (current_phase == TEST_DONE && current_time - phase_start_time > 2000)
        {
            uart_printf("# Test finished. Motor stopped.\r\n");
            GM6020_Stop(&g_gm6020_motors[0]);
            break;
        }
    }
}

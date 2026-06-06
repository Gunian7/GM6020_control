/**
 * @file gm6020_cmd.c
 * @brief GM6020 串口指令解析模块
 * 
 * 通过串口发送指令控制电机，支持：
 *   POS <angle>   — 位置模式，转到指定角度（度），如 POS 90
 *   SPD <rpm>     — 速度模式，转到指定转速（rpm），如 SPD 50
 *   VOL <value>   — 电压开环，如 VOL 5000
 *   STOP          — 停止电机
 *   PID <Kp> <Ki> <Kd> — 临时修改速度环PID参数
 *   APID <Kp> <Ki> <Kd> — 临时修改角度环PID参数
 *   STATUS        — 打印当前状态
 *   HELP          — 打印帮助信息
 *   SAVE          — 将PID参数保存到内存（后续断电会丢失，仅运行时生效）
 * 
 * 串口配置：115200-8-N-1
 */

#include "gm6020_task.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"

/* 引用 main.c 中定义的 CAN 和 UART 句柄 */
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart6;

/* 接收缓冲区 */
#define CMD_BUF_SIZE    64
static uint8_t cmd_rx_buf[CMD_BUF_SIZE];
static uint8_t cmd_rx_index = 0;
static uint8_t cmd_ready = 0;        /* 收到完整一行的标志 */

/* 串口发送缓冲 */
static char uart_tx_buf[128];

/* ======================== 串口 I/O ======================== */

/**
 * @brief 串口 printf（根据你的工程修改 huart1 为实际句柄）
 */
static void cmd_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(uart_tx_buf, sizeof(uart_tx_buf), fmt, args);
    va_end(args);
    
    if (len > 0)
    {
        HAL_UART_Transmit(&huart6, (uint8_t *)uart_tx_buf, len, 100);
    }
}

/**
 * @brief 打印欢迎信息和帮助
 */
static void cmd_print_help(void)
{
    cmd_printf("\r\n");
    cmd_printf("========================================\r\n");
    cmd_printf("  GM6020 串口控制终端\r\n");
    cmd_printf("========================================\r\n");
    cmd_printf("可用指令:\r\n");
    cmd_printf("  POS <angle>    位置模式 - 转到指定角度(度)\r\n");
    cmd_printf("                  例: POS 90\r\n");
    cmd_printf("  SPD <rpm>     速度模式 - 以指定转速运行\r\n");
    cmd_printf("                  例: SPD 50\r\n");
    cmd_printf("  VOL <value>   电压开环 - 输出电压值(-25000~25000)\r\n");
    cmd_printf("                  例: VOL 8000\r\n");
    cmd_printf("  STOP           停止电机\r\n");
    cmd_printf("  PID <Kp> <Ki> <Kd>   设置速度环PID参数\r\n");
    cmd_printf("                  例: PID 20 1 0\r\n");
    cmd_printf("  APID <Kp> <Ki> <Kd>  设置角度环PID参数\r\n");
    cmd_printf("                  例: APID 10 0 0\r\n");
    cmd_printf("  STATUS         打印当前电机状态\r\n");
    cmd_printf("  HELP           打印帮助信息\r\n");
    cmd_printf("========================================\r\n");
    cmd_printf("\r\n");
}

/**
 * @brief 打印电机当前状态
 */
static void cmd_print_status(gm6020_control_t *motor)
{
    cmd_printf("\r\n========== GM6020 Status ==========\r\n");
    cmd_printf("  Motor ID:       %d\r\n", motor->motor_id);
    cmd_printf("  Mode:           %s\r\n",
               motor->control_mode == 0 ? "VOLTAGE (开环)" :
               motor->control_mode == 1 ? "SPEED (速度环)" :
               motor->control_mode == 2 ? "POSITION (位置环)" : "UNKNOWN");
    cmd_printf("  Target Angle:   %.1f deg\r\n", motor->target_angle_deg);
    cmd_printf("  Target Speed:   %.1f rpm\r\n", motor->target_speed_rpm);
    cmd_printf("  Actual Angle:   %.1f deg\r\n", motor->feedback.total_angle_deg);
    cmd_printf("  Actual Speed:   %d rpm\r\n", motor->feedback.speed_rpm);
    cmd_printf("  Current Output: %d\r\n", motor->current_set);
    cmd_printf("  Temperature:    %d C\r\n", motor->feedback.temperature);
    cmd_printf("  Angle PID:      Kp=%.1f  Ki=%.1f  Kd=%.1f\r\n",
               motor->angle_pid.Kp, motor->angle_pid.Ki, motor->angle_pid.Kd);
    cmd_printf("  Speed PID:      Kp=%.1f  Ki=%.1f  Kd=%.1f\r\n",
               motor->speed_pid.Kp, motor->speed_pid.Ki, motor->speed_pid.Kd);
    cmd_printf("====================================\r\n");
}

/* ======================== 指令解析 ======================== */

static void cmd_process(gm6020_control_t *motor, uint8_t *buf)
{
    char *token;
    char *saveptr;
    
    /* 取第一个 token（指令名） */
    token = strtok_r((char *)buf, " \r\n", &saveptr);
    if (token == NULL) return;
    
    /* ---- POS <angle> ---- */
    if (strcmp(token, "POS") == 0 || strcmp(token, "pos") == 0)
    {
        char *arg = strtok_r(NULL, " \r\n", &saveptr);
        if (arg)
        {
            fp32 angle = (fp32)atof(arg);
            GM6020_SetTargetAngle(motor, angle);
            cmd_printf("POS OK: target angle = %.1f deg\r\n", angle);
        }
        else
        {
            cmd_printf("ERROR: 用法 POS <angle>\r\n");
        }
    }
    /* ---- SPD <rpm> ---- */
    else if (strcmp(token, "SPD") == 0 || strcmp(token, "spd") == 0)
    {
        char *arg = strtok_r(NULL, " \r\n", &saveptr);
        if (arg)
        {
            fp32 speed = (fp32)atof(arg);
            /* 限幅到电机最大转速 */
            if (speed > GM6020_MAX_SPEED) speed = GM6020_MAX_SPEED;
            if (speed < -GM6020_MAX_SPEED) speed = -GM6020_MAX_SPEED;
            GM6020_SetTargetSpeed(motor, speed);
            cmd_printf("SPD OK: target speed = %.1f rpm\r\n", speed);
        }
        else
        {
            cmd_printf("ERROR: 用法 SPD <rpm>\r\n");
        }
    }
    /* ---- VOL <value> ---- */
    else if (strcmp(token, "VOL") == 0 || strcmp(token, "vol") == 0)
    {
        char *arg = strtok_r(NULL, " \r\n", &saveptr);
        if (arg)
        {
            int16_t val = (int16_t)atoi(arg);
            if (val > GM6020_VOLTAGE_MAX) val = GM6020_VOLTAGE_MAX;
            if (val < -GM6020_VOLTAGE_MAX) val = -GM6020_VOLTAGE_MAX;
            
            motor->control_mode = 0;
            motor->target_voltage = val;
            cmd_printf("VOL OK: voltage = %d\r\n", val);
        }
        else
        {
            cmd_printf("ERROR: 用法 VOL <value>\r\n");
        }
    }
    /* ---- STOP ---- */
    else if (strcmp(token, "STOP") == 0 || strcmp(token, "stop") == 0)
    {
        GM6020_Stop(motor);
        cmd_printf("STOP OK: motor stopped, PID cleared\r\n");
    }
    /* ---- PID <Kp> <Ki> <Kd> ---- */
    else if (strcmp(token, "PID") == 0 || strcmp(token, "pid") == 0)
    {
        char *kp_str = strtok_r(NULL, " \r\n", &saveptr);
        char *ki_str = strtok_r(NULL, " \r\n", &saveptr);
        char *kd_str = strtok_r(NULL, " \r\n", &saveptr);
        
        if (kp_str && ki_str && kd_str)
        {
            fp32 kp = (fp32)atof(kp_str);
            fp32 ki = (fp32)atof(ki_str);
            fp32 kd = (fp32)atof(kd_str);
            GM6020_SetSpeedPID(motor, kp, ki, kd, 25000.0f);
            cmd_printf("PID OK: speed PID set to Kp=%.1f Ki=%.1f Kd=%.1f\r\n", kp, ki, kd);
        }
        else
        {
            cmd_printf("ERROR: 用法 PID <Kp> <Ki> <Kd>\r\n");
        }
    }
    /* ---- APID <Kp> <Ki> <Kd> ---- */
    else if (strcmp(token, "APID") == 0 || strcmp(token, "apid") == 0)
    {
        char *kp_str = strtok_r(NULL, " \r\n", &saveptr);
        char *ki_str = strtok_r(NULL, " \r\n", &saveptr);
        char *kd_str = strtok_r(NULL, " \r\n", &saveptr);
        
        if (kp_str && ki_str && kd_str)
        {
            fp32 kp = (fp32)atof(kp_str);
            fp32 ki = (fp32)atof(ki_str);
            fp32 kd = (fp32)atof(kd_str);
            GM6020_SetAnglePID(motor, kp, ki, kd, 300.0f);
            cmd_printf("APID OK: angle PID set to Kp=%.1f Ki=%.1f Kd=%.1f\r\n", kp, ki, kd);
        }
        else
        {
            cmd_printf("ERROR: 用法 APID <Kp> <Ki> <Kd>\r\n");
        }
    }
    /* ---- STATUS ---- */
    else if (strcmp(token, "STATUS") == 0 || strcmp(token, "status") == 0)
    {
        cmd_print_status(motor);
    }
    /* ---- HELP ---- */
    else if (strcmp(token, "HELP") == 0 || strcmp(token, "help") == 0)
    {
        cmd_print_help();
    }
    else
    {
        cmd_printf("ERROR: 未知指令 '%s'，输入 HELP 查看可用指令\r\n", token);
    }
}

/* ======================== 串口接收中断 ======================== */

/**
 * @brief 串口接收中断回调（在 stm32f4xx_it.c 的 USART1_IRQHandler 中触发）
 * 
 * 需要先开启中断接收：
 *   HAL_UART_Receive_IT(&huart1, cmd_rx_buf + offset, 1);
 * 
 * 或者在 CubeMX 中开启 UART 全局中断，然后在 main 中：
 *   __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
 * 
 * HAL 库方式（推荐）：
 *   在 main 中调用:
 *     HAL_UART_Receive_IT(&huart1, &cmd_rx_byte, 1);
 *   然后在此回调中逐字节拼接。
 */
static uint8_t cmd_rx_byte;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 收到回车或换行，认为指令结束 */
        if (cmd_rx_byte == '\n' || cmd_rx_byte == '\r')
        {
            if (cmd_rx_index > 0)
            {
                cmd_rx_buf[cmd_rx_index] = '\0';
                cmd_ready = 1;
                cmd_rx_index = 0;
            }
        }
        else
        {
            /* 防止缓冲区溢出 */
            if (cmd_rx_index < CMD_BUF_SIZE - 1)
            {
                cmd_rx_buf[cmd_rx_index++] = cmd_rx_byte;
            }
        }
        
        /* 继续接收下一个字节 */
        HAL_UART_Receive_IT(huart, &cmd_rx_byte, 1);
    }
}

/* ======================== 串口控制任务 ======================== */

/**
 * @brief 串口控制任务
 * 
 * 在 FreeRTOS 中以 10ms 周期运行，检查是否有新的串口指令。
 * 有指令则解析执行，同时持续运行电机控制。
 * 
 * 创建方式（在 main.c 中）:
 *   osThreadDef(cmd_task, GM6020_CmdTask, osPriorityBelowNormal, 0, 256);
 *   osThreadCreate(osThread(cmd_task), NULL);
 */
void GM6020_CmdTask(void *argument)
{
    uint8_t i;
    
    /* 等待电机任务先完成初始化 */
    osDelay(500);
    
    /* 启动串口中断接收 */
    HAL_UART_Receive_IT(&huart6, &cmd_rx_byte, 1);
    
    /* 打印欢迎信息 */
    cmd_printf("\r\n\r\n");
    cmd_printf("========================================\r\n");
    cmd_printf("  GM6020 控制终端已就绪\r\n");
    cmd_printf("  输入 HELP 查看指令列表\r\n");
    cmd_printf("========================================\r\n");
    cmd_printf("\r\n> ");
    
    for (;;)
    {
        osDelay(10);
        
        /* 检查是否有新指令 */
        if (cmd_ready)
        {
            cmd_ready = 0;
            
            /* 回显指令 */
            cmd_printf("\r\n> %s\r\n", cmd_rx_buf);
            
            /* 解析并执行（控制第一个电机） */
            cmd_process(&g_gm6020_motors[0], cmd_rx_buf);
            
            cmd_printf("\r\n> ");
        }
    }
}

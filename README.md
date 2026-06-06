# GM6020 电机控制 - CubeMX 配置与使用指南

基于 **RoboMaster 开发板 C 型**（STM32F407IG）控制 **GM6020 直流无刷电机**。

---

## 一、硬件连接

| 开发板 C 型 | GM6020 |
|------------|--------|
| CAN1 (2-pin接口): BLACK=CANL, RED=CANH | CAN 信号线: BLACK=CAN_L, RED=CAN_H |
| XT30 电源输入: 24V DC | XT30 电源线: 24V DC |
| — | 拨码开关: 设置电机 ID |

**注意**：
- CAN 总线两端需要接入 120Ω 终端电阻（GM6020 拨码开关第4位拨至 ON）
- 电源电压范围 8V~28V，额定 24V
- CAN 波特率固定 **1Mbps**
- **开发板 C 型 CAN1 引脚**：PD0 (CAN1_RX), PD1 (CAN1_TX)
- **开发板 C 型 CAN2 引脚**：PB5 (CAN2_RX), PB6 (CAN2_TX)

---

## 二、CubeMX 配置（按顺序操作）

### 2.1 新建工程

1. 打开 CubeMX → `File` → `New Project`
2. 搜索 `STM32F407IG`，选你对应的型号（IGH6/IGT6 均可）
3. 点 `Start Project`

### 2.2 配置时钟

1. 切换到 `Clock Configuration` 标签页
2. 在 **HCLK** 输入框填 **168** 后回车
3. 确认 **APB1** = 84MHz（CAN 时钟源）

### 2.3 配置 USART1（串口，用于发指令和打印数据）

1. 左侧 `Connectivity` → `USART1`
2. **Mode** = `Asynchronous`
3. 确认引脚：TX = PA9, RX = PB7（开发板 C 型 4-pin 接口，外壳丝印"UART2"）
4. **Parameter Settings** → Baud Rate = **115200**

### 2.4 配置 CAN1

1. 左侧 `Connectivity` → `CAN1`
2. 勾选 **Activated**
3. **Mode** = `Normal`
4. **Parameter Settings**（APB1=84MHz, 配置 1Mbps）：
   - `Prescaler (for Time Quantum)` = 4
   - `Time Quanta in Bit Segment 1` = 16
   - `Time Quanta in Bit Segment 2` = 4
   - `ReSynchronization Jump Width` = 4
   - `Time Triggered Communication Mode` = Disable
   - `Automatic Bus-Off Management` = Disable
   - `Automatic Wake-Up Mode` = Disable
   - `Automatic Retransmission` = Enable
   - `Receive Fifo0 Locked Mode` = Disable
   - `Transmit Fifo Priority` = Disable
   - `Mode` = Normal

5. **NVIC Settings**：
   - 勾选 `CAN1 RX0 interrupt`
   - 勾选 `CAN1 SCE interrupt`
   - 优先级保持默认

6. **GPIO Settings**（**关键！需要手动改**）：
   - CubeMX 会自动把 CAN1 分配到 PB8/PB9（这是 CAN1 的另一组 AF 映射）
   - **但开发板 C 型硬件上 CAN1（2-pin接口）实际连的是 PD0/PD1**
   - 切换到 `Pinout View` 芯片引脚图，找到：
     - **PD0** → 左键点它，选 `CAN1_RX`
     - **PD1** → 左键点它，选 `CAN1_TX`
   - 找到 PB8/PB9，**右键 → `Reset to Default`** 释放占用

### 2.5 配置 FreeRTOS

1. 左侧 `Middleware` → `FREERTOS`
2. **Interface** = `CMSIS_V2`（不要选 V1）
3. 在 `Tasks and Queues` 中：
   - 默认有一个 `defaultTask`，保留不动
   - 点 **Add** 新建一个任务：

     | 字段 | 值 |
     |------|-----|
     | Task Name | `gm6020_control` |
     | Priority | `Normal` |
     | Stack Size | `512` |
     | Entry Function | `GM6020_ControlTask` |

   - 再点 **Add** 新建第二个任务：

     | 字段 | 值 |
     |------|-----|
     | Task Name | `cmd_task` |
     | Priority | `BelowNormal` |
     | Stack Size | `256` |
     | Entry Function | `GM6020_CmdTask` |

### 2.6 确认 NVIC（串口中断）

1. 左侧 `System Core` → `NVIC`
2. 确保 `USART1 global interrupt` 已勾选（默认已勾选）

### 2.7 生成代码

1. 切换到 `Project Manager` 标签页
2. **Project Name**：填 `gm6020_control`（纯英文，不要有中文或空格）
3. **Project Location**：选一个纯英文目录（如 `/home/setsuna/MCU/stm32_project/`）
4. **Toolchain / IDE**：选 `MDK-ARM`（Keil）或 `STM32CubeIDE`
5. 点右上角 **GENERATE CODE**

---

## 三、程序集成步骤

### 步骤 1：复制源文件

将 `/home/setsuna/MCU/code/gm6020_control/` 下的文件复制到 CubeMX 生成的工程目录中：

```
你的工程文件夹/
├── Core/
│   ├── Src/           ←  放 .c 文件
│   │   ├── pid.c
│   │   ├── gm6020.c
│   │   ├── gm6020_task.c
│   │   └── gm6020_cmd.c
│   └── Inc/           ←  放 .h 文件
│       ├── pid.h
│       ├── gm6020.h
│       ├── gm6020_task.h
│       └── gm6020_cmd.h
└── ...
```

### 步骤 2：在 main.c 中声明全局变量

在 `/* Includes */` 区域添加：

```c
#include "gm6020_task.h"
```

在 `/* Private variables */` 区域添加：

```c
/* 全局电机数组 */
gm6020_control_t g_gm6020_motors[GM6020_MAX_MOTORS];
```

### 步骤 3：在 main() 中启动 CAN

在 `MX_FREERTOS_Init()` 之前或之后添加（如果 CubeMX 生成的是带 FreeRTOS 的工程，不需要手动启动调度器）：

```c
/* 启动 CAN */
HAL_CAN_Start(&hcan1);
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
```

### 步骤 4：确认 CAN 中断

确保 `stm32f4xx_it.c` 中有：

```c
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
```

（这些 CubeMX 会自动生成，不需要手动添加，检查一下就好。）

---

## 四、使用方法

烧录后，打开串口助手（115200-8-N-1），会看到：

```
========================================
  GM6020 控制终端已就绪
  输入 HELP 查看指令列表
========================================

>
```

### 可用指令

| 指令 | 说明 | 示例 |
|------|------|------|
| `POS <角度>` | 位置模式，转到指定角度 | `POS 90` |
| `SPD <转速>` | 速度模式，以指定转速运行 | `SPD 50` |
| `VOL <值>` | 电压开环模式 | `VOL 5000` |
| `STOP` | 停止电机 | `STOP` |
| `PID <Kp> <Ki> <Kd>` | 设置速度环 PID 参数 | `PID 20 1 0` |
| `APID <Kp> <Ki> <Kd>` | 设置角度环 PID 参数（外环） | `APID 8 0 0` |
| `STATUS` | 打印当前电机状态 | `STATUS` |
| `HELP` | 打印帮助信息 | `HELP` |

### 使用示例

```
> POS 90
POS OK: target angle = 90.0 deg
> POS -45
POS OK: target angle = -45.0 deg
> SPD 50
SPD OK: target speed = 50.0 rpm
> STATUS
========== GM6020 Status ==========
  Mode:           POSITION (位置环)
  Target Angle:   90.0 deg
  Actual Angle:   89.5 deg
  Speed PID:      Kp=20.0  Ki=1.0  Kd=0.0
====================================
```

---

## 五、PID 参数调优建议

### 速度环调参步骤

1. **先将 Ki、Kd 设为 0**，只调 Kp
2. 从小到大增加 Kp，直到电机产生轻微震荡
3. 取当前 Kp 的 60% 作为最终 Kp
4. 加入 Ki（约为 Kp 的 0.05~0.1 倍）消除静差
5. 如有超调或震荡，加入少量 Kd

**参考初始值**（GM6020）：
- 速度环: Kp=15, Ki=0.5, Kd=0, max_out=25000, max_iout=5000
- 角度环: Kp=8, Ki=0, Kd=0, max_out=300, max_iout=100

> 直接通过串口调参，无需重新编译烧录。例如：
> ```
> > PID 20 1 0          ← 改速度环
> > APID 10 0.5 0       ← 改角度环
> ```

---

## 六、故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 绿灯每1秒闪N次 | 正常，N=ID | 正常状态 |
| 橙灯每1秒闪2次 | CAN总线有相同ID | 检查拨码开关，确保ID唯一 |
| 红灯每1秒闪1次 | 电压过高 | 检查电源，不要超过28V |
| 红灯每1秒闪4次 | 电机温度 >125℃ | 停止运行，等待冷却 |
| 电机不转 | CAN配置错误 | 检查PD0/PD1引脚配置，检查波特率 |
| 电机抖动 | PID参数过大 | 减小Kp |
| 位置有静差 | 积分不够 | 增加Ki或增大max_iout |
| 串口无输出 | USART1引脚不对 | 检查PA9(TX)/PB7(RX)是否正确 |
| 串口输出乱码 | 波特率不匹配 | 确认串口助手设为115200 |

---

## 七、参考资料

- `RoboMaster GM6020直流无刷电机使用说明 V1.4` — CAN 协议详细定义
- `RoboMaster 开发板 C 型用户手册 V1.0` — 硬件接口、引脚定义
- `RoboMaster 开发板 C 型嵌入式软件教程 V1.0` — PID实现（第16章）、串级PID（第19章）
- `RoboMaster Assistant 调参软件` — 用于配置电机参数、开启电流环

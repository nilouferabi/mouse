# mouse —— 启诚智能鼠核心代码
---

## 一、文件总览

共 19 个文件（7 个 `.c`、11 个 `.h`、1 个 README），约 10800 行。

### 参与构建的源文件（6 个）

| 文件 | 行数 | 职责 | 备注 |
|---|---:|---|---|
| `main.c` | 2148 | 迷宫算法 + 主状态机 | `WAIT → START → MAZESEARCH → SPURTL` 四态循环；等高图、搜索法则、冲刺、Flash 存取、串口 |
| `Mouse_Drive.c` | 6444 | 底层驱动 + 全部运动控制 | 占全工程 60% 代码；1 ms 控制环、PID、红外、陀螺、转向/直行动作 |
| `Mouse_Drive_Init.c` | 400 | 外设初始化 | SysTick 1 ms、GPIO、ADC1+DMA（5 通道）、TIM1 PWM（25 kHz）、TIM2/TIM3 编码器、TIM6、LED、TIM4 |
| `zlg7289.c` | 241 | 数码管 + 键盘驱动 | GPIO 位操作模拟串行：`PB4`=CS、`PB6`=CLK、`PB5`=DIO、`PB3`=KEY |
| `stmflash.c` | 150 | 内部 Flash 读写 | 地图 `0x08018800`、起点 `0x08019000`、红外阈值 `0x08010000` |
| `stm32f10x_it.c` | 154 | 异常处理 | 8 个 handler 全空/`while(1)`；`startup` 里已有 weak 等价实现，**删掉不影响行为** |

### 不参与构建（1 个）

| 文件 | 行数 | 状态 |
|---|---:|---|
| `Mouse_Srv.c` | 302 | **未编译**。实现 45° 斜线路规划：`Diagonalway()`、`actiongenerate()`，并定义 `actionsequence[]`、`MapDiagonalway[]`、`DiagonalwayAction[]`。目前唯一调用点在 `main.c` 的 `mouseSpurt_45()` 里，而该函数没有被调用、已被链接器丢弃——所以整体是死代码。想启用必须同时改 `../linux/Makefile` 和调用链。 |

### 头文件（11 个）

| 文件 | 行数 | 内容 | 是否常改 |
|---|---:|---|---|
| `Mouse_Config.h` | 89 | 迷宫类型、终点坐标、搜索法则、速度、一格脉冲数 | ⭐ 最常改 |
| `Mouse_Drive.h` | 261 | 引脚定义、状态枚举、`__PID`/`__MOTOR` 结构体、`U_MAX`/`Deadband`、`__KP/__KI/__KD` | ⭐ 调 PID |
| `Maze.h` | 131 | 状态常量（`WAIT/START/MAZESEARCH/SPURTL`）、全局变量 extern 声明 | 改算法时 |
| `Micromouse.h` | 93 | 搜索法则编号、`MOUSEWAY_L/R/F/B` 相对方向宏 | 改算法时 |
| `Type.h` | 73 | `uint8/int8/uint16/int16/uint32/fp32/fp64` 等 | 基本不动 |
| `BitBand.h` | 82 | Cortex-M3 位带宏（`PBout(n)` 等） | 不动，ZLG7289 依赖它 |
| `zlg7289.h` | 77 | 数码管引脚宏 + 命令码（`0xA4` 复位、`0x88` 闪烁…） | 改显示时 |
| `stmflash.h` | 30 | 三个 Flash 地址常量 | 换芯片时 |
| `Mouse_Drive_Init.h` | 45 | 初始化函数声明 | 不动 |
| `stm32f10x_it.h` | 53 | 异常处理声明 | 不动 |
| `Mouse_Srv.h` | 7 | 声明 `Diagonalway()` / `actiongenerate()`（实现不存在） | 见下方坑 4 |

> 参考：各文件"定义的函数数 / 实际链接进固件的数量"——
> `main.c` 40/25、`Mouse_Drive.c` 73/29、`Mouse_Drive_Init.c` 14/13、
> `zlg7289.c` 11/7、`stmflash.c` 7/4。
> 差值都是没人调用、被 `--gc-sections` 丢掉或被内联掉的函数。

---

## 二、它实现了哪些功能

| 功能 | 实现位置 | 状态 |
|---|---|---|
| 迷宫搜索（右手/左手/中右/中左/中心五种法则） | `main.c` 的 `*Method()` | ✅ 实际只用中心法则 |
| 等高图 / 洪水填充 | `mapStepEdit()`、`floodMethodnew()` | ✅ |
| 岔路记录与回溯 | `crosswayCheck()`、`crosswayChoice()`、`GmcCrossway[]` | ✅ |
| 按等高图走回起点/终点 | `objectGoTo1()` | ✅ |
| 一冲（读 Flash 地图直接冲刺） | `mouseSpurt_CC()` | ✅ 需先完整探索一次并保存地图 |
| 45° 斜走 | `mouseSpurt_45()`、`objectGoTo_45()` | ❌ 未链接（死代码） |
| 定距直行 | `ONEBLOCK`（51600 脉冲/格）+ 编码器计数 | ✅ |
| 原地转向（90°/135°/180°） | 陀螺累加量 `GW` 到阈值即停 | ✅ |
| 贴墙定中 | `leftdis/rightdis` → `GsTpusle_T = 增益×(距离−目标)` | ⚠️ 受坑 3 影响 |
| 速度闭环 | 增量式 PI（`__GmSPID`） | ✅ |
| 方向闭环 | 转向速率 PI（`__GmWPID`） | ✅ |
| 红外分时采样 + 环境光差分 | `TIM6_IRQHandler()`（8 ms 一轮） | ✅ |
| 陀螺去极值滤波 | `GyroscopeFilter()`（8 点滑窗，去最大最小取平均） | ✅ |
| 7 段数码管显示 | `zlg7289.c` + `__mazeInfDebug()` / `sensorDebug()` | ✅ 显示坐标、方向、墙标志 |
| START 按键选模式 | `main.c` 中 `start` 计数 | ✅ 1 次=搜索，3~4 次=冲刺，8 次=只校准 |
| 地图/起点掉电保存 | `stmflash.c` | ✅ |
| 串口通信（USART3，9600 8N1，PB10/PB11） | `uart_sendchar`/`SendStr`/`Sendmap` | ⚠️ 函数在，但没有调用点 |

---

## 三、关键数据结构

| 变量 | 含义 |
|---|---|
| `GucMapBlock[16][16]` | 当前墙图。**每一位为 1 表示该方向可走**，bit0/1/2/3 对应绝对方向 0/2/4/6 |
| `GucMapBlock0` | 探索记录（判断"这个格子走没走过"） |
| `GucMapBlock1` | 洪水算法用的副本（边界在 `floodwall()` 里初始化） |
| `GucMapStep[16][16]` | 等高值，从终点往起点铺 |
| `GmcMouse`（`MAZECOOR`） | 当前坐标 `cX`/`cY` |
| `GucMouseDir` | 当前朝向：0=上、2=右、4=下、6=左 |
| `__GmSPID` / `__GmWPID` | 速度环 / 方向环的 `__PID` 结构体 |
| `__GmLeft` / `__GmRight` | 左右电机的 `__MOTOR` 结构体（脉冲目标、已走脉冲、PWM 值） |
| `jiao` / `GW` | 陀螺积分的两条角度量，转向判据分别用它们 |

---

## 四、改代码前必须知道的 6 个坑

1. **`SEARCHMETHOD` 宏是摆设**
   `Mouse_Config.h` 里定义了它，但 `crosswayChoice()` 写的是 `switch (2)`（= 中心法则）。
   想换法则必须改那一行 `switch`。

2. **转向函数有 5 套，只有 `_C` 那套生效**
   `Mouse_Drive.c` 定义了 73 个函数，链接进固件的只有 29 个。同一动作有
   `_C` / `_t` / `_v` / `_Y` / `yuandi` 多个版本，实际生效的是
   `mouseTurnleft_C` / `mouseTurnright_C` / `mouseTurnback`，
   加上 `mouseGoahead_L`、`mazeSearch`。
   **改 `_t`、`_v` 版本不会有效果**（它们被 `--gc-sections` 丢了）。

3. **`ADCmv()` 没有返回值**
   `Mouse_Drive.c:143` 原文：
   ```c
   float ADCmv(int AD)
   {
     float mvdianya;
     mvdianya=AD*3300/4096;      /* 算完就丢了，没有 return */
   }
   ```
   调用方拿到的是寄存器里的残留值，导致 `leftdis/rightdis/fldis/frdis` 变成常数、贴墙校正失效。
   正确写法：`return (float)AD * 3300.0f / 4096.0f;`

4. **`Mouse_Srv.h` 是"有声明没实现"**
   它被 `main.c` 和 `Mouse_Drive.h` include，但 `Diagonalway()` / `actiongenerate()`
   只在未编译的 `Mouse_Srv.c` 里。因为调用点在死函数 `mouseSpurt_45()` 内，链接才没报错。
   删这个头文件时必须同步删掉两处 `#include`。

5. **启动门槛：`start==1 && GucGoHead`**
   `GucGoHead` 只在"左前传感器看到墙、右前传感器没看到墙"时为 1（`Mouse_Drive.c:1022`）。
   空地上四个红外读数都低于阈值 → 永远不置位 → **按 START 没反应**。
   起跑格必须靠左墙摆放。另外 `startCheck()` 是"等松手"才返回的，按住不放会卡住。

---

## 五、最常改的地方

| 想改什么 | 去哪里 |
|---|---|
| 迷宫大小 / 终点坐标 / 搜索法则 | `Mouse_Config.h` |
| 速度上限、一格脉冲数 | `Mouse_Config.h`（`MAXSPEED` / `SEARCHSPEED` / `ONEBLOCK`） |
| PID 增益、死区、输出限幅 | `Mouse_Drive.h`（`__KP/__KI/__KD`、`Deadband`、`U_MAX`） |
| 红外判墙阈值 | `Mouse_Drive.c:68-80`（`GusDistance_*`，单位是 ADC 计数） |
| 贴墙目标距离 / 校正增益 | `Mouse_Drive.c:1208-1231`（`GsTpusle_T = 增益×(距离−目标)`） |
| 转向角度阈值 | `mouseTurnleft_C()` / `mouseTurnright_C()` 里的 `GW > 84000 / 93000` |
| 陀螺零位 | `Mouse_Drive.c:64`（`voltageDetectRef`） |

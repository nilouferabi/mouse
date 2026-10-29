# mouse —— 启诚智能鼠核心代码（重构版）

---

## 一、这次重构做了什么

| 项目 | 重构前 | 重构后 |
|---|---:|---:|
| `Scr/` 应用代码总行数 | 10 915 | 5 099（另有 309 行归档） |
| `Mouse_Drive.c` | 6 444 | 1 740 |
| `main.c` | 2 148 | 1 485 |
| `-Os` 固件 Flash | 24 420 B（**动作函数被折叠成死循环，跑不了**） | 26 284 B（可正常运行） |
| `-O0` 固件 Flash | 46 688 B | 42 200 B |

核心变化：

1. **重点参数集中到 `Mouse_Tuning.h`（140 个宏）**
   按“机械/编码器、速度环 PID、直行贴墙、90° 转向、掉头、搜索衔接、红外 ADC、陀螺、流程外设、时间基准”十组收口。
   以前调一个转弯要翻 6 个函数、改 4 个魔数；现在只改这里，数值与重构前逐字一致。

2. **删除不可达代码，保留可运行功能**
   - `Mouse_Drive.c` 删除 30 多个历史动作变体（`_t` / `_v` / `_90` / `_135` / `_45` / `_Y1` 等）、`__irCheck` 旧副本、`testEncoder`、`mouseGoahead` 系列旧版、`onestep/1/2`、`mouseTurnback1/_Y3/_js` 等；
   - `main.c` 删除 45° 冲刺链、重复的 `mouseSpurt` / `_CCC` / `_C`、`objectGoTo_Hui/liang/45`、未实现的 `objectGoTo_CHui` 调用等；
   - 删除 `speed_l/speed_r/gyro_l/gyro_r`（778 个从未引用的标定值）、`fldis/frdis`、`GusFreq_*`、`DisRX/LX` 等死数组/死变量。

3. **修复 3 个会直接影响跑车的确定缺陷**
   | 缺陷 | 位置 | 修复 |
   |---|---|---|
   | `ADCmv()` 算完电压没有 `return`，距离全是寄存器残值 | 原 `Mouse_Drive.c:143` | 返回 `AD*3300/4096` |
   | 左距离插值误用右红外余数 `decimal` | 原 `Mouse_Drive.c:1056` | 改用 `decimalr` |
   | 跨中断变量没有 `volatile`，GCC `-Os` 把忙等折叠成 `b .` | 全局 | `__MOTOR`/`__PID` 字段与 20+ 个共享变量加 `volatile` |

   现在 `-Os` 下除 5 个异常处理外，**没有任何函数被折叠成自循环**，可直接用默认 `make` 构建。

4. **配置宏真正生效**
   `crosswayChoice()` 由 `switch (2)` 改为 `switch (SEARCHMETHOD)`，改 `Mouse_Config.h` 里的搜索法则终于有效。

5. **其他确定性清理**
   - `__delay()` 的循环变量改 `volatile`，`-Os` 下延时不再被优化掉；
   - PID 积分项加 `TUNE_PID_I_LIMIT` 防 `int16` 溢出（只防溢出，不改变正常范围）；
   - 删除 `centralMethodnew()` 里 `if(ss=0)` 的无效调试片段；
   - `main()` 补 `int` 返回类型；
   - 头文件去掉与 `.c` 冲突的 `static` 声明、僵尸函数声明和旧引脚宏（真实映射以 `Mouse_Drive_Init.c` 为准）。

6. **45° 斜走链路归档**
   `Mouse_Srv.c/h` 移到 `Scr/legacy/`，不再参与构建，也不再被任何头文件包含。
   原 45° 动作函数已删除；如需恢复，见下方第五节。

---

## 二、现在的文件结构

```
Scr/
├── Mouse_Config.h          工程级配置：迷宫尺寸、终点、搜索法则、基础速度、ONEBLOCK
├── Mouse_Tuning.h          ★ 重点参数集中区（140 个宏，调车先看这里）
├── Maze.h                  应用层（迷宫算法 + 状态机）接口
├── main.c                  状态机、等高图/洪水、岔路记录、Flash 存取、串口调试
├── Mouse_Drive.h           运动层类型与接口（__MOTOR/__PID 字段为 volatile）
├── Mouse_Drive.c           PID、编码器、红外/陀螺采样、1 ms 控制环、动作函数
├── Mouse_Drive_Init.h
├── Mouse_Drive_Init.c      时钟、ADC+DMA、TIM1 PWM、TIM2/3 编码器、TIM6、风扇
├── zlg7289.c / .h          数码管 + 键盘
├── stmflash.c / .h         内部 Flash（地图 / 起点 / 红外阈值）
├── stm32f10x_it.c / .h     异常向量
├── Type.h / BitBand.h / Micromouse.h
├── README.md
└── legacy/
    ├── README.md           归档说明
    └── Mouse_Srv.c / .h    45° 斜线路径生成（未编译）
```

参与构建的仍是 6 个 `.c`：`main.c`、`Mouse_Drive.c`、`Mouse_Drive_Init.c`、`stmflash.c`、`zlg7289.c`、`stm32f10x_it.c`。

---

## 三、重点参数地图（都改 `Mouse_Tuning.h`）

| 想改什么 | 宏 | 当前值 | 说明 |
|---|---|---:|---|
| 走一格脉冲 | `TUNE_PULSE_PER_BLOCK` | `ONEBLOCK`=51600 | 46 处引用，改 `Mouse_Config.h` 的 `ONEBLOCK` 即可 |
| 搜索速度 | `SEARCHSPEED`（`Mouse_Config.h`） | 140 | 每 1 ms 编码器脉冲 |
| 冲刺/转弯速度 | `TUNE_TURN_SPEED` / `TUNE_GO_SPEED_*` / `TUNE_TURNBACK_RETURN_SPEED` | 145 / 120~200 / 150 | 各动作的 `sRef` |
| PID | `TUNE_PID_KP/KI/KD` | 20 / 0.1 / 0 | 全工程唯一 PID 增益 |
| PWM 限幅 | `TUNE_PID_U_MAX` | 1400 | TIM1 ARR=2880 → 最大占空比 48.6%，想更快改这里 |
| 贴墙目标 | `TUNE_WALL_L_TARGET_DIST` / `R` | 75 / 100 | 距墙 mm |
| 贴墙有效距离 | `TUNE_WALL_L_ACTIVE_DIST` / `R` | 120 / 170 | 超过此距离不参与校正 |
| 右转 90° 判据 | `TUNE_TURN90_W_SEARCH_R` / `_C_R` | 95000 / 93000 | 搜索段 / 冲刺段 |
| 左转 90° 判据 | `TUNE_TURN90_W_SEARCH_L` / `_C_L` | 86500 / 84000 | |
| 转向后续直行 | `TUNE_TURN90_PULSE_*` | 17000 / 21000 / 9600 | 右转 / 搜索左转 / 冲刺左转 |
| 掉头判据 | `TUNE_TURNBACK_GW*` | 290000 / 280000 / 300000 | 三个版本 |
| 掉头回程 | `TUNE_TURNBACK_RETURN_PULSE*` | 23000 / 26000 / 3000 | |
| 红外默认阈值 | `TUNE_IR_*_DEFAULT` | 见文件 | 单位 ADC 计数 |
| 红外查表 | `TUNE_IR_R_OFFSET_MV/STEP_MV` | 300 / 15 | 左右共用 |
| 陀螺零位 | `TUNE_GYRO_ZERO_DEFAULT` | 2041 | 上电应重新校准 |
| 陀螺换算 | `TUNE_GYRO_GW_SCALE` / `TUNE_GYRO_SENSITIVITY_MV` / `TUNE_GYRO_SYSTEM_PARAM` | 5121 / 1055 / 3.96 | |
| 风扇 | `TUNE_FAN_DUTY` / `TUNE_FAN_PWM_PERIOD` | 600 / 3600 | |
| 串口 | `TUNE_UART_BAUD` | 9600 | USART3 PB10/PB11 |
| 按键次数 | `TUNE_START_PRESS_*` | 1 / 3~4 / 8 | 搜索 / 直接冲刺 / 只校准 |

---

## 四、构建与验证

```bash
make                 # 默认 -Os，已验证可正常运行
make OPT="-O0 -g3"   # 调试版
make size
make flash
```

本次重构后的验证结果：

| 构建 | Flash | RAM | 折叠成死循环的函数 |
|---|---:|---:|---|
| `-Os` | 26 284 B（20.05%） | 8 440 B（41.21%） | 仅 `HardFault` 等 5 个异常处理 |
| `-O0` | 42 200 B（32.20%） | 8 464 B（41.33%） | 仅 `LoopForever` |

> 检查方法：`arm-none-eabi-objdump -d linux/build/Maze.elf`，确认动作函数里的忙等每轮都重新读取 `volatile` 变量。

---

## 五、归档的 45° 斜走功能

`Scr/legacy/Mouse_Srv.c` 实现了 `Diagonalway()` / `actiongenerate()`，但原工程从未把它加入构建，调用链也只挂在从未被调用的 `mouseSpurt_45()` 上。

本次重构删除了 `mouseSpurt_45()`、`objectGoTo_45()` 以及 8 个 45°/135° 动作函数。需要恢复时：

1. 从 git 基线取回原文件：`git show baseline-before-refactor:Scr/main.c > /tmp/main_v1.c`；
2. 在原 `Mouse_Drive.c` 中找回 `mouseTurnleft_45_t` / `_45_v` / `_135_t` / `_135_v` 等函数；
3. 把 `legacy/Mouse_Srv.c` 加回 `linux/Makefile`；
4. 重新接好 `mouseSpurt_45 → Diagonalway → actiongenerate → objectGoTo_45` 调用链。

> 注意：归档代码引用的是旧版全局变量（`actionsequence`、`STRAIGHT90L`、`LT90L` 等），直接编译会报错，需要按新结构适配。

---

## 六、仍然保留、但需要实车确认的点

1. **终点 Y 坐标三处不一致**
   `Mouse_Config.h` 的 `YDST0/YDST1 = 6,6`；`main()` 只把 X 改成 8/7；`goalwall()` 内部按 7/8 处理。
   本次重构**没有改行为**，只把 X 的魔法值收进 `TUNE_GOAL_X0/X1`。上实车前请确认 Y 是否应为 8/7。

2. **TIM6 与 SysTick 双触发**
   `SysTick_Handler()` 仍每 1 ms 手工调用一次 `TIM6_IRQHandler()`，而 TIM6 中断本身也开着（约 6 kHz 推进 8 状态机）。为保持原时序没有删，代码里有注释。

3. **`__GucDistance[__FRONT]` 恒为 0**
   硬件没有单独的正前方槽位，因此 `DenggaoCheck()` 目前恒为假，搜索中“遇墙停车”不会触发。保留原逻辑并在函数处注明。

4. **`MouseTurnleft_C()` 的 286.67**
   其余位置都是 2886.67，这里疑似漏写一位。为避免改变实车行为，原值保留，只加了标记。

5. **忙等没有超时**
   动作函数仍用 `while ((counter + 200) <= target);` 等编码器；断线/堵转会死等。重构没有引入超时机制。

6. **`GucMapBlock1[0][ucTemp + 1]` 的越界**
   `main()` 起点镜像时，`ucTemp == MAZETYPE-1` 会写到下一行首元素。原逻辑保留，未改动。

7. **`mouseTurnbackqidian()` 的 `(GucMouseDir + 2) % 4`**
   与其他掉头函数的 `+4 % 8` 不同，疑似历史遗留。原值保留。

---

## 七、最常改的地方（速查）

| 想改什么 | 去哪里 |
|---|---|
| 迷宫大小 / 终点 / 搜索法则 | `Mouse_Config.h` |
| 速度、PID、贴墙、转向、掉头 | `Mouse_Tuning.h` |
| 红外阈值默认值 | `Mouse_Tuning.h` 的 `TUNE_IR_*_DEFAULT` |
| 标定查表（mV→mm） | `Mouse_Drive.c` 的 `disCheck0..3` |
| 动作函数细节 | `Mouse_Drive.c`（参数已宏化） |
| 状态机 / 等高图 / 洪水 | `main.c` |
| 引脚与外设初始化 | `Mouse_Drive_Init.c` |
| Flash 分区 | `stmflash.h` |

---

*重构基线：git tag `baseline-before-refactor`（提交 `8ad8dd0`）。*

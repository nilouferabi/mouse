# mouse —— 启诚智能鼠固件

STM32F103CB 电脑鼠固件：4 向迷宫搜索、4 向冲刺（8 向代码保留，当前默认关闭）、红外墙图、编码器/陀螺转向、负压风扇和状态灯。

---

## 一、当前功能

- 4 向探索：每次正交走一格，用左右/前左/前右红外更新墙图；
- 8 向冲刺代码保留；当前 `TUNE_8DIR_ENABLE = 0`，冲刺先走 4 向；
- 搜索阶段不使用斜向，保证识墙可靠；
- 冲刺阶段支持斜向，但斜向必须满足 2×2 方块内部无墙，避免切角；
- 负压风扇只在冲刺阶段打开；
- 状态灯：搜索慢闪，靠近终点快闪，冲刺常亮；
- 地图、起点、红外阈值可通过内部 Flash 保存/读取；
- 提供主机侧 8 邻域规划器测试，不依赖硬件。

## 二、目录结构

```text
Scr/
├── app/                        应用层
│   ├── main.c                  主状态机
│   ├── maze.h                  共享迷宫数据、状态常量、外部变量
│   ├── maze_search.c/h         等高图、洪水填充、岔路决策、搜索
│   ├── path_4dir.c/h           4 向等高图路径执行
│   ├── maze_storage.c/h        地图/起点/红外阈值 Flash 存档
│   ├── path_8dir.c/h           8 邻域规划 + 冲刺执行
│   ├── serial_debug.c/h        USART3 调试串口、地图发送
│   └── maze_search_internal.h  搜索与 4 向路径的内部共享声明
├── control/
│   ├── motion_control.c/h      PID、编码器、红外/陀螺采样、1 ms 控制环、4 向动作
│   ├── motion_8dir.c/h         8 向转向、正交/斜向走一格
│   └── motion_internal.h       控制层内部共享声明
├── bsp/
│   ├── board_init.c/h          时钟、TIM1 PWM、TIM2/3 编码器、按键、MouseInit
│   ├── sensor_init.c/h         ADC+DMA、红外发射 GPIO、TIM6
│   ├── fan.c/h                 负压风扇（TIM4_CH3 / PB8）
│   ├── status_led.c/h          状态灯（PB12）
│   ├── display.c/h             ZLG7289 数码管 + 键盘
│   ├── flash.c/h               内部 Flash 读写
│   └── interrupts.c/h          异常向量/中断入口
├── config/
│   ├── mouse_config.h          迷宫尺寸、终点、搜索法则、ONEBLOCK
│   ├── tuning.h                ★ 重点参数集中区
│   ├── types.h                 uint8/int8/fp32 等类型
│   └── micromouse.h            迷宫坐标/方向宏
├── util/
│   ├── delay.c/h               简单忙等延时
│   └── bitband.h               位带操作
├── test/test_path_8dir.c       主机侧 8 邻域规划器测试
└── README.md
```

参与固件构建的 16 个 `.c`：

- `app/`：`main.c`、`maze_search.c`、`path_4dir.c`、`maze_storage.c`、`path_8dir.c`、`serial_debug.c`
- `control/`：`motion_control.c`、`motion_8dir.c`
- `bsp/`：`board_init.c`、`sensor_init.c`、`fan.c`、`status_led.c`、`display.c`、`flash.c`、`interrupts.c`
- `util/`：`delay.c`

GCC 包含路径：

```make
-I$(ROOT)/Scr
-I$(ROOT)/Scr/app
-I$(ROOT)/Scr/control
-I$(ROOT)/Scr/bsp
-I$(ROOT)/Scr/config
-I$(ROOT)/Scr/util
```

IAR 需要手动加入上述 16 个 `.c`，并设置同样的 include path。

## 三、构建与烧录

```bash
make                 # 默认 -Os
make OPT="-O0 -g3"   # 调试版
make test-nav        # 主机侧 8 邻域规划器测试
make size
make flash
```

当前构建结果：

| 构建 | Flash | RAM |
|---|---:|---:|
| `-Os` | 25 144 B（19.18%） | 6 832 B（33.36%） |
| `-O0` | 41 524 B（31.68%） | 6 856 B（33.48%） |

`make test-nav` 覆盖：

- 空地 16×16 斜向距离；
- 缺墙时禁止斜向；
- 2×2 内部有墙时禁止切角；
- 未知地图无路径；
- 执行器到终点并对齐正交方向。

## 四、运行状态机

`app/main.c` 当前使用四个状态：

```text
WAIT → START → MAZESEARCH → SPURTL → WAIT
```

| 状态 | 当前行为 |
|---|---|
| `WAIT` | 扫描红外；1 秒窗口内按 1 次进入探图，按 2 次检查地图通路后冲刺；3 次及以上回停止；陀螺零位在上电时自动校准 |
| `START` | 执行第一段 `mazeSearch()`，必要时处理左侧起点镜像 |
| `MAZESEARCH` | 中心法则探索到终点/无支路，保存地图，回到起点后停止 |
| `SPURTL` | 读取地图，冲刺前用 `Path4_CanReachGoal()` 检查 4 向通路；有路执行 `mouseSpurt_CC()`，到终点停止；无路回 `WAIT` |

冲刺结束或中止后回到 `WAIT`。

> 当前 1/2 次模式仍要求 `GucGoHead == 1`（起点姿态判据）；否则按键只计数，不启动。

## 五、格子、坐标和终点

代码里的格子是**逻辑格子**，不是毫米坐标：

- `MAZETYPE = 16`，坐标范围 `x/y = 0..15`；
- 一格 = `ONEBLOCK = 51600` 个编码器脉冲；
- 斜向一格 = `ONEBLOCK_45 = 36410` 个脉冲；
- `x` 向东增加，`y` 向北增加；
- 初始 `GmcMouse = (0,0)`，初始方向 `UP`；
- 从左侧起点出发时，启动阶段会把地图镜像到 `x=15` 一侧。

每个格子的低 4 位表示四个方向：

| bit | 方向 | 1 | 0 |
|---|---|---|---|
| bit0 | 北 | 有路 | 有墙 |
| bit1 | 东 | 有路 | 有墙 |
| bit2 | 南 | 有路 | 有墙 |
| bit3 | 西 | 有路 | 有墙 |

三张地图：

- `GucMapBlock`：物理墙图，搜索/规划使用；
- `GucMapBlock0`：探索记录；
- `GucMapBlock1`：洪水填充副本；
- `GucMapStep`：等高值，实时计算，不保存。

### 终点判断

当前终点判断是**纯坐标匹配**，没有物理终点传感器：

```c
if (((GmcMouse.cX == GucXGoal0) && (GmcMouse.cY == GucYGoal0)) ||
    ((GmcMouse.cX == GucXGoal1) && (GmcMouse.cY == GucYGoal1)) ||
    ((GmcMouse.cX == GucXGoal0) && (GmcMouse.cY == GucYGoal1)) ||
    ((GmcMouse.cX == GucXGoal1) && (GmcMouse.cY == GucYGoal0)))
{
    /* 认为到达终点 */
}
```

当前坐标配置：

- `XDST0/XDST1 = 6/6`，`YDST0/YDST1 = 6/6`；
- `START` 分支在左侧起点时把 `GucXGoal0/1` 改成 `8/7`；
- `goalwall()` 内部按中心 `7/8` 处理。

因此当前终点定义并不完全一致，实际搜索目标可能落在 `Y=6`，而中心 2×2 是 `Y=7/8`。这是当前代码的已知风险。

## 六、搜索阶段

### 1. 直行

`mazeSearch()` 是一段连续直行，不是走一格就停：

```c
__GucMouseState = __GOAHEAD;
__GiMaxSpeed    = SEARCHSPEED;
__GmRight.uiPulse = MAZETYPE * TUNE_PULSE_PER_BLOCK;
__GmLeft.uiPulse  = MAZETYPE * TUNE_PULSE_PER_BLOCK;
__GmRight.cState = __MOTORRUN;
__GmLeft.cState  = __MOTORRUN;
```

每累计一个 `ONEBLOCK`：

```c
__mouseCoorUpdate();   /* 坐标加一格 + __wallCheck() */
```

搜索直行不启用陀螺航向保持，只靠：

- 左右轮速度环保持平均速度；
- 左墙/右墙贴墙差速；
- 两侧都无墙时，仅靠左右轮编码器速度相等。

贴墙优先级：

```c
if (leftdis < 120) {
    GsTpusle_T = 1 * (leftdis - 75);
}
else if (rightdis < 170) {
    GsTpusle_T = -1 * (rightdis - 100);
}
else {
    GsTpusle_T = 0;
}
```

两侧都有墙时只使用左侧墙，不做左右同时居中。

### 2. 什么时候退出直行

- 前方两侧都检测到墙，且左右两侧没有近处侧墙：判定死路，退出；
- 左侧从有墙变为开口：向前补 `TUNE_SEARCH_LEFT_ADVANCE`，到岔路口中心后退出；
- 右侧从有墙变为开口：向前补 `TUNE_SEARCH_RIGHT_ADVANCE`，到岔路口中心后退出；
- 走出当前段后，由 `crosswayCheck()` / `crosswayChoice()` 决定转向。

### 3. 岔路决策

- `crosswayCheck()` 统计当前格未探索支路；
- `crosswayChoice()` 按 `SEARCHMETHOD` 选择右手、左手、中右、中左等法则；
- 到终点或无未探索支路后，`centralMethodnew()` 结束搜索；
- 无支路时用 `floodMethodnew()` 继续洪水/回程。

## 七、4 向回程与 8 向冲刺

> 当前 `TUNE_8DIR_ENABLE = 0`，实际冲刺使用 `mouseSpurt_CC()` 的 4 向路径。
> 下面的 8 向流程是保留的可选实现，改回 1 后启用。


### 4 向路径

`path_4dir.c`：

- `objectGoTo1(cXdst,cYdst)`：建等高图，按等高值下降走到目标；
- `mouseSpurt_CC()`：从四个终点格中选最近的一个，调用 `objectGoTo1()`。

### 8 向冲刺

`path_8dir.c`：

1. `mouseSpurt_8_to_goal()` 对四个终点格分别建 8 邻域距离图，选最近目标；
2. `Nav_NextStep()` 选“最短路 + 平局少转向”的下一步；
3. `mouseTurnTo(dir)` 原地转到目标方向；
4. `mouseGoOneBlock(dir)` 走一格正交或斜向；
5. 每走一格更新 `GmcMouse`；
6. 到目标后把朝向对齐到最近正交方向。

斜向规则：

```text
(x,y) → (x+dx, y+dy) 允许，当且仅当 2×2 方块四条内边都通。
```

斜向运动中如果两个前向近距同时触发，立即停车并返回失败。

## 八、1 ms 控制环

`SysTick_Handler()` 每 1 ms 执行：

- `Led_Tick()`；
- 编码器采样；
- 手动调用 `TIM6_IRQHandler()`；
- 电机状态机；
- 速度/差速 PID；
- 加减速。

### 直行控制公式

```text
平均速度反馈 = (右轮速度 + 左轮速度) / 2
差速反馈     = (右轮速度 - 左轮速度) / 2

左轮 PWM = 速度环输出 - 差速环输出
右轮 PWM = 速度环输出 + 差速环输出
```

差速目标：

```text
4 向直行：TUNE_PID_WALL_FEEDFORWARD * GsTpusle_T
8 向斜向：TUNE_PID_WALL_FEEDFORWARD * (TUNE_8DIR_GYRO_FF * jiao)
```

当前参数：

| 参数 | 值 |
|---|---:|
| `TUNE_PID_KP/KI/KD` | 20 / 0.1 / 0 |
| `TUNE_PID_U_MAX` | 1400（TIM1 ARR=2880，约 48.6% 占空比） |
| `TUNE_PID_DEADBAND` | 3 |
| `TUNE_PID_ERROR_IMAX` | 10 |
| `TUNE_PID_WALL_FEEDFORWARD` | 0.5 |
| `TUNE_WALL_L_TARGET_DIST` | 75 mm |
| `TUNE_WALL_R_TARGET_DIST` | 100 mm |
| `TUNE_WALL_L_ACTIVE_DIST` | 120 mm |
| `TUNE_WALL_R_ACTIVE_DIST` | 170 mm |

### 8 向斜向

- `mouseGoOneBlock(1)` 设置 `__GucMouseState = __GOAHEAD_45`；
- `jiao = 0`；
- 陀螺积分角偏离 0 时，`GsTpusle_T` 产生反向差速；
- 走完 `ONEBLOCK_45` 后停车；
- `mouseGoOneBlock(0)` 正交直行仍使用 4 向贴墙/轮速逻辑，不使用陀螺航向保持。

## 九、转向

### 4 向 90°/180°

- `mouseTurnright()`：`GucMouseDir = (dir + 2) % 8`；
- `mouseTurnleft()`：`GucMouseDir = (dir + 6) % 8`；
- `mouseTurnback()`：`GucMouseDir = (dir + 4) % 8`；
- 90° 判据使用 `GW`（陀螺偏离零位的绝对值累加）；
- 掉头判据使用更高的 `GW` 阈值，转完后补一段回程脉冲。

### 8 向原地转向

`__turnInPlace(steps45)`：

- `steps45` 正数右转，负数左转；
- 45° = ±1，90° = ±2，135° = ±3，180° = ±4；
- `jiao` 为有符号陀螺积分角；
- 有 2 s 超时；
- `TUNE_8DIR_TURN_TRIM_R/L` 修正左右角度差。

## 十、红外与陀螺

### 红外采样

TIM6 8 状态机循环：

```text
关发射 → 采环境光 → 开发射 → 采反射光
ucIRCheck[i] = ADC - Sunlight
```

四路模拟红外：

| 槽位 | 位置 | 用途 |
|---|---|---|
| `__LEFT` | 左 | 左墙、左岔路 |
| `__FRONTL` | 前左 | 前墙、前近距 |
| `__FRONTR` | 前右 | 前墙、前近距 |
| `__RIGHT` | 右 | 右墙、右岔路 |

### 墙判定

`__wallCheck()` 用 `ucIRCheck` 和 `GusDistance_*` 阈值生成墙位：

- `GusDistance_L/R/FL/FR_Near`；
- `GusDistance_L/R/FL/FR_Far`；
- `__GucDistance[]` 使用位标志：`0x01` 远、`0x02` 近。

墙图同时写入：

- `GucMapBlock`；
- `GucMapBlock0`；
- `GucMapBlock1`。

### 连续距离

左右墙距离 `leftdis/rightdis` 使用四路独立 6 点表做分段线性插值：

```c
/* mV 升序、mm 降序；先做上下界钳位，再找相邻两点 */
if (mv <= mv[0]) return mm[0];
if (mv >= mv[5]) return mm[5];

for (i = 0; i < 5; i++)
    if (mv <= mv[i + 1]) break;

return mm[i] + (mm[i + 1] - mm[i]) * (mv - mv[i]) / (mv[i + 1] - mv[i]);
```

当前状态：

- 四路独立校准：`FL / FR / L / R`；
- 每路 6 个标定点，`Ir_Distance()` 做分段线性插值；
- 数组按 mV 升序、mm 降序排列；
- 插值前做上下界钳位，不会访问表外索引；
- `disCheck0/1` 是保留的旧 201 点参考表，当前不参与计算；
- 旧 `disCheck2/3` 表已移除，新逻辑使用 6 点表；
- 前方距离也会计算到 `fldis/frdis`，但墙图仍以阈值为准；
- 没有 magic/CRC 的红外阈值存档，`Storage_LoadIrThresholds()` 默认未被调用。

### 陀螺

上电后 `Gyro_Calibrate()` 在电机停止、车辆静止时采样约 3 秒，
同时更新 `zijiaozheng`（用于 `jiao`）和 `voltageDetectRef`（用于 `GW`）。
校准期间不要移动车体。


- `GW`：陀螺偏离零位的绝对值累加，用于 90°/180° 判据；
- `jiao`：有符号陀螺积分角，用于 8 向斜向和原地转向；
- `voltageDetectRef`：上电自动零漂校准值；
- 陀螺零位、比例、灵敏度在 `config/tuning.h`。

## 十一、风扇与状态灯

### 负压风扇

- 硬件：TIM4_CH3 → PB8，AF 推挽；
- `Fan_Init()` 在 `MouseInit()` 中初始化，默认关闭；
- 只在 `SPURTL` 冲刺阶段 `Fan_On()`，结束/中止 `Fan_Off()`；
- 搜索、校准、`WAIT` 不打开；
- `TUNE_FAN_DUTY = 600`，`TUNE_FAN_PWM_PERIOD = 3600`，约 16.7% 占空比。

### 状态灯

- 硬件：PB12，高电平点亮（`TUNE_LED_ACTIVE_HIGH = 1`）；
- `Led_Tick()` 由 SysTick 每 1 ms 调用；
- `LED_MODE_SEARCH`：100 ms 亮 / 1 s 周期；
- 搜索中到任一终点格曼哈顿距离 ≤ 2：80 ms 亮 / 160 ms 周期快闪；
- `LED_MODE_SPRINT`：常亮；
- `LED_MODE_OFF`：灭。

## 十二、地图存档

保存的是墙/连通性地图，不是红外电压或距离：

- `Storage_SaveMap()` 把 `GucMapBlock` 写到 `MAPE_SAVE_ADDRESS = 0x08018800`；
- 写 `TUNE_FLASH_MAP_WORDS = 256` 个半字 = 512 B；
- `Storage_LoadMap()` 读回并还原 `GucMapBlock`；
- 起点写到 `START_SAVE_ADDRESS = 0x08019000`；
- 红外阈值地址 `IR_SAVE_ADDRESS = 0x08010000`；
- 没有 magic/CRC/版本/地图完整度；
- 空片读成 `0xFF` 会得到“所有方向都通”的假图。

## 十三、重点参数

> 单位：速度 = 每 1 ms 编码器脉冲；PWM = TIM1 CCR；距离 = mm；
> 红外阈值 = ADC 计数；陀螺角度 = 度。

| 分组 | 宏 | 当前值 / 说明 |
|---|---|---|
| 迷宫 | `MAZETYPE` | 16 |
| 终点 | `XDST0/1`、`YDST0/1` | 6/6、6/6 |
| 运行期终点 | `TUNE_GOAL_X0/X1` | 8/7 |
| 一格脉冲 | `ONEBLOCK` | 51600 |
| 斜向脉冲 | `ONEBLOCK_45` | 36410 |
| 搜索速度 | `SEARCHSPEED` | 由 `mouse_config.h` 配置 |
| 冲刺模式 | `TUNE_8DIR_ENABLE` | 0（当前 4 向） |
| 8 向速度 | `TUNE_8DIR_SPEED_ORTHO/DIAG` | 180 / 150 |
| 90° 转向速度 | `TUNE_TURN_SPEED` | 145 |
| 掉头回程 | `TUNE_TURNBACK_RETURN_SPEED` | 150 |
| PID | `TUNE_PID_KP/KI/KD` | 20 / 0.1 / 0 |
| PWM 上限 | `TUNE_PID_U_MAX` | 1400 |
| 贴墙目标 | `TUNE_WALL_L/R_TARGET_DIST` | 75 / 100 |
| 贴墙有效距离 | `TUNE_WALL_L/R_ACTIVE_DIST` | 120 / 170 |
| 8 向旋转 | `TUNE_8DIR_TURN_SPIN` | 120 |
| 8 向角度修正 | `TUNE_8DIR_TURN_TRIM_R/L` | 0.95 |
| 8 向陀螺保持 | `TUNE_8DIR_GYRO_FF` | 3 |
| 8 向超时 | `TUNE_8DIR_TURN/MOVE_TIMEOUT_MS` | 2000 / 6000 |
| 风扇 | `TUNE_FAN_DUTY` / `TUNE_FAN_PWM_PERIOD` | 600 / 3600 |
| 灯 | `TUNE_LED_ACTIVE_HIGH` | 1 |
| 按键窗口 | `TUNE_START_WINDOW_MS` | 1000 ms |
| 红外标定 | 4 路 × 6 点 | 见 `motion_control.c` |
| 地图存档 | `TUNE_FLASH_MAP_WORDS` | 256 |

### 需要实车测量的参数

标记说明：

- ★ 必须实测/标定，否则不要直接高速跑；
- ○ 可以先给初值，再试跑微调；
- — 配置项，不需要测。

| 标记 | 参数 | 去哪里改 | 怎么测 | 修改方式 |
|---|---|---|---|---|
| ★ | `ONEBLOCK` | `config/mouse_config.h` | 架空或空地走一格，量车体中心实际位移；重复 3~5 次取平均 | 实际走多了就减小，走少了就增大；按 `旧值 × 目标距离 / 实际距离` 调 |
| ★ | `ONEBLOCK_45` | `config/mouse_config.h` | 2×2 空方块走斜向一格，量实际对角线位移 | 同 `ONEBLOCK`，按比例调整；先保证正交准确再调斜向 |
| ★ | 4 路 6 点表 `irMvFL/FR/L/R`、`irMmFL/FR/L/R` | `control/motion_control.c` | 每路单独：墙面放在 6 个已知距离（如 50/75/100/150/200/300 mm），读对应 `ucIRCheck → mV` | 每个传感器 6 组 `(mV, mm)`；`irMv*` 必须升序、`irMm*` 对应降序；改完重新编译 |
| ★ | 4 路红外阈值 `TUNE_IR_FL/FR/L/R_*_DEFAULT` | `config/tuning.h` | 在“应该判有墙/无墙”的距离读 `ucIRCheck`，取中间值并留回差 | 按每个传感器分别改 Near/Far；不要四路共用一个值 |
| ★ | 红外饱和 `TUNE_IR_FL/FR/L/R_SAT` | `config/tuning.h` | 近距离贴墙，观察 `ucIRCheck` 最大值和噪声 | 超过实际最大读数并留余量；不要设得比正常读数还低 |
| ★ | `TUNE_WALL_L/R_TARGET_DIST` | `config/tuning.h` | 车在走廊中间，量左右墙实际距离；看 `leftdis/rightdis` | 目标墙距按车体宽度和赛道调；左 75/右 100 只是初值 |
| ★ | `TUNE_WALL_L/R_ACTIVE_DIST` | `config/tuning.h` | 逐渐拉远墙，直到贴墙修正开始/结束 | 保证目标距离以内有效，又不会把远处墙误当侧墙 |
| ★ | `TUNE_TURN90_W_SEARCH_R/L`、`_C_R/L` | `config/tuning.h` | 用角度尺/陀螺仪看 90° 转到位时的 `GW` 值 | 转不够就减小阈值，转过头就增大；搜索段和冲刺段分开标 |
| ★ | `TUNE_TURN90_PULSE_*` | `config/tuning.h` | 90° 转完后量车是否到下一格中心 | 转后走多了减小脉冲，走少了增大 |
| ★ | `TUNE_TURNBACK_GW_*`、`TUNE_TURNBACK_*_PULSE` | `config/tuning.h` | 180° 掉头，量最终朝向和回程位置 | 先调 `GW` 判据，再调回程脉冲 |
| ○ | `SEARCHSPEED` | `config/mouse_config.h` | 低速跑完整迷宫，看是否丢步/撞墙/停车 | 先低后高；不要用高速值直接做第一次实车 |
| ○ | `TUNE_PID_KP/KI/KD` | `config/tuning.h` | 直行给阶跃目标，看速度响应和过冲 | 先 `KP`，再加少量 `KI`；`KD` 一般先 0 |
| ○ | `TUNE_PID_U_MAX` | `config/tuning.h` | 观察急加速/转弯时 PWM 是否饱和 | TIM1 ARR=2880，1400 约 48.6%；根据驱动和电池调 |
| ○ | `TUNE_8DIR_TURN_TRIM_R/L` | `config/tuning.h` | 只在使用 8 向时：转 45/90/135/180，量实际角度 | 转不够调大，转过头调小；左右分开 |
| ○ | `TUNE_8DIR_SPEED_ORTHO/DIAG` | `config/tuning.h` | 只在使用 8 向时：空地和斜向试跑 | 先 80~100，确认不丢步再提速 |
| ○ | `TUNE_8DIR_GYRO_FF` | `config/tuning.h` | 只在使用 8 向时：斜向直行看是否跑偏 | 跑偏方向回正不够就增大；抖动就减小 |
| ○ | `TUNE_FAN_DUTY` | `config/tuning.h` | 测负压吸力、泵电流、电池压降 | 600 是 16.7%；泵起不来可先试 1800/3600，注意电流 |
| — | `TUNE_START_WINDOW_MS` | `config/tuning.h` | 人手连按节奏 | 当前 1000 ms；不好按可改 600~1200 |
| — | `TUNE_8DIR_ENABLE` | `config/tuning.h` | 选择 4 向/8 向冲刺 | 当前 0；改 1 启用 8 向 |
| — | `XDST0/1`、`YDST0/1`、`TUNE_GOAL_X0/X1`、`goalwall()` | `mouse_config.h` / `maze_search.c` | 终点坐标是配置，不是测量 | 当前坐标不一致，先统一成中心 2×2 再跑 |

> `TUNE_GYRO_ZERO_DEFAULT` 是自动校准失败时的后备值，正常不需要手动测；
> 如果更换陀螺或发现 `jiao/GW` 比例不对，再用转台标定 `TUNE_GYRO_GW_SCALE` 和 `TUNE_GYRO_SENSITIVITY_MV`。

**建议标定顺序：**

1. `ONEBLOCK` / `ONEBLOCK_45`；
2. 四路 6 点红外表和四路阈值；
3. 贴墙目标/有效距离；
4. 90°/180° 的 `GW` 和脉冲；
5. PID、搜索速度；
6. 最后才开 8 向、调 45°/135° 和风扇。

## 十四、调试

### 断点

| 位置 | GDB 命令 | 用途 |
|---|---|---|
| 状态机入口 | `break main` | 启动流程 |
| 初始化完成 | `ready` | 停到 `app/main.c:72`，可调用运动原语 |
| 红外测压 | `./debug.sh --ir` | 停在 `Sensor_IrqInit` 前 |
| 搜索到终点 | `break centralMethodnew` | 探索循环 |
| 洪水/回程 | `break floodMethodnew` | 搜索结束后的洪水与回程 |
| 4 向路径执行 | `break objectGoTo1` | 按等高图走 |
| 等高图 | `break mapStepEdit` | 看 `GucMapStep` |
| 4 向冲刺 | `break mouseSpurt_CC` | 选最近终点格 |
| 单格搜索 | `break mazeSearch` | 每格墙图更新 |
| 8 向转向 | `break mouseTurnTo` | 45°/90°/135°/180° |
| 8 向单格 | `break mouseGoOneBlock` | 正交/斜向走一格 |
| 1 ms 控制环 | `break SysTick_Handler` | PID、编码器、墙跟随 |
| 墙图更新 | `break __wallCheck` | 当前格四方向判定 |
| 风扇 | `break Fan_On` / `break Fan_Off` | 只在冲刺打开 |
| 灯光 | `break Led_SetMode` | SEARCH/SPRINT/OFF |

### GDB 命令

```text
st        状态机 / 红外 / PID / 电机
params    PID、sRef、红外阈值、陀螺零位
ready     运行到 app/main.c:72
fwd       沿当前朝向直行一格（mouseGoOneBlock(0)）
irvol     红外测压：ADC + mV + 环境光差值
iron 1..4 打开 FL/FR/L/R 发射管并测电压
irtest    自动扫四路发射管
c         继续
```

直行测试：

```text
./debug.sh
(gdb) ready
(gdb) fwd
```

`mouseGoOneBlock(0)` 不会自动转向、不会更新 `GmcMouse` 坐标、不会查正交墙。换方向先：

```text
(gdb) call mouseTurnTo(2)   # 0=北 2=东 4=南 6=西
(gdb) fwd
```

### 注意

- `-Os` 下静态函数可能被内联；需要稳定断点时用 `make OPT="-O0 -g3"`；
- `SysTick_Handler` 每 1 ms 执行，直接断会像死机；用 `ignore` 或条件断点；
- 静态函数可用文件限定，例如 `break 'maze_search.c'::floodMethodnew`；
- 正常调试用 `./debug.sh`，红外测压用 `./debug.sh --ir`。

## 十五、当前限制和风险

1. **终点坐标不一致**
   `XDST/YDST = 6/6`，运行期 X 被改成 8/7，`goalwall()` 按 7/8 处理；实际目标可能落在 `Y=6`。

2. **没有物理终点识别**
   只有 `GmcMouse` 坐标匹配，没有摄像头、信标、颜色或专用终点传感器。

3. **坐标是开环推算**
   编码器走够 `ONEBLOCK` 就按方向加一格；打滑、丢步、转向误差不会被传感器纠正。

4. **`mouseTurnbackqidian()` 方向更新错误**
   当前是 `(GucMouseDir + 2) % 4`，180° 应为 `(GucMouseDir + 4) % 8`。

5. **搜索直行没有陀螺航向保持**
   有墙时靠贴墙，无墙时只靠左右轮编码器速度相等；轮径/摩擦差异会跑偏。

6. **搜索直行没有超时**
   `mazeSearch()` 的 `while` 没有 `GucSysTickMs` 超时；红外/墙检测失效时可能一直走。

7. **`uiPulse = MAZETYPE * ONEBLOCK` 不是硬停止条件**
   每格会递减，但循环没有在 `uiPulse` 到 0 时强制退出。

8. **启用 8 向时中止没有错误码**
   `mouseSpurt_8_to_goal()` 返回 `NAV_ABORT` 时主状态机停车回 `WAIT`，但不区分转向超时、直行超时、前向安全还是无路。

9. **两侧都有墙时只跟左墙**
   `if / else if` 的结构决定右侧不参与同时居中。

10. **红外距离表**
    当前四路各 6 点标定，`disCheck0/1` 旧表仅保留参考；6 点初始值由旧表抽样，必须按实车传感器/墙面重新标定。

11. **TIM6 与 SysTick 双触发**
    `SysTick_Handler()` 每 1 ms 手动调用一次 `TIM6_IRQHandler()`，实际约 6 kHz 推进 8 状态机。

12. **`DenggaoCheck()` 恒为假**
    `__GucDistance[__FRONT]` 从未写入，搜索中“遇墙停车”不会触发。

13. **`mouseTurnleft_C()` 的 286.67**
    其他位置是 2886.67，左转补偿系数不一致。

14. **Flash 存档无校验**
    没有 magic/CRC/版本/完整度；空片读成 `0xFF` 会得到全通假图。

## 十六、最常改的位置

| 想改什么 | 去哪里 |
|---|---|
| 迷宫大小 / 终点 / 搜索法则 | `config/mouse_config.h` |
| 速度、PID、贴墙、转向、掉头 | `config/tuning.h` |
| 4/8 向冲刺开关 | `config/tuning.h` 的 `TUNE_8DIR_ENABLE` |
| 8 向速度 / 角度修正 / 超时 | `config/tuning.h` 的 `TUNE_8DIR_*` |
| 斜向一格脉冲 | `config/mouse_config.h` 的 `ONEBLOCK_45` |
| 红外阈值默认值 | `config/tuning.h` 的 `TUNE_IR_*_DEFAULT` |
| 红外 6 点表 | `control/motion_control.c` 的 `irMv*/irMm*` |
| 8 邻域规划 | `app/path_8dir.c` |
| 4 向动作/PID/传感器 | `control/motion_control.c` |
| 8 向转向/单格 | `control/motion_8dir.c` |
| 状态机 | `app/main.c` |
| 地图/起点/红外阈值存档 | `app/maze_storage.c` |
| 调试串口/地图发送 | `app/serial_debug.c` |
| 负压风扇 | `bsp/fan.c` |
| 状态灯 | `bsp/status_led.c` |
| 等高图 / 洪水 / 岔路 / 搜索 | `app/maze_search.c` |
| 4 向等高图执行 | `app/path_4dir.c` |
| 引脚与外设初始化 | `bsp/board_init.c` |
| ADC/红外/TIM6 初始化 | `bsp/sensor_init.c` |
| 数码管/键盘 | `bsp/display.c` |
| Flash 分区/读写 | `bsp/flash.c` |

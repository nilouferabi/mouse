# mouse —— 启诚智能鼠核心代码（重构 + 八向冲刺）

---

## 一、这次重构做了什么

| 项目 | 重构前 | 重构后 |
|---|---:|---:|
| `Scr/` 应用代码总行数 | 10 915 | 6 156 |
| `control/motion_control.c` | 6 444 | 1 791 |
| `app/main.c` | 2 148 | 204 |
| `app/maze_search.c`（新增） | — | 968 |
| `app/path_4dir.c`（新增） | — | 188 |
| `app/path_8dir.c`（新增） | — | 295 |
| `control/motion_8dir.c`（新增） | — | 138 |
| `bsp/board_init.c` | — | 233 |
| `bsp/sensor_init.c`（新增） | — | 158 |
| `app/maze_storage.c`（新增） | — | 85 |
| `app/serial_debug.c`（新增） | — | 137 |
| `-Os` 固件 Flash | 24 420 B，动作函数被折叠成死循环 | **28 260 B（21.56%），可运行** |
| `-O0` 固件 Flash | 46 688 B | 45 960 B（35.06%） |

核心变化：

1. **重点参数集中到 `tuning.h`**
   按机械/编码器、PID、贴墙、90° 转向、掉头、搜索衔接、红外 ADC、陀螺、**八向冲刺**、流程外设、时间基准分组。

2. **删除不可达代码**
   30 多个历史动作变体、`__irCheck` 旧副本、`mouseGoahead` 旧系列、重复的 `mouseSpurt`、死数组/死变量等。
   `control/motion_control.c` 从 6 444 行降到 1 791 行。

3. **修复确定性缺陷**
   - 跨中断变量加 `volatile`，`-Os` 下忙等不再被折叠成 `b .`；
   - `ADCmv()` 补 `return`；
   - 左距离插值由 `decimal` 改为 `decimalr`；
   - `delay()`/`__delay()` 加 `volatile`；
   - `crosswayChoice()` 改用 `SEARCHMETHOD`；
   - PID 积分加防溢出限幅；
   - 修正 `main()` 返回类型、`if(ss=0)`、重复定义、空 `else` 等。

4. **新增八向冲刺（本次重点）**
   - 搜索阶段仍然只走 4 向，保证识墙可靠；
   - 地图完整后的冲刺阶段，用 8 邻域 Dijkstra 规划，允许斜向；
   - 斜向必须满足 **2×2 方块内部无墙**，不切角；
   - 动作层新增 `mouseTurnTo()`（原地 45°/90°/135°/180°）和 `mouseGoOneBlock()`（正交/斜向一格）；
   - 规划失败自动回退原来的 4 向 `mouseSpurt_CC()`；
   - 运动超时/斜向前方近距双墙时立即停车，不再无限忙等。

5. **45° 旧代码没有直接搬回**
   旧 `Mouse_Srv.c` 的 `Diagonalway/actiongenerate/actionsequence` 是一套只服务单次冲刺的特殊动作序列，且原工程从未编译过。
   这次用 `path_8dir.c` 重写成通用 8 邻域规划，避免把旧思路的隐式约束带回来。

6. **寻路算法从主文件拆出**
   等高图、洪水填充、岔路决策、搜索移到 `app/maze_search.c/h`，4 向路径执行移到 `app/path_4dir.c/h`。
   `app/main.c` 从 1 502 行降到 204 行，只保留状态机。

7. **按职责分类**
   - `app/`：状态机、迷宫搜索、4/8 向路径、地图存档、调试串口；
   - `control/`：电机/传感器/4 向动作、8 向运动原语；
   - `bsp/`：板级初始化、ADC/红外/TIM6、风扇、状态灯、数码管、Flash、中断向量；
   - `config/`：工程配置、调参、类型、迷宫结构；
   - `util/`：延时、位带。

8. **继续拆分主文件**
   `app/main.c` 再拆出 `maze_storage`、`serial_debug`；
   `bsp/board_init.c` 再拆出 `sensor_init`；
   `control/motion_control.c` 再拆出 `motion_8dir`。

---

## 二、文件结构

```
Scr/
├── app/                        应用层
│   ├── main.c                  主状态机：WAIT/START/MAZESEARCH/SPURTL
│   ├── maze.h                  共享迷宫数据、状态常量、外部变量
│   ├── maze_search.h / .c      等高图、洪水填充、岔路决策、搜索
│   ├── path_4dir.h / .c        4 向等高图路径执行
│   ├── maze_storage.h / .c     地图/起点/红外阈值 Flash 存档
│   ├── path_8dir.h / .c        8 邻域规划 + 冲刺执行
│   └── serial_debug.h / .c     USART3 调试串口、地图发送
├── control/                    控制层
│   ├── motion_control.h / .c   PID、编码器、红外/陀螺采样、1 ms 控制环、4 向动作
│   ├── motion_8dir.h / .c      8 向转向、正交/斜向走一格
│   └── motion_internal.h       motion_control 与 motion_8dir 的内部共享声明
├── bsp/                        板级支持
│   ├── board_init.h / .c       时钟、TIM1 PWM、TIM2/3 编码器、按键、MouseInit
│   ├── sensor_init.h / .c      ADC+DMA、红外发射 GPIO、TIM6 初始化
│   ├── fan.h / .c              负压风扇（TIM4_CH3/PB8，仅冲刺）
│   ├── status_led.h / .c       状态灯（PB12：搜索慢闪/近终点快闪/冲刺常亮）
│   ├── display.h / .c          ZLG7289 数码管 + 键盘
│   ├── flash.h / .c            内部 Flash 读写
│   └── interrupts.h / .c       异常向量/中断入口
├── config/                     配置
│   ├── mouse_config.h          迷宫尺寸、终点、搜索法则、ONEBLOCK/ONEBLOCK_45
│   ├── tuning.h                ★ 重点参数集中区（调车先看这里）
│   ├── types.h                 uint8/int8/fp32 等类型
│   └── micromouse.h            迷宫坐标/宏定义
├── util/
│   ├── delay.h / .c            简单忙等延时
│   └── bitband.h               位带操作
├── test/test_path_8dir.c       主机侧 8 邻域规划器测试（不参与固件）
└── README.md
```

参与构建的 16 个 `.c`：

- `app/`：`main.c`、`maze_search.c`、`path_4dir.c`、`maze_storage.c`、`path_8dir.c`、`serial_debug.c`
- `control/`：`motion_control.c`、`motion_8dir.c`
- `bsp/`：`board_init.c`、`sensor_init.c`、`fan.c`、`status_led.c`、`display.c`、`flash.c`、`interrupts.c`
- `util/`：`delay.c`

GCC 的包含路径是 `Scr/` 加上各子目录：

```make
-I$(ROOT)/Scr
-I$(ROOT)/Scr/app
-I$(ROOT)/Scr/control
-I$(ROOT)/Scr/bsp
-I$(ROOT)/Scr/config
-I$(ROOT)/Scr/util
```

> IAR 工程（`Trash/Maze.ewp` 是旧工程）需要手动把上述 16 个 `.c` 加入源文件列表，
> 并把 `app/control/bsp/config/util` 加入 include path。旧的单目录工程路径已不再适用。

### 编码约定

- `.c` / `.h`：**UTF-8 with BOM**，CRLF 换行；
- `.md`：UTF-8（无 BOM）；
- GCC 默认按 UTF-8 读，BOM 会被忽略；
- IAR 读到 BOM 会自动按 UTF-8 解码，不需要额外编译选项；
  如果以后去掉 BOM，则要在
  `Project > Options > C/C++ Compiler > Encodings > Default source file encoding`
  选 UTF-8（等价于 `--source_encoding utf8`）。
- 如果 IAR 编辑器仍显示中文乱码，把编辑器的默认编码也设为 UTF-8；
  这只是显示问题，不影响带 BOM 文件的编译结果。

---

## 三、4 向搜索 + 8 向冲刺怎么配合

```
WAIT → START → MAZESEARCH（4 向探索，建墙图）
                    ↓
                SPURTL
                    ↓
        ┌───────────┴───────────┐
        │ TUNE_8DIR_ENABLE = 1  │
        │ 8 邻域规划 + 斜向冲刺 │
        └───────────┬───────────┘
                    │ 无斜向通路
                    ↓
              mouseSpurt_CC()（4 向回退）
```

- 搜索阶段 `mazeSearch()` 每次只走一格正交方向，靠左右/前左/前右红外更新墙图。
- 冲刺阶段 `mouseSpurt_8_to_goal()`：
  1. 分别以四个终点格为目标建 8 邻域距离图，选离起点最近的；
  2. 每步按“最短路 + 最少转向”选方向；
  3. `mouseTurnTo()` 原地转到目标方向；
  4. `mouseGoOneBlock()` 走一格正交或斜向；
  5. 更新 `GmcMouse.cX/cY`；
  6. 到终点后把朝向对齐到最近的正交方向，交回原 4 向返回流程。

### 斜向通行规则

```
允许 (x,y) → (x+dx, y+dy) 当且仅当：
  (x,y) → (x+dx,y)   通
  (x,y) → (x,y+dy)   通
  (x+dx,y) → (x+dx,y+dy) 通
  (x,y+dy) → (x+dx,y+dy) 通
```

也就是四个格子组成的 2×2 方块内部没有墙。这样可以避免“斜着切墙角”撞墙。

---

## 四、转弯和识墙是怎么实现的

### 4.1 90° / 180° 转向（原有）

以 `mouseTurnright_C()` 为例：

```c
__GucMouseState = __TURNRIGHT;
__GmRight.cState = __MOTORRUN;
__GmLeft.cState  = __MOTORRUN;
GucMouseDir = (GucMouseDir + 2) % 8;      // 先更新方向标记
__GmSPID.sRef = 145;                      // 两个轮子的平均前进速度
__GmWPID.sRef = -95;                      // 差速，使车原地/弧线右转
while (1) {
    W = (float)GW * 5121 / 4096 / 2;
    if (W > 93000) break;                 // 陀螺积分到阈值
}
__GmWPID.sRef = 0;
__GmSPID.sRef = 150;
__GmLeft.uiPulse = 17000 - piancha_r * 2886.67;  // 转完后补一段直行
...
```

- `__SPID` 速度环保证两轮平均速度；
- `__WPID` 方向环给左右轮差速；
- `GW` 是 `voltageDetect1()` 对陀螺偏离零位的**绝对值累加**，所以不论左转右转都单调增大；
- 到阈值后停方向环，再走固定脉冲把车带到下一格中心；
- 180° 用 `mouseTurnback()`，判据是 `GW > 290000`，回程 23000 脉冲。

### 4.2 八向原地转向（新增）

`__turnInPlace(steps45)`：

```c
target = steps45 * 45.0f * trim;   // 右转为正，左转为负
__GmSPID.sRef = 0;                 // 平均速度 0
__GmWPID.sRef = right ? -120 : +120;  // 纯差速，原地旋转
jiao = 0;                          // 有符号陀螺积分角
while (1) {
    if (right ? (jiao >= target) : (jiao <= target)) break;
    if (超时) { 停车; return 0; }
}
```

- `jiao` 是 `voltageDetect()` 对陀螺角速度的**有符号积分**，单位近似为度；
- 90° 用 `steps45 = ±2`，45° 用 ±1，135° 用 ±3，180° 用 ±4；
- 纯旋转不前进，因此不会把车带离当前格中心；
- 有 2 s 超时，陀螺坏掉/卡住不会无限等；
- 45°/135° 的角度系数需要实车标定：`TUNE_8DIR_TURN_TRIM_R/L`。

### 4.3 识墙（原有）

时间域：

- TIM6 5 kHz 中断推进 8 状态机；
- `SysTick_Handler()` 里又手工调用一次 `TIM6_IRQHandler()`，实际约 6 kHz 推进（原行为保留）。

每个状态做一件事：

```
关发射 → 采环境光 → 开发射 → 采反射光
ucIRCheck[i] = ADC - Sunlight
```

四路模拟红外：

| 槽位 | 位置 | 用途 |
|---|---|---|
| `__LEFT` | 左 | 左墙距离、左岔路 |
| `__FRONTL` | 前左 | 前墙、前近距 |
| `__FRONTR` | 前右 | 前墙、前近距 |
| `__RIGHT` | 右 | 右墙距离、右岔路 |

- `__GucDistance[]` 用位标志：`0x01` 远、`0x02` 近；
- `leftdis/rightdis` 通过 `disCheck2/3` 查表把 mV 换成 mm；
- `__wallCheck()` 根据阈值生成当前格的 4 位墙图：
  - `MOUSEWAY_L/R/F/B` 把“相对方向”转成绝对方向；
  - 同时更新相邻格的对应墙位；
  - 写入 `GucMapBlock`（路径）、`GucMapBlock0`（探索记录）、`GucMapBlock1`（洪水）。
- `mazeSearch()` 每走完一格调用 `__mouseCoorUpdate()` → `__wallCheck()`；
- 搜索决策用 `crosswayCheck()` 数未走过支路，`crosswayChoice()` 按 `SEARCHMETHOD` 选法则。

### 4.4 地图保存

保存的是**墙/连通性地图**，不是红外电压或距离：

- 每格 4 bit：bit0 北、bit1 东、bit2 南、bit3 西；`1`=有路，`0`=有墙；
- `GucMapBlock` 是物理墙图，`GucMapBlock0` 是探索记录，`GucMapBlock1` 是洪水副本；
- 只有 `GucMapBlock` 会被 `wallsave()` 保存：
  ```c
  STMFLASH_Write(MAPE_SAVE_ADDRESS, (u16 *)GucMapBlock4, 256);
  ```
  `0x08018800` 起写 256 个半字 = 512 B（实际数据 256 B，高字节为 0）。
- `wallget()` 读回 256 个半字，取低字节还原 `GucMapBlock`；
- 起点由 `StartSave()` 写到 `0x08019000`，`StartGet()` 读回；`start == 15` 表示车在另一侧起点；
- 红外阈值地址是 `0x08010000`，`__ir_Get()` 目前仍被注释，实际用 `TUNE_IR_*_DEFAULT`；
- 没有 magic/CRC/版本/地图完整度；空片读成 `0xFF` 会变成“所有方向都通”的假图。

### 4.5 状态灯与负压风扇

**负压风扇**（`fan.c/h`）：

- 硬件：TIM4_CH3 → PB8，AF 推挽；
- `Fan_Init()` 在 `MouseInit()` 里初始化，默认 `CCR3 = 0`（关闭）；
- 只在 `SPURTL` 冲刺阶段调用 `Fan_On()`，结束/中止时 `Fan_Off()`；
- 搜索、校准、WAIT 都不打开；
- 占空比：`Fan_SetDuty(0..3600)`，`TUNE_FAN_DUTY = 600` 约 16.7%；
- 如果泵需要全速，先试 `Fan_SetDuty(TUNE_FAN_PWM_PERIOD)`，但注意电池和驱动电流。

**状态灯**（`status_led.c/h`）：

- 硬件：PB12，默认高电平点亮（`TUNE_LED_ACTIVE_HIGH = 1`）；
- `Led_Tick()` 在 `SysTick_Handler()` 里每 1 ms 调用一次；
- 模式：
  - `LED_MODE_SEARCH`：慢闪（默认 100 ms 亮 / 1 s 周期）；
  - 搜索中离任一终点格曼哈顿距离 ≤ `TUNE_LED_NEAR_GOAL_CELLS`（默认 2）时自动快闪（80 ms 亮 / 160 ms 周期）；
  - `LED_MODE_SPRINT`：常亮；
  - `LED_MODE_OFF`：灭。
- 原来 SysTick 里 1 s 闪烁和左右贴墙时直接写 PB12 的逻辑已移除，统一由 `Led_Tick()` 管理。

---

## 五、代码需要改进的地方

### 5.1 当前主要问题（按优先级）

| 优先级 | 问题 | 现状 | 影响 |
|---|---|---|---|
| P0 | 终点坐标不一致 | X 运行期 8/7，Y 仍是 6/6，`goalwall()` 按 7/8 | 三个模式可能都跑错终点 |
| P0 | 没有物理终点识别 | 只有 `GmcMouse` 坐标匹配 | 终点换位置必须改配置 |
| P0 | 搜索首次到终点就停 | `centralMethodnew()` 到点直接 `goto End2` | 地图可能不完整，模式 2 受限 |
| P0 | 没有模式抽象 | `WAIT` 里按 `start` 计数直接跳状态 | 加模式 2 会继续堆 `if` |
| P0 | 规划与执行耦合 | `mouseSpurt_8_to_goal()` 算完立刻跑 | 无法只算路线、保存路线 |
| P1 | 8 向未计转向代价 | Dijkstra 距离 + 平局少转向 | 不是严格最优路线 |
| P1 | 没有路径缓存 | 模式 3 每次重新规划 | 冲刺前多花时间，路线不可复用 |
| P1 | 4 向 `objectGoTo1()` 是贪心 | 只看等高值下降 | 路径可能多绕 |
| P1 | 冲刺逐格停车 | `mouseGoOneBlock()` 每格 `Motion_Stop()` | 不是真正高速冲刺 |
| P1 | 45°/135° 未实车标定 | 参数按几何推导 | 可能转不准、斜向跑偏 |
| P2 | 旧动作没有统一超时 | `motion_control.c` 里 10+ 个 `while(1)` | 堵转/断线会卡死 |
| P2 | `DenggaoCheck()` 恒为假 | `__GucDistance[__FRONT]` 从未写入 | 搜索中“遇墙停车”失效 |
| P2 | 地图边界检查不足 | `crosswayCheck()`、`objectGoTo1()` 直接访问 `cX+1/cY+1` | 边缘可能越界 |
| P2 | 三份墙图分散更新 | `GucMapBlock`、`GucMapBlock0`、`GucMapBlock1` | 可能不一致 |
| P2 | Flash 存档无校验 | 没有 magic/CRC/版本 | 空片可能读成 0xFF |
| P3 | 可观测性弱 | 没有模式号、路径长度、单圈时间 | 调试靠猜 |

### 5.2 识墙

1. **连续距离代替三档标志**
   现在 `__GucDistance` 只有远/近/更近三档。可直接用 `leftdis/rightdis/fldis/frdis`（mm）做比例控制，墙判定用双阈值 + 回差。
2. **每路单独标定**
   四个红外一致性差，建议每个传感器单独存 offset/scale，而不是共用 300/700 mV 起点。
3. **时间一致性**
   加 3~5 次中值/投票，避免单次 ADC 噪声导致墙图翻转；墙图从“有/无”升级为“置信度”。
4. **补前向中心传感器**
   当前“前方有墙”= 前左 **且** 前右都超阈值，斜向时容易误判；有中心前 IR 会更稳。
5. **斜向安全包络**
   现在斜向只检查双前近距；可再加左右近距、电流/堵转检测，作为最后保险。
6. **地图一致性**
   `GucMapBlock`、`GucMapBlock0`、`GucMapBlock1` 三份图建议统一由一个 `SetWall()` 维护，避免只更新一份。

### 5.3 转向

1. **陀螺 + 编码器融合**
   现在 `jiao` 只靠陀螺积分，长时间会漂；可用 `互补滤波`：陀螺积分 + 左右轮差推算角度。
2. **角度闭环改为速度规划**
   现在到角度阈值就停，惯性会过冲；可做梯形/S 曲线角速度规划，或到阈值前减速。
3. **绝对航向**
   每次转向后把 `GucMouseDir` 和陀螺绝对角对齐，减少累计误差；现在只更新离散方向编号。
4. **所有忙等加超时**
   旧动作函数仍是 `while (...)`；新八向函数已加 ms 超时，建议逐步推广到 `mouseGoahead_L`、`mazeSearch`。
5. **堵转检测**
   编码器脉冲长时间不增、电流过大时停车，而不是等超时。

### 5.4 八向规划

1. **转向代价进 A\***
   现在是“最短距离 + 平局时少转向”；可把状态扩成 `(x,y,dir)`，把 45°/90° 转向代价直接放进 A*。
2. **连续运动**
   现在每格停车再转，速度优势被抵消；可把“转向 + 下一格”合成连续动作，像原 `_C` 函数那样边走边转。
3. **路径平滑**
   对连续同向斜线合并成多格斜向直行，减少停车次数。
4. **斜向物理包络**
   2×2 无墙只是拓扑条件；还要按车体宽度检查实际通过间隙，必要时禁用某些斜向。

---

## 六、重点参数说明

> 单位约定：速度 = 每 1 ms 编码器脉冲数；PWM = TIM1 CCR 计数；距离 = mm；
> 红外阈值 = ADC 原始计数；陀螺角度 = 度。

| 分组 | 宏 / 变量 | 当前值 | 作用与影响 |
|---|---|---:|---|
| 迷宫 | `MAZETYPE` | 16 | 迷宫边长；改 8 要同时改终点和数组 |
| 终点 | `XDST0/1`、`YDST0/1` | 6 / 6 / 6 / 6 | 编译期终点；当前与运行期 X=8/7 不一致 |
| 终点 | `TUNE_GOAL_X0/X1` | 8 / 7 | START 分支覆盖的 X 目标 |
| 一格脉冲 | `ONEBLOCK` | 51600 | 正交走一格的编码器脉冲 |
| 斜向脉冲 | `ONEBLOCK_45` | 36410 | 斜向走一格；≈ `ONEBLOCK/√2` |
| 搜索速度 | `SEARCHSPEED` | 140 | 搜索阶段最高速度 |
| 8 向速度 | `TUNE_8DIR_SPEED_ORTHO/DIAG` | 180 / 150 | 冲刺正交/斜向最高速 |
| 转向速度 | `TUNE_TURN_SPEED` | 145 | 90° 转向时前进速度设定 |
| 掉头回程 | `TUNE_TURNBACK_RETURN_SPEED` | 150 | 180° 后回程速度 |
| PID | `TUNE_PID_KP/KI/KD` | 20 / 0.1 / 0 | 全工程唯一一组 PID 增益 |
| PWM 限幅 | `TUNE_PID_U_MAX` | 1400 | TIM1 ARR=2880，最大占空比约 48.6% |
| PID 死区 | `TUNE_PID_DEADBAND` | 3 | 速度环死区 |
| 积分分离 | `TUNE_PID_ERROR_IMAX` | 10 | 误差大于 10 时关积分 |
| 贴墙目标 | `TUNE_WALL_L/R_TARGET_DIST` | 75 / 100 | 左右墙目标距离 |
| 贴墙有效距离 | `TUNE_WALL_L/R_ACTIVE_DIST` | 120 / 170 | 超过此距离不参与校正 |
| 90° 判据 | `TUNE_TURN90_W_SEARCH_R/L` | 95000 / 86500 | 搜索段 `GW` 阈值 |
| 90° 判据 | `TUNE_TURN90_W_C_R/L` | 93000 / 84000 | 冲刺段 `GW` 阈值 |
| 转向后续直行 | `TUNE_TURN90_PULSE_*` | 17000 / 21000 / 9600 | 右转 / 搜索左转 / 冲刺左转 |
| 8 向转向 | `TUNE_8DIR_TURN_SPIN` | 120 | 原地旋转的差速 PWM 设定 |
| 8 向角度修正 | `TUNE_8DIR_TURN_TRIM_R/L` | 0.95 | 实际转不够就调大，转过头就调小 |
| 8 向陀螺保持 | `TUNE_8DIR_GYRO_FF` | 3 | 斜向直行的航向保持增益 |
| 8 向超时 | `TUNE_8DIR_TURN/MOVE_TIMEOUT_MS` | 2000 / 6000 | 转向/单格直行超时，防卡死 |
| 红外默认阈值 | `TUNE_IR_*_DEFAULT` | 见文件 | 远/中/近判定，单位 ADC 计数 |
| 红外查表 | `TUNE_IR_R_OFFSET_MV/STEP_MV` | 300 / 15 | 左右距离表起点/步长 |
| 红外饱和 | `TUNE_IR_R_SAT/FRONT_SAT` | 35000 / 30000 | 超过就丢弃本次读数 |
| 陀螺零位 | `TUNE_GYRO_ZERO_DEFAULT` | 2041 | 上电零漂校准起点 |
| 陀螺换算 | `TUNE_GYRO_GW_SCALE` | 5121 | `GW → W` 比例 |
| 陀螺灵敏度 | `TUNE_GYRO_SENSITIVITY_MV` | 1055 | mV/(°/s) |
| 陀螺校准 | `TUNE_GYRO_CAL_SAMPLES` | 3000 | 1 kHz 下约 3 s |
| 按键 | `TUNE_START_PRESS_*` | 1 / 3 / 8 | 搜索 / 直接冲刺 / 校准 |
| 风扇 | `TUNE_FAN_DUTY` / `TUNE_FAN_PWM_PERIOD` | 600 / 3600 | 仅冲刺打开；占空比 = 600/3600 ≈ 16.7% |
| 状态灯 | `TUNE_LED_ACTIVE_HIGH` | 1 | PB12 高电平点亮 |
| 搜索慢闪 | `TUNE_LED_SEARCH_ON_MS` / `PERIOD_MS` | 100 / 1000 | 亮 100 ms，周期 1 s |
| 近终点快闪 | `TUNE_LED_NEAR_ON_MS` / `PERIOD_MS` | 80 / 160 | 亮 80 ms，周期 160 ms |
| 近终点判定 | `TUNE_LED_NEAR_GOAL_CELLS` | 2 | 到任一终点格曼哈顿距离 ≤ 2 |
| 地图存档 | `TUNE_FLASH_MAP_WORDS` | 256 | 256 个半字 = 512 B |

**推荐调参顺序：**

1. `ONEBLOCK` / `ONEBLOCK_45`：先保证走一格、斜一格的实际距离；
2. 红外阈值/查表：保证墙判定和 mm 读数可信；
3. `TUNE_WALL_*`：调贴墙目标与有效距离；
4. PID：先 `KP`，再少量 `KI`，最后考虑 `KD`；
5. 转向阈值/脉冲：90° 先准，再 45°/135°/180°；
6. 8 向速度/陀螺增益：最后提速。

## 七、八向冲刺怎么标定

建议顺序（先在空地上，不要直接上迷宫）：

1. **降速**
   `TUNE_8DIR_SPEED_ORTHO` / `TUNE_8DIR_SPEED_DIAG` 先设 80~100；
   `TUNE_8DIR_TURN_SPIN` 先设 60~80。
2. **原地 45°**
   用 `mouseTurnTo(1)` 从方向 0 转到 1，量实际角度；
   转不够→调大 `TUNE_8DIR_TURN_TRIM_R`；转过头→调小。
   左转同理调 `TUNE_8DIR_TURN_TRIM_L`。
3. **原地 90°/135°/180°**
   确认 `jiao` 判据在四个角度都线性；否则先查陀螺零位/比例。
4. **斜向一格**
   在 2×2 空方块里走 `mouseGoOneBlock(1)`，量实际位移；
   调 `ONEBLOCK_45`（当前 36410）。
5. **斜向直行**
   连续走 2~3 格斜向，看是否跑偏；调 `TUNE_8DIR_GYRO_FF`（当前 3）。
6. **整路径**
   用已知地图跑 `mouseSpurt_8_to_goal()`；失败会停车并回 WAIT。
7. **确认回退**
   把 `TUNE_8DIR_ENABLE` 设 0，确认原 4 向 `mouseSpurt_CC()` 仍正常。

> 注意：45°/135° 的角度和斜向脉冲没有实车验证过，默认值只是从 90° 参数和几何关系推出来的起点，必须按上面步骤标定。

---

## 八、构建与验证

```bash
make                 # 默认 -Os
make OPT="-O0 -g3"   # 调试版
make test-nav        # 主机侧 8 邻域规划器测试（gcc）
make size
make flash
```

红外测压（不发射/发射时 ADC 电压）：

```bash
./debug.sh --ir
# 停在 Sensor_IrqInit 前：IR 发射管关闭、ADC/DMA 已启动
# 进入 gdb 后：irvol / irsave / iron 1..4 / iroff / irtest / c
```

本次验证结果：

| 构建 | Flash | RAM | 折叠成死循环的函数 |
|---|---:|---:|---|
| `-Os` | 28 260 B（21.56%） | 9 224 B（45.04%） | 仅 `HardFault` 等 5 个异常处理 |
| `-O0` | 45 960 B（35.06%） | 9 240 B（45.12%） | 仅 `LoopForever` |

主机侧 8 邻域规划器测试（`make test-nav`，不接硬件）：

- 空地 16×16：15 步斜向到 (15,15)，距离 210；
- 缺北墙时 (0,0)→(1,1) 斜向被禁止，改走正交；
- 2×2 内部有墙时禁止切角；
- 全未知地图返回无路径；
- 执行器能到终点并对齐正交方向。

---

## 九、断点位置与调试

| 想停在哪 | GDB 命令 | 用途 |
|---|---|---|
| 状态机入口 | `break main` | 看启动流程；此时外设还没初始化 |
| 红外测压 | `./debug.sh --ir` | 停在 `Sensor_IrqInit` 前：发射管关闭、ADC 已启动 |
| 搜索到终点 | `break centralMethodnew` | 模式 1 的探索循环 |
| 洪水/返回 | `break floodMethodnew` | 搜索结束后的洪水与回程 |
| 4 向寻路执行 | `break objectGoTo1` | 按等高图逐格走 |
| 等高图生成 | `break mapStepEdit` | 看 `GucMapStep` 怎么铺 |
| 4 向冲刺 | `break mouseSpurt_CC` | 选最近终点格并执行 |
| 单格搜索 | `break mazeSearch` | 一格一格的墙图更新 |
| 8 向转向 | `break mouseTurnTo` | 45°/90°/135°/180° 原地转 |
| 8 向单格 | `break mouseGoOneBlock` | 正交/斜向走一格 |
| 纯旋转细节 | `break __turnInPlace` | `jiao` 角度闭环（静态函数） |
| 1 ms 控制环 | `break SysTick_Handler` | PID、编码器、墙跟随；1 kHz，建议加条件 |
| 墙图更新 | `break __wallCheck` | 看当前格四方向判定（静态函数） |
| 风扇开关 | `break Fan_On` / `break Fan_Off` | 确认只在 SPURTL 打开 |
| 灯光模式 | `break Led_SetMode` | 看当前进入 SEARCH/SPRINT/OFF |
| 墙图变化 | `watch GucMapBlock[7][7]` | 只盯中心格，别 watch 整个数组 |

调试命令：

```text
st        打印状态机 / 红外标志 / PID / 电机
params    打印 PID、sRef、红外阈值、陀螺零位
ready     运行到 app/main.c:68（初始化完成，可发运动原语）
fwd       沿当前朝向直行一格（mouseGoOneBlock(0)）
irvol     红外测压：ADC 原始值 + mV + 环境光差值
iron 1..4 打开 FL/FR/L/R 发射管并测电压
irtest    自动扫四路发射管
c         继续
```

直行调试流程：

```text
./debug.sh
(gdb) ready      # 或 c：从 main 跑到 app/main.c:68，初始化完成
(gdb) fwd        # 沿当前朝向直行一格
(gdb) st         # 看电机/坐标/状态
```

注意：`mouseGoOneBlock(0)` 只是“向前走一格”的原语，**不会自动转向，也不会更新
`GmcMouse` 坐标、不会查正交墙**。要换方向先：

```text
(gdb) call mouseTurnTo(2)   # 0=北 2=东 4=南 6=西
(gdb) fwd
```

直行测试请架空轮子或在空地/走廊进行，避免直接撞墙。

注意：

- `-Os` 下静态函数可能被内联，断点不一定能停；要稳定打断点先用
  `make OPT="-O0 -g3"` 再 `./debug.sh`。
- `SysTick_Handler` 每 1 ms 进一次，直接断会像“死机”；用
  `ignore <断点号> 100` 或条件断点。
- 同名静态函数建议用文件限定：
  `break 'maze_search.c'::floodMethodnew`、`break 'motion_control.c'::__wallCheck`。
- 正常调试用 `./debug.sh`，红外测压用 `./debug.sh --ir`。

---

## 十、仍然保留、需要实车确认的点

1. **终点 Y 坐标三处不一致**
   `mouse_config.h` 的 `YDST0/YDST1 = 6,6`；`main()` 只把 X 改成 8/7；`goalwall()` 内部按 7/8 处理。本次没有改行为。
2. **TIM6 与 SysTick 双触发**
   `SysTick_Handler()` 仍每 1 ms 手工调用一次 `TIM6_IRQHandler()`，实际约 6 kHz 推进 8 状态机。为保持原时序保留。
3. **`__GucDistance[__FRONT]` 恒为 0**
   `DenggaoCheck()` 目前恒为假，搜索中“遇墙停车”不会触发。
4. **`MouseTurnleft_C()` 的 286.67**
   其余位置是 2886.67，疑似漏写一位；原值保留。
5. **忙等没有统一超时**
   旧 4 向动作仍是 `while (...)`；新八向动作已加 ms 超时。
6. **`GucMapBlock1[0][ucTemp + 1]` 越界**
   `main()` 起点镜像时 `ucTemp == MAZETYPE-1` 会写到下一行首元素，原逻辑保留。
7. **`mouseTurnbackqidian()` 的 `(GucMouseDir + 2) % 4`**
   与其他掉头函数的 `+4 % 8` 不同，疑似历史遗留。
8. **斜向机械间隙**
   2×2 无墙只是拓扑条件，车体宽度/轮距是否够过，必须实车确认。

---

## 十一、最常改的地方

| 想改什么 | 去哪里 |
|---|---|
| 迷宫大小 / 终点 / 搜索法则 | `mouse_config.h` |
| 速度、PID、贴墙、转向、掉头 | `tuning.h` |
| 八向速度 / 45° 修正 / 超时 | `tuning.h` 的 `TUNE_8DIR_*` |
| 斜向一格脉冲 | `mouse_config.h` 的 `ONEBLOCK_45` |
| 红外阈值默认值 | `tuning.h` 的 `TUNE_IR_*_DEFAULT` |
| 标定查表（mV→mm） | `control/motion_control.c` 的 `disCheck0..3` |
| 8 邻域规划 / 转向代价 | `app/path_8dir.c` |
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

---

*重构基线：git tag `baseline-before-refactor`（提交 `8ad8dd0`）。*

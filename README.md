# mouse —— 启诚智能鼠核心代码（重构 + 八向冲刺）

---

## 一、这次重构做了什么

| 项目 | 重构前 | 重构后 |
|---|---:|---:|
| `Scr/` 应用代码总行数 | 10 915 | 5 660 |
| `Mouse_Drive.c` | 6 444 | 1 928 |
| `main.c` | 2 148 | 1 502 |
| `Mouse_Nav.c`（新增） | — | 295 |
| `-Os` 固件 Flash | 24 420 B，动作函数被折叠成死循环 | **28 040 B（21.39%），可运行** |
| `-O0` 固件 Flash | 46 688 B | 45 444 B（34.67%） |

核心变化：

1. **重点参数集中到 `Mouse_Tuning.h`**
   按机械/编码器、PID、贴墙、90° 转向、掉头、搜索衔接、红外 ADC、陀螺、**八向冲刺**、流程外设、时间基准分组。

2. **删除不可达代码**
   30 多个历史动作变体、`__irCheck` 旧副本、`mouseGoahead` 旧系列、重复的 `mouseSpurt`、死数组/死变量等。
   `Mouse_Drive.c` 从 6 444 行降到 1 928 行。

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
   这次用 `Mouse_Nav.c` 重写成通用 8 邻域规划，避免把旧思路的隐式约束带回来。

---

## 二、文件结构

```
Scr/
├── Mouse_Config.h          工程级配置：迷宫尺寸、终点、搜索法则、ONEBLOCK / ONEBLOCK_45
├── Mouse_Tuning.h          ★ 重点参数集中区（调车先看这里）
├── Maze.h                  应用层（迷宫算法 + 状态机）接口
├── main.c                  状态机、等高图/洪水、岔路记录、Flash 存取、串口
├── Mouse_Nav.h / .c        ★ 8 邻域路径规划 + 冲刺执行
├── Mouse_Drive.h / .c      PID、编码器、红外/陀螺采样、1 ms 控制环、动作函数
├── Mouse_Drive_Init.h/.c   时钟、ADC+DMA、TIM1 PWM、TIM2/3 编码器、TIM6、风扇
├── zlg7289.c / .h          数码管 + 键盘
├── stmflash.c / .h         内部 Flash（地图 / 起点 / 红外阈值）
├── stm32f10x_it.c / .h     异常向量
├── Type.h / BitBand.h / Micromouse.h
├── test/test_nav.c         主机侧 8 邻域规划器测试（不参与固件）
└── README.md
```

参与构建的 7 个 `.c`：
`main.c`、`Mouse_Drive.c`、`Mouse_Nav.c`、`Mouse_Drive_Init.c`、`stmflash.c`、`zlg7289.c`、`stm32f10x_it.c`。

> IAR 工程需要手动把 `Mouse_Nav.c` 加入源文件列表。

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

---

## 五、可以怎么改进

### 5.1 识墙

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

### 5.2 转向

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

### 5.3 八向规划

1. **转向代价进 A\***
   现在是“最短距离 + 平局时少转向”；可把状态扩成 `(x,y,dir)`，把 45°/90° 转向代价直接放进 A*。
2. **连续运动**
   现在每格停车再转，速度优势被抵消；可把“转向 + 下一格”合成连续动作，像原 `_C` 函数那样边走边转。
3. **路径平滑**
   对连续同向斜线合并成多格斜向直行，减少停车次数。
4. **斜向物理包络**
   2×2 无墙只是拓扑条件；还要按车体宽度检查实际通过间隙，必要时禁用某些斜向。

---

## 六、八向冲刺怎么标定

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

## 七、构建与验证

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
# 停在 TIM6_IRInit 前：IR 发射管关闭、ADC/DMA 已启动
# 进入 gdb 后：irvol / irsave / iron 1..4 / iroff / irtest / c
```

本次验证结果：

| 构建 | Flash | RAM | 折叠成死循环的函数 |
|---|---:|---:|---|
| `-Os` | 28 040 B（21.39%） | 9 208 B（44.96%） | 仅 `HardFault` 等 5 个异常处理 |
| `-O0` | 45 444 B（34.67%） | 9 232 B（45.08%） | 仅 `LoopForever` |

主机侧 8 邻域规划器测试（`make test-nav`，不接硬件）：

- 空地 16×16：15 步斜向到 (15,15)，距离 210；
- 缺北墙时 (0,0)→(1,1) 斜向被禁止，改走正交；
- 2×2 内部有墙时禁止切角；
- 全未知地图返回无路径；
- 执行器能到终点并对齐正交方向。

---

## 八、仍然保留、需要实车确认的点

1. **终点 Y 坐标三处不一致**
   `Mouse_Config.h` 的 `YDST0/YDST1 = 6,6`；`main()` 只把 X 改成 8/7；`goalwall()` 内部按 7/8 处理。本次没有改行为。
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

## 九、最常改的地方

| 想改什么 | 去哪里 |
|---|---|
| 迷宫大小 / 终点 / 搜索法则 | `Mouse_Config.h` |
| 速度、PID、贴墙、转向、掉头 | `Mouse_Tuning.h` |
| 八向速度 / 45° 修正 / 超时 | `Mouse_Tuning.h` 的 `TUNE_8DIR_*` |
| 斜向一格脉冲 | `Mouse_Config.h` 的 `ONEBLOCK_45` |
| 红外阈值默认值 | `Mouse_Tuning.h` 的 `TUNE_IR_*_DEFAULT` |
| 标定查表（mV→mm） | `Mouse_Drive.c` 的 `disCheck0..3` |
| 8 邻域规划 / 转向代价 | `Mouse_Nav.c` |
| 动作函数细节 | `Mouse_Drive.c` |
| 状态机 / 等高图 / 洪水 | `main.c` |
| 引脚与外设初始化 | `Mouse_Drive_Init.c` |
| Flash 分区 | `stmflash.h` |

---

*重构基线：git tag `baseline-before-refactor`（提交 `8ad8dd0`）。*

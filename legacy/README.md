# legacy —— 归档代码（不参与构建）

本目录存放重构时移出的、原工程就没有实际运行的代码。保留是为了以后做 45° 斜走时参考，**不要直接加进构建**。

## Mouse_Srv.c / Mouse_Srv.h

- 原工程（`baseline-before-refactor`）就没有把 `Mouse_Srv.c` 加入 IAR 工程或 `linux/Makefile`。
- 它实现 45° 斜线路径：`Diagonalway()`、`actiongenerate()`，并定义 `actionsequence[]`、`MapDiagonalway[]`、`DiagonalwayAction[]`。
- 唯一调用链是 `main.c` 的 `mouseSpurt_45() → objectGoTo_45() → mouseTurnleft/right_45_*`，而 `mouseSpurt_45()` 从未被调用。
- 重构后该调用链已删除，因此这里只剩“算法参考”，与当前 `main.c` / `Mouse_Drive.c` 的全局变量和动作函数不再兼容。

## 想恢复 45° 功能

1. 从 git 基线取回原文件：
   ```bash
   git -C Scr show baseline-before-refactor:Scr/main.c > /tmp/main_v1.c
   git -C Scr show baseline-before-refactor:Scr/Mouse_Drive.c > /tmp/Mouse_Drive_v1.c
   ```
2. 参考原 `mouseSpurt_45()`、`objectGoTo_45()` 和 `mouseTurnleft_45_t/_v`、`mouseTurnright_45_t/_v`、`_135_*`、`_180` 等函数，按新结构重写；
3. 把 `Mouse_Srv.c` 加入 `linux/Makefile` 的 `C_SOURCES`（IAR 工程同理）；
4. 补回 `actionsequence` 等共享变量，并重新接好状态机。

> 建议先在 `-O0` 下验证，再切回 `-Os`；45° 动作函数同样依赖 `volatile` 忙等，移植时不要丢掉 `volatile`。

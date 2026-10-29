/*********************************************************************************************************
** File Name:           Mouse_Nav.c
** Description:         8 邻域路径规划 + 冲刺执行
**
**   坐标/方向约定（与原工程一致）：
**     方向 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW
**     迷宫墙位：bit0=N, bit1=E, bit2=S, bit3=W，1 表示有路
**
**   斜向规则：只有 2x2 方块内部完全连通时才允许走对角线，避免切角撞墙。
*********************************************************************************************************/

#include "Mouse_Nav.h"
#include "Maze.h"
#include "Mouse_Drive.h"

#define NAV_INF 0xFFFF

static uint16 GucNavDist[MAZETYPE][MAZETYPE];
static uint8  GucNavVisited[MAZETYPE][MAZETYPE];

static const int8 NavDx[8] = { 0,  1,  1,  1,  0, -1, -1, -1 };
static const int8 NavDy[8] = { 1,  1,  0, -1, -1, -1,  0,  1 };

/* 两点之间的墙位步长：正交 10，对角 14（≈√2 倍） */
#define NAV_COST_ORTHO 10
#define NAV_COST_DIAG  14

static uint8 Nav_MoveCost(uint8 dir)
{
    return (dir & 0x01) ? NAV_COST_DIAG : NAV_COST_ORTHO;
}

static uint8 Nav_TurnDistance(uint8 from, uint8 to)
{
    uint8 d = (uint8)((to + 8 - from) & 0x07);
    if (d > 4)
        d = (uint8)(8 - d);
    return d;
}

uint16 Nav_Distance(uint8 x, uint8 y)
{
    if ((x >= MAZETYPE) || (y >= MAZETYPE))
        return NAV_INF;

    return GucNavDist[x][y];
}

uint8 Nav_CanMove(uint8 x, uint8 y, int8 dx, int8 dy)
{
    int16 nx, ny;

    if ((x >= MAZETYPE) || (y >= MAZETYPE))
        return 0;

    nx = (int16)x + dx;
    ny = (int16)y + dy;
    if ((nx < 0) || (nx >= MAZETYPE) || (ny < 0) || (ny >= MAZETYPE))
        return 0;

    /* 正交一步：两边都得认为有路（墙图一致时等价） */
    if ((dx == 0) && (dy == 1))
        return ((GucMapBlock[x][y] & 0x01) && (GucMapBlock[nx][ny] & 0x04)) ? 1 : 0;
    if ((dx == 1) && (dy == 0))
        return ((GucMapBlock[x][y] & 0x02) && (GucMapBlock[nx][ny] & 0x08)) ? 1 : 0;
    if ((dx == 0) && (dy == -1))
        return ((GucMapBlock[x][y] & 0x04) && (GucMapBlock[nx][ny] & 0x01)) ? 1 : 0;
    if ((dx == -1) && (dy == 0))
        return ((GucMapBlock[x][y] & 0x08) && (GucMapBlock[nx][ny] & 0x02)) ? 1 : 0;

    /* 对角一步：2x2 方块四条内边都必须通，绝不切角 */
    if ((dx != 0) && (dy != 0))
    {
        return (Nav_CanMove(x, y, dx, 0) &&
                Nav_CanMove(x, y, 0, dy) &&
                Nav_CanMove((uint8)(x + dx), y, 0, dy) &&
                Nav_CanMove(x, (uint8)(y + dy), dx, 0)) ? 1 : 0;
    }

    return 0;
}

uint8 Nav_BuildDistance(uint8 gx, uint8 gy)
{
    uint8 x, y, dir;
    uint16 best;
    uint8 bx = 0, by = 0;
    uint16 cand;

    if ((gx >= MAZETYPE) || (gy >= MAZETYPE))
        return 0;

    for (x = 0; x < MAZETYPE; x++)
    {
        for (y = 0; y < MAZETYPE; y++)
        {
            GucNavDist[x][y] = NAV_INF;
            GucNavVisited[x][y] = 0;
        }
    }

    GucNavDist[gx][gy] = 0;

    /* 节点很少（256），用 O(N^2) 的 Dijkstra 足够 */
    while (1)
    {
        best = NAV_INF;
        for (x = 0; x < MAZETYPE; x++)
        {
            for (y = 0; y < MAZETYPE; y++)
            {
                if ((!GucNavVisited[x][y]) && (GucNavDist[x][y] < best))
                {
                    best = GucNavDist[x][y];
                    bx = x;
                    by = y;
                }
            }
        }

        if (best == NAV_INF)
            break;

        GucNavVisited[bx][by] = 1;

        for (dir = 0; dir < 8; dir++)
        {
            uint8 nx, ny;

            if (!Nav_CanMove(bx, by, NavDx[dir], NavDy[dir]))
                continue;

            nx = (uint8)(bx + NavDx[dir]);
            ny = (uint8)(by + NavDy[dir]);
            cand = (uint16)(best + Nav_MoveCost(dir));
            if (cand < GucNavDist[nx][ny])
                GucNavDist[nx][ny] = cand;
        }
    }

    return 1;
}

uint8 Nav_NextStep(uint8 x, uint8 y, uint8 curDir,
                   int8 *dx, int8 *dy, uint8 *moveDir)
{
    uint8 dir;
    uint8 found = 0;
    uint8 bestTurn = 0xFF;
    uint16 cost;
    uint16 here;

    if ((x >= MAZETYPE) || (y >= MAZETYPE))
        return 0;

    here = GucNavDist[x][y];
    if (here == NAV_INF)
        return 0;

    for (dir = 0; dir < 8; dir++)
    {
        uint8 nx, ny;
        uint8 turn;

        if (!Nav_CanMove(x, y, NavDx[dir], NavDy[dir]))
            continue;

        nx = (uint8)(x + NavDx[dir]);
        ny = (uint8)(y + NavDy[dir]);
        if (GucNavDist[nx][ny] == NAV_INF)
            continue;

        cost = Nav_MoveCost(dir);
        /* 只走“最短路”上的边 */
        if ((uint16)(GucNavDist[nx][ny] + cost) != here)
            continue;

        turn = Nav_TurnDistance(curDir, dir);
        if (turn < bestTurn)
        {
            bestTurn = turn;
            *dx = NavDx[dir];
            *dy = NavDy[dir];
            *moveDir = dir;
            found = 1;
        }
    }

    return found;
}

/* 在当前已构建的距离图上，从 GmcMouse 走到 (gx,gy) */
static uint8 Nav_RunPath(uint8 gx, uint8 gy)
{
    int16 x, y;
    uint16 guard;
    int8 dx, dy;
    uint8 dir;

    x = GmcMouse.cX;
    y = GmcMouse.cY;

    for (guard = 0; guard < (MAZETYPE * MAZETYPE * 2); guard++)
    {
        if ((x == gx) && (y == gy))
            break;

        if (!Nav_NextStep((uint8)x, (uint8)y, GucMouseDir, &dx, &dy, &dir))
            return NAV_ABORT;

        if (!mouseTurnTo(dir))
            return NAV_ABORT;

        if (!mouseGoOneBlock(dir))
            return NAV_ABORT;

        x = (int16)(x + dx);
        y = (int16)(y + dy);
        if ((x < 0) || (x >= MAZETYPE) || (y < 0) || (y >= MAZETYPE))
            return NAV_ABORT;

        GmcMouse.cX = (int8)x;
        GmcMouse.cY = (int8)y;
    }

    if ((x != gx) || (y != gy))
        return NAV_ABORT;

    /* 回到 4 向代码前，把朝向对齐到最近的正交方向 */
    if (GucMouseDir & 0x01)
    {
        if (!mouseTurnTo((uint8)(GucMouseDir & 0xFE)))
            return NAV_ABORT;
    }

    return NAV_OK;
}

uint8 mouseSpurt_8_to(uint8 gx, uint8 gy)
{
    uint8 ret;

    if (!Nav_BuildDistance(gx, gy))
        return NAV_NO_PATH;

    if (Nav_Distance((uint8)GmcMouse.cX, (uint8)GmcMouse.cY) == NAV_INF)
        return NAV_NO_PATH;

    ret = Nav_RunPath(gx, gy);
    if (ret != NAV_OK)
        mouseStop();

    return ret;
}

uint8 mouseSpurt_8_to_goal(uint8 gx0, uint8 gx1, uint8 gy0, uint8 gy1)
{
    uint8 i;
    uint8 xs[4];
    uint8 ys[4];
    uint8 bestX = 0, bestY = 0;
    uint16 best = NAV_INF;
    uint16 d;

    xs[0] = gx0; ys[0] = gy0;
    xs[1] = gx0; ys[1] = gy1;
    xs[2] = gx1; ys[2] = gy0;
    xs[3] = gx1; ys[3] = gy1;

    for (i = 0; i < 4; i++)
    {
        if (!Nav_BuildDistance(xs[i], ys[i]))
            continue;

        d = Nav_Distance((uint8)GmcMouse.cX, (uint8)GmcMouse.cY);
        if (d < best)
        {
            best = d;
            bestX = xs[i];
            bestY = ys[i];
        }
    }

    if (best == NAV_INF)
        return NAV_NO_PATH;

    /* 重新按选中的终点建图，再执行 */
    if (!Nav_BuildDistance(bestX, bestY))
        return NAV_NO_PATH;

    if (Nav_Distance((uint8)GmcMouse.cX, (uint8)GmcMouse.cY) == NAV_INF)
        return NAV_NO_PATH;

    return Nav_RunPath(bestX, bestY);
}

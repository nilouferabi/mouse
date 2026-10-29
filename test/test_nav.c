/*
 * Host-side test for the 8-neighbour planner.  Builds Mouse_Nav.c with
 * minimal stubs so the path/corner rules can be checked without hardware.
 */
#include <stdio.h>
#include <string.h>

#define __Maze_h
#define __Mouse_Drive_h
#define __MOUSE_NAV_H

typedef unsigned char  uint8;
typedef signed char    int8;
typedef unsigned short uint16;
typedef signed short   int16;

#define MAZETYPE 16
#define NAV_OK      1
#define NAV_NO_PATH 0
#define NAV_ABORT   2

typedef struct { int8 cX; int8 cY; } MAZECOOR;

uint8 GucMapBlock[MAZETYPE][MAZETYPE];
MAZECOOR GmcMouse;
uint8 GucMouseDir;

/* stubs used by Mouse_Nav.c */
uint8 mouseTurnTo(uint8 targetDir) { GucMouseDir = targetDir; return 1; }
uint8 mouseGoOneBlock(uint8 dir) { (void)dir; return 1; }
void  mouseStop(void) { }

#include "../Mouse_Nav.c"

static int failures = 0;

static void check(int cond, const char *msg)
{
    if (!cond) {
        printf("FAIL: %s\n", msg);
        failures++;
    } else {
        printf("ok  : %s\n", msg);
    }
}

static void map_all_open(void)
{
    int x, y;
    for (x = 0; x < MAZETYPE; x++)
        for (y = 0; y < MAZETYPE; y++)
            GucMapBlock[x][y] = 0x0F;
}

static void map_all_blocked(void)
{
    memset(GucMapBlock, 0, sizeof(GucMapBlock));
}

int main(void)
{
    int8 dx, dy;
    uint8 dir;
    uint16 d;
    int steps;

    /* 1. open 16x16: 15 diagonal steps => 15*14 = 210 */
    map_all_open();
    check(Nav_BuildDistance(15, 15) == 1, "build distance on open map");
    d = Nav_Distance(0, 0);
    check(d == 210, "open map distance = 15 diagonals");

    /* 2. next step should be NE (1), not N/E */
    check(Nav_NextStep(0, 0, 0, &dx, &dy, &dir) == 1, "open map has next step");
    check((dx == 1) && (dy == 1) && (dir == 1), "open map first step is NE");

    /* 3. simulate the whole path, count steps and directions */
    GmcMouse.cX = 0; GmcMouse.cY = 0; GucMouseDir = 0;
    steps = 0;
    while ((GmcMouse.cX != 15) || (GmcMouse.cY != 15)) {
        if (!Nav_NextStep((uint8)GmcMouse.cX, (uint8)GmcMouse.cY,
                          GucMouseDir, &dx, &dy, &dir)) break;
        GucMouseDir = dir;
        GmcMouse.cX = (int8)(GmcMouse.cX + dx);
        GmcMouse.cY = (int8)(GmcMouse.cY + dy);
        steps++;
        if (steps > 100) break;
    }
    check((GmcMouse.cX == 15) && (GmcMouse.cY == 15), "greedy path reaches (15,15)");
    check(steps == 15, "greedy path uses 15 steps");

    /* 4. close the north wall of (0,0): diagonal (0,0)->(1,1) must be illegal,
          and first step must become E (2). */
    map_all_open();
    GucMapBlock[0][0] &= (uint8)~0x01;   /* no N */
    GucMapBlock[0][1] &= (uint8)~0x04;   /* no S from (0,1) */
    check(Nav_CanMove(0, 0, 1, 1) == 0, "corner cut blocked by missing N wall");
    check(Nav_BuildDistance(1, 1) == 1, "rebuild distance for (1,1)");
    check(Nav_NextStep(0, 0, 0, &dx, &dy, &dir) == 1, "(0,0) still has a step");
    check(dir == 2, "first step detours to E when NE is blocked");

    /* 5. remove an internal wall in the 2x2 block: (1,0) north is blocked */
    map_all_open();
    GucMapBlock[1][0] &= (uint8)~0x01;   /* (1,0) no N */
    GucMapBlock[1][1] &= (uint8)~0x04;   /* (1,1) no S */
    check(Nav_CanMove(0, 0, 1, 1) == 0, "corner cut blocked by target-side wall");

    /* 6. fully unknown map: no path from (0,0) to (15,15) */
    map_all_blocked();
    check(Nav_BuildDistance(15, 15) == 1, "distance map can be built on blocked map");
    check(Nav_Distance(0, 0) == 0xFFFF, "blocked map has no path");

    /* 7. executor path on open map: should reach (15,15) and snap to a cardinal dir */
    map_all_open();
    GmcMouse.cX = 0; GmcMouse.cY = 0; GucMouseDir = 0;
    check(mouseSpurt_8_to(15, 15) == NAV_OK, "mouseSpurt_8_to reaches target");
    check((GmcMouse.cX == 15) && (GmcMouse.cY == 15), "executor ends at (15,15)");
    check((GucMouseDir & 1) == 0, "executor snaps to cardinal heading");

    /* 8. no path returns NAV_NO_PATH */
    map_all_blocked();
    GmcMouse.cX = 0; GmcMouse.cY = 0;
    check(mouseSpurt_8_to(15, 15) == NAV_NO_PATH, "blocked map returns NO_PATH");

    printf("%s (%d failures)\n", failures ? "TEST FAILED" : "ALL TESTS PASSED", failures);
    return failures ? 1 : 0;
}

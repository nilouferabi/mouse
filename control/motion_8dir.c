/****************************************Copyright (c)****************************************************
** File Name:           motion_8dir.c
** Description:         8 向运动原语：原地 45/90/135/180 度转向、正交/斜向走一格
*********************************************************************************************************/

#include "motion_8dir.h"
#include "motion_internal.h"
#include "motion_control.h"

static uint8 __turnInPlace(int8 steps45)
{
    uint8  right;
    float  target;
    uint32 start;

    if (steps45 == 0)
        return 1;

    right = (steps45 > 0) ? 1 : 0;
    target = (float)steps45 * 45.0f;
    target *= right ? TUNE_8DIR_TURN_TRIM_R : TUNE_8DIR_TURN_TRIM_L;

    Motion_ResetPID();
    Motion_Stop();

    __GmSPID.sRef = 0;
    __GmWPID.sRef = right ? -TUNE_8DIR_TURN_SPIN : TUNE_8DIR_TURN_SPIN;
    __GucMouseState = right ? __TURNRIGHT : __TURNLEFT;

    __GmLeft.cState  = __MOTORRUN;
    __GmRight.cState = __MOTORRUN;

    jiao = 0;
    start = GucSysTickMs;

    while (1)
    {
        if (right)
        {
            if (jiao >= target)
                break;
        }
        else
        {
            if (jiao <= target)
                break;
        }

        if ((GucSysTickMs - start) > TUNE_8DIR_TURN_TIMEOUT_MS)
        {
            Motion_Stop();
            return 0;
        }
    }

    Motion_Stop();
    GucMouseDir = (uint8)((GucMouseDir + 8 + steps45) % 8);
    __delay(TUNE_8DIR_STOP_DELAY_LOOPS);

    return 1;
}



uint8 mouseTurnTo(uint8 targetDir)
{
    int8 delta;

    if (targetDir > 7)
        return 0;

    delta = (int8)((targetDir + 8 - GucMouseDir) % 8);
    if (delta > 4)
        delta = (int8)(delta - 8);

    if (delta == 0)
        return 1;

    return __turnInPlace(delta);
}



uint8 mouseGoOneBlock(uint8 dir)
{
    uint32 distance;
    uint32 start;
    uint8  diagonal;

    if (dir > 7)
        return 0;

    diagonal = (uint8)(dir & 0x01);
    distance = diagonal ? TUNE_8DIR_PULSE_DIAG : TUNE_8DIR_PULSE_ORTHO;

    Motion_ResetPID();
    Motion_Stop();

    jiao = 0;
    __GucMouseState = diagonal ? __GOAHEAD_45 : __GOAHEAD;
    GuiSpeedCtr = __SPEEDUP;
    __GiMaxSpeed = diagonal ? TUNE_8DIR_SPEED_DIAG : TUNE_8DIR_SPEED_ORTHO;

    __GmRight.uiPulse = distance;
    __GmLeft.uiPulse  = distance;
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr  = 0;
    __GmRight.cState = __MOTORRUN;
    __GmLeft.cState  = __MOTORRUN;

    start = GucSysTickMs;

    while ((__GmRight.uiPulseCtr < __GmRight.uiPulse) ||
           (__GmLeft.uiPulseCtr  < __GmLeft.uiPulse))
    {
        if ((GucSysTickMs - start) > TUNE_8DIR_MOVE_TIMEOUT_MS)
        {
            Motion_Stop();
            return 0;
        }

        /* 斜向安全：两个前向近距都触发，说明前方/角落有障碍，立即停车 */
        if (TUNE_8DIR_FRONT_SAFETY && diagonal &&
            (ucIRCheck[0] > GusDistance_FL_Near) &&
            (ucIRCheck[1] > GusDistance_FR_Near))
        {
            Motion_Stop();
            return 0;
        }
    }

    Motion_Stop();
    __delay(TUNE_8DIR_STOP_DELAY_LOOPS);

    return 1;
}



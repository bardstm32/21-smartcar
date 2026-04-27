#include "element.h"
#include "imu.h"
#include <math.h>

// 状态与偏差全局定义
TrackState_e TrackState = NORMAL;

// 核心阈值定义
#define THRES_CROSS 800
#define THRES_ROUND 1200
#define PITCH_RAMP 12.0
#define PITCH_SEE 6.0
#define ROLL_ROLLER 15.0
#define WALL_SUM_LOW 200

// 注意这里：把 int16_t 改成了 int16
void Element_Control(int16 *param)
{
    // 注意这里：把 int32_t 改成了 int32
    int32 total_sum = param[0] + param[1] + param[2] + param[3];

    switch (TrackState)
    {
    case NORMAL:
        // --- 滚筒判定：依据 Roll 角 ---
        if (fabs(first_complement.angle.roll) > ROLL_ROLLER)
        {
            TrackState = ROLLER;
        }
        // --- 竖直墙面判定：电感集体失灵，但姿态平稳 ---
        else if (total_sum < WALL_SUM_LOW && fabs(first_complement.angle.pitch) < 3.0)
        {
            TrackState = WALL;
        }
        // --- 坡道与跷跷板判定：依据 Pitch 角 ---
        else if (first_complement.angle.pitch > PITCH_SEE)
        {
            if (first_complement.angle.pitch > PITCH_RAMP)
                TrackState = RAMP;
            else
                TrackState = SEESAW;
        }
        // --- 环岛判定：单侧远端电感激增 ---
        else if (param[0] > THRES_ROUND || param[3] > THRES_ROUND)
        {
            TrackState = ROUNDABOUT;
        }
        // --- 十字判定：两侧远端电感同时激增 ---
        else if (param[0] > THRES_CROSS && param[3] > THRES_CROSS)
        {
            TrackState = CROSS;
        }
        break;

    case WALL:
        if (total_sum > 400)
            TrackState = NORMAL;
        break;

    case ROLLER:
        if (fabs(first_complement.angle.roll) < 5.0)
            TrackState = NORMAL;
        break;

    case SEESAW:
        if (first_complement.angle.pitch < -2.0)
            TrackState = NORMAL;
        break;

    case ROUNDABOUT:
        if (param[0] < 300 && param[3] < 300)
            TrackState = NORMAL;
        break;

    case CROSS:
        if (param[0] < THRES_CROSS && param[3] < THRES_CROSS)
            TrackState = NORMAL;
        break;
    }
}
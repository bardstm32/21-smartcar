#include "element.h"

// 状态与偏差全局定义
TrackState_e TrackState = NORMAL;

// 核心阈值定义
#define THRES_CROSS 800  // 十字路口远端阈值
#define THRES_ROUND 1200 // 环岛触发阈值
#define PITCH_RAMP 12.0  // 坡道触发俯仰角
#define PITCH_SEE 6.0    // 跷跷板触发角
#define ROLL_ROLLER 15.0 // 滚筒横滚角阈值
#define WALL_SUM_LOW 200 // 墙面信号骤减阈值

void Element_Control(uint16_t *param)
{

    switch (TrackState)
    {

    case NORMAL:
        // --- 滚筒判定：依据 Roll 角 ---
        if (fabs(Roll) > ROLL_ROLLER)
        {
            TrackState = ROLLER;
        }
        // --- 竖直墙面判定：电感集体失灵，但姿态平稳 ---
        else if (total_sum < WALL_SUM_LOW && fabs(Pitch) < 3.0)
        {
            TrackState = WALL;
        }
        // --- 坡道与跷跷板判定：依据 Pitch 角 ---
        else if (Pitch > PITCH_SEE)
        {
            // 坡道倾角通常较大且稳定，跷跷板会有明显的入板冲击（Pitch变化率大）
            if (Pitch > PITCH_RAMP)
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
        // 墙面逻辑：通常保持直行，建议锁定 TrackError = 0 或保持上一次值
        if (total_sum > 400)
            TrackState = NORMAL;
        break;

    case ROLLER:
        // 滚筒逻辑：车身倾斜，巡线偏差需要根据 Roll 角进行物理补偿
        if (fabs(Roll) < 5.0)
            TrackState = NORMAL;
        break;

    case SEESAW:
        // 跷跷板逻辑：检测到 Pitch 反转（从正变负）代表过支点，准备下板
        if (Pitch < -2.0)
            TrackState = NORMAL;
        break;

    case ROUNDABOUT:
        // 环岛逻辑：此处可配合 Gyro_Z 积分判断转向度数，判定出环
        // 简单逻辑：两侧电感恢复正常分布则退出
        if (param[0] < 300 && param[3] < 300)
            TrackState = NORMAL;
        break;

    case CROSS:
        // 十字逻辑：远端信号消失后回归直道
        if (param[0] < THRES_CROSS && param[3] < THRES_CROSS)
            TrackState = NORMAL;
        break;
    }
}
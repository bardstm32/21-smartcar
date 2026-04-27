#ifndef __ELEMENT_H
#define __ELEMENT_H

#include "zf_common_headfile.h" // 或者直接包含 "zf_common_typedef.h"

// 状态枚举定义
typedef enum
{
    NORMAL = 0,
    CROSS,      // 十字
    ROUNDABOUT, // 环岛
    SEESAW,     // 跷跷板
    WALL,       // 竖直墙面
    ROLLER,     // 滚筒
} TrackState_e;

extern TrackState_e TrackState;

// 注意这里：把 int16_t 改成了 int16
void Element_Control(int16 *param);

#endif
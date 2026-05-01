#ifndef __ELEMENT_H
#define __ELEMENT_H

#include "zf_common_headfile.h" // 或者直接包含 "zf_common_typedef.h"

// 状态枚举定义
typedef enum
{
    NORMAL = 0,
    CROSS = 1,      // 十字
	ROUNDAPPROCH = 2,
	ROUNDIN = 3,
    ROUNDOUT = 4, // 环岛
    SEESAW = 5,     // 跷跷板
    WALL = 6,       // 竖直墙面
    ROLLER = 7,     // 滚筒
} TrackState_e;

extern TrackState_e TrackState;
extern float Nowangel;

// 注意这里：把 int16_t 改成了 int16
void Element_Control(int16 *param);

#endif
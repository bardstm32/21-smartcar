#ifndef __ELEMENT_H
#define __ELEMENT_H

#include "zf_common_headfile.h" // 或者直接包含 "zf_common_typedef.h"

// 状态枚举定义
typedef enum
{
    NORMAL = 0,
    CROSS = 1,      // 十字
	RIGHT_ROUNDAPPROCH = 2,
	LEFT_ROUNDAPPROCH = 3,
	RIGHT_ROUND = 4,
	LEFT_ROUND = 5,
	ROUNDIN = 6,
    ROUNDOUT = 7, // 环岛
    SEESAW = 8,     // 跷跷板
    WALL = 9,       // 竖直墙面
    ROLLER = 10,     // 滚筒
} TrackState_e;

extern TrackState_e TrackState;
extern float Nowangel;
extern uint8 ring_dir;

// 注意这里：把 int16_t 改成了 int16
void Element_Control(uint16 *param);

#endif
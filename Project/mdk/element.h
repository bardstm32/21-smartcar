#ifndef __ELEMENT_H__
#define __ELEMENT_H__

#include "zf_common_headfile.h"

// 状态枚举定义
typedef enum
{
    NORMAL = 0,
    CROSS,      // 十字
    ROUNDABOUT, // 环岛
    SEESAW,     // 跷跷板
    WALL,       // 竖直墙面
    ROLLER      // 滚筒
} TrackState_e;

// 全局变量声明（需在.c中定义）
extern TrackState_e TrackState;

// 函数声明
void Element_Control(int16_t *param);

#endif
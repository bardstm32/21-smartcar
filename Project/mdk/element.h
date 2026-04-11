#ifndef ELEMENT_H
#define ELEMENT_H

#include "zf_common_headfile.h"

typedef enum
{
    Normal = 0,     // 常规赛道（直道、普通弯道）
    RightAngle = 2, // 直角弯道
    Crossroads = 3, // 十字交叉路口
    Loop = 4,       // 环岛（包括今年新增的六边形环岛）

    Vertical = 5, // 垂直面
    Roller = 5,   // 滚筒面
    Seesaw = 6,   // 跷跷板
} Element_Type;

#endif
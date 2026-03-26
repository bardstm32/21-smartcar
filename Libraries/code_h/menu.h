#ifndef __MENU_H__
#define __MENU_H__

#include "zf_common_headfile.h"

// 菜单初始化函数，用于刷白屏和显示静态文本
void Menu_Init(void);

// 菜单刷新函数，传入包含4个元素的电感数据数组
void Menu_Display(uint16 *inductance_data);

#endif // __MENU_H__
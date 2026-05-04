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

float Nowangel = 0.0f;
// 注意这里：把 int16_t 改成了 int16
void Element_Control(int16 *param)
{
    switch (TrackState)
    {
        case NORMAL:
            // 环岛判定：单侧远端电感激增
            if ((((param[1] > 45) && (param[4] > 60) && (param[3] >=25))) || ((param[4] > (35 + param[1])) 
				&& param[1]>=38) || (param[1] + param[4])>=110)
            {
                TrackState = ROUNDAPPROCH;	
                Nowangel = Daty_Z; // 记录初始角度
            }
            break;
            
        case ROUNDAPPROCH:
            // 入环强制差速中... 直到车体实际上偏转了约60度，认为已经成功入环
            if (My_abs(Nowangel - Daty_Z) >= 40) 
            {
                TrackState = ROUNDIN;
            }
            break;
            
        case ROUNDIN:
            // 环内循迹中... 转了差不多一圈（例如270-300度），准备出环
            if (My_abs(Nowangel - Daty_Z) >= 300) // 提前一点出环，避免错过
            {
                TrackState = ROUNDOUT;
                Nowangel = Daty_Z; // 重新记录角度用于出环判定
            }
            break;
            
        case ROUNDOUT:
            // 出环强制差速中... 当反向转出一定角度，或者检测到双边电感恢复对称时，回到NORMAL
            if (My_abs(Nowangel - Daty_Z) >= 30) // 出环转出30度左右
            {
                TrackState = NORMAL;
            }
            break;
    }
    Send_Data_To_PC();
}
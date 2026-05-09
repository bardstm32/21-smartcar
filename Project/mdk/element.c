#include "element.h"
#include "imu.h"

// 状态与偏差全局定义
TrackState_e TrackState = NORMAL;
uint16 times = 0;
// 核心阈值定义
float Nowangel = 0;
void Element_Control(int16 *param)
{
    switch (TrackState)
    {
        case NORMAL:
            // 环岛判定：单侧远端电感激增
		if ((param[3] >= 20) && (param[3] <= 40) && (param[4] >= 50) && (param[2] <= 35) && param[2] >= 20)
            {
				if(param[1] >= 30){times++;}
				if(times >= 3)
				{                
					TrackState = RIGHT_ROUNDAPPROCH;	
					times = 0;
				}

            }
			
			else if ((param[2] >= 20) &&(param[2] <= 40)&& (param[1] >= 50) && (param[3] <= 35))//左侧横电感>75 && 右侧横电感 >40 
            {
				if(param[4] >= 30){times++;}
				if(times >= 3)
				{                
					TrackState = LEFT_ROUNDAPPROCH;	//识别为左环岛
					times = 0;
				}
            }
            break;
			
		case RIGHT_ROUNDAPPROCH:
			if((param[4] >=50) && (param[3] >= 40))
			{
				times++;
				if(times >=2)
				{
					TrackState = RIGHT_ROUND;	
					Nowangel = Daty_Z; // 记录初始角度
					times = 0;
				}
			}
			break;
			
		case LEFT_ROUNDAPPROCH:
            if((param[1] >= 50) && param[2] >= 40)
			{
				times++;
				if(times >=2)
				{
					TrackState = LEFT_ROUND;	
					Nowangel = Daty_Z; // 记录初始角度
					times = 0;
				}
			}		
			break;
		case LEFT_ROUND:
		case RIGHT_ROUND:
            // 入环强制差速中... 直到车体实际上偏转了约40度，认为已经成功入环
            if (My_abs(Nowangel - Daty_Z) >= 35) 
            {
                TrackState = ROUNDIN;
            }
            break;
            
        case ROUNDIN:
            // 环内循迹中... 转了差不多一圈（例如270-300度），准备出环
            if (My_abs(Nowangel - Daty_Z) >= 260) // 提前一点出环，避免错过
            {
                TrackState = ROUNDOUT;
                Nowangel = Daty_Z; // 重新记录角度用于出环判定
            }
            break;
            
        case ROUNDOUT:
			Distance_Add();
            // 出环强制差速中... 当反向转出一定角度，或者检测到双边电感恢复对称时，回到NORMAL
            if (distance >= 20000) // 出环转出30度左右
            {
                TrackState = NORMAL;
				distance = 0;
            }
            break;
    }
}
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

extern float Nowangel = 0.0f;
// 注意这里：把 int16_t 改成了 int16
void Element_Control(int16 *param)
{
    switch (TrackState)
    {
		case NORMAL:
			// --- 环岛判定：单侧远端电感激增 ---
			if (((param[1] > 38) && (param[4] > 60) && (param[1] + param[4]>=100)) || ((param[4] > (30 + param[1]))))
			{
				TrackState = ROUNDAPPROCH;	
				Nowangel = Daty_Z;							
			}
			break;
		case ROUNDAPPROCH:
			if ((Nowangel - Daty_Z)>=40)
			{
				TrackState = ROUNDIN;
//				Send_Data_To_PC();
			}
			break;
		case ROUNDIN:
			if ((Nowangel - Daty_Z)>=310)
			{
				TrackState = ROUNDOUT;
//				Send_Data_To_PC();
			}
			break;
//		case ROUNDOUT:
//			if (My_abs(Daty_Z - Nowangel)<5)
//			{
//				TrackState = NORMAL;
////				Send_Data_To_PC();
//			}
//			break;
    }
	Send_Data_To_PC();
}
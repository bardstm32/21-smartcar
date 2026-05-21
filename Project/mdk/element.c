#include "element.h"
#include "imu.h"

TrackState_e TrackState = NORMAL;
uint8 times = 0;

float Nowangel = 0;
uint8 dur_time = 0;
uint8 ring_dir = 0;
float ang_diff = 0;
/* 赛道元素状态机：根据电感与陀螺角度切换 NORMAL / 环岛各阶段 */
void Element_Control(uint16 *param)
{
    switch (TrackState)
    {
        case NORMAL:

		if ((param[3] >= 6) && (param[3] <= 20) && (param[4] >= 40) && (param[2] <= 25) && first_complement.angle.pitch <=30)
            {
//				times++;
//				if(times >= 2)
//				{
					TrackState = RIGHT_ROUNDAPPROCH;
//					times = 0;
//				}

            }

		else if ((param[2] >= 6) &&(param[2] <= 20)&& (param[1] >= 40) && (param[3] <= 25) && first_complement.angle.pitch <=30)
            {
//				times++;
//				if(times >= 2)
//				{
					TrackState = LEFT_ROUNDAPPROCH;
					times = 0;
//				}
            }
            
		else if ((param[2]+param[3])>=90)
            {
				TrackState = CROSS;
				times = 0;
            }
            break;
		case RIGHT_ROUNDAPPROCH:
			dur_time++;
			if((param[4] >=45) && (param[3] >= 35))
			{
//				times++;
//				if(times >=2)
//				{
					TrackState = RIGHT_ROUND;
					Nowangel = Daty_Z;
					times = 0;
					ring_dir = 1;
//				}
			}
			if(dur_time >= 100){TrackState = NORMAL;dur_time = 0;}
			break;

		case LEFT_ROUNDAPPROCH:
			dur_time++;
            if((param[1] >= 45) && param[2] >= 35)
			{
//				times++;
//				if(times >=2)
//				{
					TrackState = LEFT_ROUND;
					Nowangel = Daty_Z;
					times = 0;
					ring_dir = 2;
//				}
			}
			if(dur_time >= 100){TrackState = NORMAL;dur_time = 0;}
			break;
		case LEFT_ROUND:
		case RIGHT_ROUND:
			dur_time = 0;
            {
                float ang_diff = Nowangel - Daty_Z;
                if (ang_diff < 0) ang_diff = -ang_diff;
                if (ang_diff >= 35) TrackState = ROUNDIN;
            }
            break;

        case ROUNDIN:

            {
                ang_diff = Nowangel - Daty_Z;
                if (ang_diff < 0) ang_diff = -ang_diff;
                if (ang_diff >= 290)
                {
                    TrackState = ROUNDOUT;
                    Nowangel = Daty_Z;
                }
            }
            break;

        case ROUNDOUT:
			ang_diff = Nowangel - Daty_Z;
            if (ang_diff < 0) ang_diff = -ang_diff;
			Distance_Add();
            if (distance >= 20000 || ang_diff >= 25)
            {
                TrackState = NORMAL;
				ring_dir = 0;
				distance = 0;
            }
            break;
		case CROSS:
			dur_time++;
            if (IABS(param[2]-param[3])<=5)
            {
                TrackState = NORMAL;
            }
			if(dur_time >= 20){TrackState = NORMAL;dur_time = 0;}
            break;
    }
}
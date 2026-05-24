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

		if ((param[3] >= 4) && (param[3] <= 20) && (param[4] >= 40) && (param[2] <= 25))
        {
			TrackState = RIGHT_ROUNDAPPROCH;
        }
		
//		if(first_complement.angle.roll <=-15 && first_complement.angle.roll >=-20)
//		{
//			TrackState = SEESAW;
//		}
		
		else if ((param[2] >= 6) &&(param[2] <= 20)&& (param[1] >= 40) && (param[3] <= 25))
        {
			TrackState = LEFT_ROUNDAPPROCH;
        }
            
		else if ((param[2]>=55 && param[3]>=55))
        {
			TrackState = CROSS;
        }
        break;
		
		case RIGHT_ROUNDAPPROCH:
			dur_time++;
			if((param[4] >=30) && (param[3] >= 40))
			{
					TrackState = RIGHT_ROUND;
					Nowangel = Daty_Z;
					times = 0;
					ring_dir = 1;
			}
			if(dur_time >= 80){TrackState = NORMAL;dur_time = 0;}
			break;

		case LEFT_ROUNDAPPROCH:
			dur_time++;
            if(((param[1] >= 50) && param[2] >= 35))
			{
					TrackState = LEFT_ROUND;
					Nowangel = Daty_Z;
					times = 0;
					ring_dir = 2;
			}
			if(dur_time >= 80){TrackState = NORMAL;dur_time = 0;}
			break;
		case LEFT_ROUND:
		case RIGHT_ROUND:
			dur_time = 0;
            {
                float ang_diff = Nowangel - Daty_Z;
                if (ang_diff < 0) ang_diff = -ang_diff;
                if (ang_diff >= 20) TrackState = ROUNDIN;
            }
            break;

        case ROUNDIN:
            {
                ang_diff = Nowangel - Daty_Z;
                if (ang_diff < 0) ang_diff = -ang_diff;
                if (ang_diff >= 300)
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
			if((ring_dir == 1 && param[3] <= 45 && param[4] <= 45) || distance >=20000)
			{
				TrackState = NORMAL;
				Nowangel = Daty_Z;
				ring_dir = 0;
				distance = 0;
			}
            else if((ring_dir == 2 && param[1] <= 45 && param[2] <= 45) || distance >=20000)
            {
                TrackState = NORMAL;
				Nowangel = Daty_Z;
				ring_dir = 0;
				distance = 0;
            }
            break;
			
		case CROSS:
			dur_time++;
            if (IABS(param[2]-param[3])<=10)
            {
                TrackState = NORMAL;
            }
			if(dur_time >= 20){TrackState = NORMAL;dur_time = 0;}
            break;
		
//		case SEESAW:
//			BASE_SPEED = 100;
//			if(first_complement.angle.roll >= 15 || first_complement.angle.roll <=-20)
//			{
//				TrackState = NORMAL;
//				BASE_SPEED = 295;
//			}
    }
}
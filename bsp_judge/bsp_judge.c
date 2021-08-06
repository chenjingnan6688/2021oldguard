#include "bsp_judge.h"
#include "main.h"
#include "judge.h"
#include "pid.h"
#include "stm32f4xx_it.h"
#include "bsp_can.h"
#include "Classic.h"

extern fp32 fspeed;
void Check_Hit(void)
{
	if(Hurt_Data_Update==true&&range_choice==0)
	{
		direction =-direction;
		Last_Dir=direction;
		fspeed=direction*speed;
		range_choice=rand()%2;
    for(int i=0;i<2;i++)
			 {
					PID_calc(&motor_pid[i],gear_motor_data[ Moto_ID[i] ].speed_rpm,fspeed);
				  Motor_Output[ Moto_ID[i] ]=motor_pid[i].out;
			 }		
		HAL_Delay (1);
	}
			
}


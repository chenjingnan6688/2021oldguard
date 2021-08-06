#include "Classic.h"
#include "pid.h"
#include <math.h>
#include <stdlib.h>
#include "bsp_can.h"
#include "stm32f4xx_it.h"
#include "judge.h"
#include "crc.h"
pid_type_def motor_pid[2];
fp32 PID[3]={1.5,0.10,0};
uint8_t Moto_ID[2]={Chassic_L,Chassic_R};
int direction=1;
uint8_t Last_Dir=1;
//float Rail_Position=0.f;
fp32 tspeed=0;
pid_type_def motor_pid[2];
//uint8_t Changing_Speed_Flag=0;	//改变速度方向时置1，由变速函数置0		光电开关和闪避检测置1
void Classic_init(void)
{
	for(int i=0;i<2;i++)
	{
		PID_init(&motor_pid[i],PID_POSITION,(const fp32*)&PID,30000,30000);
	}
}

#define Sample_Times 6
uint8_t Measuer_State=Ready_Measure;
uint32_t Rail_Len=0;
uint32_t Rail_Len_Buff[Sample_Times]={0};
uint16_t Rail_Len_Buff_cnt=0;
void Measuer_Rail_Len(void)
{
	if(Measuer_State==End_Measure)
		return;
	if(direction!=Last_Dir)
	{
		if(Measuer_State==Measuring)
		{
			Rail_Len_Buff[Rail_Len_Buff_cnt]=abs(gear_motor_data[Moto_ID[0]].round_cnt);
			++Rail_Len_Buff_cnt;
			if(Rail_Len_Buff_cnt!=Sample_Times)
			{
				Measuer_State=Ready_Measure;
			}
			else
			{
				uint32_t sum=0;
				for(uint8_t i=1; i<Sample_Times; ++i)
					sum += Rail_Len_Buff[i];
				Rail_Len = sum/(Sample_Times-1);
				Measuer_State=End_Measure;
			}
		}
	}
	else
	{
		if(Measuer_State==Ready_Measure)
		{
			gear_motor_data[Moto_ID[0]].round_cnt=0;
			gear_motor_data[Moto_ID[1]].round_cnt=0;
			Measuer_State=Measuring;
		}
	}
}
void Go_To_Middle(uint16_t Speed)	//回到轨道中间
{
	int dir=0;
	if(Rail_Position<0.4f)
		dir=-1;
	else if(Rail_Position>0.6f)
		dir=1;
	for (uint8_t i=0; i<2; i++)
	{
		tspeed=dir*Speed;
		PID_calc(&motor_pid[i],gear_motor_data[ Moto_ID[i] ].speed_rpm,tspeed);
		Motor_Output[ Moto_ID[i] ]=motor_pid[i].out;
		//motor_pid[i].f_cal_pid(&motor_pid[i], gear_motor_data[ Moto_ID[i] ].speed_rpm);
	}
	if(dir!=0)
	{
		for (uint8_t i=0; i<2; i++)
		Motor_Output[ Moto_ID[i] ]=motor_pid[i].out;
	}
	else
	{
		for (uint8_t i=0; i<2; i++)
		Motor_Output[ Moto_ID[i] ]=0;
	}
}

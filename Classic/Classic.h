#ifndef CLASSIC_H
#define CLASSIC_H

#include "main.h"
#include "pid.h"
#include <stdbool.h>
#define Motor_Base 0x201
#define Chassic_Slow 300

extern uint8_t Last_Dir;
extern int direction;
extern uint8_t Moto_ID[2];
//extern float Rail_Position;
extern pid_type_def motor_pid[2];
extern uint32_t Rail_Len;
extern uint8_t Measuer_State;
extern uint8_t Changing_Speed_Flag;
void Classic_init(void);
typedef enum
{
	Chassic_Left_3508_ID = 0x206,
	Chassic_Right_3508_ID = 0x208,
	Cartridge_2006_ID = 0x203,
}CAN_Message_ID;



typedef enum
{
	Chassic_L = Chassic_Left_3508_ID	-Motor_Base,
	Chassic_R = Chassic_Right_3508_ID	-Motor_Base,
	Cartridge = Cartridge_2006_ID			-Motor_Base,
}Motor_Data_ID;

enum
{
	Ready_Measure,
	Measuring,
	End_Measure
};
//void Spring(int dir,uint16_t Speed);
void Measuer_Rail_Len(void);
void Go_To_Middle(uint16_t Speed);
#endif

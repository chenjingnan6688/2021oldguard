/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usartinfo.h"
#include "bsp_usart.h"
#include "Remote_Control.h"
#include "Classic.h"
#include "bsp_can.h"
#include "pid.h"
#include <stdlib.h>
#include <stdbool.h>
#include "gpio.h"
#include "can.h"
#include "judge.h"
#include "crc.h"
//#include "bsp_judge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define touch_Left -1
#define touch_Right 1 
#define Cold_time 200
#define FAST_SPEED 6000
//#define Stop 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//uint8_t Motor_Power_Up=0;	//判断电机上电，1上电完成
int eliminate_dithering_left = 0;
int eliminate_dithering_right = 0;
//extern int direction=1;
//extern uint8_t dir=1;
fp32 fspeed=0;
uint16_t Speed_change_time;
//uint32_t range_choice;
uint32_t timecount=0;
uint32_t hurt_time_count=0;
uint32_t speed_up_cnt=0;
uint32_t speed_time=0u;
uint8_t L_=0;
uint8_t R_=0;
uint32_t count_cnt=0;
//uint32_t judge_time=0;
float Rail_Position=0.f;
extern bool Hurt_Data_Update;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
  static uint32_t TIM1_cnt=0;
	++TIM1_cnt;
	if(TIM1_cnt==0xFFFFFFFF)
		TIM1_cnt=0;
	
	//++timecount;
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
	 L_=HAL_GPIO_ReadPin(REDL_GPIO_Port, REDL_Pin);
	 R_=HAL_GPIO_ReadPin(REDR_GPIO_Port, REDR_Pin);
		   switch(remote_control.switch_right)
		{
			case 1:
			case 3:
			{
				if(HAL_GPIO_ReadPin(REDL_GPIO_Port, REDL_Pin) == GPIO_PIN_RESET)	//左边检测到墙壁，开始反转
			{
				eliminate_dithering_right = 0;
				eliminate_dithering_left++;
				if (eliminate_dithering_left >= 20) //消抖
				{
					direction = touch_Left;		
				}
			}
			 else if(HAL_GPIO_ReadPin(REDR_GPIO_Port, REDR_Pin) == GPIO_PIN_RESET)
			{
				eliminate_dithering_right++;
				eliminate_dithering_left = 0;
				if (eliminate_dithering_right >= 20)
				{
					direction = touch_Right;
				}
			}
			  fspeed=direction*speed;
			  Last_Dir=direction;
				if(Measuer_State!=End_Measure)
				{ 
					for(int i=0;i<2;i++)
			   {
					PID_calc(&motor_pid[i],gear_motor_data[ Moto_ID[i] ].speed_rpm,fspeed);
				  Motor_Output[ Moto_ID[i] ]=motor_pid[i].out;
				 }
					Measuer_Rail_Len();
				 if(Hurt_Data_Update==true)
				 {
					  Hurt_Data_Update=false;
				 }
				}
				else
				 {
					  Rail_Position=(abs((gear_motor_data[Moto_ID[0]].round_cnt))*1.f)/(Rail_Len*1.f);//轨道位置0-1
					  if(hurt_time_count!=0)
								{
								--hurt_time_count;
								}
						if(speed_time!=0)
						{
							--speed_time; 								
							 fspeed=direction*FAST_SPEED;//加速一秒，如果边缘反向时不加速了	
     						if(L_==1&&R_==0||L_==0&&R_==1)
							{
								speed_time=0;
								fspeed=direction*speed;
							}						
						}
						else
							fspeed=direction*speed;
									
					  if(Hurt_Data_Update==true)
					 {  
              Hurt_Data_Update=false;						 
					    if(hurt_time_count==0)
					    {   
							hurt_time_count=Cold_time;
               if(rand()%2==0)
							 {	
                  ++count_cnt;
                  timecount=0;								 
									direction =-direction;
									Last_Dir=direction;
									fspeed=direction*speed;
							 }
							 else if (PowerHeatData.chassis_power_buffer>50)
							 {
								 ++speed_up_cnt;
								 speed_time=200;		     
								 timecount=0;								 
							 }
							 
							}
													
							for(int i=0;i<2;i++)
							{
								PID_calc(&motor_pid[i],gear_motor_data[ Moto_ID[i] ].speed_rpm,fspeed);
								Motor_Output[ Moto_ID[i] ]=motor_pid[i].out;
							}
						}			
						else 
					 {
						 if(Rail_Position >0.1f&&Rail_Position <0.9f)
							{
							 ++timecount;
							}					 					 					 			 	 
							 if(timecount==Speed_change_time)//执行随机变向
							{
							timecount=0;
							Speed_change_time=(rand()%300)+300;
							direction =-direction;
							Last_Dir=direction;
							fspeed=direction*speed;
							}
								for(int i=0;i<2;i++)
							 {
							 PID_calc(&motor_pid[i],gear_motor_data[ Moto_ID[i] ].speed_rpm,fspeed);
							 Motor_Output[ Moto_ID[i] ]=motor_pid[i].out;
							 }
						}
					    
				 } 
				 break;
			 }				 
			case 2:
			{
				for(int i=0;i<2;i++)
				{
				  Motor_Output[ Moto_ID[i] ]=0;
					Last_Dir=0;
				}
				break;
			}
			default:
			{
				for(int i=0;i<2;i++)
				{
				  Motor_Output[ Moto_ID[i] ]=0;
				}
				break;
			}
	}			
		if(TIM1_cnt%2==0)		//2 freq div
		{
			CAN_Motor_Ctrl(&hcan1,Motor_Output);
		}
		//Changing_Speed_Flag=0;
	
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  Dma_UsartIdleHanlder(&huart1, 36);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */
  Dma_UsartIdleHanlder(&huart6, JUDGEMENT_BUF_LEN);
  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

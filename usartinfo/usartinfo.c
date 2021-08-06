#include "usartinfo.h"
#include "usart.h"
#include "string.h"
#include "Remote_Control.h"
#include "main.h"
#include "judge.h"
#include "crc.h"
#include "main.h"
uint8_t UART_Buffer[36];
uint8_t Judgement_Buf[JUDGEMENT_BUF_LEN];
uint8_t rx_judge_buf[200];

/**
  * @brief  ���ڿ����ж�DMA���ջص�����
  * @param  ����ͨ����ַ UART_HandleTypeDef *
  * @retval None
  */

void UART_IdleRxCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		Callback_RC_Handle(&remote_control,UART_Buffer);
	}
	else if (huart==&huart6 )
	{
		memcpy(&rx_judge_buf,Judgement_Buf,200);
		Judge_Read_Data(Judgement_Buf);
	}
	
}	

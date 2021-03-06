#include "bsp_can.h"
#include "main.h"
#include "Classic.h"

extern CAN_HandleTypeDef hcan1;
//extern CAN_HandleTypeDef hcan2;
#define ABS(x) ((x > 0) ? x : -x)

gear_moto_measure_t gear_motor_data[];
int16_t Motor_Output[12]={0};

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

//    can_filter_st.SlaveStartFilterBank = 14;
//    can_filter_st.FilterBank = 14;
//    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void get_gear_motor_measure(gear_moto_measure_t *ptr, uint8_t rxd[])
{
    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(rxd[0] << 8 | rxd[1]);
    ptr->speed_rpm = (int16_t)(rxd[2] << 8 | rxd[3]);
    ptr->real_current = (uint16_t)(rxd[4] << 8 | rxd[5]);
    ptr->temperate = rxd[6];
    if (ptr->angle - ptr->last_angle > 4096)
    {
        ptr->round_cnt--;
    }
    else if (ptr->angle - ptr->last_angle < -4096)
    {
        ptr->round_cnt++;
    }
		int res1, res2, delta;
		if(ptr->angle < ptr->last_angle)
		{			//可能的情况
			res1 = ptr->angle + 8192 - ptr->last_angle;	//正转，delta=+
			res2 = ptr->angle - ptr->last_angle;				//反转	delta=-
		}
		else
		{	//angle > last
			res1 = ptr->angle - 8192 - ptr->last_angle ;//反转	delta -
			res2 = ptr->angle - ptr->last_angle;				//正转	delta +
		}
	//不管正反转，肯定是转的角度小的那个是真的
		if(ABS(res1)<ABS(res2))
			delta = res1;
		else
			delta = res2;

	ptr->total_angle += delta;
	(ptr)->real_total_angle = ((((ptr)->total_angle)%(36*8192))*360)/(8192*36);	
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8]={0};
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if (hcan == &hcan1)
	{
		switch (rx_header.StdId)
		{
			case 0x201:
			case 0x202:
			case 0x203:
			case 0x204:
			case 0x205:
			case 0x206:
			case 0x207:
			case 0x208:
			case 0x209:
			case 0x20A:
			case 0x20B:
			{
				static uint8_t i = 0;
				i=rx_header.StdId-Motor_Base;
				get_gear_motor_measure(&gear_motor_data[i], rx_data);
				break;
			}
		default:
				break;
		}
	}
}
void CAN_Motor_Ctrl(CAN_HandleTypeDef *hcan, int16_t Motor_Data[12])
{
	CAN_TxHeaderTypeDef can_tx_message;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;
	
	uint8_t can_send_data[8];
	uint32_t send_mail_box;
	uint16_t Std_ID[3]={0x200,0x1FF,0x2FF};
	
	for(uint8_t i=0; i<2; ++i)	//没用到，改成2
	{
		can_tx_message.StdId = Std_ID[i];
		can_send_data[0] = Motor_Data[4*i] >> 8;
		can_send_data[1] = Motor_Data[4*i];
		can_send_data[2] = Motor_Data[4*i+1] >> 8;
		can_send_data[3] = Motor_Data[4*i+1];
		can_send_data[4] = Motor_Data[4*i+2] >> 8;
		can_send_data[5] = Motor_Data[4*i+2];
		can_send_data[6] = Motor_Data[4*i+3] >> 8;
		can_send_data[7] = Motor_Data[4*i+3];
		HAL_CAN_AddTxMessage(hcan, &can_tx_message, can_send_data, &send_mail_box);
	}
}
//void CAN_Send_Gimbal(CAN_HandleTypeDef *hcan, uint8_t Data[], uint8_t Len)
//{
//	CAN_TxHeaderTypeDef can_tx_message;
//	can_tx_message.IDE = CAN_ID_STD;
//	can_tx_message.RTR = CAN_RTR_DATA;
//	can_tx_message.DLC = Len;
//	uint8_t can_send_data[8];
//	uint32_t send_mail_box;
//	can_tx_message.StdId = 0x1AA;
//	memcpy(can_send_data,Data,Len);
//	HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
//}
//void CAN_Send_Gimbal2(CAN_HandleTypeDef *hcan, uint8_t Data[], uint8_t Len)
//{
//	CAN_TxHeaderTypeDef can_tx_message;
//	can_tx_message.IDE = CAN_ID_STD;
//	can_tx_message.RTR = CAN_RTR_DATA;
//	can_tx_message.DLC = Len;
//	uint8_t can_send_data[8];
//	uint32_t send_mail_box;
//	can_tx_message.StdId = 0x1BB;
//	memcpy(can_send_data,Data,Len);
//	HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
//}
///**
//  * @brief          return the chassis 3508 motor data point
//  * @param[in]      i: motor number,range [0,3]
//  * @retval         motor data point
//  */
///**
//  * @brief          返回底盘电机 3508电机数据指针
//  * @param[in]      i: 电机编号,范围[0,3]
//  * @retval         电机数据指针
//  */
//const gear_moto_measure_t *get_chassis_motor_measure_point(uint8_t i)
//{
//    return &gear_motor_data[(i & 0x08)];
//}


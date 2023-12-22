#include "can.h"
#include "../inc/motor.h"
#include "../inc/bsp_can.h"
#include "stm32f4xx_hal_can.h"

moto_measure_t moto_chassis[4] = {0};//4 chassis moto



void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void can_filter_init(void)
{
    CAN_FilterTypeDef filter;
    filter.FilterIdHigh=0x0000;
    filter.FilterIdLow=0x0000;
    filter.FilterMaskIdHigh=0x0000;
    filter.FilterMaskIdLow=0x0000;
    filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;
    //filter.FilterBank;
    filter.FilterMode=CAN_FILTERMODE_IDMASK;
    filter.FilterScale=CAN_FILTERSCALE_32BIT;
    filter.FilterActivation=CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank=14;

    filter.FilterBank=0;
    HAL_CAN_ConfigFilter(&hcan1,&filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

    filter.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &filter);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/

uint8_t monitor_Rx_message[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;

    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,monitor_Rx_message);

    MotorControlCANRx(hcan,&rx_header,monitor_Rx_message);
    //if(ultra_cap.canRxMsgCheck(hcan,rx_header))
    //{
    //    ultra_cap.canRxMsgCallback(hcan, rx_header, rx_data);
    //}
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/

/*
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{

    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
    ptr->speed_rpm  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
    ptr->real_current = (hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5])*5.f/16384.f;

    ptr->hall = hcan->pRxMsg->Data[6];


    if(ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt --;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt ++;
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
*/
/*this function should be called after system+can init */

/*
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
    ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
    ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )

*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
*/
/*
void get_total_angle(moto_measure_t *p){

    int res1, res2, delta;
    if(p->angle < p->last_angle){			//可能的情况
        res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
        res2 = p->angle - p->last_angle;				//反转	delta=-
    }else{	//angle > last
        res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
        res2 = p->angle - p->last_angle;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if(ABS(res1)<ABS(res2))
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta;
    p->last_angle = p->angle;
}
 */
/*
void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){

    hcan->pTxMsg->StdId = 0x200;
    hcan->pTxMsg->IDE = CAN_ID_STD;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->DLC = 0x08;
    hcan->pTxMsg->Data[0] = (iq1 >> 8);
    hcan->pTxMsg->Data[1] = iq1;
    hcan->pTxMsg->Data[2] = (iq2 >> 8);
    hcan->pTxMsg->Data[3] = iq2;
    hcan->pTxMsg->Data[4] = iq3 >> 8;
    hcan->pTxMsg->Data[5] = iq3;
    hcan->pTxMsg->Data[6] = iq4 >> 8;
    hcan->pTxMsg->Data[7] = iq4;

    HAL_CAN_Transmit(hcan, 100);
}
*/

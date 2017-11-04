		   /******************** (C) COPYRIGHT 2013 InMotion ********************
* File Name          : USART_FUN.c
* Author             : ������
* Version            : V1.0.1
* Date               : 2013.12.22
* Description        : Function file
**********************************************************************/
#ifndef  __USART_FUN_C__
#define  __USART_FUN_C__

#include "my_typedef.h"
#include "drv_usart.h"
//keys add note
/*
����ṹ:	AA AA + ������ + У��� + 55 55
��ͷ:		AA AA
��β:		55 55
У���:	��������ۼӺ�
ת�崦��: 	AA->A5 AA
		55->A5 55
		A5->A5 A5
*/

typedef struct
{
	bool Rec_Flag;		//���ձ�־
	bool Ctrl_Flag;		//ת�����־
	u16 Offset;		    //��ǰbufλ��
	u8 Last_Byte;		//�ϴ�����
	u8 Check_Sum;		//У���
}USART_FRAME_DEF;

//���ڰ��ṹ
#define USART_FRAMECTRL 0xA5                                                  
#define USART_FRAMEHEAD 0xAA
#define USART_FRAMETAIL 0x55

//ASCII�����س�����
#define USART_CARRIAGERETURN 0x0D
#define USART_LINEFEED 		 0x0A

bool USART_RecFrame(u8 data, u8 *pbuf, u16 *iolen, const USART_OBJ_DEF *pobj);
bool USART_SendFrame(u8 *data, u8 *pbuf, u16 *iolen, const USART_OBJ_DEF *pobj);



#endif

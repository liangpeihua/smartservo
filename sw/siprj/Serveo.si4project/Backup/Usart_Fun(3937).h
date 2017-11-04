		   /******************** (C) COPYRIGHT 2013 InMotion ********************
* File Name          : USART_FUN.c
* Author             : 刘泽勇
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
组包结构:	AA AA + 数据体 + 校验和 + 55 55
包头:		AA AA
包尾:		55 55
校验和:	数据体的累加和
转义处理: 	AA->A5 AA
		55->A5 55
		A5->A5 A5
*/

typedef struct
{
	bool Rec_Flag;		//接收标志
	bool Ctrl_Flag;		//转义符标志
	u16 Offset;		    //当前buf位置
	u8 Last_Byte;		//上次数据
	u8 Check_Sum;		//校验和
}USART_FRAME_DEF;

//串口包结构
#define USART_FRAMECTRL 0xA5                                                  
#define USART_FRAMEHEAD 0xAA
#define USART_FRAMETAIL 0x55

//ASCII分析回车换行
#define USART_CARRIAGERETURN 0x0D
#define USART_LINEFEED 		 0x0A

bool USART_RecFrame(u8 data, u8 *pbuf, u16 *iolen, const USART_OBJ_DEF *pobj);
bool USART_SendFrame(u8 *data, u8 *pbuf, u16 *iolen, const USART_OBJ_DEF *pobj);



#endif

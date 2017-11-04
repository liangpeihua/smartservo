
#ifndef  __USART_FUN_C__
#define  __USART_FUN_C__

#include "M051Series.h"
#include "sysinit.h"

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
	boolean Rec_Flag;		//接收标志
	boolean Ctrl_Flag;		//转义符标志
	uint16_t Offset;		    //当前buf位置
	uint8_t Last_Byte;		//上次数据
	uint8_t Check_Sum;		//校验和
}USART_FRAME_DEF;

//串口包结构
#define USART_FRAMECTRL 0xA5                                                  
#define USART_FRAMEHEAD 0xAA
#define USART_FRAMETAIL 0x55

//ASCII分析回车换行
#define USART_CARRIAGERETURN 0x0D
#define USART_LINEFEED 		 0x0A

boolean USART_RecFrame(uint8_t data, uint8_t *pbuf, uint16_t *iolen);
boolean USART_SendFrame(uint8_t *data, uint8_t *pbuf, uint16_t *iolen);

void USART_RecvPackage(uint8_t data);
void COMSendBuffer(uint32_t id, void *buf, uint32_t bufsize);


#endif

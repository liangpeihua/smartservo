
#include "usart_Fun.h"
#include "sysinit.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define  SUPPORT_MY_DEBUG		1

#define RX_MAX_SIZE		50
#define TX_MAX_SIZE		50

static USART_FRAME_DEF UsartFrameParam;


/*******************************************************************************
* Function Name  : USART_RecFrame
* Description    : 解包函数	 格式(AA 55 A5)
* Input          : data
* Output         : 解析出的数据存入buff
* Return         : None
*******************************************************************************/
boolean USART_RecFrame(uint8_t data, uint8_t *pbuf, uint16_t *iolen)
{
	static uint8_t bugflag = 0;	
	USART_FRAME_DEF *pframe = &UsartFrameParam;

	if( (*iolen > RX_MAX_SIZE) ||   		 //当接收的数据长度超过接收SIZE
		((data == USART_FRAMEHEAD) && (pframe->Last_Byte == USART_FRAMEHEAD))) {
		//复位
		pframe->Ctrl_Flag = FALSE;
		pframe->Offset = 0;
		pframe->Rec_Flag = TRUE;
		pframe->Check_Sum = 0;
		*iolen = 0;
		return false;
	}
	if( (data == USART_FRAMETAIL) && (pframe->Last_Byte == USART_FRAMETAIL) && (pframe->Rec_Flag) ) {
		//收到结束�?
        if(pframe->Offset >= 2)
            pframe->Offset -= 2;
		pframe->Check_Sum -= (USART_FRAMETAIL + *(pbuf + pframe->Offset));
		if(pframe->Check_Sum == *(pbuf + pframe->Offset)) {                          		   		
			*iolen = pframe->Offset;
			pframe->Rec_Flag = FALSE;

			return true;
		} else {
			pframe->Offset = 0;
			pframe->Check_Sum = 0;
			*iolen = 0;
			pframe->Rec_Flag = FALSE;

			return false;
		}
	}

	if(pframe->Rec_Flag) {
		if(pframe->Ctrl_Flag) {
			if((data == USART_FRAMEHEAD) || (data == USART_FRAMETAIL) || (data == USART_FRAMECTRL)) {
				*(pbuf + pframe->Offset) = data;
				pframe->Offset++;
				pframe->Ctrl_Flag = FALSE;
				pframe->Check_Sum += data;
				if (data == USART_FRAMETAIL) {
					bugflag = 1;
				}
			} else {
				pframe->Ctrl_Flag = FALSE;
				pframe->Offset = 0;
				pframe->Check_Sum = 0;
				pframe->Rec_Flag = FALSE;
			}
		} else {
			if(data == USART_FRAMECTRL) {
				pframe->Ctrl_Flag = TRUE;
			} else {
				*(pbuf + pframe->Offset) = data;
				pframe->Offset++;
				pframe->Check_Sum += data;
			}
		}
		*iolen = pframe->Offset;
	}

	if (!bugflag) {
		pframe->Last_Byte = data;
	} else {
		bugflag = 0;
	}

	return false;
}

/*******************************************************************************
* Function Name  : USART_SendFrame
* Description    : 打包函数
* Input          : data所指向的数�?
* Output         : 打包好的数据存入pBuf，数据长度为n
* Return         : None
*******************************************************************************/
boolean USART_SendFrame(uint8_t *data, uint8_t *pbuf, uint16_t *iolen)
{
	uint16_t i;
	uint8_t check_sum = 0;           		//从这里看来，txsize应该是打包后的最大的数据量，所以，这个可以设置为TX_MEM_SIZE
	uint8_t *pdata_buf = pbuf;
	uint8_t ident_len = 0;			//标示符长�?

	if(*iolen > TX_MAX_SIZE - ident_len)	//当发送的数据长度超过发送SIZE - 包头字节�?- 包尾字节�?- 校验位字节数
		return false;
	
	*pdata_buf++ = USART_FRAMEHEAD;		//加头
	*pdata_buf++ = USART_FRAMEHEAD;

	for (i = 0; i < (*iolen); i++) {
		if( (*data == USART_FRAMECTRL) || (*data == USART_FRAMEHEAD)|| (*data == USART_FRAMETAIL) ) {
			ident_len++;		//溢出判断
			if(*iolen > TX_MAX_SIZE - ident_len)			  
				return false;
			*pdata_buf++ = USART_FRAMECTRL;

		}
		*pdata_buf++ = *data;
		check_sum += *data;
		data ++;
	}

	//校验�?
	if ( (check_sum == USART_FRAMECTRL) || (check_sum == USART_FRAMEHEAD) || (check_sum == USART_FRAMETAIL) ) {
		ident_len++;			//溢出判断
		if(*iolen > TX_MAX_SIZE - ident_len)
			return false;
		*pdata_buf++ = USART_FRAMECTRL;
		
	}
	*pdata_buf++ = check_sum;

	*pdata_buf++ = USART_FRAMETAIL; 	//加尾
	*pdata_buf++ = USART_FRAMETAIL;

	*iolen = pdata_buf - pbuf;

	return true;
}


//api test
#if 1
#include "servo_detect.h"
#include "servo_driver.h"
#include "servo_control.h"
#include "math.h"

typedef struct  
{
	uint32_t id;                 /* 29 bit identifier                               */
	uint8_t  data[8];            /* Data field                                      */
	uint8_t  len;                /* Length of data field in bytes                   */
	uint8_t  ch;                 /* Object channel                                  */
	uint8_t  format;             /* 0 - STANDARD, 1- EXTENDED IDENTIFIER            */
	uint8_t  type;               /* 0 - DATA FRAME, 1 - REMOTE FRAME                */
}  CAN_msg;

uint8_t  g_ucCOMRxBuf[RX_MAX_SIZE];
uint8_t  g_ucCOMTxBuf[TX_MAX_SIZE];
uint16_t g_ucRXLen = 0;
uint16_t g_ucTXLen = 0;

void COMSendBuffer(uint32_t id, void *buf, uint32_t bufsize)
{
	uint32_t i;
	CAN_msg msg;

	msg.id = id;
	msg.format = 1;
	msg.type = 0;
	msg.len = bufsize;	
	memcpy(msg.data, buf, bufsize);

	g_ucTXLen = sizeof(CAN_msg);
	USART_SendFrame((uint8_t *)&msg, &g_ucCOMTxBuf[0], &g_ucTXLen);

	for(i=0; i<g_ucTXLen; i++)
	{
		UART_WRITE(UART0, g_ucCOMTxBuf[i]);
		UART_WAIT_TX_EMPTY(UART0);
	}
}

void USART_RecvPackage(uint8_t data)
{
#if SUPPORT_MY_DEBUG
	extern STRUCT_PID speed_ctrl;
	extern STRUCT_PID torque_ctrl;
	
	CAN_msg *prxmsg;
	if(USART_RecFrame(data, &g_ucCOMRxBuf[0], &g_ucRXLen) == true)
	{ 
		prxmsg = (CAN_msg *)g_ucCOMRxBuf;

		switch(prxmsg->id)
		{
			case 0x00660011:
					speed_ctrl.Kp = prxmsg->data[0];
					speed_ctrl.Ki = prxmsg->data[1];
				break;
			
			case 0x00660012:
				torque_ctrl.Kp = prxmsg->data[0];
				torque_ctrl.Ki = prxmsg->data[1];
				break;

			case 0x0066000F:
				servodriver_run_debug(prxmsg->data[0], *(int16_t*)&prxmsg->data[2], *(int16_t*)&prxmsg->data[4], *(int16_t*)&prxmsg->data[6]);
				break;

			default:

				break;
		}

	}
	else
	{

	}
#endif
}

void USART_SendPackage(void)
{
#if SUPPORT_MY_DEBUG
	extern STRUCT_PID speed_ctrl;
	extern STRUCT_PID pos_ctrl;
	extern int32_t s_output_pwm;
	extern int32_t target_speed;
	extern float NTC;
	extern uint32_t s_limit_current;
	extern int32_t set_current;
	extern int16_t s_driver_pwm;
	uint8_t buff[8] = {0};
	static int32_t count = -1000;

	count++;
	if(count < 30)
	{
		return;
	}
	count = 0;

	//speed
	*(int16_t*)&buff[0] = g_servo_info.cur_speed;
	*(int16_t*)&buff[2] = g_servo_info.cur_pos/10;
	*(int16_t*)&buff[4] = g_servo_info.posmode_tarspeed;
	*(int16_t*)&buff[6] = g_servo_info.tar_pos/10;
	COMSendBuffer(0x00660001, buff, 8);

	//detect
	*(int16_t*)&buff[0] = g_servo_info.voltage;
	*(int16_t*)&buff[2] = g_servo_info.current;
	*(int16_t*)&buff[4] = g_servo_info.temperature;
	*(int16_t*)&buff[6] = g_servo_info.over_current;
	COMSendBuffer(0x00660002, buff, 8);

	//torque
	*(int16_t*)&buff[0] = g_servo_info.current;
	*(int16_t*)&buff[2] = g_servo_info.tar_torque;
	*(int16_t*)&buff[4] = 0;
	*(int16_t*)&buff[6] = 0;
	COMSendBuffer(0x00660003, buff, 8);

	//pwm
	*(int16_t*)&buff[0] = pos_ctrl.output;
	*(int16_t*)&buff[2] = speed_ctrl.output;
	*(int16_t*)&buff[4] = torque_ctrl.output;
	*(int16_t*)&buff[6] = g_servo_info.limit_pwm;
	COMSendBuffer(0x00660004, buff, 8);

#endif
}



#endif


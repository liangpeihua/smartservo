						 /******************** (C) COPYRIGHT 2013 InMotion ********************
* File Name          : Function.c
* Author             : 刘泽勇
* Version            : V1.0.1
* Date               : 2013.12.22
* Description        : Function file
**********************************************************************/
#include  "drv_usart.h"
#include  "Usart_Fun.h"

/*******************************************************************************
* Function Name  : USART_RecFrame
* Description    : 解包函数	 格式(AA 55 A5)
* Input          : data
* Output         : 解析出的数据存入buf所指空间
* Return         : None
*******************************************************************************/
bool USART_RecFrame(u8 data, u8 *pbuf, u16 *iolen, const USART_OBJ_DEF *pobj)
{
	static u8 bugflag = 0;	
	USART_FRAME_DEF *pframe = (USART_FRAME_DEF *)(pobj->func_param->prxpackageparam);

	if( (*iolen > pobj->rxmaxsize) ||   		 //当接收的数据长度超过接收SIZE
		((data == USART_FRAMEHEAD) && (pframe->Last_Byte == USART_FRAMEHEAD))) {
		//复位
		pframe->Ctrl_Flag = FALSE;
		pframe->Offset = 0;
		pframe->Rec_Flag = TRUE;
		pframe->Check_Sum = 0;
		*iolen = 0;
		return FALSE;
	}
	if( (data == USART_FRAMETAIL) && (pframe->Last_Byte == USART_FRAMETAIL) && (pframe->Rec_Flag) ) {
		//收到结束符
        if(pframe->Offset >= 2)
            pframe->Offset -= 2;
		pframe->Check_Sum -= (USART_FRAMETAIL + *(pbuf + pframe->Offset));
		if(pframe->Check_Sum == *(pbuf + pframe->Offset)) {                          		   		
			*iolen = pframe->Offset;
			pframe->Rec_Flag = FALSE;
	
			return TRUE;
		} else {
			pframe->Offset = 0;
			pframe->Check_Sum = 0;
			*iolen = 0;
			pframe->Rec_Flag = FALSE;
		
			return FALSE;
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

	return FALSE;
}

/*******************************************************************************
* Function Name  : USART_SendFrame
* Description    : 打包函数
* Input          : data所指向的数据
* Output         : 打包好的数据存入pBuf，数据长度为n
* Return         : None
*******************************************************************************/
bool USART_SendFrame(u8 *data, u8 *pbuf, u16 *iolen, const USART_OBJ_DEF *pobj)
{
	u16 i;
	u8 check_sum = 0;           		//从这里看来，txsize应该是打包后的最大的数据量，所以，这个可以设置为TX_MEM_SIZE
	u8 *pdata_buf = pbuf;
	u8 ident_len = 0;			//标示符长度

	if(*iolen > pobj->txmaxsize - ident_len)	//当发送的数据长度超过发送SIZE - 包头字节数 - 包尾字节数 - 校验位字节数
		return FALSE;
	
	*pdata_buf++ = USART_FRAMEHEAD;		//加头
	*pdata_buf++ = USART_FRAMEHEAD;

	for (i = 0; i < (*iolen); i++) {
		if( (*data == USART_FRAMECTRL) || (*data == USART_FRAMEHEAD)|| (*data == USART_FRAMETAIL) ) {
			ident_len++;		//溢出判断
			if(*iolen > pobj->txmaxsize - ident_len)			  
				return FALSE;
			*pdata_buf++ = USART_FRAMECTRL;

		}
		*pdata_buf++ = *data;
		check_sum += *data;
		data ++;
	}

	//校验和
	if ( (check_sum == USART_FRAMECTRL) || (check_sum == USART_FRAMEHEAD) || (check_sum == USART_FRAMETAIL) ) {
		ident_len++;			//溢出判断
		if(*iolen > pobj->txmaxsize - ident_len)
			return FALSE;
		*pdata_buf++ = USART_FRAMECTRL;
		
	}
	*pdata_buf++ = check_sum;

	*pdata_buf++ = USART_FRAMETAIL; 	//加尾
	*pdata_buf++ = USART_FRAMETAIL;

	*iolen = pdata_buf - pbuf;

	return TRUE;
}




/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/05/19 11:45a $
 * @brief    for neuron product
 *
 * @note
 * Copyright (C) 2015 myan@makeblock.cc. All rights reserved.
 *
 ******************************************************************************/
#include "sysinit.h"
#include <stdio.h>
#include <stdlib.h>
#include "uart_printf.h"
#include "mygpio.h"
#include "protocol.h"
#include "systime.h"
#include "MePwm.h"
#include "Interrupt.h"
#include "dataflash.h"
#include "servo_control.h"
#include "servo_driver.h"
#include "servo_detect.h"
#include "rgb.h"
#include "mp9960.h"
#include "main.h"

#define Voltage_14V
//#define Voltage_10V
//#define HIGH_VOLT_COMPATIBLE  //high voltage compatible

#define FIRMWARE_SIZE_STORE_ADDRESS      0xF800          //LDROM´æ´¢¹Ì¼þ´óÐ¡
#define FIRMWARE_CRC32_STORE_ADDRESS    FIRMWARE_SIZE_STORE_ADDRESS+0x200  //ÏÂÒ»Ò³´æ´¢crc32Öµ
#define FIRMWARE_SIZE_STORE_DUP_ADDRESS      FIRMWARE_CRC32_STORE_ADDRESS+0x200         //ÏÂÒ»Ò³´æ´¢¹Ì¼þ´óÐ¡µÄ±¸·Ý
#define FIRMWARE_CRC32_STORE_DUP_ADDRESS    FIRMWARE_SIZE_STORE_DUP_ADDRESS+0x200  //ÏÂÒ»Ò³crc32ÖµµÄ±¸·Ý

#define MOTOR_CTRL_FREQ	     10
#define SERVO_DETECT_FREQ    1

/* timer variables */
uint32_t report_time;
volatile uint32_t system_time = 0;
extern volatile boolean timer2ms_flag;
extern volatile boolean timer5ms_flag;

char type0 = '0';   //for smart servo type
char type1 = '0';



//
//  configuration parameter
//
uint16_t limit_pwm;


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
//uint32_t au32Config0[1] = {0x780FFB7F};
/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
int32_t set_current_enhance_count(void);
typedef void (FUNC_PTR)(void);
void entry_update(void *arg);
/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
  volatile uint32_t u32InitCount = 0;
  float randTime = 0;

  /* Unlock protected registers */
  SYS_UnlockReg();

  /* Init System, peripheral clock and multi-function I/O */
  SYS_Init();

  /* Lock protected registers */
  SYS_LockReg();

  /* Init UART0 for printf */
  UART0_Init(115200);

  /* Init UART1*/
  UART1_Init(115200);

  /* Set UART Configuration */
  UART_SetLine_Config(UART0, 115200, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

  /* Set UART Configuration */
  UART_SetLine_Config(UART1, 115200, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

  /* UART sample function */
  UART_Function_Init();

  TMR0_Init(10000);

  /* Enable I2C0 module clock */
  CLK_EnableModuleClock(I2C0_MODULE);

  /* Open SPI */
  mp9960_init();
	
  //printf("chip is running applicatio!\r\n");

  set_data_flash_fix_4k();
  command_mode = flash_read_data_mode();
  command_mode = command_mode & 0x01;
  mVersion[4] = command_mode+1+'0';
  
 	servodet_init();
 	rgb_init();
 	
  pinMode(SMART_SERVO_HW_Verison_D0,GPIO_PMD_INPUT);
  pinMode(SMART_SERVO_HW_Verison_D1,GPIO_PMD_INPUT);
 	
  device_id = 0;
  device_id_2 = 0;
  device_type = SMART_SERVO;
  delay(10);
  uart_printf(UART0,"V%s\r\n",mVersion);

  while(1)
  {
    parse_uart0_recv_buffer();
    flush_uart1_forward_buffer();
    flush_uart0_local_buffer();
	    
    if(timer2ms_flag)
    {
    	timer2ms_flag = false;
			servodet_process();
    }
    
    if(timer5ms_flag)
    {
    	timer5ms_flag = false;
			motor_process();
			rgb_process();
    }
    
		randTime = rand();
		randTime = round((10*randTime)/RAND_MAX);
    if (millis() - report_time > (3 * MOTOR_CTRL_FREQ + randTime))
    {
      report_time = millis();
      send_sensor_report();
    }
  }
}


/**
***********************************************************************************************
* @brief       entry bootloader for updating firmware
* @details     mcu will run bootloader program when the function is invoked
* @param[in]   null
* @param[out]  null
*
* @return     null
***********************************************************************************************
*/
void entry_update(void *arg)
{
  NVIC->ICER[0] = 0xFFFFFFFF;
  SYS_UnlockReg();
  FMC_Open();
  CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
  FMC_EnableLDUpdate();
  FMC_Erase(FIRMWARE_CRC32_STORE_ADDRESS);
  FMC_Erase(FIRMWARE_CRC32_STORE_DUP_ADDRESS);
  FMC_DisableLDUpdate();
  SYS_ResetChip();
}

void servodriver_get_type(void)
{
  int8_t type0_index = -1;
  int8_t type1_index = -1;
  float smart_servo_type0_vol = 0.0;
  float smart_servo_type1_vol = 0.0;

  smart_servo_type0_vol = (float)analogRead(SMART_SERVO_TYPE0_AD)/4096*3300;//for servo type low bit
  type0_index = constrain(round((float)(smart_servo_type0_vol/206)), 0 ,15);
  smart_servo_type1_vol = (float)analogRead(SMART_SERVO_TYPE1_AD)/4096*3300;//for servo type low bit
  type1_index = constrain(round((float)(smart_servo_type1_vol/206)), 0 ,15);

  if(type0_index <= 9 && type0_index >= 0)
  {
    type0 = type0_index + '0';
  }
  else if(type0_index <= 15 && type0_index > 9)
  {
    type0 = (type0_index - 10) + 'A'; //max is 'F' - 16th type
  }

  if(type1_index <= 9 && type1_index >= 0)
  {
    type1 = type1_index + '0';
  }
  else if(type1_index <= 15 && type1_index > 9)
  {
    type1 = (type1_index - 10) + 'A'; //max is 'F' - 16th type
  }
}

char servodriver_get_Hardware_Version(void)
{
  char _Hardware_Version = '0';
  
  if(digitalRead(SMART_SERVO_HW_Verison_D1))
  {
    if(digitalRead(SMART_SERVO_HW_Verison_D0))  // 11
    {
      _Hardware_Version = '4';
    }
    else                              // 10
    {
      _Hardware_Version = '3';
    }
  }
  else
  {
    if(digitalRead(SMART_SERVO_HW_Verison_D0))  // 01
    {
      _Hardware_Version = '2';
    }
    else                              // 00
    {
      _Hardware_Version = '0';
    }
  }

  return _Hardware_Version;
}


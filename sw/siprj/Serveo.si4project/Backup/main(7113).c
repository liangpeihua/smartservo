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
#include "smartservo.h"
#include "mp9960.h"
#include "main.h"

#define Voltage_14V
//#define Voltage_10V
//#define HIGH_VOLT_COMPATIBLE  //high voltage compatible

#define RESISTANCE_MAX_DEVIATION    30
#define RESISTANCE_VALUE_NOT_EXIST  0xff

#define FIRMWARE_SIZE_STORE_ADDRESS      0xF800          //LDROM´æ´¢¹Ì¼þ´óÐ¡
#define FIRMWARE_CRC32_STORE_ADDRESS    FIRMWARE_SIZE_STORE_ADDRESS+0x200  //ÏÂÒ»Ò³´æ´¢crc32Öµ
#define FIRMWARE_SIZE_STORE_DUP_ADDRESS      FIRMWARE_CRC32_STORE_ADDRESS+0x200         //ÏÂÒ»Ò³´æ´¢¹Ì¼þ´óÐ¡µÄ±¸·Ý
#define FIRMWARE_CRC32_STORE_DUP_ADDRESS    FIRMWARE_SIZE_STORE_DUP_ADDRESS+0x200  //ÏÂÒ»Ò³crc32ÖµµÄ±¸·Ý

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned long report_time;
volatile unsigned long system_time = 0;

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
void get_sensor_data(void);
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

  TMR0_Init(100000);

  /* Enable I2C0 module clock */
  CLK_EnableModuleClock(I2C0_MODULE);

  /* Open SPI */
  mp9960_init();

  //printf("chip is running applicatio!\r\n");

  set_data_flash_fix_4k();
  command_mode = flash_read_data_mode();
  command_mode = command_mode & 0x01;
  mVersion[4] = command_mode+1+'0';

  angle_pos_offset = (int16_t)flash_read_angle_offset();
  smart_servo_init();
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
    get_sensor_data();
    currentMillis = millis();
    if (currentMillis - previousMillis > (SAMPLING_INTERVAL_TIME - 1))
    {
      period_time = currentMillis - previousMillis;
      previousMillis = currentMillis;
      if(shake_hand_flag == true)
      {
        if(blink_count < 10)
        {
          smart_led_blink(500,255,255,255);
        }
        else
        {
          shake_hand_flag = false;
        }
      }
      else
      {
        motor_protection();
      }
      device_neep_loop_in_sampling();
	  //uart_printf(UART0,"cur_pos:%d\r\n",smart_servo_angle_val);
    }
		randTime = rand();
		randTime = round((10*randTime)/RAND_MAX);

    if (millis() - report_time > (3 * SAMPLING_INTERVAL_TIME + randTime))
    {
      report_time = millis();
      send_sensor_report();
    }
  }
}

/**
***********************************************************************************************
* @brief       for getting data from sensors
* @details     getting the data from sensor such as angle, pos, temperature, voltage and current
* @param[in]   null
* @param[out]  null
*
* @return     null
***********************************************************************************************
*/
void get_sensor_data(void)
{
  if(device_type == SMART_SERVO)
  {
    smart_servo_angle_val = (long)round((smart_servo_cur_pos * 360.0)/RAW_ANGLE_MAX_FLOAT);
    smart_servo_pos_val = (abs_user(smart_servo_cur_pos) % RAW_ANGLE_MAX_INT);
    smart_servo_temp_val = adc_get_temperature_value();
    smart_servo_voltage_val = adc_get_voltage_value();
    smart_servo_current_val = adc_get_current_value();
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

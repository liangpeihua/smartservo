#include "servo_detect.h"
#include "servo_driver.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"
#include "usart_fun.h"



//GPIO define
#define SMART_SERVO_TEMP_AD   	P1_3
#define SMART_SERVO_VOL_AD    	P1_5
#define SMART_SERVO_CURR_AD   	P1_4


//故障ID
#define NONE_ERROR                   0
#define OVER_VLOT_ERR          			(1<<1) //过压
#define LOW_VLOT_ERR           			(1<<2) //欠压
#define HARDWARE_ERR                (1<<3) //硬件故障， 管子损坏，电流运放损坏等 
#define OVER_TEMPERATURE_ERR				(1<<4) //过温
#define OVER_CURRENT_ERR						(1<<5) //母线电流过流
#define MOTOR_STALL_ERR		        	(1<<6)//电机堵转
#define OVER_LOADMAN_ERR            (1<<7)//过载报警

//监测阈值
#define LOW_VLOT_THSD		        				5000			  //1mv,电池电压欠压保护阈值	
#define OVER_VLOT_THSD             			10000			  //1mv,电池电压过压保护阈值
#define VLOT_HYSTERESIS_THSD            1000        //1V,滞回电压

#define OVER_CURRENT_THSD			    			8000					//1mA,总电流过流保护阈值,必须比滞回电流大
#define CURR_HYSTERESIS_THSD            100         //1A,滞回电流

#define OVER_TEMPERATURE_THSD		    		70		    	//°C,mos管过温保护阈值
#define TEMPERATURE_HYSTERESIS_THSD    	10          //10°C，滞回温度


uint32_t g_error_id = 0;
SERVO_DETECT g_servo_info = {FALSE,0};

void servodet_init(void)
{
	servodriver_init();
	delay(20);
	g_servo_info.current_zero_offset = analogRead(SMART_SERVO_CURR_AD);
	g_servo_info.angle_zero_offset = (int16_t)flash_read_angle_offset();
}

SpiFlashOpResult servodet_set_angle_zero(void)
{
  int value = -1;
  SpiFlashOpResult result = SPI_FLASH_RESULT_OK;
  value =  mp9960_read_raw_angle();
  if(flash_write_angle_offset((uint32_t)value) == SPI_FLASH_RESULT_OK)
  {
    g_servo_info.angle_zero_offset = value;
    result = SPI_FLASH_RESULT_OK;
  }
  else
  {
    result = SPI_FLASH_RESULT_ERR;
  }
  g_servo_info.tar_pos = g_servo_info.cur_pos = 0;
  g_servo_info.circular_count = 0;

  return result;
}


static void servodet_pos_handle(void)
{
	static boolean first_run = true;
	static int32_t pre_pos = 0;
  int32_t cur_pos;
	
	if(first_run)
	{
		first_run = false;
		pre_pos = cur_pos = mp9960_read_raw_angle();;
	}
	else
	{
		cur_pos =  mp9960_read_raw_angle();
	}

  if(abs_user(cur_pos - pre_pos) > RAW_ANGLE_MAX_INT/2)
  {
    if((pre_pos > RAW_ANGLE_MAX_INT/2) && (cur_pos < RAW_ANGLE_MAX_INT/2))
    {
      g_servo_info.circular_count++;
    }
    else if((pre_pos < RAW_ANGLE_MAX_INT/2) && (cur_pos > RAW_ANGLE_MAX_INT/2))
    {
      g_servo_info.circular_count--;
    }
  }
  pre_pos = cur_pos;
  cur_pos = cur_pos - g_servo_info.angle_zero_offset;
  cur_pos = g_servo_info.circular_count * RAW_ANGLE_MAX_INT + cur_pos;

  g_servo_info.cur_pos = cur_pos * 3600 / 4096;//0.1°
}

static void servodet_speed_handle(void)
{
//最小转速(1rmp/min)对应的最小分辨时间:
//1(rpm/min) = (360/RAW_ANGLE_MAX_INT) / Tx
//Tx = (360°/4096) / (360°/60000ms) = (360 * 60000) / (4096*360) = 14.6484375ms
	static uint32_t pre_time = 0;
	static int32_t pre_pos = 0;
	uint32_t time_diff,cur_time;
  int32_t pos_diff;
	int32_t cur_pos = g_servo_info.cur_pos;

	static uint8_t index = 0; 
	static int32_t tempvalue[8] = {0};
	static int32_t sum = 0;

	cur_time = us100();//millis();
	time_diff = cur_time - pre_time;
	pre_time = cur_time;
	pos_diff = cur_pos - pre_pos;
	pre_pos = cur_pos;
	tempvalue[index] = (int32_t)((int32_t)(pos_diff * 60 * 10000) / (int32_t)(time_diff * 10 * 360));//rpm / min
	sum += tempvalue[index];
	g_servo_info.cur_speed = (sum / 8); 
	index++;
	index %= 8;
	sum -= tempvalue[index];

}

static void servodet_temperature_handle(void)
{
	static uint16_t error_count = 0;
	static uint16_t value = 0;
	int cur_value = analogRead(SMART_SERVO_TEMP_AD);

	cur_value /= 512;//V_div = 3.3 * 2.2K/(R_ntc + 2.2K) NTC型号 ：MF52	固定硬件参数

	value = (value*99 + cur_value) / 100;
	
	if(value<2 || value>OVER_TEMPERATURE_THSD)
	{
		error_count++;
		if(error_count>16)
		{
	    error_count = 16;
	    g_error_id |= OVER_TEMPERATURE_ERR;
		}      
	}
	else
	{
		error_count = 0;
		if(value < (OVER_TEMPERATURE_THSD-TEMPERATURE_HYSTERESIS_THSD))
		{
		  g_error_id &= ~(OVER_TEMPERATURE_ERR);
		}
	}

	g_servo_info.temperature = value;
}

static void servodet_current_handle(void)
{
	static uint16_t error_count = 0;
	static uint8_t index = 0; 
	static int32_t tempvalue[8] = {0};
	static int32_t sum = 0;
	int32_t value;

	tempvalue[index] =  analogRead(SMART_SERVO_CURR_AD);

	sum += tempvalue[index];
	value = (sum / 8); 
	index++;
	index %= 8;
	sum -= tempvalue[index];

	//0.005omu 、Aop = 22*11/23、 I = ADC_value*3.3/(65536*R*Aop) *100 (单位：10mA) ， 3.3/(65536*R*Aop) *100 = 1/10.4477 = 49/512
	value = (int32_t)(value - g_servo_info.current_zero_offset)*49/512;
		
	if(abs_user(value)>OVER_CURRENT_THSD)
	{
		error_count++;
		if(error_count>16)
		{
	    error_count = 16;
	    g_error_id |= OVER_CURRENT_ERR;
		}      
	}
	else
	{
		error_count = 0;
		if(value < (OVER_CURRENT_THSD-CURR_HYSTERESIS_THSD))
		{
		  g_error_id &= ~(OVER_CURRENT_ERR);
		}
	}

	g_servo_info.current = value;	 
}

static void servodet_voltage_handle(void)
{
	static uint16_t error_count = 0;
	static uint32_t value = 0;
	int cur_value;

  cur_value = analogRead(SMART_SERVO_VOL_AD);

	cur_value = value * 349 / 100;   //voltage_temp = value*(3.3/4096)*(130/30)
	if(g_servo_info.current > 0){
		cur_value = cur_value + (uint32_t)(g_servo_info.current* 200/1000);//200mΩ
	}
	else{
		cur_value = cur_value;
	}
	value = (value*63 + cur_value) / 64;
	
	if(value > OVER_VLOT_THSD)
	{
		error_count++;
		if(error_count>16)
		{
	    error_count = 16;
	    g_error_id |= OVER_VLOT_ERR;
		}      
	}
	else if(value < LOW_VLOT_THSD)
	{
		error_count++;
		if(error_count>16)
		{
	    error_count = 16;
	    g_error_id |= LOW_VLOT_ERR;
		}      
	}
	else
	{
		error_count = 0;
		if(value < (OVER_VLOT_THSD-VLOT_HYSTERESIS_THSD))
		{
		  g_error_id &= ~(OVER_VLOT_ERR);
		}

		if(value > (LOW_VLOT_THSD+VLOT_HYSTERESIS_THSD))
		{
		  g_error_id &= ~(LOW_VLOT_ERR);
		}
	}

	g_servo_info.voltage = value;	 	 
}

/* nfault pin of motor driver chip mp6515 */


static void servodet_nfault_handle(void)
{
	static uint16_t error_count = 0;
	
	if(servodriver_get_nfault_value() == 0)
	{
		error_count++;
		if(error_count > 1)
		{
			 g_error_id |= HARDWARE_ERR;
			 error_count = 1000;//10s
		}
	}
	else
	{
		if(error_count > 0)
		{
			error_count--;
		}
		else
		{
			g_error_id &= ~(HARDWARE_ERR);
		}
	}

}


/***********************************************************
* Function Name : SystemDetectProcess
* Description   : 系统监测,2ms执行一次
* Input         : NONE
* Output        : NONE
* Return        : NONE
************************************************************/
void servodet_process(void)
{
	static uint8_t timecount = 0;
	static int8_t state = -1;
	
	if(g_servo_info.ready == false)
	{
		if(++timecount > 100)
		{
			g_servo_info.ready = true;
		}
	}

	state++;
	switch(state)
	{
		case 1:
			servodet_temperature_handle();
			break;
		case 2:
			servodet_current_handle();
			break;
		case 3:
			servodet_voltage_handle();
			break;
		case 4:
			servodet_nfault_handle();
			break; 
		case 5:
//			servodet_pos_handle();
//			servodet_speed_handle();
//			USART_SendPackage();
			state = 0;
			break;
		default:
			//state = 0;
			break;    
	}

	servodet_pos_handle();
	servodet_speed_handle();
	USART_SendPackage();
	
	
 {

 static uint8_t flag = 0;

 if(flag ==0)
 {
		flag = 1;
	 digitalWrite(P3_6,0);
 }
 else
 {
	 digitalWrite(P3_6,1);
		flag = 0;
 }
 }
}


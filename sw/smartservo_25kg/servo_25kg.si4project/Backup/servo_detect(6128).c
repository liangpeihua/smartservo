#include "servo_detect.h"
#include "servo_driver.h"
#include <string.h>
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
#define OVER_CURRENT_ERR						(1<<5) //电流过流
#define MOTOR_STALL_ERR		        	(1<<6)//电机堵转
#define OVER_LOADMAN_ERR            (1<<7)//过载报警

//监测阈值
#define SHORT_CURRENT_THSD			    			3000			//1mA,驱动芯片短路保护阈值
#define CURR_HYSTERESIS_THSD            	200         //100mA,滞回电流

#define OVER_TEMPERATURE_THSD		    		70		    	//°C,mos管过温保护阈值
#define TEMPERATURE_HYSTERESIS_THSD    	20          //20°C，滞回温度

//static STRUCT_PID current_ctrl = {0};
SERVO_DETECT g_servo_info = {FALSE,0};

void servodet_init(void)
{
	servodriver_init();
	memset((uint8_t*)&g_servo_info, 0, sizeof(g_servo_info));
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

static void servodet_voltage_handle(void)
{
	static uint32_t value = 0;
	int cur_value;

  cur_value = analogRead(SMART_SERVO_VOL_AD);
	
	cur_value = cur_value * 349 / 100 + 150;   //voltage_temp = value*(3.3/4096)*(130/30),error=150mv
	if(g_servo_info.current > 0){
		cur_value = cur_value + (uint32_t)(g_servo_info.current* 500/1000);//500mΩ
	}
	else{
		cur_value = cur_value;
	}
	value = (uint32_t)(value*0.95 + cur_value*0.05);

	g_servo_info.voltage = value;	 	 
}

static void servodet_temperature_handle(void)
{
	static boolean first_run = true;
	static float value = 0;
	float NTC;
	int cur_value = analogRead(SMART_SERVO_TEMP_AD);

	//ADC / 4096 = NTC / (10K + NTC)
	//NTC(KΩ) = (ADC*10K) / (4096-ADC)
	NTC = (cur_value * 10.0) / (4096-cur_value);

	//0℃ ~ 20℃：Y = -1.388*X + 37.781 
	//20℃ ~ 40℃：Y = -3.188*X + 58.511, NTC = 12.081KΩ
	//40℃ ~ 60℃：Y = -7.096*X + 81.38, NTC = 5.834KΩ
	//60℃ ~ 80℃：Y = -14.87*X + 104.82, NTC = 3.014KΩ

	if(NTC > 12.081){
		cur_value = -1.388*NTC + 37.781;
	}
	else if(NTC > 5.834){
		cur_value = -3.188*NTC + 58.511;
	}
	else if(NTC > 3.014){
		cur_value = -7.096*NTC + 81.38;
	}
	else{
		cur_value = -14.87*NTC + 104.82;
	}
	
	if(cur_value < 0)
	{
		cur_value = 0;
	}

	if(first_run)
	{
		value = cur_value;
		g_servo_info.errorid |= OVER_TEMPERATURE_ERR; 
		first_run = false;
	}
	
	value = value*0.9 + cur_value*0.1;
	
	if(value > OVER_TEMPERATURE_THSD)
	{
	  g_servo_info.errorid |= OVER_TEMPERATURE_ERR;    
	}
	else if(value < (OVER_TEMPERATURE_THSD-TEMPERATURE_HYSTERESIS_THSD))
	{
		g_servo_info.errorid &= ~(OVER_TEMPERATURE_ERR);
	}
	
	g_servo_info.temperature = constrain(value, 10, OVER_TEMPERATURE_THSD);
}

static void servodet_overcurrent_handle(void)
{
	static uint16_t over_count = 0;
	static uint8_t index = 0; 
	static int32_t tempvalue[8] = {0};
	static int32_t sum = 0;
	int32_t value;

	tempvalue[index] =	analogRead(SMART_SERVO_CURR_AD);

	sum += tempvalue[index];
	value = (sum / 8); 
	index++;
	index %= 8;
	sum -= tempvalue[index];

	//1V -> 1A, voltage = ADC*(3.3/4096)，amp=75
	value = (int32_t)(value - g_servo_info.current_zero_offset)*3300/4096 / 0.375;
		
	if(abs_user(value) > SHORT_CURRENT_THSD)
	{
		over_count++;
		if(over_count > 50)//0.5s
		{
			over_count = 20;
	  	g_servo_info.errorid |= OVER_CURRENT_ERR;
		} 		 
	}
	else
	{
		over_count = 0;
		if(value < (SHORT_CURRENT_THSD-CURR_HYSTERESIS_THSD))
		{
		 // g_servo_info.errorid &= ~(OVER_CURRENT_ERR);
		}
	}

	g_servo_info.current = value;  
}

static void servodet_limitcurrent_handle(void)
{
#define LIMIT_MAX_VOLTAGE		7800//7400		
	int32_t limit_current;
	int32_t limit_current_7_4v;
	int32_t limit_pwm;

	//电压7.4V测试：堵转电流和温度的关系
	//y = -17.353x + 4161.6
	//max_current(7.4V) = -17.353*temp + 4161.6, 单位mA
	limit_current_7_4v =  (int32_t)(-35*g_servo_info.temperature + 4000);
	
	if(g_servo_info.voltage < LIMIT_MAX_VOLTAGE)
	{
		limit_pwm = MAX_OUTPUT_PWM; 

		limit_current = (int32_t)(g_servo_info.voltage * limit_current_7_4v) / LIMIT_MAX_VOLTAGE;
	}
	else
	{
		//当电压大于8V时，保持输出转速和功率不变，减低PWM占空比，为了保护舵机寿命
		limit_pwm =  LIMIT_MAX_VOLTAGE * MAX_OUTPUT_PWM / g_servo_info.voltage; //7.4v -> max output
	
		//功率守恒
		//Ix = LIMIT_MAX_VOLTAGE * I_74V / Ux
		limit_current = (int32_t)(LIMIT_MAX_VOLTAGE * limit_current_7_4v / g_servo_info.voltage);
	}
	
	limit_pwm = constrain(limit_pwm, 0, MAX_OUTPUT_PWM);
	g_servo_info.limit_pwm = limit_pwm;

	limit_current -= 500;
	limit_current = constrain(limit_current, 500, SHORT_CURRENT_THSD);
	g_servo_info.limit_current = limit_current;
}

static void servodet_nfault_handle(void)
{
	static uint16_t error_count = 0;
	int32_t value;

	value = digitalRead(SMART_SERVO_NFAULT);
	
	if(value == 0)
	{
		error_count++;
		if(error_count > 1)
		{
			 g_servo_info.errorid |= HARDWARE_ERR;
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
			g_servo_info.errorid &= ~(HARDWARE_ERR);
		}
	}
}

static void servodet_stall_handle(void)
{
	static uint16_t stall_count = 0;
	static int32_t pre_pos = 0;
	uint16_t cur_pwmvalue = abs_user( servodriver_getpwmvalue() );

	if( (cur_pwmvalue > (MAX_OUTPUT_PWM/5)) &&
			(((g_eSysMotionStatus==SPEED_MODE) && (g_servo_info.tar_speed!=0 ) && (abs_user(pre_pos-g_servo_info.cur_pos)<30)) ||
			((g_eSysMotionStatus==PWM_MODE) && (g_servo_info.tar_pwm!=0 ) && (abs_user(pre_pos-g_servo_info.cur_pos)<30)) || 
			((g_eSysMotionStatus==POS_MODE) && (g_servo_info.posmode_tarspeed!=0 ) && (abs_user(pre_pos-g_servo_info.cur_pos)<30)) ||
			((g_eSysMotionStatus==TORQUE_MODE) && (g_servo_info.posmode_tarspeed!=0 ) && (abs_user(pre_pos-g_servo_info.cur_pos)<30)) ))
	{
		stall_count++;
		if(stall_count > 150)//1.5s
		{
			 g_servo_info.errorid |= MOTOR_STALL_ERR;
		}
	}
	else
	{
		stall_count = 0;
		pre_pos = g_servo_info.cur_pos;
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
		  servodet_voltage_handle();
			break;
		case 2:
			servodet_overcurrent_handle();
			break;
		case 3:
			servodet_limitcurrent_handle();
			break;
		case 4:
			servodet_stall_handle();
			break; 
		case 5:
			servodet_temperature_handle();
		  //servodet_nfault_handle();
			state = 0;
			break;
		default:
			//state = 0;
			break;    
	}

	servodet_pos_handle();
	servodet_speed_handle();
	USART_SendPackage();
}


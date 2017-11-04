#include "servo_protect.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"


int16_t adc_get_position_value(void)
{
  int value = -1;
  value = RAW_ANGLE_MAX_INT-1 - mp9960_read_raw_angle();;
  value = value - angle_pos_offset;
  if(value < 0)
  {
    value = RAW_ANGLE_MAX_INT + value;
  }
  return value;
}

int16_t adc_get_temperature_value(void)
{
  int value = -1;
  value = analogRead(SMART_SERVO_TEMP_AD);
  return value;
}

int16_t adc_get_voltage_value(void)
{
  int value = -1;
  value = analogRead(SMART_SERVO_VOL_AD);
  return value;
}

int16_t adc_get_current_value(void)
{
  int value = -1;
  value = analogRead(SMART_SERVO_CURR_AD);
  return value;
}

float calculate_temp(int16_t In_temp)
{
  float media;
  float temperatura;
  media = (float)In_temp;
  // Convert the thermal stress value to resistance
  media = 4095.0 / media - 1;
  media = SERIESRESISTOR / media;
  //Calculate temperature using the Beta Factor equation

  temperatura = media / TERMISTORNOMINAL;              // (R/Ro)
  temperatura = log(temperatura); // ln(R/Ro)
  temperatura /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
  temperatura += 1.0 / (TEMPERATURENOMINAL + 273.15);  // + (1/To)
  temperatura = 1.0 / temperatura;                     // Invert the value
  temperatura -= 273.15;                               // Convert it to Celsius
  return temperatura;
}

float calculate_current(int16_t In_cur)
{
  float current_temp;
  current_temp = (In_cur * 3300.0)/4096.0;
  //uart_printf(UART0,"current_temp:%.2f\r\n",current_temp);
  return current_temp;
}

float calculate_voltage(int16_t In_vol)//In_vol: adc voltage
{
  float voltage_temp;
  voltage_temp = In_vol * 0.0035;   //voltage_temp = In_vol*(3.3/4096)*(130/30)
  //uart_printf(UART0,"voltage_temp:%.2f\r\n",voltage_temp);
  return voltage_temp;
}

void servoprotect_temperature_handle(void)
{

	float temp = adc_get_temperature_value();

	temp_debounced_count ++;

	temp = calculate_temp(temp);

	/* temperature protection */
	if(temp > 70)
	{
		temp_match_max_count++;
	}
	if(temp_debounced_count == (3000/SAMPLING_INTERVAL_TIME))
	{
		temp_debounced_count = 0;
		if(temp_match_max_count > (1000/SAMPLING_INTERVAL_TIME))
		{
			smart_servo_break(true);
			protect_flag = true;
		}
		temp_match_max_count = 0;
	}


}

/*

  calculating parameters per 10ms for protection

 */
void motor_protection(void)
{

  float current = adc_get_current_value();
  float voltage = adc_get_voltage_value();
  float locked_electric_current = 0.00;

  cur_debounced_count ++;
 
  vol_debounced_count ++;
  
  current = calculate_current(current);
  voltage = calculate_voltage(voltage);



  /* voltage protection */
  if(voltage >= 13.5)
  {
   vol_match_max_count++;
  }
  if(vol_debounced_count == (4000/SAMPLING_INTERVAL_TIME))
  {
   vol_debounced_count = 0;
   if(vol_match_max_count > (2000/SAMPLING_INTERVAL_TIME))
   {
     smart_servo_break(true);
     protect_flag = true;
   }
   vol_match_max_count  = 0;
  }

  /* protection when locked by external force */
  if(temp < 45)
  {
    locked_electric_current = (float)((0.093*voltage+0.2014)*1000 - 120.0);
  }
  if((temp >= 45) && (temp < 55))
  {
    locked_electric_current = (float)((0.093*voltage+0.2014)*1000 - 120.0 - 100.0);
  }
  if((temp >= 55) && (temp < 60))
  {
    locked_electric_current = (float)((0.093*voltage+0.2014)*1000 - 120.0 - 150.0);
  }
  if((temp >= 60) && (temp < 70))
  {
    locked_electric_current = (float)((0.093*voltage+0.2014)*1000 - 120.0 - 250.0);
  }
  if(current >= locked_electric_current)
  {
    locked_cur_match_max_count++;
  }
  if(cur_debounced_count == (3000/SAMPLING_INTERVAL_TIME))
  {
    cur_debounced_count = 0;
    if(locked_cur_match_max_count > (2000/SAMPLING_INTERVAL_TIME))
    {
      smart_servo_break(true);
      protect_flag = true;
    }
    locked_cur_match_max_count = 0;
  }
  // uart_printf(UART0,"locked_electric_current: %f\r\n",locked_electric_current);

  /* overheat protection */
  if(current >= 100)
  {
    s_counter_for_heat += (int32_t)round(abs_user(current - 100)/80.0);
  }
  else
  {
    s_counter_for_heat -= (int32_t)round(abs_user(current - 200)/300.0);
    if(s_counter_for_heat <= 0)
    {
      s_counter_for_heat = 0;
    }
  }
  if(s_counter_for_heat >= 120000)
  {
    smart_servo_break(true);
    protect_flag = true;
  }
  //uart_printf(UART0, "temp: %.2f, current: %.2f, s_counter_for_heat: %d\r\n", temp, current, s_counter_for_heat);

  /* nfault pin of motor driver chip mp6515 */
  if(servodriver_get_nfault_value() == 0)
  {
    smart_servo_break(true);
    protect_flag = true;
  }

  if(protect_flag == true)
  {
    smart_led_blink(500,255,0,0);
    return;
  }
	// uart_printf(UART0, "temp: %.2f,  cur: %.2f,  vol: %.2f,  pos:%d\r\n", temp, current, voltage);
}



#include "smartservo.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"

#define FILTER_SHIFT 3

PID  PID_pos, PID_speed, PID_speed_lock;
smart_led_type smart_led = {0,0,255};

volatile boolean pos_lock_flag = false;
volatile boolean protect_flag = false;
volatile boolean release_state_flag = false;

volatile long smart_servo_cur_pos = 0;
volatile long smart_servo_target_pos = 0;
volatile long smart_servo_pre_target_pos = 0;
volatile long smart_servo_circular_turn_count = 0;

volatile unsigned long period_time = 1;

volatile float smart_servo_target_speed = SMART_SERVO_PER_SPEED_MAX;
volatile float smart_servo_cur_speed = 0;

volatile int16_t pre_pos = 0;
volatile int16_t smart_servo_output = 0;
volatile int16_t angle_pos_offset = 0;
volatile int16_t motion_mode = SMART_SERVO_COMMON_MODE;
volatile int16_t smart_servo_pwm = 0;

volatile uint16_t blink_count = 0;
volatile long pre_blink_time = 0;

volatile int32_t s_counter_for_heat = 0;

static volatile float current = 0.0;

static volatile boolean blink_flag = false;
static volatile boolean dir_lock_flag = false;
volatile boolean reach_pos_flag = false;


static volatile uint16_t vol_debounced_count = 0;
static volatile uint16_t vol_match_max_count = 0;
static volatile uint16_t vol_match_min_count = 0;

static volatile uint16_t cur_debounced_count = 0;
static volatile uint16_t cur_match_max_count = 0;
static volatile uint16_t cur_match_min_count = 0;

static volatile uint16_t temp_debounced_count = 0;
static volatile uint16_t temp_match_max_count = 0;
static volatile uint16_t temp_match_min_count = 0;

static volatile uint16_t locked_cur_match_max_count = 0;

static volatile long smart_servo_speed_integral = 0;

static int16_t TEMPERATURENOMINAL     = 25;    //Nominl temperature depicted on the datasheet
static int16_t SERIESRESISTOR         = 10000; // Value of the series resistor
static int16_t BCOEFFICIENT           = 3377;  // Beta value for our thermistor(3350-3399)
static int16_t TERMISTORNOMINAL       = 10000; // Nominal temperature value for the thermistor

static float filter_pos = 0.0;
static void smart_servo_ccw(uint8_t speed);

void smart_servo_init(void)
{
  pinMode(SMART_SERVO_LED_R,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_LED_G,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_LED_B,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_HW_Verison_D0,GPIO_PMD_INPUT);
  pinMode(SMART_SERVO_HW_Verison_D1,GPIO_PMD_INPUT);

  pinMode(SMART_SERVO_ENA,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_ENB,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_PWMA,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_PWMB,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_SLEEP,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_NFAULT,GPIO_PMD_INPUT);

  digitalWrite(SMART_SERVO_ENA,1);
  digitalWrite(SMART_SERVO_ENB,1);

  pwm_init(SMART_SERVO_LED_R,1000);
  pwm_init(SMART_SERVO_LED_G,1000);
  pwm_init(SMART_SERVO_LED_B,1000);
  pwm_init(SMART_SERVO_PWMA,20000);  //20KHz  PWM Timer period = 50us
  pwm_init(SMART_SERVO_PWMB,20000);

  smart_servo_led(0,0,255);
  s_counter_for_heat = (int32_t)(round(calculate_temp((int16_t)adc_get_temperature_value()) - 25)*4200);//get motor initial temperature for motor_protection()

  smart_servo_cur_pos = adc_get_position_value();
  if(smart_servo_cur_pos > RAW_ANGLE_MAX_INT/2)
  {
    smart_servo_cur_pos = smart_servo_cur_pos - RAW_ANGLE_MAX_INT;
  }
  pre_pos = smart_servo_cur_pos;
  smart_servo_target_pos = smart_servo_cur_pos;
  // filter_pos = smart_servo_cur_pos;
  smart_servo_target_speed = SMART_SERVO_PER_SPEED_MAX;
  smart_servo_speed_integral = 0;
  smart_servo_circular_turn_count = 0;
  protect_flag = false;

#if defined (USED_MP9960)
  PID_pos.P = 0.3;  //0.3    //2.0
  PID_pos.I = 0.6;  //0.6    //0.75
  PID_pos.D = 1.5;
#elif defined (USED_MA730)
  PID_pos.P = 0.3;  //0.3    //2.0
  PID_pos.I = 0.6;  //0.6    //0.75
  PID_pos.D = 1.5;
#endif
  PID_pos.Setpoint = smart_servo_target_pos;
  PID_pos.Integral = 0;

#if defined (USED_MP9960)
  PID_speed.P = 4.6;        //3.2
#elif defined (USED_MA730)
  PID_speed.P = 3.2;  //4.6;        //3.2
#endif
  PID_speed.I = 0.0;
  PID_speed.D = 0.0;
  PID_speed.Setpoint = smart_servo_target_speed;
  PID_speed.Integral = 0;

  PID_speed_lock.P = 1.8;   //1.8
  PID_speed_lock.I = 0;
  PID_speed_lock.D = 0;
  PID_speed_lock.Setpoint = 0;
  PID_speed_lock.Integral = 0;
  smart_servo_break(true);
}

void smart_servo_led(uint8_t led_r,uint8_t led_g,uint8_t led_b)
{
  pwm_write(SMART_SERVO_LED_R,(255-led_r),0,255);
  pwm_write(SMART_SERVO_LED_G,(255-led_g),0,255);
  pwm_write(SMART_SERVO_LED_B,(255-led_b),0,255);
}

void smart_led_change(uint8_t led_color)
{
  switch(led_color)
  {
    case 0:
      smart_servo_led(255,0,0);
      break;
    case 1:
      smart_servo_led(0,255,0);
      break;
    case 2:
      smart_servo_led(0,0,255);
      break;
    default:
      smart_servo_led(0,0,0);
      break;
  }
}

void smart_led_blink(uint16_t blink_time,uint8_t led_r,uint8_t led_g,uint8_t led_b)
{
  if(millis() - pre_blink_time > blink_time)
  {
    if(blink_flag == false)
    {
      blink_flag = true;
    }
    else
    {
      blink_flag = false;
    }
    blink_count++;
    pre_blink_time = millis();
  }
  if(blink_flag == true)
  {
    smart_servo_led(led_r,led_g,led_b);
  }
  else
  {
    smart_servo_led(0,0,0);
  }
}

void smart_servo_break(boolean status)
{
  if(status == true)
  {
    digitalWrite(SMART_SERVO_SLEEP,0);
    release_state_flag = true;
  }
  else
  {
    digitalWrite(SMART_SERVO_SLEEP,1);
    release_state_flag = false;
    // uart_printf(UART0,"smart_servo_cur_pos: %d  smart_servo_target_pos: %d\r\n",smart_servo_cur_pos,smart_servo_target_pos);
    if(abs_user(smart_servo_cur_pos - smart_servo_target_pos) > 11)
    {
      smart_servo_circular_turn_count = 0;
      smart_servo_cur_pos = adc_get_position_value();
      pre_pos = smart_servo_cur_pos;
      smart_servo_target_pos = smart_servo_cur_pos;
    }
  }
}

/**
***********************************************************************************************
* @brief       update some parameter
* @details     update some parameter
* @param[in]   null
* @param[out]  null
*
* @return     null
***********************************************************************************************
*/
static void update_config_parameter(void)
{
  /* update maximum pwm parameter for motor according to the motor voltage */
  if(smart_servo_voltage_val > 2400)//when voltage is bigger than 8.4V (voltage adc value: 2400 = 8.4V / 0.0035)
  {
    limit_pwm = (uint16_t)(smart_servo_voltage_val*255.0/2400.0);
  }
  else
  {
    limit_pwm = 255;
  }
}


static void smart_servo_ccw(uint8_t speed)
{
  pwm_write(SMART_SERVO_PWMA,speed,0,limit_pwm);
  pwm_write(SMART_SERVO_PWMB,0,0,limit_pwm);
}

static void smart_servo_cw(uint8_t speed)
{
  pwm_write(SMART_SERVO_PWMA,0,0,limit_pwm);
  pwm_write(SMART_SERVO_PWMB,speed,0,limit_pwm);
}

void smart_servo_speed_update(int16_t pwm)
{
  int16_t speed_temp = pwm;

  if((SMART_SERVO_PWM_MODE != motion_mode) && (release_state_flag == true))
  {
    return;
  }

  if(speed_temp > 0)
  {
    smart_servo_cw((uint8_t)speed_temp);
  }
  else
  {
    smart_servo_ccw((uint8_t)abs_user(speed_temp));
  }
}

void smart_servo_rotation(int16_t speed_pwm)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  motion_mode = SMART_SERVO_PWM_MODE;
  PID_speed.Integral = 0;
  release_state_flag = false;
  smart_servo_target_speed = (float)speed_pwm;
  if(smart_servo_voltage_val < 2000)//when voltage is less than 7.0V (voltage adc value: 2000 = 7.0V / 0.0035)
  {
    smart_servo_target_speed = constrain(floor(smart_servo_target_speed), -40, 40);
  }
  PID_speed.Output = smart_servo_output;
}

void smart_servo_move_to(long absolute_angle,float speed)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  motion_mode = SMART_SERVO_COMMON_MODE;
  pos_lock_flag = false;
  PID_speed.Integral = 0;
  PID_pos.Integral = 0;
  filter_pos = 0.0;
  smart_servo_target_pos = absolute_angle;
  smart_servo_target_speed = (float)abs_user(speed);
  smart_servo_target_speed = constrain(floor(smart_servo_target_speed),-SMART_SERVO_PER_SPEED_MAX,SMART_SERVO_PER_SPEED_MAX);
  release_state_flag = false;
  if(abs_user(smart_servo_distance_togo()) >= smart_servo_target_speed * DECELERATION_DISTANCE_PITCH)
  {
    PID_speed.Output = (255.0/SMART_SERVO_PER_SPEED_MAX) * smart_servo_target_speed;

    if(smart_servo_distance_togo() > 0)
    {
      dir_lock_flag = true;
      PID_speed.Output = max(abs_user(PID_speed.Output),abs_user(smart_servo_output));
    }
    else
    {
      dir_lock_flag = false;
      PID_speed.Output = min(-abs_user(PID_speed.Output),-abs_user(smart_servo_output));
    }
  }
  else
  {
    PID_speed.Output = (255.0/SMART_SERVO_PER_SPEED_MAX) * abs_user(smart_servo_distance_togo())/(float)DECELERATION_DISTANCE_PITCH;
    if(smart_servo_distance_togo() > 0)
    {
      dir_lock_flag = true;
      PID_speed.Output = max(abs_user(PID_speed.Output),abs_user(smart_servo_output));
    }
    else
    {
      dir_lock_flag = false;
      PID_speed.Output = min(-abs_user(PID_speed.Output),-abs_user(smart_servo_output));
    }
  }
  smart_servo_output = PID_speed.Output;
}

void smart_servo_move(long angle,float speed)
{
  smart_servo_move_to((smart_servo_cur_pos + angle),speed);
}

SpiFlashOpResult smart_servo_set_current_angle_zero_degrees(void)
{
  int value = -1;
  SpiFlashOpResult result = SPI_FLASH_RESULT_OK;
  value = RAW_ANGLE_MAX_INT-1 - mp9960_read_raw_angle();
  if(flash_write_angle_offset((uint32_t)value) == SPI_FLASH_RESULT_OK)
  {
    angle_pos_offset = value;
    result = SPI_FLASH_RESULT_OK;
  }
  else
  {
    result = SPI_FLASH_RESULT_ERR;
  }
  smart_servo_cur_pos = 0;
  smart_servo_target_pos = smart_servo_cur_pos;
  smart_servo_circular_turn_count = 0;
  pre_pos = 0;
  smart_servo_speed_integral = 0;
  return result;
}

void smart_servo_circular_turn_calc(int16_t cur_pos, int16_t pre_pos, float speed_input)
{
  smart_servo_speed_integral += speed_input;
  if(abs_user(cur_pos - pre_pos) > RAW_ANGLE_MAX_INT/2)
  {
    if((pre_pos > RAW_ANGLE_MAX_INT/2) && (cur_pos < RAW_ANGLE_MAX_INT/2))
    {
      smart_servo_circular_turn_count = smart_servo_circular_turn_count + 1;
      // uart_printf(UART1,"%ld\r\n",smart_servo_circular_turn_count);
    }
    else if((pre_pos < RAW_ANGLE_MAX_INT/2) && (cur_pos > RAW_ANGLE_MAX_INT/2))
    {
      smart_servo_circular_turn_count = smart_servo_circular_turn_count - 1;
      // uart_printf(UART1,"%ld\r\n",smart_servo_circular_turn_count);
    }
  }
}

void smart_servo_update(void)
{
  int16_t cur_pos;
  int16_t speed_temp;
  cur_pos = adc_get_position_value();
  speed_temp = normalize_position_difference(cur_pos - pre_pos);
  // speed_temp = Kalman(speed_temp);
  smart_servo_circular_turn_calc(cur_pos,pre_pos,speed_temp);
  // uart_printf(UART1,"cur_pos: %d  pre_pos: %d  speed_temp: %d  ",cur_pos,pre_pos,speed_temp);
  pre_pos = cur_pos;
  smart_servo_cur_speed = ((speed_temp * (1000.0/period_time))/RAW_ANGLE_MAX_FLOAT) * 60;
  // uart_printf(UART1,"cur_speed: %0.2f\r\n",smart_servo_cur_speed);
  smart_servo_cur_pos = smart_servo_circular_turn_count * RAW_ANGLE_MAX_INT + cur_pos;

  /* update some parameter */
  update_config_parameter();
}

long smart_servo_distance_togo(void)
{
  if(release_state_flag == false)
  {
    return smart_servo_target_pos - smart_servo_cur_pos;
  }
  else
  {
    return 0;
  }
}

int16_t filter_position(int16_t input)
{
  filter_pos = 0.65 * filter_pos + 0.35 * input;
  return (int16_t)filter_pos;
}


int16_t normalize_position_difference(int16_t posdiff)
{
  if(abs_user(posdiff) <= 1)
  {
    posdiff = 0;
  }
  if (posdiff > ((SMART_SERVO_MAX_POSITION - SMART_SERVO_MIN_POSITION) / 2))
  {
    posdiff -= (SMART_SERVO_MAX_POSITION - SMART_SERVO_MIN_POSITION);
  }

  if (posdiff < -((SMART_SERVO_MAX_POSITION - SMART_SERVO_MIN_POSITION) / 2))
  {
    posdiff += (SMART_SERVO_MAX_POSITION - SMART_SERVO_MIN_POSITION);
  }
  return posdiff;
}

void check_whether_reach_the_postion(void)
{
  uint8_t checksum;
    //response mesaage to UART0
    write_byte_uart0(START_SYSEX);
    write_byte_uart0(device_id);
    write_byte_uart0(SMART_SERVO);
    write_byte_uart0(CHECK_WHETHER_REACH_THE_SET_POSITION);
    write_byte_uart0((uint8_t)reach_pos_flag);
    checksum = (device_id + SMART_SERVO + CHECK_WHETHER_REACH_THE_SET_POSITION+(uint8_t)reach_pos_flag);
    checksum = checksum & 0x7f;
    write_byte_uart0(checksum);
    write_byte_uart0(END_SYSEX);
}

int16_t pid_speed_to_pwm(void)
{
  float speed_error;

  speed_error = (float)(smart_servo_cur_speed - smart_servo_target_speed);
  if(speed_error > 1.0)
  {
   if(abs_user(speed_error) > PID_speed.P)
   {
     PID_speed.Output -= PID_speed.P * (abs_user(speed_error)/abs_user(speed_error));
   }
   else
   {
     PID_speed.Output -= abs_user(speed_error) * (abs_user(speed_error)/abs_user(speed_error));
   }
  }
  else if(speed_error < -1.0 )
  {
   if(abs_user(speed_error) > PID_speed.P)
   {
     PID_speed.Output += PID_speed.P * (abs_user(speed_error)/abs_user(speed_error));
   }
   else
   {
     PID_speed.Output += abs_user(speed_error) * (abs_user(speed_error)/abs_user(speed_error));
   }
  }
  PID_speed.Output = constrain(PID_speed.Output,SMART_SERVO_MIN_OUTPUT,SMART_SERVO_MAX_OUTPUT);
  smart_servo_output = PID_speed.Output;
  // uart_printf(UART1,"device_id: %d, speed_error: %.2f, cur_speed:%d, Output:%.2f\r\n", device_id, speed_error, (int)smart_servo_cur_speed,PID_speed.Output);

  return smart_servo_output;
}

int16_t pid_position_to_pwm(void)
{
  float speed_error;
  float pos_error;
  float d_component;
  float seek_velocity;

   if(smart_servo_target_speed <= SMART_SERVO_MINIMUM_SPEED)
  {
    smart_servo_output = 0;
    return smart_servo_output;
  }

  //speed pid;
  if(abs_user(smart_servo_distance_togo()) >= smart_servo_target_speed * DECELERATION_DISTANCE_PITCH)
  {
    speed_error = abs_user(smart_servo_cur_speed) - smart_servo_target_speed;

    if((speed_error != 0) && (smart_servo_distance_togo() > 0))
    {
      if(abs_user(speed_error) > PID_speed.P)
      {
        PID_speed.Output -= PID_speed.P * (speed_error/abs_user(speed_error));
      }
      else
      {
        PID_speed.Output -= abs_user(speed_error) * (speed_error/abs_user(speed_error));
      }
    }
    else if((speed_error !=0) && (smart_servo_distance_togo() < 0))
    {
      if(abs_user(speed_error) > PID_speed.P)
      {
        PID_speed.Output += PID_speed.P * (speed_error/abs_user(speed_error));
      }
      else
      {
        PID_speed.Output += abs_user(speed_error) * (speed_error/abs_user(speed_error));
      }
    }
    PID_speed.Output = constrain(PID_speed.Output,SMART_SERVO_MIN_OUTPUT,SMART_SERVO_MAX_OUTPUT);
    smart_servo_output = PID_speed.Output;
    // uart_printf(UART1,"speed_error: %.2f, cur_speed:%d, Output:%.2f, Pos:%d\r\n",speed_error, (int)smart_servo_cur_speed,PID_speed.Output,smart_servo_cur_pos);
  }

  //position pid;
  else if(abs_user(smart_servo_distance_togo()) < smart_servo_target_speed * DECELERATION_DISTANCE_PITCH)
  {
    pos_error = smart_servo_target_pos - smart_servo_cur_pos;
    smart_servo_output = (255.0/SMART_SERVO_PER_SPEED_MAX)* smart_servo_distance_togo()/DECELERATION_DISTANCE_PITCH_FLOAT;

    if(abs_user(pos_error) >= SMART_SERVO_POS_DEADBAND)
    {
      reach_pos_flag = false;
      if(pos_error > 0)
      {
        if((pos_lock_flag == true) && (dir_lock_flag == false))
        {
          PID_pos.Integral = 0;
        }
        dir_lock_flag = true;
        seek_velocity = sqrt(abs_user(smart_servo_target_speed * DECELERATION_DISTANCE_PITCH * pos_error))/DECELERATION_DISTANCE_PITCH_FLOAT;
        d_component = smart_servo_cur_speed - seek_velocity;
      }
      else
      {
        if((pos_lock_flag == true) && (dir_lock_flag == true))
        {
          PID_pos.Integral = 0;
        }
        dir_lock_flag = false;
        seek_velocity = -sqrt(abs_user(smart_servo_target_speed * DECELERATION_DISTANCE_PITCH * pos_error))/DECELERATION_DISTANCE_PITCH_FLOAT;
        d_component = smart_servo_cur_speed - seek_velocity;
      }
      PID_pos.Integral += d_component;
      if(pos_lock_flag == true)
      {
        PID_pos.Output = smart_servo_output + 2* PID_pos.P * pos_error - PID_pos.D * d_component;
        //PID_pos.Output = smart_servo_output + PID_pos.P * pos_error * 2 - PID_pos.I * PID_pos.Integral;
      }
      else
      {
        PID_pos.Output = smart_servo_output - PID_pos.D * d_component - PID_pos.I * PID_pos.Integral;
        PID_pos.Output = PID_pos.Output * abs_user(pos_error) / 4096.0 * 20.5;
        //PID_pos.Output = smart_servo_output + PID_pos.P * pos_error - PID_pos.D * d_component - PID_pos.I * PID_pos.Integral;
      }
    }
    else
    {
      // uart_printf(UART0,"smaller than deadband    ");
      pos_error = filter_position(pos_error);
      pos_lock_flag = true;
      PID_pos.Integral = 0;
      seek_velocity = 0;
      d_component = smart_servo_cur_speed - seek_velocity;
      PID_pos.Output = smart_servo_output + PID_pos.P * pos_error;
      if(reach_pos_flag == false)
      {
        reach_pos_flag = true;
        check_whether_reach_the_postion();
      }
    }
    PID_pos.Output = constrain(PID_pos.Output,SMART_SERVO_MIN_OUTPUT,SMART_SERVO_MAX_OUTPUT);
    // uart_printf(UART1,"pos_error: %.2f, d_component: %.2f, PID_pos.Integral: %.2f, seek_velocity: %.2f, smart_servo_output: %d\r\n",pos_error, d_component, PID_pos.Integral, seek_velocity,smart_servo_output);
    smart_servo_output = PID_pos.Output;
  }
  return smart_servo_output;
}

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
/**************************************************************************************/
int16_t adc_get_smart_servo_type0_value(void) //for servo type low bit
{
  int value = -1;
  value = analogRead(SMART_SERVO_TYPE0_AD);
  return value;
}

int16_t adc_get_smart_servo_type1_value(void) //for servo type high bit
{
  int value = -1;
  value = analogRead(SMART_SERVO_TYPE1_AD);
  return value;
}

int16_t io_get_HW_Version_D0_value(void)  //for hardware type low bit
{
  int value = -1;
  value = digitalRead(SMART_SERVO_HW_Verison_D0);
  return value;
}

int16_t io_get_HW_Version_D1_value(void)  //for hardware type high bit
{
  int value = -1;
  value = digitalRead(SMART_SERVO_HW_Verison_D1);
  return value;
}
/**************************************************************************************/
int16_t io_get_nfault_value(void)
{
  int value = -1;
  value = digitalRead(SMART_SERVO_NFAULT);
  return value;
}

void servo_move_test(float speed)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  if(speed > 0)
  {
    smart_servo_cw((uint8_t)speed);
  }
  else
  {
    smart_servo_ccw((uint8_t)abs_user(speed));
  }
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

/*

  calculating parameters per 10ms for protection

 */
void motor_protection(void)
{
  float temp = adc_get_temperature_value();
  float current = adc_get_current_value();
  float voltage = adc_get_voltage_value();
  float locked_electric_current = 0.00;

  cur_debounced_count ++;
  temp_debounced_count ++;
  vol_debounced_count ++;
  temp = calculate_temp(temp);
  current = calculate_current(current);
  voltage = calculate_voltage(voltage);

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
  if(io_get_nfault_value() == 0)
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

/**
***********************************************************************************************
* @brief       for getting the types of smart servo
* @details     for getting the types of smart servo
* @param[in]   null
* @param[out]  null
*
* @return     null
***********************************************************************************************
*/
void get_smart_servo_type(void)
{
  int8_t type0_index = -1;
  int8_t type1_index = -1;
  float smart_servo_type0_vol = 0.0;
  float smart_servo_type1_vol = 0.0;

  smart_servo_type0_vol = (float)adc_get_smart_servo_type0_value()/4096*3300;
  type0_index = constrain(round((float)(smart_servo_type0_vol/206)), 0 ,15);
  smart_servo_type1_vol = (float)adc_get_smart_servo_type1_value()/4096*3300;
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

/*
 *  function for get hardware version
*/
char get_Hardware_Version(void)
{
  char _Hardware_Version = '0';

  if(io_get_HW_Version_D1_value())
  {
    if(io_get_HW_Version_D0_value())  // 11
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
    if(io_get_HW_Version_D0_value())  // 01
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




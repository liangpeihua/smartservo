/**
 * @file    servo_control.c
 * @author  Payton
 * @version V1.0.0
 * @date    2017/11/17
 * @brief   
 *
 * \par Description
 * This file is servo control algorithm.
 *
 * \par History:
 * <pre>
 * `<Author>`        `<Time>`         `<Version>`        `<Descr>`
 * Payton            2017/11/17         1.0.0            create
 * </pre>
 *
 */
#include "servo_control.h"
#include "servo_driver.h"
#include "servo_detect.h"
#include "systime.h"
#include "sysinit.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"
#include "usart_Fun.h"
	
#define MAX_SPEED         60	//1rpm/min,实测最大转
#define _PWM(Speed)      ((Speed) * MAX_OUTPUT_PWM / MAX_SPEED)	//拟合pwm

STRUCT_PID speed_ctrl = {0};
STRUCT_PID pos_ctrl = {0};
STRUCT_PID torque_ctrl = {0};
int32_t s_output_pwm = 0;

//运动控制状态
static boolean s_bMotionStatusChanged = TRUE;

static MOTOR_CTRL_STATUS motor_idle_mode(void *param);
static MOTOR_CTRL_STATUS motor_pwm_mode(void *param);
static MOTOR_CTRL_STATUS motor_speed_mode(void *param);
static MOTOR_CTRL_STATUS motor_pos_mode(void *param);
static MOTOR_CTRL_STATUS motor_torque_mode(void *param);
static MOTOR_CTRL_STATUS motor_error_mode(void *param);
static MOTOR_CTRL_STATUS motor_debug_mode(void *param);

extern void check_whether_reach_the_postion(void);

//运动控制处理函数
typedef struct
{
  MOTOR_CTRL_STATUS (*motion_control)(void *param);//运动控制模式
  void (*ack_event)(void);//到达目标位置时，产生的事件
  int32_t param; 
}MOTION_INFO;

MOTION_INFO MotionProcessors[] = 
{
  {motor_idle_mode,    NULL, 0},
  {motor_pwm_mode,     NULL, 0},
  {motor_speed_mode,   NULL, 0},
  {motor_pos_mode,     check_whether_reach_the_postion, 0},
  {motor_torque_mode,  check_whether_reach_the_postion, 0},
  {motor_error_mode,   NULL, 0},
  {motor_debug_mode,   NULL, 0},
};

static MOTOR_CTRL_STATUS motor_idle_mode(void *param)
{
  if(s_bMotionStatusChanged)
  {
    speed_ctrl.integral = 0;
    speed_ctrl.output = 0;
    pos_ctrl.integral = 0;
    pos_ctrl.output = 0;
    s_output_pwm = 0;
    servodriver_run_idle();
    //servodriver_set_pwm(s_output_pwm);
    g_servo_info.reach_tar_pos = true;
  }
  servodriver_set_pwm(s_output_pwm);
  return IDLE_MODE;
}

static MOTOR_CTRL_STATUS motor_pwm_mode(void *param)
{
  int32_t set_pwm = 0; 
  static int32_t step_len = 0;

  if(s_bMotionStatusChanged)
  {
    step_len = 30;
    s_output_pwm = 0;
  }

  if(g_servo_info.errorid != 0)
  {
    return ERROR_MODE;
  }

  set_pwm = g_servo_info.tar_pwm;

  if(set_pwm > s_output_pwm+step_len)
  {
    s_output_pwm += step_len; 
  }
  else if(set_pwm < s_output_pwm-step_len)
  {
    s_output_pwm -= step_len;
  }
  else
  {
    s_output_pwm = set_pwm;
  }

  s_output_pwm = constrain(s_output_pwm, -g_servo_info.limit_pwm, g_servo_info.limit_pwm);
  servodriver_set_pwm(s_output_pwm);

  return PWM_MODE;
}

static MOTOR_CTRL_STATUS motor_speed_mode(void *param)
{
  int32_t speed_error;

  if(s_bMotionStatusChanged)
  {
  speed_ctrl.integral = _PWM(g_servo_info.cur_speed) / speed_ctrl.Ki;
  speed_ctrl.output = 0;
  speed_ctrl.Kp = 200;
  speed_ctrl.Ki = 20;
  }

  if(g_servo_info.errorid != 0)
  {
    return ERROR_MODE;
  }
  g_servo_info.posmode_tarspeed = g_servo_info.tar_speed;

  speed_error = g_servo_info.tar_speed - g_servo_info.cur_speed;
  speed_error = constrain(speed_error,-20,20);
  speed_ctrl.integral += speed_error;
  speed_ctrl.integral = constrain(speed_ctrl.integral,-(10*g_servo_info.limit_pwm/speed_ctrl.Ki),(10*g_servo_info.limit_pwm/speed_ctrl.Ki));
  speed_ctrl.output = (speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral) / 10;
  speed_ctrl.output = constrain(speed_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);

  s_output_pwm = speed_ctrl.output;
  servodriver_set_pwm(s_output_pwm);

  return SPEED_MODE;
}

//占用时间T= 70us
static MOTOR_CTRL_STATUS motor_pos_mode(void *param)
{
  int32_t speed_error;
  int32_t pos_error;
  int32_t abspos_error;
  int32_t target_speed;
  float A;
  int32_t H;
  int32_t K;

  if(s_bMotionStatusChanged)
  {
    speed_ctrl.integral = _PWM(g_servo_info.cur_speed) / speed_ctrl.Ki;
    speed_ctrl.output = 0;
    speed_ctrl.Kp = 200;
    speed_ctrl.Ki = 10;

    pos_ctrl.output = 0;
    pos_ctrl.Kp = 10;
    pos_ctrl.Kd = 20;
  }

  if(g_servo_info.errorid != 0)
  {
    return ERROR_MODE;
  }

  //pos pid
  pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
  LIMIT_DEATH(pos_error, 5);
  pos_error = constrain(pos_error,-20,20);
  pos_ctrl.output = pos_ctrl.Kp * pos_error + pos_ctrl.Kd * (pos_error - pos_ctrl.last_error);
  pos_ctrl.output = constrain(pos_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);
  pos_ctrl.last_error = pos_error;

  //speed pid
  H = 1000;
  K = MAX_TAR_SPEED;
  A = (float)(-MAX_TAR_SPEED) / pow(H,2);
  pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
  LIMIT_DEATH(pos_error, 5);
  abspos_error = abs_user(pos_error);
  if(abspos_error == 0)
  {
    target_speed = 0;
    if(g_servo_info.reach_tar_pos == FALSE)
    {
      MotionProcessors[POS_MODE].ack_event();
    }
    g_servo_info.reach_tar_pos = TRUE;
  }
  else if(abspos_error < H)
  {
    target_speed = A*pow((abspos_error - H),2) + K;//二次函数，顶点式：y=a(x-h)²+k
    if(target_speed < 1){
      target_speed = 1;
    }
  }
  else
  {
    target_speed = abs_user(g_servo_info.tar_speed);
  }

  target_speed = constrain(target_speed,0,abs_user(g_servo_info.tar_speed));
  g_servo_info.posmode_tarspeed = target_speed;
  if(pos_error > 0){
    target_speed= abs_user(target_speed);
  }
  else{
    target_speed= -abs_user(target_speed);
  }
  speed_error = target_speed - g_servo_info.cur_speed;
  speed_error = constrain(speed_error,-20,20);
  speed_ctrl.integral += speed_error;
  speed_ctrl.integral = constrain(speed_ctrl.integral,-(10*g_servo_info.limit_pwm/speed_ctrl.Ki),(10*g_servo_info.limit_pwm/speed_ctrl.Ki));
  speed_ctrl.output = (speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral) / 10;
  speed_ctrl.output = constrain(speed_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);

  s_output_pwm = pos_ctrl.output + speed_ctrl.output;
  if(abs_user(s_output_pwm) < 50){
    s_output_pwm = 0;
  }
  s_output_pwm = constrain(s_output_pwm,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);
  servodriver_set_pwm(s_output_pwm);

  return POS_MODE;
}

static MOTOR_CTRL_STATUS  motor_torque_mode(void *param)
{
  int32_t speed_error;
  int32_t pos_error;
  int32_t abspos_error;
  int32_t target_speed;
  int32_t torque_error;
  float A;
  int32_t H;
  int32_t K;

  if(s_bMotionStatusChanged)
  {
    speed_ctrl.integral = _PWM(g_servo_info.cur_speed) / speed_ctrl.Ki;
    speed_ctrl.output = 0;
    speed_ctrl.Kp = 200;
    speed_ctrl.Ki = 10;

    pos_ctrl.output = 0;
    pos_ctrl.Kp = 10;
    pos_ctrl.Kd = 20;

    torque_ctrl.integral = 0;
    torque_ctrl.output = 0;
    torque_ctrl.Kp = 100;
    torque_ctrl.Ki = 2;
  }

  if(g_servo_info.errorid != 0)
  {
    return ERROR_MODE;
  }

  //pos pid
  pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
  LIMIT_DEATH(pos_error, 5);
  pos_error = constrain(pos_error,-20,20);
  pos_ctrl.output = pos_ctrl.Kp * pos_error + pos_ctrl.Kd * (pos_error - pos_ctrl.last_error);
  pos_ctrl.output = constrain(pos_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);
  pos_ctrl.last_error = pos_error;

  //speed pid
  H = 1000;
  K = MAX_TAR_SPEED;
  A = (float)(-MAX_TAR_SPEED) / pow(H,2);
  pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
  LIMIT_DEATH(pos_error, 5);
  abspos_error = abs_user(pos_error);
  if(abspos_error == 0)
  {
    target_speed = 0;
    if(g_servo_info.reach_tar_pos == FALSE)
    {
      MotionProcessors[TORQUE_MODE].ack_event();
    }
    g_servo_info.reach_tar_pos = TRUE;
  }
  else if(abspos_error < H)
  {
    target_speed = A*pow((abspos_error - H),2) + K;//二次函数，顶点式：y=a(x-h)²+k
    if(target_speed < 1){
      target_speed = 1;
    }
  }
  else
  {
    target_speed = abs_user(g_servo_info.tar_speed);
  }

  target_speed = constrain(target_speed,0,abs_user(g_servo_info.tar_speed));
  g_servo_info.posmode_tarspeed = target_speed;
  if(pos_error > 0){
    target_speed= abs_user(target_speed);
  }
  else{
    target_speed= -abs_user(target_speed);
  }
  speed_error = target_speed - g_servo_info.cur_speed;
  speed_error = constrain(speed_error,-20,20);
  speed_ctrl.integral += speed_error;
  speed_ctrl.integral = constrain(speed_ctrl.integral,-(10*g_servo_info.limit_pwm/speed_ctrl.Ki),(10*g_servo_info.limit_pwm/speed_ctrl.Ki));
  speed_ctrl.output = (speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral) / 10;
  speed_ctrl.output = constrain(speed_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);

#if 0
  //torque pid
  torque_error = abs_user(g_servo_info.tar_torque*4 - g_servo_info.current);

  torque_ctrl.Kp = 500;
  torque_ctrl.Ki = 5;

  if(g_servo_info.current > g_servo_info.tar_torque*4)
  {
		torque_error = constrain(torque_error,-200,200);
		torque_ctrl.integral += torque_error;
		torque_ctrl.integral = constrain(torque_ctrl.integral,-(int32_t)(100*1500/torque_ctrl.Ki),(int32_t)(100*1500/torque_ctrl.Ki));
		torque_ctrl.output = (torque_ctrl.Kp * torque_error + torque_ctrl.Ki * torque_ctrl.integral) / 100;
		torque_ctrl.output = constrain(torque_ctrl.output,-(int32_t)(1500), (int32_t)(1500));

  }
  else if(g_servo_info.current > (int32_t)(g_servo_info.tar_torque*4-20))//20mA滞回
  {
  }
  else
  {
    if(torque_ctrl.output > 2){
      torque_ctrl.output -= 2;
    }
  }

  if(speed_ctrl.output > 0)
  {
     s_output_pwm = pos_ctrl.output + speed_ctrl.output - torque_ctrl.output;
  }
  else
  {
     s_output_pwm = pos_ctrl.output + speed_ctrl.output + torque_ctrl.output;
  }

  //s_output_pwm = pos_ctrl.output + speed_ctrl.output + torque_ctrl.output;
  s_output_pwm = constrain(s_output_pwm,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);
  servodriver_set_pwm(s_output_pwm);
#else
  s_output_pwm = pos_ctrl.output + speed_ctrl.output;
  s_output_pwm = constrain(s_output_pwm,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);
  servodriver_set_limitcurrent(g_servo_info.tar_torque*4);
  servodriver_set_pwm(s_output_pwm);
#endif

  return TORQUE_MODE;
}

static MOTOR_CTRL_STATUS motor_error_mode(void *param)
{
  static int32_t step_len = 0;
  int32_t set_pwm = 0; 
  	
  if(s_bMotionStatusChanged)
  {
    speed_ctrl.integral = 0;
    speed_ctrl.output = 0;
    pos_ctrl.integral = 0;
    pos_ctrl.output = 0;
    step_len = 0;
  }

  if(s_output_pwm == 0)
  {
    return IDLE_MODE;
  }

  if(abs_user(s_output_pwm) > g_servo_info.limit_pwm/2)
  {
    step_len = 5;
  }
  else
  {
    step_len = 2;
  }

  set_pwm = 0;

  if(set_pwm > s_output_pwm+step_len)
  {
    s_output_pwm += step_len; 
  }
  else if(set_pwm < s_output_pwm-step_len)
  {
    s_output_pwm -= step_len;
  }
  else
  {
    s_output_pwm = set_pwm;
  }

  servodriver_set_pwm(s_output_pwm);

  return ERROR_MODE;
}

static MOTOR_CTRL_STATUS motor_debug_mode(void *param)
{
  int32_t set_pwm = 0; 
  static int32_t step_len = 0;

  if(s_bMotionStatusChanged)
  {
    step_len = 5;
    s_output_pwm = 100;
  }

  if(g_servo_info.errorid != 0)
  {
    return ERROR_MODE;
  }

  set_pwm = g_servo_info.tar_pwm;

  if(set_pwm > s_output_pwm+step_len)
  {
    s_output_pwm += step_len; 
  }
  else if(set_pwm < s_output_pwm-step_len)
  {
    s_output_pwm -= step_len;
  }
  else
  {
    s_output_pwm = set_pwm;
  }

  s_output_pwm = constrain(s_output_pwm, -g_servo_info.limit_pwm, g_servo_info.limit_pwm);
  servodriver_set_pwm(s_output_pwm);

  return DEBUG_MODE;
}


/***********************************************************
* Function Name : motor_process
* Description   : 运动控制线程,5ms执行一次
* Input         : NONE
* Output        : NONE
* Return        : NONE
************************************************************/
void motor_process(void)
{
  static MOTOR_CTRL_STATUS last_motion_status = (MOTOR_CTRL_STATUS)0xFF;

  if(g_servo_info.ready == false)
  {
    return;
  }

  if(last_motion_status != g_eSysMotionStatus) 
  {
    last_motion_status = g_eSysMotionStatus;
    s_bMotionStatusChanged = TRUE;
  }

  //执行函数
  g_eSysMotionStatus = MotionProcessors[g_eSysMotionStatus].motion_control(&(MotionProcessors[g_eSysMotionStatus].param));

  s_bMotionStatusChanged = FALSE;	
}


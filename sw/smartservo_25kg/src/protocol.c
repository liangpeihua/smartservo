#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "sysinit.h"
#include "protocol.h"
#include "uart_printf.h"
#include "rgb.h"
#include "servo_driver.h"
#include "servo_detect.h"
#include "dataflash.h"
#include "MePwm.h"
#include "math.h"
#include "systime.h"
#include "mp9960.h"
#include "main.h"
#include "servo_driver.h"

volatile uint8_t Uart0SendData[UART0_SEND_BUF_SIZE]={0};
volatile uint8_t Uart0RecData[UART0_REV_BUF_SIZE]={0};
volatile uint8_t Uart1RecData[UART1_REV_BUF_SIZE]={0};

volatile boolean parsingSysex = false;
volatile boolean parsingVersionSysex = false;
volatile boolean parsingSysexComplete = true;
volatile boolean parsingGcode = false;
volatile boolean parsingGcodeComplete = true;
volatile boolean isLocalOutput = false;
volatile boolean isForwardOutput = false;

volatile boolean shake_hand_flag = false;

volatile uint16_t sysexBytesRead = 0 ;
volatile int16_t ForwardBytesRead = 0;
volatile int16_t Uart0SendBytes = 0;
volatile uint16_t Uart1Revhead  = 0;
volatile uint16_t Uart1RevRtail  = 0;
volatile uint16_t Uart0Sendhead  = 0;
volatile uint16_t Uart0SendRtail  = 0;
volatile int16_t InputBytesRead = 0;
volatile uint16_t Uart0Revhead  = 0;
volatile uint16_t Uart0RevRtail  = 0;

volatile uint8_t device_id = 0;                //hardware ID, Uniqueness in a link.
volatile uint8_t device_id_2 = 0;              //hardware ID, Uniqueness in a link.

volatile uint8_t device_type = CUSTOM_TYPE;  //device type, Indicates the type of module, 0x00 indicates no external module
volatile uint8_t command_mode = FIRMATA_DATA_MODE;    //G_CODE_MODE,FIRMATA_DATA_MODE

char mVersion[10] = "20.01.101";

union sysex_message sysex = {0};

//Sensor value
volatile int16_t report_mode = 0;
volatile int16_t report_smart_pos_mode = 0;
volatile int16_t report_smart_angle_mode = 0;
volatile int16_t report_smart_temp_mode = 0;
volatile int16_t report_smart_speed_mode = 0;
volatile int16_t report_smart_voltage_mode = 0;
volatile int16_t report_smart_current_mode = 0;
volatile int8_t report_record_info = 0;

//volatile int16_t smart_servo_pos_val = 0;
volatile int16_t pre_smart_servo_pos_val = 0;

//volatile int16_t smart_servo_temp_val = 0;
volatile int16_t pre_smart_servo_temp_val = 0;

//volatile int16_t smart_servo_voltage_val = 0;
volatile int16_t pre_smart_servo_voltage_val = 0;

//volatile int16_t smart_servo_current_val = 0;
volatile int16_t pre_smart_servo_current_val = 0;

//volatile float smart_servo_speed_val = 0;
volatile float pre_smart_servo_speed_val = 0;

//volatile long smart_servo_angle_val = 0;
volatile long pre_smart_servo_angle_val = 0;

volatile uint32_t flush_uart1_forward_time = 0;
volatile uint32_t flush_uart0_local_time = 0;

static volatile uint16_t uart1_ready_forward_bytes_count = 0;

union{
  uint8_t byteVal[8];
  double doubleVal;
}val8byte;

union{
  uint8_t byteVal[4];
  float floatVal;
  long longVal;
}val4byte;

union{
  uint8_t byteVal[2];
  short shortVal;
}val2byte;

union{
  uint8_t byteVal[1];
  uint8_t charVal;
}val1byte;

uint8_t readbyte(uint8_t *argv,int idx)
{
  uint8_t temp;
  val1byte.byteVal[0] = argv[idx] & 0x7f;
  temp = argv[idx+1] << 7;
  val1byte.byteVal[0] |= temp;
  return val1byte.charVal;
}

static uint8_t check_sysex_message(void);

short readShort(uint8_t *argv,int idx,boolean ignore_high)
{
  uint8_t temp;
  val2byte.byteVal[0] = argv[idx] & 0x7f;
  temp = argv[idx+1] << 7;
  val2byte.byteVal[0] |= temp;

  val2byte.byteVal[1] = (argv[idx+1] >> 1) & 0x7f;

  //Send analog can ignored high
  if(ignore_high == false)
  {
    temp = (argv[idx+2] << 6);
    val2byte.byteVal[1] |= temp;
  }

  return val2byte.shortVal;
}

float readFloat(uint8_t *argv,int idx){
  uint8_t temp;
  val4byte.byteVal[0] = argv[idx] & 0x7f;
  temp = argv[idx+1] << 7;
  val4byte.byteVal[0] |= temp;

  val4byte.byteVal[1] =  (argv[idx+1] >> 1) & 0x7f;
  temp = (argv[idx+2] << 6);
  val4byte.byteVal[1] += temp;

  val4byte.byteVal[2] =  (argv[idx+2] >> 2) & 0x7f;
  temp = (argv[idx+3] << 5);
  val4byte.byteVal[2] += temp;

  val4byte.byteVal[3] =  (argv[idx+3] >> 3) & 0x7f;
  temp = (argv[idx+4] << 4);
  val4byte.byteVal[3] += temp;

  return val4byte.floatVal;
}

long readLong(uint8_t *argv,int idx){
  uint8_t temp;
  val4byte.byteVal[0] = argv[idx] & 0x7f;
  temp = argv[idx+1] << 7;
  val4byte.byteVal[0] |= temp;

  val4byte.byteVal[1] =  (argv[idx+1] >> 1) & 0x7f;
  temp = (argv[idx+2] << 6);
  val4byte.byteVal[1] += temp;

  val4byte.byteVal[2] =  (argv[idx+2] >> 2) & 0x7f;
  temp = (argv[idx+3] << 5);
  val4byte.byteVal[2] += temp;

  val4byte.byteVal[3] =  (argv[idx+3] >> 3) & 0x7f;
  temp = (argv[idx+4] << 4);
  val4byte.byteVal[3] += temp;

  return val4byte.longVal;
}

uint8_t sendbyte(uint8_t val)//1 byte to 2 byte
{
  uint8_t checksum;
  uint8_t val_7bit[2]={0};
  val1byte.charVal = val;
  val_7bit[0] = val1byte.byteVal[0] & 0x7f;
  write_byte_uart0(val_7bit[0]);
  val_7bit[1] = (val1byte.byteVal[0] >> 7) & 0x7f;
  write_byte_uart0(val_7bit[1]);
  checksum = val_7bit[0] + val_7bit[1];
  checksum = checksum & 0x7f;
  return checksum;
}

uint8_t sendShort(int val,boolean ignore_high)
{
  uint8_t checksum;
  uint8_t val_7bit[3]={0};
  val2byte.shortVal = val;
  val_7bit[0] = val2byte.byteVal[0] & 0x7f;
  write_byte_uart0(val_7bit[0]);
  val_7bit[1] = ((val2byte.byteVal[1] << 1) | (val2byte.byteVal[0] >> 7)) & 0x7f;
  write_byte_uart0(val_7bit[1]);
  checksum = val_7bit[0] + val_7bit[1];
  //Send analog can ignored high
  if(ignore_high == false)
  {
    val_7bit[2] = (val2byte.byteVal[1] >> 6) & 0x7f;
    checksum += val_7bit[2];
    checksum = checksum & 0x7f;
    write_byte_uart0(val_7bit[2]);
  }
  return checksum;
}

uint8_t sendFloat(float val)
{
  uint8_t checksum;
  uint8_t val_7bit[5]={0};
  val4byte.floatVal = val;
  val_7bit[0] = val4byte.byteVal[0] & 0x7f;
  write_byte_uart0(val_7bit[0]);
  val_7bit[1] = ((val4byte.byteVal[1] << 1) | (val4byte.byteVal[0] >> 7)) & 0x7f;
  write_byte_uart0(val_7bit[1]);
  val_7bit[2] = ((val4byte.byteVal[2] << 2) | (val4byte.byteVal[1] >> 6)) & 0x7f;
  write_byte_uart0(val_7bit[2]);
  val_7bit[3] = ((val4byte.byteVal[3] << 3) | (val4byte.byteVal[2] >> 5)) & 0x7f;
  write_byte_uart0(val_7bit[3]);
  val_7bit[4] = (val4byte.byteVal[3] >> 4) & 0x7f;
  write_byte_uart0(val_7bit[4]);
  checksum = (val_7bit[0] + val_7bit[1] + val_7bit[2] + val_7bit[3] + val_7bit[4]) & 0x7f;
  return checksum;
}

uint8_t sendLong(long val)
{
  uint8_t checksum;
  uint8_t val_7bit[5]={0};
  val4byte.longVal = val;
  val_7bit[0] = val4byte.byteVal[0] & 0x7f;
  write_byte_uart0(val_7bit[0]);
  val_7bit[1] = ((val4byte.byteVal[1] << 1) | (val4byte.byteVal[0] >> 7)) & 0x7f;
  write_byte_uart0(val_7bit[1]);
  val_7bit[2] = ((val4byte.byteVal[2] << 2) | (val4byte.byteVal[1] >> 6)) & 0x7f;
  write_byte_uart0(val_7bit[2]);
  val_7bit[3] = ((val4byte.byteVal[3] << 3) | (val4byte.byteVal[2] >> 5)) & 0x7f;
  write_byte_uart0(val_7bit[3]);
  val_7bit[4] = (val4byte.byteVal[3] >> 4) & 0x7f;
  write_byte_uart0(val_7bit[4]);
  checksum = (val_7bit[0] + val_7bit[1] + val_7bit[2] + val_7bit[3] + val_7bit[4]) & 0x7f;
  return checksum;
}

static void assign_dev_id_response(void *arg)
{
  volatile uint8_t device_id_temp1 = sysex.val.value[0];
  volatile uint8_t checksum;
  if(((ALL_DEVICE + CTL_ASSIGN_DEV_ID + device_id_temp1) & 0x7f) != sysex.val.value[1])
  {
    SendErrorUart0(WRONG_INPUT_DATA);
    return;
  }
  device_id = device_id_temp1 + 1;
  //response mesaage to UART0
  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id);
  write_byte_uart0(CTL_ASSIGN_DEV_ID);
  write_byte_uart0(device_type);
  checksum = (device_id + CTL_ASSIGN_DEV_ID + device_type) & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);

  //forward mesaage to UART1
  write_byte_uart1(START_SYSEX);
  write_byte_uart1(ALL_DEVICE);
  write_byte_uart1(CTL_ASSIGN_DEV_ID);
  write_byte_uart1(device_id);
  checksum = (ALL_DEVICE + CTL_ASSIGN_DEV_ID + device_id) & 0x7f;
  write_byte_uart1(checksum);
  write_byte_uart1(END_SYSEX);
}

static void process_general_cmd_response(void *arg)
{
  uint8_t sub_type;

  sub_type = sysex.val.value[0];
  switch(sub_type)
  {
    case CTL_UPDATE_FIRMWARE:
      entry_update(NULL);//update firmware
      break;
    case CTL_READ_HARDWARE_ID:
      //read_hardware_id_response(NULL);
	  case CTL_CHECK_DEVICE_STATE:
      check_device_state(NULL);
    default:
      break;
  }
}

static void set_baud_rate(void *arg)
{
  uint8_t baudrate = sysex.val.value[0];
  uint32_t baud_rate;

  SendErrorUart0(PROCESS_SUC);
  switch(baudrate)
  {
    case 0:
      baud_rate = 9600;
      break;
    case 1:
      baud_rate = 115200;
      break;
    case 2:
      baud_rate = 921600;
      break;
    default:
      baud_rate = 115200;
      break;
  }
  //Init UART0 for printf
  UART0_Init(baud_rate);

  /* Init UART1*/
  UART1_Init(baud_rate);

  /* Set UART Configuration */
  UART_SetLine_Config(UART0, baud_rate, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

  /* Set UART Configuration */
  UART_SetLine_Config(UART1, baud_rate, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

  /* UART sample function */
  UART_Function_Init();
}

static void cmd_test_response(void *arg)
{
  if((sysex.val.value[0] == 0x01) &&
     (sysex.val.value[1] == 0x02) &&
     (sysex.val.value[2] == 0x03) &&
     (sysex.val.value[3] == 0x04) &&
     (sysex.val.value[4] == 0x05))
  {
    SendErrorUart0(PROCESS_SUC);
  }
  else
  {
    SendErrorUart0(PROCESS_ERROR);
  }
}

static void system_reset_response(void *arg)
{
  command_mode = flash_read_data_mode();
  command_mode = command_mode & 0x01;
  mVersion[4] = command_mode+1+'0';
  servodriver_run_idle();
  device_id = 0;
  device_id_2 = 0;
  device_type = SMART_SERVO;
  report_mode = 0;
  uart_printf(UART0,"V%s\r\n",mVersion);

  // NVIC->ICER[0] = 0xFFFFFFFF;
  // SYS_ResetChip();
}

static void system_version_response(void *arg)//20.01.001
{
  uint8_t checksum;
  uint8_t dev_type;
  uint8_t protocol_ver;
  uint8_t software_ver;

  dev_type = atoi(mVersion);
  protocol_ver = atoi(mVersion+3);//protocol type
  software_ver = atoi(mVersion+7);//software version
  //response mesaage to UART0
  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id);
  write_byte_uart0(CTL_READ_DEV_VERSION);
  write_byte_uart0(dev_type);
  checksum = (device_id + CTL_READ_DEV_VERSION + dev_type);
  write_byte_uart0(protocol_ver);
  write_byte_uart0((uint8_t)mVersion[6]);
  write_byte_uart0(software_ver);
  checksum += (protocol_ver + (uint8_t)mVersion[6] + software_ver);
  checksum = checksum & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);
}

static void shake_hand_response(void *arg)
{
  shake_hand_flag = true;
  SendErrorUart0(PROCESS_SUC);
}

static void assign_dev_id_2_response(void *arg)
{
  uint8_t checksum;

  device_id_2 = sysex.val.value[0] + 1;
  //response mesaage to UART0
  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id_2);
  write_byte_uart0(CTL_ASSIGN_DEV_ID_2);
  write_byte_uart0(device_type);
  checksum = (device_id_2 + CTL_ASSIGN_DEV_ID_2 + device_type) & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);

  //forward mesaage to UART1
  write_byte_uart1(START_SYSEX);
  write_byte_uart1(ALL_DEVICE);
  write_byte_uart1(CTL_ASSIGN_DEV_ID_2);
  write_byte_uart1(device_id_2);
  checksum = (ALL_DEVICE + CTL_ASSIGN_DEV_ID_2 + device_id_2) & 0x7f;
  write_byte_uart1(checksum);
  write_byte_uart1(END_SYSEX);
}

static void send_smart_pos(short pos_value)
{
  uint8_t checksum;

  //response mesaage to UART0
  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id);
  write_byte_uart0(SMART_SERVO);
  write_byte_uart0(GET_SERVO_CUR_POS);
  checksum = (device_id + SMART_SERVO + GET_SERVO_CUR_POS);
  checksum += sendShort(pos_value,true);
  checksum = checksum & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);
}

static void send_smart_angle(long angle_value)
{
  uint8_t checksum;

  //response mesaage to UART0
  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id);
  write_byte_uart0(SMART_SERVO);
  write_byte_uart0(GET_SERVO_CUR_ANGLE);
  checksum = (device_id + SMART_SERVO + GET_SERVO_CUR_ANGLE);
  checksum += sendLong(angle_value);
  checksum = checksum & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);
}

static void send_smart_temp(int32_t temp_value)
{
  uint8_t checksum;
  float tmp_temp;

  //response mesaage to UART0
  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id);
  write_byte_uart0(SMART_SERVO);
  write_byte_uart0(GET_SERVO_TEMPERATURE);
  checksum = (device_id + SMART_SERVO + GET_SERVO_TEMPERATURE);
  //tmp_temp = calculate_temp(temp_value);
	tmp_temp = temp_value;
  checksum += sendFloat(tmp_temp);
  checksum = checksum & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);
}

static void send_smart_speed(int32_t speed_value)
{
  uint8_t checksum;
  float speed_temp = speed_value;

  //uart_printf(UART0,"speed_temp(%d)",speed_val);

  //response mesaage to UART0
  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id);
  write_byte_uart0(SMART_SERVO);
  write_byte_uart0(GET_SERVO_SPEED);
  checksum = (device_id + SMART_SERVO + GET_SERVO_SPEED);
  checksum += sendFloat(speed_temp);
  checksum = checksum & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);
}

static void send_smart_voltage(uint32_t vol_value)
{
  uint8_t checksum;
  float vol_temp;

  //response mesaage to UART0
  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id);
  write_byte_uart0(SMART_SERVO);
  write_byte_uart0(GET_SERVO_VOLTAGE);
  checksum = (device_id + SMART_SERVO + GET_SERVO_VOLTAGE);
  //vol_temp = calculate_voltage(vol_value);
  vol_temp = vol_value / 1000.0;
  checksum += sendFloat(vol_temp);
  checksum = checksum & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);
}

static void send_smart_current(int32_t cur_value)
{
  uint8_t checksum;
  float cur_temp;

  //response mesaage to UART0
  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id);
  write_byte_uart0(SMART_SERVO);
  write_byte_uart0(GET_SERVO_ELECTRIC_CURRENT);
  checksum = (device_id + SMART_SERVO + GET_SERVO_ELECTRIC_CURRENT);
  //cur_temp = calculate_current(cur_value);
	cur_temp	= (float)cur_value;
  checksum += sendFloat(cur_temp);
  checksum = checksum & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);
}

static void smart_servo_cmd_process(void *arg)
{
  uint8_t cmd;
  uint8_t cmd_mode;
  uint8_t status;
  uint8_t led_r;
  uint8_t led_g;
  uint8_t led_b;
//  uint8_t init_mode;
  uint16_t pos_value;
  int16_t speed_pwm;
  float speed;
  uint8_t torque;
  float raw_angle;
  long pos_angle;
  cmd = sysex.val.value[0];
  switch(cmd)
  {
    case SET_SERVO_PID:
    {
      // uint8_t pid_speed_p, pid_pos_p;
      // pid_speed_p = readShort(sysex.val.value,1,true);
      // pid_pos_p = readShort(sysex.val.value,1,true);

      // PID_pos.P = pid_pos_p;  //0.3    //2.0
      // // PID_pos.I = 0.6;  //0.6    //0.75
      // // PID_pos.D = 1.5;

      // PID_speed.P = pid_speed_p;        //3.2
      // // PID_speed.I = 0.0;
      // // PID_speed.D = 0.0;

      // // uart_printf(UART1,"pid_speed_p:%d ,pid_pos_p: %d\r\n",pid_speed_p,pid_pos_p);
      // SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_ABSOLUTE_POS:
    {
      pos_value = readShort(sysex.val.value,1,true);
      speed = readShort(sysex.val.value,3,true);
      if((speed > MAX_TAR_SPEED) || (speed < -MAX_TAR_SPEED))
      {
        speed = MAX_TAR_SPEED;
      }
      servodriver_run_abs_pos(pos_value, abs_user(speed));
//      uart_printf(UART0,"pos:%d ,speed %f, sysexBytesRead:%d\r\n",pos_value,speed,sysexBytesRead);
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_RELATIVE_POS:
    {
      pos_value = readShort(sysex.val.value,1,true);
      speed = readShort(sysex.val.value,3,true);
      if((speed > MAX_TAR_SPEED) || (speed < -MAX_TAR_SPEED))
      {
        speed = MAX_TAR_SPEED;
      }
      servodriver_run_relative_pos(pos_value,abs_user(speed));
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_BREAK:
    {
      status = sysex.val.value[1];
      if(status)
      {
      	servodriver_run_idle();
      }
      else
      {
				servodriver_run_relative_pos(0,5);
      }
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_RGB_LED:
    {
      led_r = readbyte(sysex.val.value,1);
      led_g = readbyte(sysex.val.value,3);
      led_b = readbyte(sysex.val.value,5);
      user_set_rgb(led_r,led_g,led_b);
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SERVO_SHARKE_HAND:
    {
      shake_hand_flag = true;
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_CMD_MODE:
    {
      cmd_mode = sysex.val.value[1];
      cmd_mode = (cmd_mode - 1) & 0x01;
      if(flash_write_data_mode((uint32_t)cmd_mode) == SPI_FLASH_RESULT_OK)
      {
        SendErrorUart0(PROCESS_SUC);
      }
      else
      {
        SendErrorUart0(PROCESS_ERROR);
      }
			break;
    }

    case GET_SERVO_STATUS:
      break;
    case GET_SERVO_PID:
      break;
    case GET_SERVO_CUR_POS:
    {
      report_smart_pos_mode = sysex.val.value[1];

      report_mode += report_smart_pos_mode;
      send_smart_pos(g_servo_info.cur_pos/10);
      break;
    }
    case GET_SERVO_SPEED:
    {
      report_smart_speed_mode = sysex.val.value[1];

      report_mode += report_smart_speed_mode;
      send_smart_speed(g_servo_info.cur_speed);
      break;
    }
    case GET_SERVO_TEMPERATURE:
      report_smart_temp_mode = sysex.val.value[1];

      report_mode += report_smart_temp_mode;
      send_smart_temp(g_servo_info.temperature);
      break;

    case GET_SERVO_ELECTRIC_CURRENT:
      report_smart_current_mode = sysex.val.value[1];

      report_mode += report_smart_current_mode;
      send_smart_current(g_servo_info.current);
      break;

    case GET_SERVO_VOLTAGE:
      report_smart_voltage_mode = sysex.val.value[1];

      report_mode += report_smart_voltage_mode;
      send_smart_voltage(g_servo_info.voltage);
      break;
    case SET_SERVO_CURRENT_ANGLE_ZERO_DEGREES:
      if(servodet_set_angle_zero() == SPI_FLASH_RESULT_OK)
      {
        SendErrorUart0(PROCESS_SUC);
      }
      else
      {
        SendErrorUart0(PROCESS_ERROR);
      }
      break;
    case SET_SERVO_ABSOLUTE_ANGLE:
    {
      raw_angle = readShort(sysex.val.value,1,false);
      pos_angle = raw_angle;//round((RAW_ANGLE_MAX_INT * raw_angle)/360.0);
      speed = readShort(sysex.val.value,4,true);
      speed = constrain(speed,-MAX_TAR_SPEED,MAX_TAR_SPEED);
      servodriver_run_abs_pos(pos_angle, speed);
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_RELATIVE_ANGLE:
    {
      raw_angle = readShort(sysex.val.value,1,false);
      pos_angle = raw_angle;//round((RAW_ANGLE_MAX_INT * raw_angle)/360.0);
      speed = readShort(sysex.val.value,4,true);
      speed = constrain(speed,-MAX_TAR_SPEED,MAX_TAR_SPEED);
      servodriver_run_relative_pos(pos_angle,speed);
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_ABSOLUTE_ANGLE_LONG:
    {
      raw_angle = readLong(sysex.val.value,1);
      pos_angle = raw_angle;//round((RAW_ANGLE_MAX_INT * raw_angle)/360.0);
      speed = readShort(sysex.val.value,6,true);
      speed = constrain(speed,-MAX_TAR_SPEED,MAX_TAR_SPEED);
      servodriver_run_abs_pos(pos_angle, speed);
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_RELATIVE_ANGLE_LONG:
    {
      raw_angle = readLong(sysex.val.value,1);
      pos_angle = raw_angle;//round((RAW_ANGLE_MAX_INT * raw_angle)/360.0);
      speed = readShort(sysex.val.value,6,true);
      speed = constrain(speed,-MAX_TAR_SPEED,MAX_TAR_SPEED);
      servodriver_run_relative_pos(pos_angle,speed);
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_PWM_MOVE:
    {
      speed_pwm = readShort(sysex.val.value,1,false);
      speed_pwm = constrain((float)speed_pwm,-255,255);
      speed_pwm = map(speed_pwm, 0, 255, 0,MAX_OUTPUT_PWM);
      servodriver_run_pwm((int16_t)speed_pwm);
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case GET_SERVO_CUR_ANGLE:
    {
      report_smart_angle_mode = sysex.val.value[1];
      report_mode += report_smart_angle_mode;
      send_smart_angle(g_servo_info.cur_pos/10);
      break;
    }
    case SET_SERVO_BACK_ZERO:
    {
//      init_mode = sysex.val.value[1];
      speed = readShort(sysex.val.value,2,true);
      if((speed > MAX_TAR_SPEED) || (speed < -MAX_TAR_SPEED))
      {
        speed = (float)MAX_TAR_SPEED;
      }
      //uart_printf(UART0,"init_mode:%d,cur_pos:%ld,speed:%f\r\n",init_mode,smart_servo_cur_pos,speed);
      servodriver_run_abs_pos(0, speed);
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_ABSOLUTE_ANGLE_TURQUE:
    {
      raw_angle = readLong(sysex.val.value,1);
      pos_angle = raw_angle;//round((RAW_ANGLE_MAX_INT * raw_angle)/360.0);
      speed = readShort(sysex.val.value,6,true);
      speed = constrain(speed,-20,20);
      torque = readbyte(sysex.val.value,8);
      servodriver_run_abspos_torque(pos_angle, speed, torque);
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    case SET_SERVO_RELATIVE_ANGLE_TORQUE:
    {
      raw_angle = readLong(sysex.val.value,1);
      pos_angle = raw_angle;//round((RAW_ANGLE_MAX_INT * raw_angle)/360.0);
      speed = readShort(sysex.val.value,6,true);
      speed = constrain(speed,-20,20);
      torque = readbyte(sysex.val.value,8);
      servodriver_run_relativepos_torque(pos_angle, speed, torque);
      SendErrorUart0(PROCESS_SUC);
      break;
    }
    default:
      SendErrorUart0(WRONG_TYPE_OF_SERVICE);
      break;
  }
}

const Cmd_list_tab_type cmd_list_tab[]={
{CTL_ASSIGN_DEV_ID,NULL,assign_dev_id_response},              //assign ID
{CTL_SYSTEM_RESET,NULL,system_reset_response},                //system reset
{CTL_READ_DEV_VERSION,NULL,system_version_response},          //read system version
{CTL_SHAKE_HANDS,NULL,shake_hand_response},
{CTL_SET_BAUD_RATE,NULL,set_baud_rate},
{CTL_CMD_TEST,NULL,cmd_test_response},
{CTL_ASSIGN_DEV_ID_2,NULL,assign_dev_id_2_response},
{SMART_SERVO,NULL,smart_servo_cmd_process},
{CTL_GENERAL,NULL,process_general_cmd_response},
};

#define CMD_LIST_TAB_ARRAY_SIZE sizeof(cmd_list_tab)/ sizeof(cmd_list_tab[0])
/*******************************************************************************
* Function Name  : SendErroruart0
* Description    : .....
* Input          :
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void SendErrorUart0(uint8_t errorcode)
{
  uint8_t checksum;
  if(command_mode == G_CODE_MODE)
  {
    uart0_printf("G%d T%d E%d\r\n",device_id,SMART_SERVO,errorcode);
  }
  else
  {
    write_byte_uart0(START_SYSEX);
    write_byte_uart0(device_id);
    write_byte_uart0(CTL_ERROR_CODE);
    write_byte_uart0(errorcode);
    checksum = (device_id + CTL_ERROR_CODE + errorcode) & 0x7f;
    write_byte_uart0(checksum);
    write_byte_uart0(END_SYSEX);
  }
}

/*******************************************************************************
* Function Name  : processSysexMessage
* Description    : .....
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void processSysexMessage(void)
{
  uint8_t i;

  if(sysex.val.dev_id == ALL_DEVICE || sysex.val.dev_id == device_id)
  {
    if(check_sysex_message() == false)//check the checksum value
    {
     SendErrorUart0(WRONG_INPUT_DATA);
     return;
    }
    //CTL_ASSIGN_DEV_ID should processed one by one
    if((sysex.val.dev_id == ALL_DEVICE) &&
       (sysex.val.srv_id != CTL_ASSIGN_DEV_ID) &&
       (sysex.val.srv_id != CTL_ASSIGN_DEV_ID_2))
    {
      if(device_id == 0)
      {
        device_id = ALL_DEVICE;
      }
      write_byte_uart1(START_SYSEX);
      flush_uart0_forward_buffer();
      write_byte_uart1(END_SYSEX);
    }

    for(i=0;i<CMD_LIST_TAB_ARRAY_SIZE;i++)
    {
      if(cmd_list_tab[i].service_id == sysex.val.srv_id)
      {
        // if((device_type != CUSTOM_TYPE) &&
        //    (sysex.val.srv_id != device_type))
        if((device_type != CUSTOM_TYPE) &&
           (sysex.val.srv_id != device_type) &&
           ((sysex.val.srv_id >> 4) != 1) && (sysex.val.srv_id != CTL_GENERAL))
        {
          SendErrorUart0(WRONG_TYPE_OF_SERVICE);
          return;
        }
        else if(cmd_list_tab[i].response_fun != NULL)
        {
          cmd_list_tab[i].response_fun((void*)NULL);
        }
        break;
      }
    }
  }
  else
  {
    write_byte_uart1(START_SYSEX);
    flush_uart0_forward_buffer();
    write_byte_uart1(END_SYSEX);
  }
}

/*******************************************************************************
* Function Name  : flush_uart0_forward_buffer
* Description    : .....
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void flush_uart0_forward_buffer(void)
{
  uint8_t i;
  for(i = 0;i < sysexBytesRead;i++)
  {
    UART_WRITE(UART1, sysex.storedInputData[i]);
    UART_WAIT_TX_EMPTY(UART1);
  }
  sysexBytesRead = 0;
}

/*******************************************************************************
* Function Name  : flush_uart1_forward_buffer
* Description    : flush the uart1 forward data on UART0
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void flush_uart1_forward_buffer(void)
{
  volatile uint8_t inputData = 0xFF;
//  volatile uint16_t tmp;
//  tmp = Uart1RevRtail;

  if(isLocalOutput == true)
  {
    return;
  }

  flush_uart1_forward_time = millis();

  while(Uart1Revhead != Uart1RevRtail)
  {
//    tmp = Uart1RevRtail;
    inputData = Uart1RecData[Uart1Revhead];
    if(((command_mode == FIRMATA_DATA_MODE) && (inputData == START_SYSEX)) ||
       ((command_mode == G_CODE_MODE) && (inputData == 'G')))
    {
      isForwardOutput = true;
    }

    UART_WRITE(UART0, inputData);
    UART_WAIT_TX_EMPTY(UART0);
    Uart1Revhead = (Uart1Revhead == (uint16_t)(UART1_REV_BUF_SIZE - 1)) ? 0 : (Uart1Revhead + 1);
    ForwardBytesRead--;

    if(((command_mode == FIRMATA_DATA_MODE) && (inputData == END_SYSEX)) ||
       ((command_mode == G_CODE_MODE) && (inputData == '\n')))
    {
      isForwardOutput = false;
      break;
    }
    else if((millis() - flush_uart1_forward_time) > 200)
    {
      isForwardOutput = false;
      break;
    }
  }
}

/*******************************************************************************
* Function Name  : flush_uart0_local_buffer
* Description    : flush the uart0 send buffer
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void flush_uart0_local_buffer(void)
{
  volatile uint8_t inputData = 0xFF;
//  volatile uint16_t tmp;
//  tmp = Uart0SendRtail;

  if(isForwardOutput == true)
  {
    return;
  }

  flush_uart0_local_time = millis();

  while(Uart0Sendhead != Uart0SendRtail)
  {
//    tmp = Uart0SendRtail;
    inputData = Uart0SendData[Uart0Sendhead];
    if(((command_mode == FIRMATA_DATA_MODE) && (inputData == START_SYSEX)) ||
       ((command_mode == G_CODE_MODE) && (inputData == 'G')))

    {
      isLocalOutput = true;
    }

    UART_WRITE(UART0, inputData);
    UART_WAIT_TX_EMPTY(UART0);
    Uart0Sendhead = (Uart0Sendhead == (UART0_SEND_BUF_SIZE - 1)) ? 0 : (Uart0Sendhead + 1);
    Uart0SendBytes--;

    if(((command_mode == FIRMATA_DATA_MODE) && (inputData == END_SYSEX)) ||
       ((command_mode == G_CODE_MODE) && (inputData == '\n')))

    {
      isLocalOutput = false;
      break;
    }
    else if((millis() - flush_uart0_local_time) > 200)
    {
      isLocalOutput = false;
      break;
    }
  }
}

void parse_uart0_recv_buffer(void)
{
  volatile uint8_t inputData = 0xFF;

  while(Uart0Revhead != Uart0RevRtail)
  {
    inputData = Uart0RecData[Uart0Revhead];
    if(command_mode == FIRMATA_DATA_MODE)
    {
      if(true == parsingSysex)
      {
        if (inputData == END_SYSEX)
        {
          //stop sysex byte
          parsingSysex = false;
          //fire off handler function
          processSysexMessage();
        }
        else
        {
          //normal data byte - add to buffer
          sysex.storedInputData[sysexBytesRead] = inputData;
          sysexBytesRead++;
          if(sysexBytesRead > DEFAULT_UART_BUF_SIZE-1)
          {
            parsingSysex = false;
            sysexBytesRead = 0;
          }
        }
      }
      else if(inputData == START_SYSEX)
      {
        parsingSysex = true;
        sysexBytesRead = 0;
      }
      else if(true == parsingVersionSysex)
      {
        //normal data byte - add to buffer
        sysex.storedInputData[sysexBytesRead] = inputData;
        sysexBytesRead++;
        if(sysexBytesRead == 5)
        {
          parsingVersionSysex = false;
          sysexBytesRead = 0;
          if((sysex.storedInputData[0] == 0x55) &&
             (sysex.storedInputData[1] == 0x03) &&
             (sysex.storedInputData[3] == 0x01) &&
             (sysex.storedInputData[4] == 0x00))
          {
            int16_t i = 0;
            UART_WRITE(UART0, 0xff);
            UART_WRITE(UART0, 0x55);
            UART_WRITE(UART0, sysex.storedInputData[2]);//0x3e
            UART_WRITE(UART0, 0x04);
            UART_WRITE(UART0, 0x09);//20.A2.B01
            for(i=0;i<9;i++)
            {
              UART_WRITE(UART0, mVersion[i]);
            }
            UART_WRITE(UART0, 0x0d);
            UART_WRITE(UART0, 0x0a);
          }
        }
      }
      else if(inputData == VERSION_READ_SYSEX)
      {
        parsingVersionSysex = true;
        sysexBytesRead = 0;
      }
    }
    else if(command_mode == G_CODE_MODE)
    {
      if(parsingGcode)
      {
        if ((char)inputData == '\n')
        {
          //stop sysex byte
          parsingGcode = false;
          //fire off handler function
          parse_gcommand(sysex.gcode_buffer);
        }
        else
        {
          //normal data byte - add to buffer
          sysex.storedInputData[sysexBytesRead] = inputData;
          sysexBytesRead++;
          if(sysexBytesRead > DEFAULT_UART_BUF_SIZE-1)
          {
            parsingGcode = false;
            sysexBytesRead = 0;
          }
        }
      }
      else if(inputData == 'G')
      {
        parsingGcode = true;
        sysexBytesRead = 0;
      }
    }
    Uart0Revhead = (Uart0Revhead == (uint16_t)(UART0_REV_BUF_SIZE - 1)) ? 0 : (Uart0Revhead + 1);
    InputBytesRead--;
  }
}

/*******************************************************************************
* Function Name  : write_byte_uart0
* Description    : write a byte to the uart0
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void write_byte_uart0(uint8_t inputData)
{
  if(Uart0SendBytes < UART0_SEND_BUF_SIZE - 1)
  {
    /* Enqueue the character */
    Uart0SendData[Uart0SendRtail] = inputData;
    Uart0SendRtail = (Uart0SendRtail == (UART0_SEND_BUF_SIZE - 1)) ? 0 : (Uart0SendRtail + 1);
    Uart0SendBytes++;
  }
  else
  {
    Uart0SendBytes = 0;
    Uart0SendRtail = 0;
    Uart0Sendhead = 0;
  }
}

/*******************************************************************************
* Function Name  : send_uart0_string
* Description    : send string to the uart0 buffer
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void send_uart0_string(char *str)
{
  while(*str)
  {
    write_byte_uart0(*str++);
  }
}

/*******************************************************************************
* Function Name  : uart0_printf
* Description    : send format string to the uart0 buffer
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void uart0_printf(char *fmt,...)
{
  va_list ap;
  char string[64];
  va_start(ap,fmt);
  vsprintf(string,fmt,ap);
  send_uart0_string(string);
  va_end(ap);
}

/*******************************************************************************
* Function Name  : write_byte_uart1
* Description    : write a byte to the uart1
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void write_byte_uart1(uint8_t inputData)
{
  UART_WRITE(UART1, inputData);
  UART_WAIT_TX_EMPTY(UART1);
}

/*******************************************************************************
* Function Name  : send_uart1_string
* Description    : send string to the uart1 buffer
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void send_uart1_string(char *str)
{
  while(*str)
  {
    write_byte_uart1(*str++);
  }
}

/*******************************************************************************
* Function Name  : uart1_printf
* Description    : send format string to the uart1 buffer
* Input          : ....
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void uart1_printf(char *fmt,...)
{
  va_list ap;
  char string[64];
  va_start(ap,fmt);
  vsprintf(string,fmt,ap);
  send_uart1_string(string);
  va_end(ap);
}

void check_digital_report(void)
{

}

void check_analog_report(void)
{

}

void send_sensor_report(void)
{
  if(report_mode != 0x00)
  {
    switch(device_type)
    {
      case SMART_SERVO:
      {
        if(((report_smart_pos_mode == 0x01) &&
            (abs_user(pre_smart_servo_pos_val - g_servo_info.cur_pos/10) > 5)) ||
            (report_smart_pos_mode == 0x02))
        {
          pre_smart_servo_pos_val = g_servo_info.cur_pos/10;
          send_smart_pos(g_servo_info.cur_pos/10);
        }

        if(((report_smart_angle_mode == 0x01) &&
            (abs_user(pre_smart_servo_angle_val - g_servo_info.cur_pos/10) > 0)) ||
            (report_smart_angle_mode == 0x02))
        {
          pre_smart_servo_angle_val = g_servo_info.cur_pos/10;
          send_smart_angle(g_servo_info.cur_pos/10);
        }

        if(((report_smart_temp_mode == 0x01) &&
            (abs_user(pre_smart_servo_temp_val - g_servo_info.temperature) > 5)) ||
            (report_smart_temp_mode == 0x02))
        {
          pre_smart_servo_temp_val = g_servo_info.temperature;
          send_smart_temp(g_servo_info.temperature);
        }

        if(((report_smart_speed_mode == 0x01) &&
            (abs_user(pre_smart_servo_speed_val - g_servo_info.cur_speed) > 2)) ||
            (report_smart_speed_mode == 0x02))
        {
          pre_smart_servo_speed_val = g_servo_info.cur_speed;
          send_smart_speed(g_servo_info.cur_speed);
        }

        if(((report_smart_voltage_mode == 0x01) &&
            (abs_user(pre_smart_servo_voltage_val - g_servo_info.voltage) > 10)) ||
            (report_smart_voltage_mode == 0x02))
        {
          pre_smart_servo_voltage_val = g_servo_info.voltage;
          send_smart_voltage(g_servo_info.voltage);
        }

        if(((report_smart_current_mode == 0x01) &&
            (abs_user(pre_smart_servo_current_val - g_servo_info.current) > 10)) ||
            (report_smart_current_mode == 0x02))
        {
          pre_smart_servo_current_val = g_servo_info.current;
          send_smart_current(g_servo_info.current);
        }

        break;
      }
      default:
        break;
    }
  }
}

float K_P  =  1.0;
float varP = 0.0004;
float varM = 0.25;
float K = 1.0;
float Kalmanq = 0.0;

int Kalman(float inByte)
{
  K_P = K_P+ varP;
  K = K_P/ (K_P+ varM);
  Kalmanq = K * inByte + (1 - K) * Kalmanq;
  K_P = (1 - K) * K_P;
  return (int)Kalmanq;
}

static void cmd_assign_dev_id(char *cmd)
{
  char * tmp;
  char * str;
  int8_t dev_id;
  str = strtok_r(cmd, " ", &tmp);
  if(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
  }
  if(str[0]=='D')
  {
    dev_id = atoi(str+1);
  }
  device_id = dev_id + 1;
  uart0_printf("G%d M10 T%d\r\n",device_id,device_type);

  uart_printf(UART1,"G%d M10 D%d\r\n",ALL_DEVICE,device_id);
}

static void cmd_reset_smart_servo(char *cmd)
{
  command_mode = flash_read_data_mode();
  command_mode = command_mode & 0x01;
  mVersion[4] = command_mode+1+'0';
  servodriver_run_idle();
  device_id = 0;
  device_type = SMART_SERVO;
  uart_printf(UART0,"V%s\r\n",mVersion);
  //uart_printf(UART0,"V%s\r\n",SmartServo_Version);
  // uart_printf(UART0,"Angle: %d\r\n",round((smart_servo_cur_pos * 360.0)/4096.0));
}

static void cmd_get_smart_version(char *cmd)
{
  uart0_printf("G%d M12 V%s\r\n",device_id,mVersion);
  //uart0_printf("G%d M12 V%s\r\n",device_id,SmartServo_Version);
}

static void cmd_set_command_mode(char *cmd)
{
  char * tmp;
  char * str;
  uint8_t cmd_mode;
  str = strtok_r(cmd, " ", &tmp);
  if(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
  }
  if(str[0]=='C')
  {
    cmd_mode = atoi(str+1);
    cmd_mode = (cmd_mode - 1) & 0x01;
    if(flash_write_data_mode((uint32_t)cmd_mode) == SPI_FLASH_RESULT_OK)
    {
      SendErrorUart0(PROCESS_SUC);
    }
    else
    {
      SendErrorUart0(PROCESS_ERROR);
    }
  }
  else
  {
    SendErrorUart0(WRONG_INPUT_DATA);
  }
}


static void cmd_shake_hand(char *cmd)
{
  shake_hand_flag = true;
  SendErrorUart0(PROCESS_SUC);
}

static void cmd_absolute_pos(char *cmd)
{
  char * tmp;
  char * str;
  int16_t angle;

  float speed = MAX_TAR_SPEED;

  str = strtok_r(cmd, " ", &tmp);

  while(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='P')
    {
      angle = atoi(str+1);
    }
    else if(str[0]=='S')
    {
      speed = atof(str+1);
    }
  }
  speed = constrain(speed,-MAX_TAR_SPEED,MAX_TAR_SPEED);
  servodriver_run_abs_pos((long)angle,speed);
  SendErrorUart0(PROCESS_SUC);
}

static void cmd_relative_pos(char *cmd)
{
  char * tmp;
  char * str;
  int16_t angle;
  float speed = MAX_TAR_SPEED;
  str = strtok_r(cmd, " ", &tmp);
  while(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='P')
    {
      angle = atoi(str+1);
    }
    else if(str[0]=='S')
    {
      speed = atof(str+1);
    }
  }
  speed = constrain(speed,-MAX_TAR_SPEED,MAX_TAR_SPEED);
  servodriver_run_relative_pos((long)angle,speed);
  SendErrorUart0(PROCESS_SUC);
}

static void cmd_absolute_angle(char *cmd)
{
  char * tmp;
  char * str;
  int16_t angle_raw = g_servo_info.cur_pos/10;
  double angle;
  float speed = MAX_TAR_SPEED;

  str = strtok_r(cmd, " ", &tmp);

  while(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='A')
    {
      angle_raw = atoi(str+1);
    }
    else if(str[0]=='S')
    {
      speed = atof(str+1);
    }
  }

  angle = angle_raw;// round((RAW_ANGLE_MAX_INT * angle_raw)/360.0);
  speed = constrain(speed,-MAX_TAR_SPEED,MAX_TAR_SPEED);
  servodriver_run_abs_pos((long)angle,speed);
  SendErrorUart0(PROCESS_SUC);
}

static void cmd_relative_angle(char *cmd)
{
  char * tmp;
  char * str;
  int16_t angle_raw = g_servo_info.cur_pos/10;
  double angle;
  float speed = MAX_TAR_SPEED;

  str = strtok_r(cmd, " ", &tmp);
  while(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='A')
    {
      angle_raw = atoi(str+1);
    }
    else if(str[0]=='S')
    {
      speed = atof(str+1);
    }
  }
  angle = angle_raw;//round((RAW_ANGLE_MAX_INT * angle_raw)/360.0);
  speed = constrain(speed,-MAX_TAR_SPEED,MAX_TAR_SPEED);
  servodriver_run_relative_pos((long)angle,speed);
  SendErrorUart0(PROCESS_SUC);
}

static void cmd_absolute_angle_long(char *cmd)
{
  char * tmp;
  char * str;
  long angle_raw = g_servo_info.cur_pos/10;
  double angle;
  float speed = MAX_TAR_SPEED;

  str = strtok_r(cmd, " ", &tmp);

  while(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='A')
    {
      angle_raw = atol(str+1);
    }
    else if(str[0]=='S')
    {
      speed = atof(str+1);
    }
  }
  angle = angle_raw;//round((RAW_ANGLE_MAX_INT * angle_raw)/360.0);
  speed = constrain(speed,-MAX_TAR_SPEED,MAX_TAR_SPEED);
  servodriver_run_abs_pos((long)angle,speed);
  SendErrorUart0(PROCESS_SUC);
}

static void cmd_relative_angle_long(char *cmd)
{
  char * tmp;
  char * str;
  long angle_raw = g_servo_info.cur_pos/10;
  double angle;
  float speed = MAX_TAR_SPEED;

  str = strtok_r(cmd, " ", &tmp);
  while(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='A')
    {
      angle = atol(str+1);
    }
    else if(str[0]=='S')
    {
      speed = atof(str+1);
    }
  }
  angle = angle_raw;//round((RAW_ANGLE_MAX_INT * angle_raw)/360.0);
  speed = constrain(speed,-MAX_TAR_SPEED,MAX_TAR_SPEED);
  servodriver_run_relative_pos((long)angle,speed);
  SendErrorUart0(PROCESS_SUC);
}

static void cmd_set_pwm_motion(char *cmd)
{
  char * tmp;
  char * str;
  int16_t speed = 0;

  str = strtok_r(cmd, " ", &tmp);
  while(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='S')
    {
      speed = atoi(str+1);
    }
  }

  speed = constrain(speed,-255,255);
  speed = map(speed, 0, 255, 0,MAX_OUTPUT_PWM);
  servodriver_run_pwm(speed);
  SendErrorUart0(PROCESS_SUC);
}

static void cmd_set_current_angle_zero_degrees(char *cmd)
{
  if(servodet_set_angle_zero() == SPI_FLASH_RESULT_OK)
  {
    SendErrorUart0(PROCESS_SUC);
  }
  else
  {
    SendErrorUart0(PROCESS_ERROR);
  }
}

static void cmd_set_servo_break(char *cmd)
{
  char * tmp;
  char * str;
  uint8_t break_value;
  str = strtok_r(cmd, " ", &tmp);
  if(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
  }
  if(str[0]=='B')
  {
    break_value = atoi(str+1);
  }

	if(break_value)
	{
		servodriver_run_idle();
	}
	else
	{
		servodriver_run_relative_pos(0,5);
	}
  SendErrorUart0(PROCESS_SUC);
}

static void cmd_set_rgb_led(char *cmd)
{
  char * tmp;
  char * str;
  uint8_t r_value;
  uint8_t g_value;
  uint8_t b_value;
  str = strtok_r(cmd, " ", &tmp);
  while(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='R')
    {
      r_value = atoi(str+1);
    }
    else if(str[0]=='G')
    {
      g_value = atoi(str+1);
    }
    else if(str[0]=='B')
    {
      b_value = atoi(str+1);
    }
  }
  user_set_rgb(r_value,g_value,b_value);
  SendErrorUart0(PROCESS_SUC);
}

static void cmd_get_servo_pos(char *cmd)
{
  uart0_printf("G%d M5 P%d\r\n",device_id,(int32_t)(g_servo_info.cur_pos/10));
}

static void cmd_get_servo_speed(char *cmd)
{
  uart0_printf("G%d M6 S%.2f\r\n",device_id,(float)g_servo_info.cur_speed);
}

static void cmd_get_servo_temperature(char *cmd)
{
  uart0_printf("G%d M7 T%.2f\r\n",device_id,(float)g_servo_info.temperature);
}

static void cmd_get_servo_current(char *cmd)
{
  uart0_printf("G%d M8 I%.2f\r\n",device_id,(float)g_servo_info.current);
}

static void cmd_get_servo_voltage(char *cmd)
{
  uart0_printf("G%d M9 V%.2f\r\n",device_id,(float)(g_servo_info.voltage/1000.0));
}

static void cmd_get_servo_angle(char *cmd)
{
  uart0_printf("G%d M36 A%ld\r\n",device_id,(int32_t)(round((g_servo_info.cur_pos/10.0 * 360.0)/4096.0)));
}

static void   cmd_back_zero(char *cmd)
{
  char * tmp;
  char * str;
//  uint8_t init_mode;
  float speed = MAX_TAR_SPEED;
  str = strtok_r(cmd, " ", &tmp);
  while(str != NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='C')
    {
  //    init_mode = atoi(str+1);
    }
    else if(str[0]=='S')
    {
      speed = atof(str+1);
    }
  }

  if((speed > MAX_TAR_SPEED) || (speed < -MAX_TAR_SPEED))
  {
    speed = (float)MAX_TAR_SPEED;
  }

  servodriver_run_abs_pos(0,speed);
  SendErrorUart0(PROCESS_SUC);
}

static void parse_mcode(char *cmd)
{
  uint8_t code;

  code = atoi((const char *)cmd+1);

  switch(code)
  {
    case 1:
      cmd_absolute_pos(cmd);
      break;
    case 2:
      cmd_relative_pos(cmd);
      break;
    case 3:
      cmd_set_servo_break(cmd);
      break;
    case 4:
      cmd_set_rgb_led(cmd);
      break;
    case 5:
      cmd_get_servo_pos(cmd);
      break;
    case 6:
      cmd_get_servo_speed(cmd);
      break;
    case 7:
      cmd_get_servo_temperature(cmd);
      break;
    case 8:
      cmd_get_servo_current(cmd);
      break;
    case 9:
      cmd_get_servo_voltage(cmd);
      break;
    case 10:
      cmd_assign_dev_id(cmd);
      break;
    case 11:
      cmd_reset_smart_servo(cmd);
      break;
    case 12:
      cmd_get_smart_version(cmd);
      break;
    case 19:
      cmd_set_command_mode(cmd);
      break;
    case 20:
      cmd_shake_hand(cmd);
      break;
    case 30:
      cmd_set_current_angle_zero_degrees(cmd);
      break;
    case 31:
      cmd_absolute_angle(cmd);
      break;
    case 32:
      cmd_relative_angle(cmd);
      break;
    case 33:
      cmd_absolute_angle_long(cmd);
      break;
    case 34:
      cmd_relative_angle_long(cmd);
      break;
    case 35:
      cmd_set_pwm_motion(cmd);
      break;
    case 36:
      cmd_get_servo_angle(cmd);
      break;
    case 37:
      cmd_back_zero(cmd);
      break;
    // case 40:
    //   cmd_back_zero(cmd);
    //   break;
    default:
      break;
  }
}

void parse_gcommand(char *cmd)
{
  uint8_t devid_temp;
  char srv_type = 0;
  devid_temp = atoi((const char *)cmd);

  if(parsingGcodeComplete == false)
  {
    SendErrorUart0(PROCESS_BUSY);
    parsingGcodeComplete = true;
    return;
  }
  parsingGcodeComplete = false;

  if((devid_temp == ALL_DEVICE) || (devid_temp == device_id))
  {
    int p  = strcspn(cmd, "M");
    if(p)
    {
      srv_type = atoi(sysex.gcode_buffer + p + 1);
      if((devid_temp == ALL_DEVICE) && (srv_type != 10))
      {
        write_byte_uart1('G');
        flush_uart0_forward_buffer();
        write_byte_uart1('\n');
      }
      parse_mcode(sysex.gcode_buffer + p);
    }
    else
    {
      SendErrorUart0(WRONG_TYPE_OF_SERVICE);
    }
  }
  else
  {
    write_byte_uart1('G');
    flush_uart0_forward_buffer();
    write_byte_uart1('\n');
  }
  parsingGcodeComplete = true;
}

/**
*@ Description:check the checksum of input data
* @param null
* @param null
* @return success: 1, fail: 0
*/
static uint8_t check_sysex_message(void)
{
  uint8_t read_check_sum = 0;
  uint8_t cal_check_sum = 0;
  uint8_t i = 0;

  read_check_sum = sysex.storedInputData[sysexBytesRead - 1];
  for(i=0; i<sysexBytesRead - 1; i++)
  {
    cal_check_sum += sysex.storedInputData[i];
  }
  cal_check_sum &= 0x7f;

  if(read_check_sum == cal_check_sum)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
*@ Description:check whether smart servo be bootloader state
* @param null
* @param null
* @return success: 1, fail: 0
*/
void check_device_state(void *arg)
{
  volatile uint8_t checksum;
	
  //response mesaage to UART0
  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id);
	write_byte_uart0(CTL_GENERAL);
  write_byte_uart0(CTL_CHECK_DEVICE_STATE);
  write_byte_uart0(0x00);
  checksum = (device_id + CTL_GENERAL + CTL_CHECK_DEVICE_STATE + 0x00) & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);
}

void check_whether_reach_the_postion(void)
{
	uint8_t checksum;
	uint8_t reach_pos_flag = 1;

	//if(!g_servo_info.reach_tar_pos)
	{
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
}


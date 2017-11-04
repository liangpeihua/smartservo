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
#include "M051Series.h"
#include "Interrupt.h"
#include "uart.h"
#include <stdarg.h>

#define DEFAULT_UART_BUF_SIZE      256//128
#define UART0_SEND_BUF_SIZE        128
#define UART1_REV_BUF_SIZE         512  //uart1 just for test
#define UART0_REV_BUF_SIZE         256

#define FILE_PACK_SIZE     128
#define FILE_UNPACK_SIZE   64
#define MAX_FILE_SIZE      0xD800

#define FIRMWARE_SIZE_STORE_ADDRESS      0xF800          //APROM最后的2K空间保存固件信息和固件信息的备份
#define FIRMWARE_CRC32_STORE_ADDRESS    FIRMWARE_SIZE_STORE_ADDRESS+0x200  //下一页存储crc32值  0xFA00
#define FIRMWARE_SIZE_STORE_DUP_ADDRESS      FIRMWARE_CRC32_STORE_ADDRESS+0x200         //下一页存储存储固件长度的副本 0xFC00
#define FIRMWARE_CRC32_STORE_DUP_ADDRESS    FIRMWARE_SIZE_STORE_DUP_ADDRESS+0x200  //下一页存储存储crc32值的副本  0xFE00

#define MAIN_AREA                   0
#define DUP_AREA                    1
#define AP_START_ADDRESS            FMC_APROM_BASE+0x2000           //the range of AP is 0x2000 - 0xF800
#define AP_END_ADDRESS              AP_START_ADDRESS+0xD800    //the size of AP is 0xD800   54K

#define ALL_DEVICE              0xff

#define SMART_SERVO             0x60
/* Device control&management command */ 
#define CTL_ASSIGN_DEV_ID       0x10 // Assignment device ID
#define CTL_SYSTEM_RESET        0x11 // reset from host
#define CTL_READ_DEV_VERSION    0x12 // read the firmware version
#define CTL_SET_BAUD_RATE       0x13 // Set the bandrate
#define CTL_CMD_TEST            0x14 // Just for test
#define CTL_ERROR_CODE          0x15 // error code

/* general command for module */
#define CTL_GENERAL            0x61

//#define CTL_SET_FEEDBACK                 0x01 // set feedback.
//#define CTL_SET_RGB_LED                  0x02
//#define CTL_FIND_BLOCK                   0x03
#define CTL_UPDATE_FIRMWARE              0x05 // update firmware
//#define CTL_READ_HARDWARE_ID             0x06

#define CTL_TRANSFER_FILE   0x07

/* report error code */
#define PROCESS_SUC             0x0F
#define PROCESS_BUSY            0x10
#define PROCESS_ERROR           0x11
#define WRONG_TYPE_OF_SERVICE   0x12
#define WRONG_INPUT_DATA        0x13

#define START_SYSEX             0xF0 // start a MIDI Sysex message
// #define VERSION_READ_SYSEX      0xFF // read version Sysex message
#define END_SYSEX               0xF7 // end a MIDI Sysex message

volatile uint8_t device_id = 0xff;
volatile uint8_t device_type = SMART_SERVO;

static volatile uint32_t s_to_send_bytes_count = 0;
static volatile uint16_t s_read_bytes_count = 0;
__align(4) uint8_t  file_unpack_buffer[FILE_UNPACK_SIZE] = {0};
// __align(4) uint8_t  file_unpack_buffer[FILE_UNPACK_SIZE] = {0x78,0x05,0x00,0x20,0xB5,0x01,0x00,0x00,0xD5,0x01,0x00,0x00,0xD7,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD9,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,0x01,0x00,0x00,0xDD,0x01,0x00,0x00};
// __align(4) uint8_t  verify_flash_buffer[FILE_UNPACK_SIZE] = {0};
__align(4) static uint8_t aprom_buf[FMC_FLASH_PAGE_SIZE];

volatile uint16_t Uart0Revhead  = 0;
volatile uint16_t Uart0RevRtail  = 0;
volatile uint16_t Uart1Revhead  = 0;
volatile uint16_t Uart1RevRtail  = 0;

volatile uint8_t Uart0RecData[UART0_REV_BUF_SIZE]={0};
volatile uint8_t Uart1RecData[UART1_REV_BUF_SIZE]={0};

volatile boolean parsingSysex = false;
volatile uint16_t sysexBytesRead = 0 ;
volatile int16_t ForwardBytesRead = 0;
volatile int16_t InputBytesRead = 0;

// uint8_t response_buff[3] = {4,5,6};

static volatile uint8_t s_check_sum = 0x00;
static volatile uint8_t s_update_result = 0x00;

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

typedef struct{
  uint8_t dev_id;
  uint8_t srv_id;
  uint8_t value[DEFAULT_UART_BUF_SIZE - 2];
}sysex_message_type;

union sysex_message{
  uint8_t storedInputData[DEFAULT_UART_BUF_SIZE];
  sysex_message_type val;
};

union sysex_message sysex = {0};
union sysex_message sysex_to_send = {0};

struct firmware_info{
    uint32_t firmware_size;
    uint32_t crc_value;
};

typedef void (FUNC_PTR)(void);
void JumpToApp(uint32_t appaddr);
void parse_uart0_recv_buffer_for_update_firmware(void);
void transfer_file_response(void);
void processSysexMessage(void);
void firmware_info_store(struct firmware_info* info, uint8_t area);
void firmware_info_read(struct firmware_info* info, uint8_t area);
int8_t check_firmware(void);
void transfer_file_feedback(uint16_t pack_no, uint8_t ret);
void write_byte_uart0(uint8_t inputData);
void write_byte_uart1(uint8_t inputData);
void flush_uart0_forward_buffer(void);
int8_t WriteData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *u32Data);
int8_t ReadData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *u32Data);
int8_t  VerifyData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *u32Data);
int8_t EraseFlash(uint32_t u32StartAddr, uint32_t u32EndAddr);
void init_crc_table(void);
uint32_t crc32(unsigned int crc,unsigned char *buffer, unsigned int size);
uint32_t cal_crc(uint32_t start, uint32_t len);
void firmware_info_store(struct firmware_info* info, uint8_t area);
void firmware_info_read(struct firmware_info* info, uint8_t area);
void assign_dev_id_response(void);
void SendErrorUart0(uint8_t errorcode);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* main function                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    //SYS_ResetChip();

    /* Init system clock and multi-function I/O */
    SYS_Init(); //1KBytes

    /* Init UART0 for printf */         //396Bytes

    UART0_Init(115200);

    /* Init UART1 for printf */         //396Bytes
    UART1_Init(115200);

    //Set UART Configuration
    UART_SetLine_Config(UART0, 115200, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1); //204Bytes

    //Set UART Configuration
    UART_SetLine_Config(UART1, 115200, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1); //204Bytes

    // /* UART sample function */
    UART_Function_Init();   //40Bytes

    //Enable FMC ISP function
    FMC_Open();  //24Bytes

    GPIO_SetMode(P2, BIT4, GPIO_PMD_OUTPUT);//RGB display red meaning running bootloader
    GPIO_SetMode(P2, BIT5, GPIO_PMD_OUTPUT);
    GPIO_SetMode(P2, BIT6, GPIO_PMD_OUTPUT);

    // printf("ldrom\r\n");
    // UART_WAIT_TX_EMPTY(UART0);

    if(check_firmware() != 0)
    {
        printf("chip is running bootloader!\r\n");
        UART_WAIT_TX_EMPTY(UART0);

        P24 = 1;
        P25 = 1;
        P26 = 0;

        goto _ISP;
    }

    while(1)
    {
       goto _APROM;
    }


_ISP:
    while(1) {
        parse_uart0_recv_buffer_for_update_firmware();
    }

_APROM:
   JumpToApp(FMC_APROM_BASE+0x2000);
    /* Trap the CPU */
    while(1);
}

void JumpToApp(uint32_t appaddr)
{
    uint32_t JumpAddress; /*-- define the usrapp's address --*/
    FUNC_PTR* JumpToApplication; /*-- definethe function pointer which direct to usr app --*/
    SYS_UnlockReg();
    __disable_irq();
    FMC->ISPCMD |= FMC_ISPCON_ISPEN_Msk;

    FMC_SetVectorPageAddr(appaddr);
    __enable_irq();
    JumpAddress = *(__IO uint32_t*) (appaddr + 4);
    JumpToApplication = (FUNC_PTR*) JumpAddress;
    __set_MSP(*(__IO uint32_t*) appaddr); /*-- initialize theheap & stack pointer --*/
    JumpToApplication();
}

uint8_t readbyte(uint8_t *argv,int idx)
{
  uint8_t temp;
  val1byte.byteVal[0] = argv[idx] & 0x7f;
  temp = argv[idx+1] << 7;
  val1byte.byteVal[0] |= temp;
  return val1byte.charVal;
}

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

void sendShort(int val, uint8_t ignore_high)
{
    uint8_t val_7bit[3]={0};
    val2byte.shortVal = val;
    val_7bit[0] = val2byte.byteVal[0] & 0x7f;
    sysex_to_send.storedInputData[s_to_send_bytes_count++] = val_7bit[0];
    val_7bit[1] = ((val2byte.byteVal[1] << 1) | (val2byte.byteVal[0] >> 7)) & 0x7f;
    sysex_to_send.storedInputData[s_to_send_bytes_count++] = val_7bit[1];

    //Send analog can ignored high
    if(ignore_high == false)
    {
        val_7bit[2] = (val2byte.byteVal[1] >> 6) & 0x7f;
        sysex_to_send.storedInputData[s_to_send_bytes_count] = val_7bit[2];
        s_to_send_bytes_count++;
    }
}

int8_t WriteData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *u32Data)//对一段flash写入一个数组的数据  程序中是对64字节的flash写数据
{
    uint32_t u32Addr;

    FMC_EnableAPUpdate();
    for(u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        FMC_Write(u32Addr, *u32Data);
        u32Data++;
    }
    FMC_DisableAPUpdate();
    return 0;
}

int8_t ReadData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *u32Data)//对一段flash写入一个数组的数据  程序中是对64字节的flash写数据
{
    uint32_t u32Addr;

    for(u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        *u32Data = FMC_Read(u32Addr);
        u32Data++;
    }
    return 0;
}

int8_t  VerifyData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *u32Data)
{
    uint32_t    u32Addr;
    uint32_t    u32Data_temp;

    for(u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32Data_temp = FMC_Read(u32Addr);
        if(u32Data_temp != *u32Data)
        {
            // printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32Data_temp, u32Data);
            return -1;
        }
        u32Data++;
    }
    return 0;
}

int8_t EraseFlash(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t    u32Addr;
    //uint32_t    u32Data_temp;

    FMC_EnableAPUpdate();
    for(u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        FMC_Erase(u32Addr);

       // for(u32Addr = u32StartAddr; u32Addr < u32Addr+FMC_FLASH_PAGE_SIZE; u32Addr += 4)
       // {
       //     u32Data_temp = FMC_Read(u32Addr);
       //     if(u32Data_temp != 0xFFFFFFFF)
       //     {
       //         // printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32Data_temp, u32Data);
       //         return -1;
       //     }
       // }
    }
    FMC_DisableAPUpdate();
    return 0;
}

static uint32_t crc_table[256];

void init_crc_table(void)
{
    uint32_t c;
    uint32_t i, j;

    for (i = 0; i < 256; i++) {
        c = i;
        for (j = 0; j < 8; j++) {
            if (c & 1)
                c = 0xedb88320L ^ (c >> 1);
            else
                c = c >> 1;
        }
        crc_table[i] = c;
    }
}

uint32_t crc32(unsigned int crc,unsigned char *buffer, unsigned int size)
{
    unsigned int i;
    for (i = 0; i < size; i++) {
        crc = crc_table[(crc ^ buffer[i]) & 0xff] ^ (crc >> 8);
    }
    return crc ;
}



uint32_t cal_crc(uint32_t start, uint32_t len)  //有bug，当flash不够512的整数倍时，最后的数组buff会没填满，crc值会把buff的默认值ff加进去
{
   int i;
   uint32_t crc = 0xffffffff;

   init_crc_table();

   for(i = 0; i < len; i+=FMC_FLASH_PAGE_SIZE) {
       ReadData(start + i, start + i + FMC_FLASH_PAGE_SIZE, (uint32_t*)aprom_buf);
       if(len - i >= FMC_FLASH_PAGE_SIZE)
           crc = crc32(crc, aprom_buf, FMC_FLASH_PAGE_SIZE);
       else
           crc = crc32(crc, aprom_buf, len - i);
   }

   return crc^0xffffffff;
}

void firmware_info_store(struct firmware_info* info, uint8_t area)
{
    FMC_EnableLDUpdate();
    if(area == MAIN_AREA)
    {
        FMC_Erase(FIRMWARE_SIZE_STORE_ADDRESS);
        FMC_Write(FIRMWARE_SIZE_STORE_ADDRESS, (uint32_t)info -> firmware_size);

        FMC_Erase(FIRMWARE_CRC32_STORE_ADDRESS);
        FMC_Write(FIRMWARE_CRC32_STORE_ADDRESS, (uint32_t)info -> crc_value);
    }
    else if(area == DUP_AREA)
    {
        FMC_Erase(FIRMWARE_SIZE_STORE_DUP_ADDRESS);
        FMC_Write(FIRMWARE_SIZE_STORE_DUP_ADDRESS, (uint32_t)info -> firmware_size);

        FMC_Erase(FIRMWARE_CRC32_STORE_DUP_ADDRESS);
        FMC_Write(FIRMWARE_CRC32_STORE_DUP_ADDRESS, (uint32_t)info -> crc_value);
    }
    FMC_DisableLDUpdate();
}

void firmware_info_read(struct firmware_info* info, uint8_t area)
{
    if(area == MAIN_AREA)
    {
        info -> firmware_size = FMC_Read(FIRMWARE_SIZE_STORE_ADDRESS);
        info -> crc_value = FMC_Read(FIRMWARE_CRC32_STORE_ADDRESS);
    }
    else if(area == DUP_AREA)
    {
        info -> firmware_size = FMC_Read(FIRMWARE_SIZE_STORE_DUP_ADDRESS);
        info -> crc_value = FMC_Read(FIRMWARE_CRC32_STORE_DUP_ADDRESS);
    }
}

void processSysexMessage(void)
{
    //uint8_t i;

    if(sysex.val.dev_id == ALL_DEVICE || sysex.val.dev_id == device_id)
    {
      //CTL_ASSIGN_DEV_ID should processed one by one
      if(sysex.val.srv_id == CTL_ASSIGN_DEV_ID)
      {
          assign_dev_id_response();
      }
      else
        if((sysex.val.srv_id == SMART_SERVO)&&(sysex.val.value[0] == CTL_TRANSFER_FILE))
        {
          transfer_file_response();
        }
        else
        {
          write_byte_uart1(START_SYSEX);
          flush_uart0_forward_buffer();
          write_byte_uart1(END_SYSEX);
        }
    }
    else
    {
      write_byte_uart1(START_SYSEX);
      flush_uart0_forward_buffer();
      write_byte_uart1(END_SYSEX);
    }
}

void transfer_file_response(void)
{
    static uint32_t s_start_address, s_total_len_dup, s_start_address_dup, s_read_crc_value;
    static uint16_t s_total_len;  //max flash size is 63K of mcu
    uint8_t s_package_size;
    static uint16_t package_no;
    static uint16_t s_pre_package_no = 0;
    uint32_t crc_check_value;
    struct firmware_info info;
    int16_t i = 0;

    s_read_bytes_count = 3;
    package_no = readShort(sysex.storedInputData + s_read_bytes_count, 0, true);
    // uart_printf(UART1,"no: %d",package_no);
    s_read_bytes_count += 2;

    if(package_no == 0)
    {
        s_pre_package_no = 0;
        s_total_len = readLong(sysex.storedInputData + s_read_bytes_count, 0);//aprom file size
        if(s_total_len >= MAX_FILE_SIZE)
        {
            transfer_file_feedback(package_no, false);
            return;
        }
        s_read_bytes_count += 5;
        s_read_crc_value = readLong(sysex.storedInputData + s_read_bytes_count, 0);
        s_read_bytes_count += 5;
        s_start_address = AP_START_ADDRESS;
        s_total_len_dup = s_total_len;
        s_start_address_dup = s_start_address;
        if(EraseFlash(AP_START_ADDRESS, AP_END_ADDRESS) < 0)
        {
            transfer_file_feedback(package_no, false);//erase flash failed
            return;
        }
        transfer_file_feedback(package_no, true);//erase flash succeed
    }
    else
    {
        if(package_no != (s_pre_package_no + 1))
        {
            // TODO: wrong package no.
            transfer_file_feedback(package_no, false);
            return;
        }
        s_package_size = readShort(sysex.storedInputData + s_read_bytes_count, 0, true);
        s_pre_package_no = package_no;

        s_read_bytes_count += 2;
        s_total_len -= s_package_size/2;
        // uart_printf(UART1,"NO: %d ",package_no);
        for(i = 0; i < s_package_size/2; i++)
        {
            file_unpack_buffer[i] = readbyte(sysex.storedInputData + s_read_bytes_count, 0);
            s_read_bytes_count += 2;
        }
        if(WriteData(s_start_address, s_start_address + s_package_size/2, (uint32_t*)(file_unpack_buffer)) < 0)//write a page
        {
            transfer_file_feedback(package_no, false);
            return;
        }

        s_start_address += s_package_size/2;

        if(s_total_len == 0)
        {
            crc_check_value = cal_crc(s_start_address_dup, s_total_len_dup);
            if(s_read_crc_value == crc_check_value)
            {
                s_update_result = 1; // update successful.
                info.crc_value = s_read_crc_value;
                info.firmware_size = s_total_len_dup;
                firmware_info_store(&info, MAIN_AREA);
                firmware_info_store(&info, DUP_AREA);
                transfer_file_feedback((s_total_len_dup-1), true);//传回固件长度和升级结果

                P24 = 0;
                P25 = 1;
                P26 = 1;

                SYS_ResetChip();
            }
            else
            {
                s_update_result = 0; // update failed.
                transfer_file_feedback(package_no, false);
            }
        }
    }
    // transfer_file_feedback(package_no, true);
}

void parse_uart0_recv_buffer_for_update_firmware(void)
{
    //uint8_t byte_num;
    volatile uint8_t inputData = 0xFF;

    while(Uart0Revhead != Uart0RevRtail)
    {
      //printf("abc\n");
      inputData = Uart0RecData[Uart0Revhead];
        if(true == parsingSysex)
        {
          if (inputData == END_SYSEX)
          {
            //stop sysex byte
            parsingSysex = false;
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
      Uart0Revhead = (Uart0Revhead == (uint16_t)(UART0_REV_BUF_SIZE - 1)) ? 0 : (Uart0Revhead + 1);
      InputBytesRead--;
    }
}

void UART0_IRQHandler(void)
{
    volatile uint8_t inputData = 0xFF;
    uint32_t u32IntSts = UART0->ISR;
    if(u32IntSts & UART_ISR_RDA_INT_Msk)
    {
      /* Get all the input characters */
      while(UART_IS_RX_READY(UART0))
      {
        inputData = UART_READ(UART0);

        /* Check if buffer full */
        if(InputBytesRead < UART0_REV_BUF_SIZE)
        {
          /* Enqueue the character */
          Uart0RecData[Uart0RevRtail] = inputData;
          Uart0RevRtail = (Uart0RevRtail == (uint16_t)(UART0_REV_BUF_SIZE - 1)) ? 0 : (Uart0RevRtail + 1);
          InputBytesRead++;
        }
        else
        {
          InputBytesRead = 0;
          Uart0RevRtail = 0;
          Uart0Revhead = 0;
        }
      }
    }
}

void UART1_IRQHandler(void)
{
  volatile uint8_t inputData = 0xFF;
  uint32_t u32IntSts = UART1->ISR;
  if(u32IntSts & UART_ISR_RDA_INT_Msk)
  {
    /* Get all the input characters */
    while(UART_IS_RX_READY(UART1))
    {
      inputData = UART_READ(UART1);
      /* Check if buffer full */
      if(ForwardBytesRead < UART1_REV_BUF_SIZE)
      {
        /* Enqueue the character */
        Uart1RecData[Uart1RevRtail] = inputData;
        Uart1RevRtail = (Uart1RevRtail == (uint16_t)(UART1_REV_BUF_SIZE - 1)) ? 0 : (Uart1RevRtail + 1);
        ForwardBytesRead++;
      }
      else
      {
        ForwardBytesRead = 0;
        Uart1RevRtail = 0;
        Uart1Revhead = 0;
      }
    }
  }
}

int8_t check_firmware(void)
{
    int8_t ret = 0;
    uint32_t crc_cal_value;
    struct firmware_info info_main, info_dup;
    firmware_info_read(&info_main, MAIN_AREA);

    if((info_main.firmware_size > 0) &&(info_main.firmware_size < MAX_FILE_SIZE))//擦除之后firmware_size肯定大于MAX_FILE_SIZE
    {
        crc_cal_value = cal_crc(AP_START_ADDRESS, info_main.firmware_size);

        if(crc_cal_value == info_main.crc_value)
        {
            ret = 0;
            return ret;
        }
    }

    // read main area stored firmware info failed, read duplicate area.
    firmware_info_read(&info_dup, DUP_AREA);
    if((info_dup.firmware_size > 0) &&(info_dup.firmware_size < MAX_FILE_SIZE))
    {
        crc_cal_value = cal_crc(AP_START_ADDRESS, info_dup.firmware_size);

        if(crc_cal_value == info_dup.crc_value)
        {
            firmware_info_store(&info_dup, MAIN_AREA);
            ret = 0;
            return ret;
        }
    }
    ret = -1;
    return ret;
}

void transfer_file_feedback(uint16_t pack_no, uint8_t ret)  //
{
    int i = 0;
    s_check_sum = 0;

    s_to_send_bytes_count = 0;
    sysex_to_send.storedInputData[s_to_send_bytes_count++] = device_id;
    sysex_to_send.storedInputData[s_to_send_bytes_count++] = device_type;//srv_id
    sysex_to_send.storedInputData[s_to_send_bytes_count++] = CTL_TRANSFER_FILE;
    sysex_to_send.storedInputData[s_to_send_bytes_count++] = s_update_result;
    sendShort(pack_no, false);
    sysex_to_send.storedInputData[s_to_send_bytes_count++] = ret;


    for(i = 0; i < s_to_send_bytes_count; i++)
    {
        s_check_sum += sysex_to_send.storedInputData[i];
    }
    s_check_sum &= 0x7f;
    sysex_to_send.storedInputData[s_to_send_bytes_count++] = s_check_sum;

    // write to uart0.
    write_byte_uart0(START_SYSEX);
    for(i=0; i < s_to_send_bytes_count; i++)
    {
        write_byte_uart0(sysex_to_send.storedInputData[i]);
    }
    write_byte_uart0(END_SYSEX);
}

void write_byte_uart0(uint8_t inputData)
{
    UART_WRITE(UART0, inputData);
    UART_WAIT_TX_EMPTY(UART0);
}

void write_byte_uart1(uint8_t inputData)
{
  UART_WRITE(UART1, inputData);
  UART_WAIT_TX_EMPTY(UART1);
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

void assign_dev_id_response(void)
{
  volatile uint8_t device_id_temp1 = sysex.val.value[0];
  volatile uint8_t device_id_temp2 = sysex.val.value[1];
  volatile uint8_t checksum;
  if(((ALL_DEVICE + CTL_ASSIGN_DEV_ID + device_id_temp1) & 0x7f) != device_id_temp2)
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

void SendErrorUart0(uint8_t errorcode)
{
  uint8_t checksum;

  write_byte_uart0(START_SYSEX);
  write_byte_uart0(device_id);
  write_byte_uart0(CTL_ERROR_CODE);
  write_byte_uart0(errorcode);
  checksum = (device_id + CTL_ERROR_CODE + errorcode) & 0x7f;
  write_byte_uart0(checksum);
  write_byte_uart0(END_SYSEX);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

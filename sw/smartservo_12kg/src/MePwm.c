#include "MePwm.h"
#include "uart_printf.h"
#include "main.h"

int fq_tab[8] = {500,500,500,500,500,500,500,500};
int pw_ctl_tab[8] = {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};

static int s_max_cnt[8] = {0,0,0,0,0,0,0,0};
void pwm_writeMicroseconds(int pin,int value);

//PWM fq = 12000000 / PRESCALER / DIVIDER/ CNR0 = 12000000 / 2 / 1 / CNR0 = 20000
//CNR0 = 1250
//PWM占空比 = (CMR0+1) / (CNR0 + 1)
void pwm_init(int pin,int fq)
{
	static boolean first_run = TRUE;
	int max_cnt;
	
	fq_tab[pin-0x20] = fq;
	pw_ctl_tab[pin-0x20] = TRUE;
	s_max_cnt[pin-0x20] = SystemCoreClock/2/fq;//prescaler 2
	max_cnt = s_max_cnt[pin-0x20];

	if(first_run == TRUE)
	{
		first_run = FALSE;
		CLK->CLKSEL1 &= CLK_CLKSEL1_PWM01_S_HCLK;	
		CLK->CLKSEL2 &= CLK_CLKSEL2_PWM45_S_HCLK;
	}

	switch(pin)
	{
		case 0x20:
			CLK->CLKSEL1 |= CLK_CLKSEL1_PWM01_S_HCLK;			 //select the 50MHz HCLK clock as source
			CLK->APBCLK |= CLK_APBCLK_PWM01_EN_Msk; 			 //enable the clock
			PWMA->PCR |= PWM_PCR_CH0MOD_Msk;							 //auto reload
			PWM_SET_PRESCALER(PWMA,PWM_CH0,1); 					 //prescaler 2
			PWM_SET_DIVIDER(PWMA, PWM_CH0, PWM_CLK_DIV_1); //write the CSR to select divider
			PWMA->POE |= PWM_POE_PWM0_Msk;								 //Enable PWM Output
			SYS->P2_MFP |= SYS_MFP_P20_PWM0;							 //Set P2 multi-function pins for PWMA Channel-0
			PWMA->CNR0 = max_cnt;															 //Set PWM Timer period
			pwm_writeMicroseconds(pin, 0);//Set PWM Duty 0
			break;

		case 0x21:
			CLK->CLKSEL1 |= CLK_CLKSEL1_PWM01_S_HCLK;			 //select the 50MHz HCLK clock as source
			CLK->APBCLK |= CLK_APBCLK_PWM01_EN_Msk; 			 //enable the clock
			PWMA->PCR |= PWM_PCR_CH1MOD_Msk;							 //auto reload
			PWM_SET_PRESCALER(PWMA,PWM_CH1,1); 					 //prescaler 2
			PWM_SET_DIVIDER(PWMA, PWM_CH1, PWM_CLK_DIV_1); //write the CSR to select divider
			PWMA->POE |= PWM_POE_PWM1_Msk;								 //Enable PWM Output
			SYS->P2_MFP |= SYS_MFP_P21_PWM1;							 //Set P2 multi-function pins for PWMA Channel-0
			PWMA->CNR1 = max_cnt;															 //Set PWM Timer period
			pwm_writeMicroseconds(pin, 0);//Set PWM Duty 0
			break;

		case 0x22:
			CLK->CLKSEL1 |= CLK_CLKSEL1_PWM23_S_HCLK;			 //select the 50MHz HCLK clock as source
			CLK->APBCLK |= CLK_APBCLK_PWM23_EN_Msk; 			 //enable the clock
			PWMA->PCR |= PWM_PCR_CH2MOD_Msk;							 //auto reload
			PWM_SET_PRESCALER(PWMA,PWM_CH2,1); 					 //prescaler 2
			PWM_SET_DIVIDER(PWMA, PWM_CH2, PWM_CLK_DIV_1); //write the CSR to select divider
			PWMA->POE |= PWM_POE_PWM2_Msk;								 //Enable PWM Output
			SYS->P2_MFP |= SYS_MFP_P22_PWM2;							 //Set P2 multi-function pins for PWMA Channel-0
			PWMA->CNR2 = max_cnt;															 //Set PWM Timer period
			pwm_writeMicroseconds(pin, 0);//Set PWM Duty 0
			break;

		case 0x23:
			CLK->CLKSEL1 |= CLK_CLKSEL1_PWM23_S_HCLK;			 //select the 50MHz HCLK clock as source
			CLK->APBCLK |= CLK_APBCLK_PWM23_EN_Msk; 			 //enable the clock
			PWMA->PCR |= PWM_PCR_CH3MOD_Msk;							 //auto reload
			PWM_SET_PRESCALER(PWMA,PWM_CH3,1); 					 //prescaler 2
			PWM_SET_DIVIDER(PWMA, PWM_CH3, PWM_CLK_DIV_1); //write the CSR to select divider
			PWMA->POE |= PWM_POE_PWM3_Msk;								 //Enable PWM Output
			SYS->P2_MFP |= SYS_MFP_P23_PWM3;							 //Set P2 multi-function pins for PWMA Channel-0
			PWMA->CNR3 = max_cnt;															 //Set PWM Timer period
			pwm_writeMicroseconds(pin, 0);//Set PWM Duty 0
			break;

		case 0x24:
			CLK->CLKSEL2 |= CLK_CLKSEL2_PWM45_S_HCLK;			 //select the 50MHz HCLK clock as source
			CLK->APBCLK |= CLK_APBCLK_PWM45_EN_Msk; 			 //enable the clock
			PWMB->PCR |= PWM_PCR_CH0MOD_Msk;							 //auto reload
			PWM_SET_PRESCALER(PWMB,PWM_CH0,1); 					 //prescaler 2
			PWM_SET_DIVIDER(PWMB, PWM_CH0, PWM_CLK_DIV_1); //write the CSR to select divider
			PWMB->POE |= PWM_POE_PWM0_Msk;								 //Enable PWM Output
			SYS->P2_MFP |= SYS_MFP_P24_PWM4;							 //Set P2 multi-function pins for PWMA Channel-0
			PWMB->CNR0 = max_cnt;															 //Set PWM Timer period
			pwm_writeMicroseconds(pin, 0);//Set PWM Duty 0
			break;

		case 0x25:
			CLK->CLKSEL2 |= CLK_CLKSEL2_PWM45_S_HCLK;			 //select the 50MHz HCLK clock as source
			CLK->APBCLK |= CLK_APBCLK_PWM45_EN_Msk; 			 //enable the clock
			PWMB->PCR |= PWM_PCR_CH1MOD_Msk;							 //auto reload
			PWM_SET_PRESCALER(PWMB,PWM_CH1,1); 					 //prescaler 2
			PWM_SET_DIVIDER(PWMB, PWM_CH1, PWM_CLK_DIV_1); //write the CSR to select divider
			PWMB->POE |= PWM_POE_PWM1_Msk;								 //Enable PWM Output
			SYS->P2_MFP |= SYS_MFP_P25_PWM5;							 //Set P2 multi-function pins for PWMA Channel-0
			PWMB->CNR1 = max_cnt;															 //Set PWM Timer period
			pwm_writeMicroseconds(pin, 0);//Set PWM Duty 0
			break;

		case 0x26:
			CLK->CLKSEL2 |= CLK_CLKSEL2_PWM67_S_HCLK;			 //select the 50MHz HCLK clock as source
			CLK->APBCLK |= CLK_APBCLK_PWM67_EN_Msk; 			 //enable the clock
			PWMB->PCR |= PWM_PCR_CH2MOD_Msk;							 //auto reload
			PWM_SET_PRESCALER(PWMB,PWM_CH2,1); 					 //prescaler 2
			PWM_SET_DIVIDER(PWMB, PWM_CH2, PWM_CLK_DIV_1); //write the CSR to select divider
			PWMB->POE |= PWM_POE_PWM2_Msk;								 //Enable PWM Output
			SYS->P2_MFP |= SYS_MFP_P26_PWM6;							 //Set P2 multi-function pins for PWMA Channel-0
			PWMB->CNR2 = max_cnt;															 //Set PWM Timer period
			pwm_writeMicroseconds(pin, 0);//Set PWM Duty 0
			break;

		case 0x27:
			CLK->CLKSEL2 &= CLK_CLKSEL2_PWM67_S_HCLK;			 //select the 50MHz HCLK clock as source
			CLK->APBCLK |= CLK_APBCLK_PWM67_EN_Msk; 			 //enable the clock
			PWMB->PCR |= PWM_PCR_CH3MOD_Msk;							 //auto reload
			PWM_SET_PRESCALER(PWMB,PWM_CH3,1); 					 //prescaler 2
			PWM_SET_DIVIDER(PWMB, PWM_CH3, PWM_CLK_DIV_1); //write the CSR to select divider
			PWMB->POE |= PWM_POE_PWM3_Msk;								 //Enable PWM Output
			SYS->P2_MFP |= SYS_MFP_P27_PWM7;							 //Set P2 multi-function pins for PWMA Channel-0
			PWMB->CNR3 = max_cnt;															 //Set PWM Timer period
			pwm_writeMicroseconds(pin, 0);//Set PWM Duty 0
			break;
		default:
			break;
	}
}

void pwm_writeMicroseconds(int pin,int value)
{
  switch(pin)
  {
    case 0x20:
      PWMA->CMR0 = value;
      PWMA->PCR |= PWM_PCR_CH0EN_Msk; //Set PWM Duty
      break;
    case 0x21:
      PWMA->CMR1 = value;
      PWMA->PCR |= PWM_PCR_CH1EN_Msk; //Set PWM Duty
      break;
    case 0x22:
      PWMA->CMR2 = value;
      PWMA->PCR |= PWM_PCR_CH2EN_Msk; //Set PWM Duty
      break;
    case 0x23:
      PWMA->CMR3 = value;
      PWMA->PCR |= PWM_PCR_CH3EN_Msk; //Set PWM Duty
      break;
    case 0x24:
      PWMB->CMR0 = value;
      PWMB->PCR |= PWM_PCR_CH0EN_Msk; //Set PWM Duty
      break;
    case 0x25:
      PWMB->CMR1 = value;
      PWMB->PCR |= PWM_PCR_CH1EN_Msk; //Set PWM Duty
      break;
    case 0x26:
      PWMB->CMR2 = value;
      PWMB->PCR |= PWM_PCR_CH2EN_Msk; //Set PWM Duty
      break;
    case 0x27:
      PWMB->CMR3 = value;
      PWMB->PCR |= PWM_PCR_CH3EN_Msk; //Set PWM Duty
      break;
    default:
      break;
  }
}

void pwm_write(int pin, int value,int min,int max)
{
  int value_tmp;
  int max_cnt = s_max_cnt[pin-0x20];

	value_tmp = value;
	value_tmp = constrain(value_tmp,min,max);
	value_tmp = (int)map(value_tmp, min, max, 0, max_cnt);//value_tmp: 	motor pwm duty
	pwm_writeMicroseconds(pin,value_tmp);
}

void pwm_canceled(int pin, uint8_t state)
{
  pw_ctl_tab[pin-0x20] = FALSE;
  switch(pin)
  {
    case 0x20:
    	if(state == 0)
    	{
	      PWMA->POE &= ~PWM_POE_PWM0_Msk;                 //Disable PWM Output
	      PWMA->PCR &= ~PWM_PCR_CH0EN_Msk;
	      //digitalWrite(pin,0);
      }
      else
      {
	      PWMA->POE |= PWM_POE_PWM0_Msk;                 //Enable PWM Output
	      PWMA->PCR |= PWM_PCR_CH0EN_Msk;
      }
      break;
    case 0x21:
      if(state == 0)
    	{
	      PWMA->POE &= ~PWM_POE_PWM1_Msk;                 //Disable PWM Output
	      PWMA->PCR &= ~PWM_PCR_CH1EN_Msk;
	      //digitalWrite(pin,0);
      }
      else
      {
	      PWMA->POE |= PWM_POE_PWM1_Msk;                 //Enable PWM Output
	      PWMA->PCR |= PWM_PCR_CH1EN_Msk;
      }
      break;
    case 0x22:
      if(state == 0)
    	{
	      PWMA->POE &= ~PWM_POE_PWM2_Msk;                 //Disable PWM Output
	      PWMA->PCR &= ~PWM_PCR_CH2EN_Msk;
	      //digitalWrite(pin,0);
      }
      else
      {
	      PWMA->POE |= PWM_POE_PWM2_Msk;                 //Enable PWM Output
	      PWMA->PCR |= PWM_PCR_CH2EN_Msk;
      }
      break;
    case 0x23:
      if(state == 0)
    	{
	      PWMA->POE &= ~PWM_POE_PWM3_Msk;                 //Disable PWM Output
	      PWMA->PCR &= ~PWM_PCR_CH3EN_Msk;
	      //digitalWrite(pin,0);
      }
      else
      {
	      PWMA->POE |= PWM_POE_PWM3_Msk;                 //Enable PWM Output
	      PWMA->PCR |= PWM_PCR_CH3EN_Msk;
      }
      break;
    case 0x24:
      if(state == 0)
    	{
	      PWMB->POE &= ~PWM_POE_PWM0_Msk;                 //Disable PWM Output
	      PWMB->PCR &= ~PWM_PCR_CH0EN_Msk;
	      //digitalWrite(pin,0);
      }
      else
      {
	      PWMB->POE |= PWM_POE_PWM0_Msk;                 //Enable PWM Output
	      PWMB->PCR |= PWM_PCR_CH0EN_Msk;
      }
      break;
    case 0x25:
      if(state == 0)
    	{
	      PWMB->POE &= ~PWM_POE_PWM1_Msk;                 //Disable PWM Output
	      PWMB->PCR &= ~PWM_PCR_CH1EN_Msk;
	      //digitalWrite(pin,0);
      }
      else
      {
	      PWMB->POE |= PWM_POE_PWM1_Msk;                 //Enable PWM Output
	      PWMB->PCR |= PWM_PCR_CH1EN_Msk;
      }
      break;
    case 0x26:
      if(state == 0)
    	{
	      PWMB->POE &= ~PWM_POE_PWM2_Msk;                 //Disable PWM Output
	      PWMB->PCR &= ~PWM_PCR_CH2EN_Msk;
	      //digitalWrite(pin,0);
      }
      else
      {
	      PWMB->POE |= PWM_POE_PWM2_Msk;                 //Enable PWM Output
	      PWMB->PCR |= PWM_PCR_CH2EN_Msk;
      }
      break;
    case 0x27:
      if(state == 0)
    	{
	      PWMB->POE &= ~PWM_POE_PWM3_Msk;                 //Disable PWM Output
	      PWMB->PCR &= ~PWM_PCR_CH3EN_Msk;
	      //digitalWrite(pin,0);
      }
      else
      {
	      PWMB->POE |= PWM_POE_PWM3_Msk;                 //Enable PWM Output
	      PWMB->PCR |= PWM_PCR_CH3EN_Msk;
      }
      break;
  }
}

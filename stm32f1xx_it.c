/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "locale_ru.h"
#include <stdbool.h>
#include <stdint.h>
#include "stdio.h"
#include "BA63.h"
#include "HD44780_Driver.h"
#include "ARGB.h"
#include "stdlib.h"
#include "timers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
bool stop=0;
bool shot=0;
uint8_t speed = 0;
static bool switcher = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define d0_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define d1_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define d2_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)

#define d0_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define d1_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define d2_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)

#define d8_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define d8_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint16_t RGB[3]={0};

uint16_t shotDelay = 0;

uint8_t inputPointer=0;
uint8_t RGBpointer = 0;
uint8_t timPointer = 0;
uint8_t delayPointer = 0;
char userInputString[4] = {0x00, 0x00, 0x00, 0x00};
char timingInput[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool writeLock=1;
bool shotLock=0;
bool colorSet=0;
bool run=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

//extern VBuf;
struct dispVBuffer{
	char first[21];
	char second[21];
	bool rendered;
}VBuf;

char tmp_rgb_template[21];
char frame_cnt[21];

static char redLevel[4];
static char greenLevel[4];
static char blueLevel[4];

void delay_ms2(uint16_t ms)
{
	for(uint16_t a=0;a<ms;++a)
	{
		delay_us(1000);
	}
}

void disp_render(void)
{
	if(VBuf.rendered)	//Если уже рендерилось то выходим сразу
		return;
	BA63_SetPos(0,0);
	BA63_SendString((uint8_t*)VBuf.first);
	BA63_SetPos(0,1);
	BA63_SendString((uint8_t*)VBuf.second);
	VBuf.rendered = 1;	//Подымаем флаг "отрендерено"
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim2_ch2_ch4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */
	
  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch2_ch4);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */
	
  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
	LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	//d8_set();
	static char framChar[6];
	static uint16_t framCnt;
	HD44780_SetPos(7,1);
	sprintf(framChar, "%05d", framCnt);
	strncpy(&VBuf.second[7],framChar,5);
	VBuf.rendered=0;
	framChar[5]=0x00;
	HD44780_SendString(framChar);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HD44780_SetPos(13,1);
	VBuf.second[13]='\230';
	VBuf.second[15]=' ';
	HD44780_SendChar(0x01);
	HD44780_SendChar(0x00);	//Visual BF
	HD44780_SendChar(0x00);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	nema_start();
	//delay_ms2(500);
	++framCnt;
	shot=0;
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Stop(&htim4);
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
	//nema_stop();
	VBuf.rendered=0;
	HAL_TIM_Base_Stop(&htim6);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HD44780_SetPos(13,1);
	VBuf.second[13]='\231';
	VBuf.second[15]='\74';
	HD44780_SendChar(0x03);
	HD44780_SendChar(0x00);
	HD44780_SendChar(0x00);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	
	/*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	delay_us(16000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	delay_us(16000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	delay_us(16000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	delay_us(16000);*/
	
	d8_set();
	shot=1;
	delay_us(50000);
	delay_us(50000);
	//delay_us(50000);
	//delay_us(50000);
	d8_reset();
	//HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim4);
	/*if(shotDelay<100)
	{
		delay_ms2(100);
		d8_reset();
	}
	else
	{
		delay_ms2(shotDelay);
		d8_reset();
	}
	static char framChar[6];
	static uint16_t framCnt;
	HD44780_SetPos(7,1);
	sprintf(framChar, "%05d", framCnt);
	framChar[5]=0x00;
	HD44780_SendString(framChar);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HD44780_SetPos(13,1);
	HD44780_SendChar(0x01);
	HD44780_SendChar(0x00);	//Visual BF
	HD44780_SendChar(0x00);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	nema_start();
	//delay_ms2(500);
	++framCnt;
	HAL_TIM_Base_Start(&htim6);*/
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	
	//check and correct film tension if this need
	
	/*HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
	HAL_ADC_Stop(&hadc1);
	speed = (uint32_t)HAL_ADC_GetValue(&hadc1)/16;
	speed = 0xFF-speed;
	config_speed(speed*25,speed,speed);*/

	/*if(speed>190)
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		stop=1;
	}
	else if(speed<180&&stop)
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		stop=0;
	}
	
	disp_render();
	RGB[0]=RGB[0];
	RGB[1]=RGB[1];
	RGB[2]=RGB[2];*/
	
	//lets go scan keyb lanes 1-3
	d0_set();	//ENABLE lane 1 for scan - '3', '6', '9', '#'
	if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))
	{
		VBuf.second[19]='0';
		VBuf.rendered = 0;
		if(!writeLock)
			userInputString[RGBpointer]='0';
		if(!shotLock)
		{
			timingInput[timPointer]='0';
			timPointer++;
		}
		++RGBpointer;
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_5))
	{
		VBuf.second[19]='8';
		VBuf.rendered = 0;
		if(!writeLock)
			userInputString[RGBpointer]='8';
		if(!shotLock)
		{
			timingInput[timPointer]='8';
			timPointer++;
		}
		++RGBpointer;
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6))
	{
		VBuf.second[19]='5';
		VBuf.rendered = 0;
		if(!writeLock)
			userInputString[RGBpointer]='5';
		if(!shotLock)
		{
			timingInput[timPointer]='5';
			timPointer++;
		}
		++RGBpointer;
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_7))
	{
		VBuf.second[19]='2';
		VBuf.rendered = 0;
		if(!writeLock)
			userInputString[RGBpointer]='2';
		if(!shotLock)
		{
			timingInput[timPointer]='2';
			timPointer++;
		}
		++RGBpointer;
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}
	d0_reset();
	delay_us(10);
	d1_set(); //ENABLE lane 2 for scan - '2', '5', '8', '0'
	if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))
	{
		VBuf.second[19]='#';
		VBuf.rendered = 0;
		if(!writeLock)
		{
			RGBpointer=0;
			++inputPointer;
			if(inputPointer==3)
			{
				inputPointer=0;
				writeLock=1;
				colorSet=1;
			}
			//convert string into uint16_t
			userInputString[3]=0x00;
			RGB[2]=atoi(userInputString);
			//RGBpointer=0;
			//overflow protect
			if(RGB[2]>255)
				RGB[2]=255;
			ARGB_FillRGB(RGB[0],RGB[1],RGB[2]);
			ARGB_Show();
			memset(userInputString, 0x00, sizeof(userInputString));
		}
		/*else
		{
			inputPointer=3;
		}*/
		if(!shotLock)
		{
			htim4.Init.Period = atoi(timingInput) > 65535 ? 65535 : atoi(timingInput);
			if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
			{
				Error_Handler();
				HD44780_ClearLCD();
				HD44780_SetPos(0,0);
				HD44780_SendString("TIM4 PERIOD ERR");
			}
			shotDelay=atoi(timingInput);
			shotLock=1;
			writeLock=0;
			inputPointer=0;
			RGBpointer=0;
			HD44780_SetPos(0,1);
			HD44780_SendString("               ");
		}
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_5))
	{
		VBuf.second[19]='9';
		VBuf.rendered = 0;
		if(!writeLock)
			userInputString[RGBpointer]='9';
		if(!shotLock)
		{
			timingInput[timPointer]='9';
			timPointer++;
		}
		++RGBpointer;
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6))
	{
		VBuf.second[19]='6';
		VBuf.rendered = 0;
		if(!writeLock)
			userInputString[RGBpointer]='6';
		if(!shotLock)
		{
			timingInput[timPointer]='6';
			timPointer++;
		}
		++RGBpointer;
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_7))
	{
		VBuf.second[19]='3';
		VBuf.rendered = 0;
		if(!writeLock)
			userInputString[RGBpointer]='3';
		if(!shotLock)
		{
			timingInput[timPointer]='3';
			timPointer++;
		}
		++RGBpointer;
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}
	d1_reset();
	delay_us(10);
	d2_set();	//ENABLE lane 3 for scan - '1', '4', '7', '*'
	if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))
	{
		VBuf.second[19]='*';
		VBuf.rendered = 0;
		if(shotLock)
		{
			writeLock=0;
			inputPointer=0;
			colorSet=0;
		}
		memset(userInputString, 0x00, sizeof(userInputString));
		if(!run)
		{
			//nema_start_part(200);
		}
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_5))
	{
		VBuf.second[19]='7';
		VBuf.rendered = 0;
		if(!writeLock)
			userInputString[RGBpointer]='7';
		if(!shotLock)
		{
			timingInput[timPointer]='7';
			timPointer++;
		}
		++RGBpointer;
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6))
	{
		VBuf.second[19]='4';
		VBuf.rendered = 0;
		if(!writeLock)
			userInputString[RGBpointer]='4';
		if(!shotLock)
		{
			timingInput[timPointer]='4';
			timPointer++;
		}
		++RGBpointer;
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_7))
	{
		VBuf.second[19]='1';
		VBuf.rendered = 0;
		if(!writeLock)
			userInputString[RGBpointer]='1';
		if(!shotLock)
		{
			timingInput[timPointer]='1';
			timPointer++;
		}
		++RGBpointer;
		delay_us(65000);
		delay_us(65000);
		//delay_us(65000);
	}
	d2_reset();
	
	//out of array protect
	if(RGBpointer==3)
		RGBpointer=0;
	if(timPointer==5)
		timPointer=0;
	if(delayPointer==5)
		delayPointer=0;
	
	//for debug
	//HD44780_SetPos(12,1);
	//HD44780_SendString(userInputString);
	
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

	//emergency button
	if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)&&run)
	{
		nema_dis();
		tens_dis();
		VBuf.rendered = 0;
		strcpy(VBuf.first, ru_user_irq_first);
		strcpy(VBuf.second, ru_user_irq_second);
		disp_render();
		//HAL_TIM_Base_Stop(&htim3);
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
		HAL_TIM_Base_Stop(&htim4);
		HAL_TIM_Base_Stop(&htim6);
		HAL_TIM_Base_Stop(&htim7);
		return;
	}
	//alternative function - run
	if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)&&!run)
	{
		//HD44780_ClearLCD();
		VBuf.rendered = 0;
		strcpy(VBuf.second, frame_cnt);
		VBuf.second[12]='\241';
		VBuf.second[14]='\240';
		VBuf.second[16]='\226';
		VBuf.second[19]='\224';
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
		run=1;
		//delay_ms2(200);
		delay_us(50000);
		delay_us(50000);
		//Prog slave break-timer
		htim1.Init.Period = 3200-1;
		if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
		{
			//Exception catch
			Error_Handler();
		}
		//Enable peripheral
		HAL_TIM_Base_Start_IT(&htim1);	//Slave mode timer control count of pulses
		HAL_TIM_Base_Start_IT(&htim15);	//PWM-gen to stepper and TIM1
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
		HAL_TIM_Base_Start_IT(&htim6);	//Frame shot time
	}
	
	//visual - draw brackets around current editing field
	if(!writeLock)
	{
		switch(inputPointer)
		{
			case 0:
			{
				tmp_rgb_template[0]='[';
				tmp_rgb_template[6]=']';
				tmp_rgb_template[12]=' ';
				tmp_rgb_template[18]=' ';
				tmp_rgb_template[19]='\225';
				break;
			}
			case 1:
			{
				tmp_rgb_template[0]=' ';
				tmp_rgb_template[6]='[';
				tmp_rgb_template[12]=']';
				tmp_rgb_template[18]=' ';
				tmp_rgb_template[19]='\225';
				break;
			}
			case 2:
			{
				tmp_rgb_template[0]=' ';
				tmp_rgb_template[6]=' ';
				tmp_rgb_template[12]='[';
				tmp_rgb_template[18]=']';
				tmp_rgb_template[19]='\225';
				break;
			}
		}
	}
	else if(!shotLock)//Hide brackets if locked
	{
			tmp_rgb_template[0]='\226';
			tmp_rgb_template[6]='\226';
			tmp_rgb_template[12]='\226';
			tmp_rgb_template[18]='\226';
	}
	
	//settings timings
	if(!shotLock)
	{
		strncpy(&VBuf.second[14], timingInput, 5);
	}
	if(shotLock&&colorSet&&!run)
	{
		nema_en();
		tens_en();
		tmp_rgb_template[0]='\223';
		tmp_rgb_template[6]='\223';
		tmp_rgb_template[12]='\223';
		tmp_rgb_template[18]='\223';
		tmp_rgb_template[19]='\224';
		strcpy(VBuf.second, ru_ready);
	}
			
	VBuf.rendered=0;
	//RGB[2]=35;
	//RGB[0]=25;
	//ARGB_FillRGB(RGB[0],RGB[1],RGB[2]);
	//ARGB_Show();
	//Render values
	sprintf(redLevel, "%03d", RGB[0]);
	sprintf(greenLevel, "%03d", RGB[1]);
	sprintf(blueLevel, "%03d", RGB[2]);
	strncpy(&tmp_rgb_template[3],redLevel,3);
	strncpy(&tmp_rgb_template[9],greenLevel,3);
	strncpy(&tmp_rgb_template[15],blueLevel,3);
	strcpy(VBuf.first, tmp_rgb_template);	//Подготовка буфера
	disp_render();	//Рендерим на экран
  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
bool pausePending=0;
bool shot_delay=0;
bool cam_busy=0;
bool first_run=0;
bool govno=1;
bool govno2=0;
uint16_t govno3=0;
uint16_t speed = 0;
static bool switcher = 0;

uint16_t callback_analog = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define d0_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define d1_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define d2_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)
#define service_en() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)

#define d0_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define d1_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define d2_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)
#define service_dis() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)

#define d8_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define d9_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define d8_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define d9_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint16_t RGB[3]={0};

uint16_t shotDelay = 0;
uint16_t framCnt=0;

uint8_t inputPointer=0;
uint8_t RGBpointer = 0;
uint8_t timPointer = 0;
uint8_t delayPointer = 0;
char userInputString[4] = {0x00, 0x00, 0x00, 0x00};
char timingInput[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char adc_cnt[5]={0};
char callback_adc[3]={0};
bool writeLock=0;
bool shotLock=1;
bool colorSet=0;
bool run=0;
bool ttm=0;
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

extern bool debugMode;

//extern VBuf;
struct dispVBuffer{
	char first[21];
	char second[21];
	char Fbuffer[21];
	char Sbuffer[21];
	bool rendered;
}VBuf;

char tmp_rgb_template[21];
char frame_cnt[21];

static char redLevel[4];
static char greenLevel[4];
static char blueLevel[4];

//Функция взята с сайта eax.me
void ADC_Select_Channel(uint32_t ch) {
    ADC_ChannelConfTypeDef conf = {
        .Channel = ch,
        .Rank = 1,
        .SamplingTime = ADC_SAMPLETIME_28CYCLES_5,
    };
    if (HAL_ADC_ConfigChannel(&hadc1, &conf) != HAL_OK) {
        Error_Handler();
    }
}

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

void read_callback_sensor(void)
{
	//Обработка датчика обратной связи
	ADC_Select_Channel(ADC_CHANNEL_2);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
	HAL_ADC_Stop(&hadc1);
	callback_analog = (uint32_t)HAL_ADC_GetValue(&hadc1)/64;
	callback_analog = 0x3F-callback_analog;
	sprintf(callback_adc, "%02d", callback_analog);
	if(callback_analog>40)
		cam_busy=1;
	else
		cam_busy=0;
}

void scan_keyb(void)
{
	//lets go scan keyb lanes 1-3
	d0_set();	//ENABLE lane 1 for scan - '3', '6', '9', '#'
	if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))
	{
		if(debugMode)
		{
			VBuf.second[19]='0';
		}
		else
		{
			VBuf.rendered = 0;
			if(!writeLock)
				userInputString[RGBpointer]='0';
			++RGBpointer;
		}
		delay_us(65000);
		delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_5))
	{
		if(debugMode)
		{
			VBuf.second[19]='8';
		}
		else
		{
			VBuf.rendered = 0;
			if(!writeLock)
				userInputString[RGBpointer]='8';
			++RGBpointer;
		}
		delay_us(65000);
		delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6))
	{
		if(debugMode)
		{
			VBuf.second[19]='5';
		}
		else
		{
			VBuf.rendered = 0;
			if(!writeLock)
				userInputString[RGBpointer]='5';
			++RGBpointer;
		}
		delay_us(65000);
		delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_7))
	{
		if(debugMode)
		{
			VBuf.second[19]='2';
		}
		else
		{
			VBuf.rendered = 0;
			if(!writeLock)
				userInputString[RGBpointer]='2';
			++RGBpointer;
		}
		delay_us(65000);
		delay_us(65000);
	}
	d0_reset();
	delay_us(10);
	d1_set(); //ENABLE lane 2 for scan - '2', '5', '8', '0'
	if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))
	{
		if(debugMode)
		{
			VBuf.second[19]='#';
		}
		else
		{
			VBuf.rendered = 0;
			if(pausePending&&writeLock&&govno2)
			{
				ARGB_FillRGB(RGB[0],RGB[1],RGB[2]);
				ARGB_Show();
				govno2=0;
				return;
			}
			if(pausePending&&writeLock&&!govno2)
			{
				ARGB_FillRGB(0,0,0);
				govno2=1;
			}
			if(run&&writeLock)
			{
				pausePending=1;
			}
			if(!writeLock)
			{
				//convert string into uint16_t
				userInputString[3]=0x00;
				RGB[inputPointer]=atoi(userInputString);
				RGBpointer=0;
				//overflow protect
				if(RGB[inputPointer]>255)
					RGB[inputPointer]=255;
				++inputPointer;
				if(inputPointer==3)
				{
					inputPointer=0;
					writeLock=1;
					colorSet=1;
				}
				memset(userInputString, 0x00, sizeof(userInputString));
			}
		}
		delay_us(65000);
		delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_5))
	{
		if(debugMode)
		{
			VBuf.second[19]='9';
		}
		else
		{
			VBuf.rendered = 0;
			if(!writeLock)
				userInputString[RGBpointer]='9';
			++RGBpointer;
		}
		delay_us(65000);
		delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6))
	{
		if(debugMode)
		{
			VBuf.second[19]='6';
		}
		else
		{
			VBuf.rendered = 0;
			if(!writeLock)
				userInputString[RGBpointer]='6';
			++RGBpointer;
		}
		delay_us(65000);
		delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_7))
	{
		if(debugMode)
		{
			VBuf.second[19]='3';
		}
		else
		{
			VBuf.rendered = 0;
			if(!writeLock)
				userInputString[RGBpointer]='3';
			++RGBpointer;
		}
		delay_us(65000);
		delay_us(65000);
	}
	d1_reset();
	delay_us(10);
	d2_set();	//ENABLE lane 3 for scan - '1', '4', '7', '*'
	if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))
	{
		if(debugMode)
		{
			VBuf.second[19]='*';
		}
		else
		{
			VBuf.rendered = 0;
			if(shotLock)
			{
				writeLock=0;
				inputPointer=0;
				colorSet=0;
			}
			memset(userInputString, 0x00, sizeof(userInputString));
		}
		delay_us(65000);
		delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_5))
	{
		if(debugMode)
		{
			VBuf.second[19]='7';
		}
		else
		{
			VBuf.rendered = 0;
			if(!writeLock)
				userInputString[RGBpointer]='7';
			++RGBpointer;
		}
		delay_us(65000);
		delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6))
	{
		if(debugMode)
		{
			VBuf.second[19]='4';
		}
		else
		{
			VBuf.rendered = 0;
			if(!writeLock)
				userInputString[RGBpointer]='4';
			++RGBpointer;
		}
		delay_us(65000);
		delay_us(65000);
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_7))
	{
		if(debugMode)
		{
			VBuf.second[19]='1';
		}
		else
		{
			VBuf.rendered = 0;
			if(!writeLock)
				userInputString[RGBpointer]='1';
			++RGBpointer;
		}
		delay_us(65000);
		delay_us(65000);
	}
	d2_reset();
	//Скан сервисных клавиш
	service_en();
	if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))//4й сверху
	{
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_5))//3й сверху
	{
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6))//Корректировка кадра вправо
	{
		if(!run)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	//Промотка вправо
			htim15.Init.Prescaler = 9600-1;
			if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
			{
				Error_Handler();
			}
			nema_start_part(50000);
			htim15.Init.Prescaler = 1200-1;
			if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
			{
				Error_Handler();
			}
		}
	}else if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_7))//Корректировка кадра влево
	{
		if(!run)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	//Промотка влево
			htim15.Init.Prescaler = 9600-1;
			if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
			{
				Error_Handler();
			}
			nema_start_part(50000);
			htim15.Init.Prescaler = 1200-1;
			if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
			{
				Error_Handler();
			}
		}
	}
	service_dis();
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim2_ch2_ch4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
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
	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
	static char framChar[6];
	//static uint16_t framCnt;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	if(!first_run)
		first_run=1;
	else if(govno)
	{
		delay_ms2(50);
		ttm=0;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		d8_set();
		delay_ms2(25);
		d8_reset();
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		++framCnt;
		sprintf(framChar, "%05d", framCnt);
		strncpy(&VBuf.second[7],framChar,5);
		VBuf.second[13]='\231';
		VBuf.second[15]='\74';
		VBuf.rendered=0;
		delay_ms2(300);
		ttm=0;
		HAL_TIM_Base_Start_IT(&htim4);
		govno=0;
	}
	scan_keyb();
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */
	
	//Сюда вписать управление сенсором натяжения
	
	//check and correct film tension if this need
	ADC_Select_Channel(ADC_CHANNEL_1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
	HAL_ADC_Stop(&hadc1);
	speed = (uint32_t)HAL_ADC_GetValue(&hadc1)/4;
	speed = 0x3FF-speed;
	config_speed(speed*10,speed,speed);	
	sprintf(adc_cnt, "%04d", speed);
	if(speed>400)
		config_speed(speed*15,speed,speed);	
	if(speed>700)
		config_speed(speed*20,speed,speed);	
	if(speed>999)
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		stop=1;
	}
	else if(speed<971&&stop)
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		stop=0;
	}
	
  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	govno=1;
	
	HAL_TIM_Base_Stop_IT(&htim4);
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	/*if(pausePending)
	{
		run=0;
		TIM1->CNT = 0;
		VBuf.second[13]=' ';
		VBuf.second[15]=' ';
		strcpy(VBuf.Sbuffer, VBuf.second);
		strcpy(VBuf.second, ru_paused);
		VBuf.rendered=0;
		disp_render();
		pausePending=0;
		HAL_TIM_Base_Stop(&htim6);
	}*/
	scan_keyb();
	if(govno)
	{
		read_callback_sensor();
		if(!ttm)
		{
			if(pausePending)
			{
				run=0;
				VBuf.second[13]=' ';
				VBuf.second[15]=' ';
				strcpy(VBuf.Sbuffer, VBuf.second);
				strcpy(VBuf.second, ru_paused);
				VBuf.rendered=0;
				HAL_TIM_IRQHandler(&htim6);
				HAL_TIM_Base_Stop_IT(&htim6);
				return;
				//pausePending=0;
			}
		}
		delay_ms2(100);
		read_callback_sensor();
		if(!cam_busy)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			ttm=1;
			VBuf.second[13]='\230';
			VBuf.second[15]=' ';
			VBuf.rendered=0;
			nema_start();
		}
	}
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	if(debugMode)
		read_callback_sensor();
	
	if(!LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_9))
	{
		if(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_5))
		{
			govno3=0;
		}
		else
		{
			if(govno3>700)
			{
				char framChar[6];
				d8_reset();
				d9_reset();
				nema_dis();
				tens_dis();
				VBuf.rendered = 0;
				sprintf(framChar, "%05d", framCnt);
				strcpy(VBuf.first, ru_finish);
				strcpy(VBuf.second, frame_cnt);
				strncpy(&VBuf.second[7],framChar,5);
				//strcpy(VBuf.second, ru_user_irq_second);
				disp_render();
				
				HAL_TIM_Base_Stop(&htim4);
				HAL_TIM_Base_Stop(&htim6);
				HAL_TIM_Base_Stop(&htim7);
				HAL_TIM_Base_Stop_IT(&htim1);
				HAL_TIM_Base_Stop_IT(&htim15);
				HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
			}
			++govno3;
		}
	}
	/*
	//check and correct film tension if this need
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
	HAL_ADC_Stop(&hadc1);
	speed = (uint32_t)HAL_ADC_GetValue(&hadc1)/2;
	speed = 0x7FF-speed;
	config_speed(speed*11,5,30);	
	
	char adc_cnt[5]={0};
	sprintf(adc_cnt, "%04d", speed);
	
	if(speed>1990)
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		stop=1;
	}
	else if(speed<1960&&stop)
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		stop=0;
	}*/
	
	scan_keyb();
	
	//out of array protect
	if(RGBpointer==3)
		RGBpointer=0;
	if(timPointer==5)
		timPointer=0;
	if(delayPointer==5)
		delayPointer=0;
	
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

	if(debugMode)
	{
		
	}
	else
	{
		//emergency button
		if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)&&run)
		{
			d8_reset();
			d9_reset();
			nema_dis();
			tens_dis();
			VBuf.rendered = 0;
			strcpy(VBuf.first, ru_user_irq_first);
			strcpy(VBuf.second, ru_user_irq_second);
			disp_render();
			
			HAL_TIM_Base_Stop(&htim4);
			HAL_TIM_Base_Stop(&htim6);
			HAL_TIM_Base_Stop(&htim7);
			HAL_TIM_Base_Stop_IT(&htim1);
			HAL_TIM_Base_Stop_IT(&htim15);
			HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
			return;
		}
		//alternative function - run
		if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)&&!run)
		{
			if(govno2)
			{
				ARGB_FillRGB(RGB[0],RGB[1],RGB[2]);
				//ARGB_Show();
				govno2=0;
			}
			d8_reset();
			d9_reset();
			BA63_SetCP(866);
			//HD44780_ClearLCD();
			VBuf.rendered = 0;
			strcpy(VBuf.second, frame_cnt);
			VBuf.second[12]='\241';
			VBuf.second[14]='\240';
			VBuf.second[16]='\226';
			VBuf.second[19]='\224';
			
			first_run=0;	//Фикс мгновенного срабатывания таймера
			ttm=1;	//Если 1 то привод работает и камера не должна снимать в этот момент
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
			if(pausePending)
			{
				strcpy(VBuf.second, VBuf.Sbuffer);
			}
			pausePending=0;	//Сбрасываем флаг паузы
			//Enable peripheral
			HAL_TIM_Base_Start_IT(&htim1);	//Slave mode timer control count of pulses
			HAL_TIM_Base_Start_IT(&htim15);	//PWM-gen to stepper and TIM1
			HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);	//Запуск двигателя ЛПМ
			HAL_TIM_Base_Start_IT(&htim6);	//Frame shot time
		}
	}
	
	//visual - draw brackets around current editing field
	if(!debugMode)
	{
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
			tmp_rgb_template[19]='\224';
		}
		else if(shotLock&&run)//Edit mode
		{
			tmp_rgb_template[0]='\223';
			tmp_rgb_template[6]='\223';
			tmp_rgb_template[12]='\223';
			tmp_rgb_template[18]='\223';
			tmp_rgb_template[19]='\224';
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
			if(!pausePending)	//Если была пауза то не показываем
				strcpy(VBuf.second, ru_ready);
		}
			
		if(!run)
		{
			ARGB_Show();
		}
		if(!pausePending&&!run)
		{
			ARGB_FillRGB(RGB[0],RGB[1],RGB[2]);
		}
	}
	
	if(!debugMode)
	{
		sprintf(redLevel, "%03d", RGB[0]);
		sprintf(greenLevel, "%03d", RGB[1]);
		sprintf(blueLevel, "%03d", RGB[2]);
		strncpy(&tmp_rgb_template[3],redLevel,3);
		strncpy(&tmp_rgb_template[9],greenLevel,3);
		strncpy(&tmp_rgb_template[15],blueLevel,3);
	}
	if(debugMode)
	{
		memset(VBuf.first, ' ', 20);
		memset(VBuf.second, ' ', 20);
		strncpy(VBuf.first, ru_debug_adc, 4);
		strncpy(&VBuf.first[4], adc_cnt, 4);
		strncpy(&VBuf.first[9], ru_debug_adc, 4);
		strncpy(&VBuf.first[13], callback_adc, 2);
		VBuf.first[15]='[';
		if(cam_busy)
			VBuf.first[16]='1';
		else
			VBuf.first[16]='0';
		VBuf.first[17]=']';
		strncpy(&VBuf.second[0],ru_debug_ver,9);
		strncpy(&VBuf.second[14],ru_debug_input,5);
		VBuf.rendered=0;
	}
	else
	{
		strcpy(VBuf.first, tmp_rgb_template);	//Подготовка буфера
		//strncpy(&VBuf.second[16], adc_cnt, 4);
		//VBuf.rendered=0;
	}
	disp_render();	//Рендерим на экран
	
  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

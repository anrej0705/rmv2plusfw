#include "stm32f1xx.h"
uint16_t debud_value=0;
extern TIM_HandleTypeDef htim16;
void TIM16_usDelay_Init(void)
{
	return;
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	TIM16->PSC			= 24-1;
	TIM16->CR1			= TIM_CR1_OPM;
}
//Код взят с microtechnics.ru
void delay_us(volatile uint16_t value)
{
	debud_value = value;
	if(value==0)
		value=1;	//Защита от зависаний
	__HAL_TIM_SET_COUNTER(&htim16, 0);
  __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start(&htim16);
  while(__HAL_TIM_GET_COUNTER(&htim16) < value)
  {
    if (__HAL_TIM_GET_FLAG(&htim16, TIM_FLAG_UPDATE) != RESET)
    {
      // Error
      break;
    }
  }
	/*TIM16->ARR = value*2;
	TIM16->CNT = 0;
	TIM16->CR1 = TIM_CR1_CEN|TIM_CR1_OPM;
	while((TIM16->SR & TIM_SR_UIF)==0){}
	TIM16->SR &= ~TIM_SR_UIF;*/
}

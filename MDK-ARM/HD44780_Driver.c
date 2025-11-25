#include "stm32f1xx.h"
#include "timers.h"
#include "stm32f1xx_ll_gpio.h"

#define d0_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0)
#define d1_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_1)
#define d2_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_2)
#define d3_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_3)
#define d4_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4)
#define d5_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_5)
#define d6_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6)
#define d7_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7)
#define d8_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8)
#define d9_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9)

#define led8_set() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8)

#define d0_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0)
#define d1_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1)
#define d2_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_2)
#define d3_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3)
#define d4_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4)
#define d5_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_5)
#define d6_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6)
#define d7_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7)
#define d8_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8)
#define d9_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9)

#define led8_reset() LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8)

//###############################################
//#Pin_RW				0x08	//PD3											#
//#Pin_RS				0x04 	//PD2											#
//#Pin_EN				0x02 	//PD1											#
//#DATA_PORT		GPIOB	//GPIO Port B	PB0...PB7		#
//###############################################
void HD44780_Command(uint8_t command)
{
	//led8_set();
	delay_us(181);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
	delay_us(194);
	//small dose of HAL porn
	if(((command >> 7)&0x01)==1) {d7_set();} else {d7_reset();}
  if(((command >> 6)&0x01)==1) {d6_set();} else {d6_reset();}
  if(((command >> 5)&0x01)==1) {d5_set();} else {d5_reset();}
  if(((command >> 4)&0x01)==1) {d4_set();} else {d4_reset();}
	if(((command >> 3)&0x01)==1) {d3_set();} else {d3_reset();}
  if(((command >> 2)&0x01)==1) {d2_set();} else {d2_reset();}
  if(((command >> 1)&0x01)==1) {d1_set();} else {d1_reset();}
  if(((command >> 0)&0x01)==1) {d0_set();} else {d0_reset();}
	delay_us(56);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
	delay_us(42);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
	//led8_reset();
}
void HD44780_Data(uint8_t data)
{
	//led8_set();
	delay_us(4);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
	delay_us(2);
	//small dose of HAL porn
	if(((data >> 7)&0x01)==1) {d7_set();} else {d7_reset();}
  if(((data >> 6)&0x01)==1) {d6_set();} else {d6_reset();}
  if(((data >> 5)&0x01)==1) {d5_set();} else {d5_reset();}
  if(((data >> 4)&0x01)==1) {d4_set();} else {d4_reset();}
	if(((data >> 3)&0x01)==1) {d3_set();} else {d3_reset();}
  if(((data >> 2)&0x01)==1) {d2_set();} else {d2_reset();}
  if(((data >> 1)&0x01)==1) {d1_set();} else {d1_reset();}
  if(((data >> 0)&0x01)==1) {d0_set();} else {d0_reset();}
	delay_us(2);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
	delay_us(2);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
	//delay_us(14);
	//led8_reset();
}
void HD44780_Init(void)
{
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	delay_us(1200);
	HD44780_Command(0x3C);
	delay_us(90);
	HD44780_Command(0x08);
	delay_us(90);
	HD44780_Command(0x01);
	delay_us(90);
	HD44780_Command(0x02);
	delay_us(90);
	HD44780_Command(0x0C);
	delay_us(200);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}
void HD44780_ClearLCD(void)
{
	delay_us(65000);
	delay_us(65000);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	HD44780_Command(0x01);
	delay_us(20);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	delay_us(65000);
	delay_us(65000);
}
void HD44780_SendChar(char ch)
{
	HD44780_Data((uint8_t) ch);
	//delay_us(4);
}
void HD44780_SetPos(uint8_t x, uint8_t y)
{
	switch(y)
	{
		case 0:
			HD44780_Command(x|0x80);
			//delay_us(4);
			break;
		case 1:
			HD44780_Command((0x40+x)|0x80);
			//delay_us(4);
			break;
		case 2:
			HD44780_Command((0x14+x)|0x80);
			//delay_us(4);
			break;
		case 3:
			HD44780_Command((0x54+x)|0x80);
			//delay_us(4);
			break;
	}
}
void HD44780_SendString(char *String)
{
	uint8_t counter;
	counter=0;
	while(String[counter]!=0)
	{
		HD44780_Data(String[counter]);
		//delay_us(1);
		counter++;
	}
}
void HD44780_LoadSymbol(uint8_t number, uint8_t *source)
{
	HD44780_Command(0x40|number*8);
	//delay_us(10000);
	for(uint8_t a=0;a<8;++a)
	{
		HD44780_Data(source[a]);
		//HAL_Delay(1);
	}
}

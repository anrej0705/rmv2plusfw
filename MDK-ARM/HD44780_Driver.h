#ifndef HD44780_DRIVER_H_
#define HD44780_DRIVER_H_
void HD44780_Init(void);
void HD44780_ClearLCD(void);
void HD44780_SendChar(char ch);
void HD44780_SendString(char *String);
void HD44780_Command(uint8_t command);
void HD44780_Data(uint8_t command);
void HD44780_SetPos(uint8_t x, uint8_t y);
void HD44780_LoadSymbol(uint8_t number, uint8_t *source);
#endif

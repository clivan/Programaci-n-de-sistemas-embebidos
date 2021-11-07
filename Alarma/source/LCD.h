#include <stdint.h>

void LCD_Data(uint8_t *HIGH, uint8_t *LOW, uint8_t c);

void Write_Data(uint8_t data);

void LCD_CMD(uint8_t cmd);

void LCD_Write(uint8_t data);

void LCD_Clear();

void LCD_Return();

void LCD_Mode(uint8_t ID, uint8_t S);

void LCD_Set(uint8_t D, uint8_t C, uint8_t B);

void LCD_Cursor(uint8_t SC, uint8_t RL);

void LCD_Activate(uint8_t DL, uint8_t N, uint8_t F);

void LCD_CGRAM(uint8_t dir);

void LCD_DDRAM(uint8_t dir);

void Delay (uint32_t delay);

void Line (int i);

void DelayTPM(void);

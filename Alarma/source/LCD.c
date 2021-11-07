#include "LCD.h"
#include <stdint.h>
#include "fsl_gpio.h"
#include "fsl_tpm.h"
#include "clock_config.h"
#include "pin_mux.h"

#define DELAY 18000

void LCD_Data(uint8_t *HIGH, uint8_t *LOW, uint8_t c)
{
  *HIGH=(c & 0xF0) >> 4;
  *LOW= c & 0x0F;
}

void Write_Data(uint8_t data)
{

	for (uint8_t i=0;i<4;i++) //low
	{
		if(data & (1u<<i))
		{
			GPIO_SetPinsOutput(GPIOB, 1u<<i);
		}
		else
		{
			GPIO_ClearPinsOutput(GPIOB, 1u<<i);
		}
	}


	for (uint8_t i=0;i<4;i++) //High
	{
		if(data & (1u<<(i+4)))
		{
			GPIO_SetPinsOutput(GPIOE, 1u<<(i+20));
		}
		else
		{
			GPIO_ClearPinsOutput(GPIOE, 1u<<(i+20));
		}
	}
}

void LCD_CMD(uint8_t cmd)
{
	GPIO_ClearPinsOutput(GPIOC, 1u<<BOARD_INITPINS_LCD_RS_PIN);
	GPIO_ClearPinsOutput(GPIOC, 1u<<BOARD_INITPINS_LCD_RW_PIN);
	GPIO_SetPinsOutput(GPIOE, 1u<<BOARD_INITPINS_LCD_E_PIN);
	Write_Data(cmd);
	GPIO_ClearPinsOutput(GPIOE, 1u<<BOARD_INITPINS_LCD_E_PIN);
	Delay(DELAY);

}

void LCD_Write(uint8_t data)
{
	GPIO_SetPinsOutput(GPIOC, 1u<<BOARD_INITPINS_LCD_RS_PIN);
	GPIO_ClearPinsOutput(GPIOC, 1u<<BOARD_INITPINS_LCD_RW_PIN);
	GPIO_SetPinsOutput(GPIOE, 1u<<BOARD_INITPINS_LCD_E_PIN);
	Write_Data(data);
	GPIO_ClearPinsOutput(GPIOE, 1u<<BOARD_INITPINS_LCD_E_PIN);
	Delay(DELAY);
}

void LCD_Clear()
{
	uint8_t cmd=0x01u;
	LCD_CMD(cmd);
}

void LCD_Return()
{
	uint8_t cmd=0x02u;
	LCD_CMD(cmd);
}

void LCD_Mode(uint8_t ID, uint8_t S)
{
	uint8_t cmd=4u;
	if(ID)
		cmd |=2u;
	if(S)
		cmd |=1u;
	LCD_CMD(cmd);
}

void LCD_Set(uint8_t D, uint8_t C, uint8_t B)
{
	uint8_t cmd=8u;
	if(D)
		cmd |= 4u;
	if(C)
		cmd |= 2u;
	if(B)
		cmd |= 1u;
	LCD_CMD(cmd);
}

void LCD_Cursor(uint8_t SC, uint8_t RL)
{
	uint8_t cmd=16u;
	if(SC)
		cmd |=8u;
	if(RL)
		cmd |=4u;
	LCD_CMD(cmd);
}

void LCD_Activate(uint8_t DL, uint8_t N, uint8_t F)
{
	uint8_t cmd=32u;
	if(DL)
		cmd |=16u;
	if(N)
		cmd |=8u;
	if(F)
		cmd |=4u;
	LCD_CMD(cmd);
}

void LCD_CGRAM(uint8_t dir)
{
	uint8_t cmd = 0x40u;
	cmd |= dir & 0x3Fu;
	LCD_CMD(cmd);
}

void LCD_DDRAM(uint8_t dir)
{
	uint8_t cmd = 0x80u;
	cmd |= dir & 0x7Fu;
	LCD_CMD(cmd);
}

void Delay(uint32_t delay)
{
	for(uint32_t i=0;i<delay;i++)
		__asm("NOP");
}

void Line (int i){
	if (i==2){
	    LCD_DDRAM(40u);
	}else if(i==1){
		LCD_DDRAM(00u);
	}
}

void DelayTPM(){
	uint32_t Mask= 1u<<8u;
	uint32_t Mask_Off = Mask;

	TPM_SetTimerPeriod(TPM1, 100u);
	TPM_StartTimer(TPM1, kTPM_SystemClock);
	while(!(TPM1->STATUS & Mask)){      //Wait
	}

	if(TPM1->STATUS & Mask){
		TPM1->STATUS &=Mask_Off;
		TPM_StopTimer(TPM1);
		TPM1->CNT=0;
	}
}

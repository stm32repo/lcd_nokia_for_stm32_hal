#include "main.h"
#include "lcd.h"
#include "nokia1100_lcd_font.h"
#include <string.h>

uint8_t volatile usart_transmit_status;
//extern uint8_t nlcd_memory[NLCD_X_RES][NLCD_Y_RES/8+1]; //буффер дисплея
USART_HandleTypeDef* usart;

void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart) {
	if (husart == usart) {
		usart_transmit_status = 0;
//		CS_LCD_SET;
	}
}

void _delay_ms(uint32_t delay) {
	for(uint32_t tmp1=delay;tmp1>0;tmp1--) {
		for(uint32_t tmp2=5000;tmp2>0;tmp2--);
	}
}

void _delay_us(uint32_t delay1) {
	for(uint32_t tmp3=delay1;tmp3>0;tmp3--);
}

void lcd_init(USART_HandleTypeDef* Lusart) {
	usart = Lusart;
	CS_LCD_RESET;
	RST_LCD_RESET;
	
	_delay_ms(10);
	
	RST_LCD_SET;
	_delay_ms(10);

	CS_LCD_SET;
	_delay_ms(2);

  nlcd_SendByte2(CMD_LCD_MODE,0xE2); // *** SOFTWARE RESET

/*	nlcd_SendByte2(CMD_LCD_MODE,0x3A); // *** Use internal oscillator
	nlcd_SendByte2(CMD_LCD_MODE,0xEF); // *** FRAME FREQUENCY:
	nlcd_SendByte2(CMD_LCD_MODE,0x04); // *** 80Hz
	nlcd_SendByte2(CMD_LCD_MODE,0xD0); // *** 1:65 divider

	nlcd_SendByte2(CMD_LCD_MODE,0x20); // Запись в регистр Vop
	nlcd_SendByte2(CMD_LCD_MODE,0x90);

	nlcd_SendByte2(CMD_LCD_MODE,0xA4); // all on/normal display

	nlcd_SendByte2(CMD_LCD_MODE,0x2F); // Power control set(charge pump on/off)

	nlcd_SendByte2(CMD_LCD_MODE,0x40); // set start row address = 0
	nlcd_SendByte2(CMD_LCD_MODE,0xB0); // установить Y-адрес = 0
	nlcd_SendByte2(CMD_LCD_MODE,0x10); // установить X-адрес, старшие 3 бита
	nlcd_SendByte2(CMD_LCD_MODE,0x0);  // установить X-адрес, младшие 4 бита

	//nlcd_SendByte2(CMD_LCD_MODE,0xC8); // mirror Y axis (about X axis)
	nlcd_SendByte2(CMD_LCD_MODE,0xA1); // Инвертировать экран по горизонтали
	//nlcd_SendByte2(CMD_LCD_MODE,0xAD);
	nlcd_SendByte2(CMD_LCD_MODE,0xCF);

	nlcd_SendByte2(CMD_LCD_MODE,0xAC); // set initial row (R0) of the display
	nlcd_SendByte2(CMD_LCD_MODE,0x07);
	nlcd_SendByte2(CMD_LCD_MODE,0xAF); // экран вкл/выкл
*/	
	
	nlcd_SendByte2(CMD_LCD_MODE,0x2F);
	nlcd_SendByte2(CMD_LCD_MODE,0x38);
	nlcd_SendByte2(CMD_LCD_MODE,0xA1); // invert X
	nlcd_SendByte2(CMD_LCD_MODE,0xC8); // invert Y
	nlcd_SendByte2(CMD_LCD_MODE,0xA6);
	nlcd_SendByte2(CMD_LCD_MODE,0x90);
	nlcd_SendByte2(CMD_LCD_MODE,0xEC);
	nlcd_SendByte2(CMD_LCD_MODE,0xAF);
	nlcd_SendByte2(CMD_LCD_MODE,0xA4);
	
	_delay_ms(10);
	CS_LCD_SET;
	
	nlcd_Clear();
}

void nlcd_Clear(void)
{
	nlcd_SendByte2(CMD_LCD_MODE,0x40);
	nlcd_SendByte2(CMD_LCD_MODE,0xB0);
	nlcd_SendByte2(CMD_LCD_MODE,0x10);
	nlcd_SendByte2(CMD_LCD_MODE,0x00);	
	for(uint16_t tmp=0;tmp<864;tmp++) nlcd_SendByte2(DATA_LCD_MODE,0x10);
}

/*-------------------------- GPIO ----------------------------------*/
void nlcd_SendByte1(char mode, uint8_t c) {
	CS_LCD_RESET;
	_delay_us(10);
	SCLK_LCD_RESET;
	
	if (mode) SDA_LCD_SET;
		else SDA_LCD_RESET;
	
	SCLK_LCD_SET;
	_delay_us(NLCD_MIN_DELAY);
	
	for(uint8_t i=0;i<8;i++)
    {
    	SCLK_LCD_RESET;

        if(c & 0x80) SDA_LCD_SET;
         else	     SDA_LCD_RESET;

        SCLK_LCD_SET;
        c <<= 1;

		_delay_us(NLCD_MIN_DELAY);	// *****!!!!! 34 - Минимальная задержка, при которой работает мой LCD-контроллер
    }	
    CS_LCD_SET;
}
/*-------------------------- USART ----------------------------------*/
void nlcd_SendByte2(char mode, uint8_t c) {
	uint8_t tmp[2];
	uint32_t tmp32;
	tmp32 = c;
	if(mode) tmp32 |= 0x00000100;
	tmp32 = __RBIT(tmp32) >> 23;  // инвертировать порядок бит для usart
	tmp[0] = (uint8_t)tmp32;
  tmp[1] = *(((uint8_t*)&tmp32)+1);
	CS_LCD_RESET;
	_delay_us(10);
			while(usart_transmit_status == 1);
			usart_transmit_status = 1;
			HAL_USART_Transmit_IT(usart, tmp, 1);
	_delay_us(30);
//	CS_LCD_SET;
//	_delay_ms(3);
}

/* обновить видеобуфер */
void nlcd_update(uint8_t (*nlcd_mem)[NLCD_Y_RES/8+1]) {
	uint8_t x, y;
	nlcd_SendByte2(CMD_LCD_MODE,0x40); // Y = 0
	nlcd_SendByte2(CMD_LCD_MODE,0xB0);
	nlcd_SendByte2(CMD_LCD_MODE,0x10); // X = 0
	nlcd_SendByte2(CMD_LCD_MODE,0x00);
	for(y=0;y<(NLCD_Y_RES/8+1);y++) {
		for(x=0;x<NLCD_X_RES;x++) {
			nlcd_SendByte2(DATA_LCD_MODE,nlcd_mem[x][y]);
		}
	}
}

void nlcd_Putc(uint8_t s, uint8_t xcurr, uint8_t ycurr, uint8_t (*nlcd_mem)[NLCD_Y_RES/8+1]) {
	// out of diapozon lcd
	if ((xcurr >= (NLCD_X_RES-FONT_SIZE_X)) || (ycurr >= (NLCD_Y_RES/8))) return;
	for (uint8_t k = 0; k < FONT_SIZE_X; k++ )
		 nlcd_mem[k+xcurr][ycurr]=nlcd_Font8[s][k];
}

void nlcd_Puts(uint8_t *s, uint8_t size_str, uint8_t xcurr, uint8_t ycurr, uint8_t (*nlcd_mem)[NLCD_Y_RES/8+1]) {
	for (uint8_t l=0; l < size_str; l++) {
		if (xcurr >= (NLCD_X_RES - FONT_SIZE_X)) {
			xcurr = 0;
			if (ycurr >= (NLCD_Y_RES/8)) return;
			ycurr++;
		}
		nlcd_Putc(s[l], xcurr, ycurr, nlcd_mem);
		xcurr = xcurr + FONT_CHAR_SIZE;
	}
}

void flush_buffer(uint8_t (*nlcd_mem)[NLCD_Y_RES/8+1]) {
	memset(nlcd_mem, 0x00, NLCD_X_RES*(NLCD_Y_RES/8+1));
}

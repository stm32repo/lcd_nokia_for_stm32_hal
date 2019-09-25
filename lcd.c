#include "main.h"
#include "lcd.h"
#include "nokia1100_lcd_font.h"
#include <string.h>

extern uint8_t usart_transmit_status;
extern uint8_t nlcd_memory[NLCD_X_RES][NLCD_Y_RES/8+1]; //буффер дисплея
extern USART_HandleTypeDef husart1;

void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart) {
	if (husart == &husart1) {
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

void lcd_init(void) {
	CS_LCD_RESET;
	RST_LCD_RESET;
	
	_delay_ms(10);
	
	RST_LCD_SET;
	_delay_ms(10);

	CS_LCD_SET;
	_delay_ms(2);

	/* nlcd_SendByte2(CMD_LCD_MODE,0xE2); // *** SOFTWARE RESET

	nlcd_SendByte2(CMD_LCD_MODE,0x3A); // *** Use internal oscillator
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
	//nlcd_xcurr=0; nlcd_ycurr=0;		  // Устанавливаем в 0 текущие координаты в видеобуфере		
	for(uint16_t tmp=0;tmp<864;tmp++) nlcd_SendByte2(DATA_LCD_MODE,0x00);
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
			HAL_USART_Transmit_IT(&husart1, tmp, 1);
//	_delay_ms(10);
//	CS_LCD_SET;
//	_delay_ms(3);
}

/* обновить видеобуфер */
void nlcd_update(void) {
	uint8_t x, y;
	nlcd_SendByte2(CMD_LCD_MODE,0x40); // Y = 0
	nlcd_SendByte2(CMD_LCD_MODE,0xB0);
	nlcd_SendByte2(CMD_LCD_MODE,0x10); // X = 0
	nlcd_SendByte2(CMD_LCD_MODE,0x00);
	for(y=0;y<(NLCD_Y_RES/8+1);y++) {
		for(x=0;x<NLCD_X_RES;x++) {
			nlcd_SendByte2(DATA_LCD_MODE,nlcd_memory[x][y]);
		}
	}
}

void nlcd_Putc(uint8_t s, uint8_t nlcd_xcurr, uint8_t nlcd_ycurr) {
	// out of diapozon lcd
	if ((nlcd_xcurr > (NLCD_X_RES-6)) || (nlcd_ycurr > 7)) return;
	for (uint8_t k = 0; k < 5; k++ )
		 nlcd_memory[k+nlcd_xcurr][nlcd_ycurr]=nlcd_Font[s][k];
}

void nlcd_Puts(uint8_t *s, uint8_t size_str, uint8_t nlcd_xcurr, uint8_t nlcd_ycurr) {
	for (uint8_t l=0; l < size_str; l++) {
		if (nlcd_xcurr > (NLCD_X_RES-6)) {
			nlcd_xcurr = 0;
			if (nlcd_ycurr > 8) return;
			nlcd_ycurr++;
		}
			nlcd_Putc(s[l], nlcd_xcurr, nlcd_ycurr);
			nlcd_xcurr = nlcd_xcurr + 6;
	}
}

void flush_buffer(void) {
	memset(nlcd_memory, 0x00, sizeof(nlcd_memory));
}

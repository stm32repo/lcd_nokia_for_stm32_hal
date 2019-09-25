#define CS_LCD_RESET		GPIOB->BSRR = GPIO_BSRR_BR_5 //CS_LINE
#define CS_LCD_SET			GPIOB->BSRR = GPIO_BSRR_BS_5 
#define RST_LCD_RESET		GPIOB->BSRR = GPIO_BSRR_BR_4 // RESET LINE
#define RST_LCD_SET			GPIOB->BSRR = GPIO_BSRR_BS_4
#define SDA_LCD_RESET		GPIOB->BSRR = GPIO_BSRR_BR_6 // Not use for usart mode
#define SDA_LCD_SET			GPIOB->BSRR = GPIO_BSRR_BS_6
#define SCLK_LCD_RESET	GPIOB->BSRR = GPIO_BSRR_BR_7 // Not use for usart mode
#define SCLK_LCD_SET		GPIOB->BSRR = GPIO_BSRR_BS_7

#define LED_ON					GPIOA->BSRR = GPIO_BSRR_BS2; // Not use
#define LED_OFF					GPIOA->BSRR = GPIO_BSRR_BR2;

#define CMD_LCD_MODE	0
#define DATA_LCD_MODE	1

#define PIXEL_ON	0
#define PIXEL_OFF	1
#define PIXEL_INV 	2

#define FILL_OFF	0
#define FILL_ON		1

#define INV_MODE_ON		0
#define INV_MODE_OFF	1

#define NLCD_X_RES	96		// разрешение по горизонтали
#define NLCD_Y_RES	68		// разрешение по вертикали

#define NLCD_MIN_DELAY	50


void lcd_init(void);
void nlcd_SendByte1(char mode, uint8_t c);
void nlcd_SendByte2(char mode, uint8_t c);
void _delay_ms(uint32_t delay);
void _delay_us(uint32_t delay);
void nlcd_Clear(void);
void nlcd_Putc(uint8_t s, uint8_t nlcd_xcurr, uint8_t nlcd_ycurr);
void nlcd_Puts(uint8_t *s, uint8_t size_str, uint8_t nlcd_xcurr, uint8_t nlcd_ycurr);
void nlcd_GotoXY_pix(char x,char y);
void nlcd_update(void);
void usart_init(void);
void spi_init(void);
void Usart1_Transmit(uint32_t data);
void flush_buffer(void);

/* Initilize usart 
static void MX_USART1_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 57600;
  husart1.Init.WordLength = USART_WORDLENGTH_9B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX;
  husart1.Init.CLKPolarity = USART_POLARITY_LOW;
  husart1.Init.CLKPhase = USART_PHASE_1EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_ENABLE;
  if (HAL_USART_Init(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
*/

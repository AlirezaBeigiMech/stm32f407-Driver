#include "stm32f4xx.h"

/* Function to initialize USART2 */
void USART_Init(void)
{
    /* Enable USART2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

		/* Configure PA2 as USART2_TX and PA3 as USART2_RX */
		GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
		GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
		GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);   /* Set pin as alternate function */

		/* USART2 configuration */
		USART2->CR1 = 0x00;
		/* 115200 baud @ 8MHz based on formula w/ oversampling by 8
		   Fck/(8*BR) -> 8e6/(8*115200) = 8.680 -> mantissa = 8, fraction = 0.68*8 ~= 5*/
		USART2->BRR = (5<<0)|(8<<4);
		USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; /* Enable transmitter and receiver */
		USART2->CR1 |= USART_CR1_UE; /* Enable USART2 */
}

void LED_Init(void)
{
	/* Enable GPIOD clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  /* Configure PD12, PD13, PD14, and PD15 as output */
  GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;

}

void USART_SendData(uint8_t data)
{
	 /* Send data */
  USART2->DR = data;
  /* Wait until transmit data register is empty and TC bit is set*/
  while (!(USART2->SR & USART_SR_TC))
  {
  }
}

void USART_SendString(const char *str)
{
  // Send each character of the string
  while (*str != '\0')
  {
    USART_SendData((uint8_t)(*str));
    str++;
  }
}

uint8_t USART_ReceiveData()
{
  /* Wait until data is received and RXNE bit is set*/
  while (!(USART2->SR & USART_SR_RXNE))
  {
  }

  /* Read received data */
  return (uint8_t)(USART2->DR);
}


int main(void)
{
	uint8_t receivedData = '0';
	
  USART_Init();
  LED_Init();

	while(1){
		receivedData = USART_ReceiveData();
		if (receivedData == '1'){
			GPIOD->BSRR |= GPIO_BSRR_BS12;	/*Bit12*/
			GPIOD->BSRR |= GPIO_BSRR_BS13;	/*Bit13*/
			GPIOD->BSRR |= GPIO_BSRR_BS14;	/*Bit14*/
			GPIOD->BSRR |= GPIO_BSRR_BS15;	/*Bit15*/
			
			USART_SendString("LED ON\r\n");
		}
		if (receivedData == '0'){
			GPIOD->BSRR |= GPIO_BSRR_BR12;
			GPIOD->BSRR |= GPIO_BSRR_BR13;
			GPIOD->BSRR |= GPIO_BSRR_BR14;
			GPIOD->BSRR |= GPIO_BSRR_BR15;
			
			USART_SendString("LED OFF\r\n");
		}
	}
}

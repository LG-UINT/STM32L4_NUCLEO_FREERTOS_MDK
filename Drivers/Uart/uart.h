
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __UART_H
#define __UART_H

#if defined (STM32L476xx)
#include "stm32l4xx_hal.h"
#endif

/* Buffer size */
#define RXBUFFERSIZE 32


#define USARTx			USART2
#define BAUDRATE		115200

#define USARTx_IRQ 										USART2_IRQn
#define USARTx_PREPRIO								0
#define USARTx_SUBPRIO								0



#define USARTx_CLK_ENABLE()    				__HAL_RCC_USART2_CLK_ENABLE()
#define USARTx_CLK_DISABLE()    			__HAL_RCC_USART2_CLK_DISABLE()


#define USARTx_TX_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_DISABLE()	__HAL_RCC_GPIOA_CLK_DISABLE()

#define USARTx_RX_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_DISABLE()	__HAL_RCC_GPIOA_CLK_DISABLE()

#define USARTx_TX_GPIO_PORT						GPIOA
#define USARTx_RX_GPIO_PORT						GPIOA

#define USARTx_TX_PIN									GPIO_PIN_2
#define USARTx_RX_PIN									GPIO_PIN_3

#define USARTx_RX_AF									GPIO_AF7_USART2


#define GPIO_DeInit() HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN); \
											HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN)

#define USARTx_IRQHandler	   USART2_IRQHandler

void UART_Init( void );
int8_t uart_read( uint8_t *fmt, uint16_t time_out );
void USARTx_IRQHandler( void );

#endif /* __UART_H */


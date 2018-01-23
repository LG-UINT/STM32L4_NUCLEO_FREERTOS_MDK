
#include <stdio.h>
#include <string.h>
#include "Uart.h"  


UART_HandleTypeDef huart;


/**
	If rear and front  have the same pointer, there is no data received from uart interruption
*/
typedef struct 
{
	uint8_t *rear;
	uint8_t *front;
}uart;

uart uart_rec;

uint8_t aRxBuffer[RXBUFFERSIZE];


#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar( int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}

int8_t uart_read(uint8_t *fmt, uint16_t time_out)  
{  
	while( time_out )  
  {  
		if( uart_rec.front != uart_rec.rear )  
    { 
			do
			{
				*fmt = *uart_rec.front;  
				uart_rec.front++;
				fmt++;					
				if ( uart_rec.front >= (aRxBuffer+RXBUFFERSIZE) )  
					uart_rec.front = aRxBuffer;
						
			}while( uart_rec.front != uart_rec.rear );
			return 0;
     }
		time_out--;  
	}  
  return (int8_t)-1;  
}  

/* USART init function */
void UART_Init( void )
{
	
  huart.Instance 										= USARTx;
  huart.Init.BaudRate 							= BAUDRATE;
  huart.Init.WordLength						  = UART_WORDLENGTH_7B;
  huart.Init.StopBits 							= UART_STOPBITS_1;
  huart.Init.Parity 								= UART_PARITY_NONE;
  huart.Init.Mode 									= UART_MODE_TX_RX;
  huart.Init.HwFlowCtl 							= UART_HWCONTROL_NONE;
  huart.Init.OverSampling 					= UART_OVERSAMPLING_16;
  huart.Init.OneBitSampling 				= UART_ONE_BIT_SAMPLE_DISABLE;
  huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	
	/* DeInit USART */
	if(HAL_UART_DeInit( &huart ) != HAL_OK)
  {
    _Error_Handler( __FILE__, __LINE__ );
  }
	
	/* Init USART */
  if (HAL_UART_Init( &huart ) != HAL_OK)
  {
    _Error_Handler( __FILE__, __LINE__ );
  }
	
	/* Init Buffer*/
	memset(	aRxBuffer, 0, RXBUFFERSIZE );
	
	/* Init pointer */
	uart_rec.front = aRxBuffer;
	uart_rec.rear  = aRxBuffer;
	
	/* Put uart in receive interuption mode */
	if(HAL_UART_Receive_IT( &huart,	aRxBuffer, 1) != HAL_OK)  
  {  
    _Error_Handler( __FILE__, __LINE__ ); 
  } 

}
void HAL_UART_MspInit( UART_HandleTypeDef* huart )
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if( huart->Instance == USARTx )
  {
    /* Peripheral clock enable */
    USARTx_CLK_ENABLE();
		USARTx_TX_GPIO_CLK_ENABLE();
		USARTx_RX_GPIO_CLK_ENABLE();
  
    /**USARTx GPIO Configuration    
     USARTx_TX
    */
    GPIO_InitStruct.Pin 			= USARTx_TX_PIN;
    GPIO_InitStruct.Mode 			= GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull 			= GPIO_PULLUP;
    GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;
    HAL_GPIO_Init( USARTx_TX_GPIO_PORT, &GPIO_InitStruct );
		
		/**USARTx GPIO Configuration    
			USARTx_RX 
    */
		GPIO_InitStruct.Pin 			= USARTx_RX_PIN;
		HAL_GPIO_Init( USARTx_RX_GPIO_PORT, &GPIO_InitStruct );


		/*##-3- Configure the NVIC for UART ########################################*/
		/* NVIC for USART */
		HAL_NVIC_SetPriority( USARTx_IRQ, USARTx_PREPRIO, USARTx_SUBPRIO );
		HAL_NVIC_EnableIRQ( USARTx_IRQ );

  }

}

void HAL_UART_MspDeInit( UART_HandleTypeDef* huart )
{

  if( huart->Instance == USARTx)
  {
    /* Peripheral clock disable */
		USARTx_CLK_DISABLE();
		USARTx_TX_GPIO_CLK_DISABLE();
		USARTx_RX_GPIO_CLK_DISABLE();
  
    /**USARTx GPIO Configuration    
     USARTx_TX
     USARTx_RX 
    */
		GPIO_DeInit();

		HAL_NVIC_DisableIRQ( USARTx_IRQ );

  }

}


/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *UartHandle )
{
	uint8_t ret = HAL_OK;  
    /* Set transmission flag: trasfer complete*/  
  uart_rec.rear++;    // refresh rear pointer  
  if( uart_rec.rear >= (	aRxBuffer + RXBUFFERSIZE ) )  
		uart_rec.rear = aRxBuffer;  
  do  
  {  
		ret = HAL_UART_Receive_IT( UartHandle, uart_rec.rear, 1);  
  }while(ret != HAL_OK);
}

/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA  
  *         used for USART data transmission     
  */
void USARTx_IRQHandler( void )
{
  HAL_UART_IRQHandler( &huart );
}

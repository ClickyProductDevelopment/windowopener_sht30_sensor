#ifndef __USART_H
#define	__USART_H


#include "hk32f030m.h"
#include <stdio.h>

	
// 串口1-USART1
#define  DEBUG_USARTx                   USART1
#define  DEBUG_USART_CLK                RCC_APB2Periph_USART1
#define  DEBUG_USART_BAUDRATE           115200

// USART GPIO 引脚宏定义
#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_USART_TX_GPIO_CLK        RCC_AHBPeriph_GPIOA   
#define  DEBUG_USART_TX_GPIO_PORT       GPIOA   
#define  DEBUG_USART_TX_GPIO_PIN        GPIO_Pin_3
#define  DEBUG_USART_TX_PIN_SOURCE      GPIO_PinSource3
#define  DEBUG_USART_TX_PIN_AF          GPIO_AF_1

#define  DEBUG_USART_RX_GPIO_CLK        RCC_AHBPeriph_GPIOD   
#define  DEBUG_USART_RX_GPIO_PORT       GPIOD
#define  DEBUG_USART_RX_GPIO_PIN        GPIO_Pin_6
#define  DEBUG_USART_RX_PIN_SOURCE      GPIO_PinSource6
#define  DEBUG_USART_RX_PIN_AF          GPIO_AF_1

#define  DEBUG_USART_IRQ                USART1_IRQn
#define  DEBUG_USART_IRQHandler         USART1_IRQHandler


void USART_Config(void);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

#endif /* __USART_H */

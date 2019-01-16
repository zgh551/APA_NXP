/*
 * uart.h
 *
 *  Created on: December 2 2018
 *      Author: zhuguohua
 */

#ifndef _UART_H_
#define _UART_H_
#include "derivative.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef enum _ReceiveFrame
{
	FirstHead1 = 0,
	FirstHead2,
	ID,
	Length,
	Data,
	CheckSum
}ReceiveFrame;

void FlexLin1_Uart_Buffer_Init( uint16_t MegaHertz, uint16_t BaudRate );
void FlexLin1_Uart_FIFO_Init( uint16_t MegaHertz, uint16_t BaudRate );
void FlexLin1_DMA_TX_Init(void);
void FlexLin1_DMA_RX_Init(void);
void TransmitData(uint8_t d);

#ifdef __cplusplus
}
#endif

#endif /* INC_UART_H_ */

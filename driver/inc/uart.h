/*
 * uart.h
 *
 *  Created on: 2018Äê11ÔÂ20ÈÕ
 *      Author: zhuguohua
 */

#ifndef _UART_H_
#define _UART_H_
#include "derivative.h"
#include "project.h"

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

typedef union _byte2float
{
	vuint8_t b[4];
	float f;
}Byte2Float;

void FlexLin1_Uart_Init( unsigned int MegaHertz, unsigned int BaudRate );
void FlexLin1_DMA_TX_Init(void);
void FlexLin1_DMA_RX_Init(void);
void TransmitData(uint8_t d);

#ifdef __cplusplus
}
#endif

#endif /* INC_UART_H_ */

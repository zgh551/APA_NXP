/*
 * can.h
 *
 *  Created on: December 22 2018
 *      Author: ZhuGuohua
 */

#ifndef CAN_H_
#define CAN_H_

#include "../System/derivative.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _CAN_Packet
{
	uint32_t id;
	uint32_t length;
	uint8_t  data[8];
}CAN_Packet;

void FlexCAN0_Init(void);
void FlexCAN1_Init(void);
void FlexCAN2_Init(void);

void CAN_Configure();

void CAN0_TransmitMsg(CAN_Packet m_CAN_Packet);
void CAN1_TransmitMsg(CAN_Packet m_CAN_Packet);
void CAN2_TransmitMsg(CAN_Packet m_CAN_Packet);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H_ */

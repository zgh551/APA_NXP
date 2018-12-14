/*
 * can.h
 *
 *  Created on: Mar 1, 2016
 *      Author: B55457
 */

#ifndef CAN_H_
#define CAN_H_

#include "derivative.h"
#include "project.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NXP_DemoBoard 0
#define MotovisBoard_V1 1
typedef struct _CAN_Packet
{
	uint32_t id;
	uint32_t length;
	uint8_t  data[8];
}CAN_Packet;

void FlexCAN0_Init(void);

void ReceiveMsg (void);

void CAN0_TransmitMsg(CAN_Packet m_CAN_Packet);
void CAN1_TransmitMsg(CAN_Packet m_CAN_Packet);
void CAN2_TransmitMsg(CAN_Packet m_CAN_Packet);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H_ */

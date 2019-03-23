/*
 * terminal.h
 *
 *  Created on: January 8 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: terminal.h                          COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: Interaction terminal            					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 8 2019      Initial Version                  */
/*****************************************************************************/

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include "property.h"
#include "derivative.h"
#include "ChangAn/chang_an_message.h"
#include "Interface/vehicle_controller.h"
#include "Interface/message_manager.h"
#include "Interface/vehicle_state.h"
#include "Ultrasonic.h"
#include "planning.h"
#include "percaption.h"
#include "ultrasonic_abstacle_percption.h"

typedef union _byte2float
{
	uint8_t b[4];
	float f;
}Byte2Float;

typedef union _Byte2Int
{
	uint8_t b[2];
	uint16_t u16;
	int16_t  i16;
}Byte2Int;

class Terminal
{
public:
	Terminal();
	virtual ~Terminal();

	// CAN Module:Vehicle information receive
	void Parse(vuint32_t id,vuint8_t dat[],VehicleController *ctl);
	void Parse(vuint32_t id,vuint8_t dat[],MessageManager *msg);

	void Parse(vuint32_t id,vuint8_t dat[],Ultrasonic *u);
	void Parse(vuint32_t id,vuint8_t dat[],Percaption *pi,Planning *pp);

	void Parse(vuint32_t id,vuint8_t dat[],Planning *msg);
	void Parse(vuint32_t id,vuint8_t dat[]);
	// Terminal Control
//	void Push(MessageManager *msg);
	void Push(ChangAnMessage *msg);

	void Push(VehicleController *msg);

	void Push(VehicleState *msg);

	void Push(Ultrasonic *u);

	void Push(Percaption *p);

//	void Push(UltrasonicAbstaclePercption *p);

	void Push(Planning *p);
///////////////////////////////////////////////////////////////////////////
	void UltrasonicSend(uint8_t id,LIN_RAM *msg);
	void UltrasonicSend(uint8_t id,Ultrasonic_Packet *msg_pk);

	void UltrasonicLocationSend(uint8_t id,LIN_RAM *msg);
	void UltrasonicLocationSend(uint8_t id,Ultrasonic_Packet *msg_pk);
	void UltrasonicLocationSend(uint8_t id,Vector2d v1);
	void UltrasonicLocationSend(uint8_t id,Vector2d v1,Vector2d v2);

	void UltrasonicGroundLocationSend(uint8_t id,Vector2d v1,Vector2d v2);

	void Ack(void);

	void VehicleInitPositionSend(VehicleBody v);
	void ParkingMsgSend(Percaption *pi,float fm,float rm);
	void FrontTrialPositionSend(VehicleBody v,uint8_t cnt);
	void RearTrialPositionSend(VehicleBody v,uint8_t cnt);
	void EnterParkingPositionSend(VehicleBody v,uint8_t cnt,uint8_t s);
	void TurnPointSend(Turn v,uint8_t cnt);
	void ParkingCenterPointSend(Vector2d v);

	/*** Property ***/
	// AckValid
	uint8_t getAckValid();
	void setAckValid(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> AckValid;

	// Commond
	uint8_t getCommand();
	void    setCommand(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> Command;

	// Commond
	uint8_t getPushActive();
	void    setPushActive(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> PushActive;

private:
	/// terminal command
	uint8_t _command;

	/// push state
	uint8_t _push_active;
	/// terminal receive frame state machine
	ReceiveFrame _terminal_frame;
	uint8_t _data_buffer[32];
	uint8_t _send_data_buffer[32];
	uint8_t _frame_id,_frame_length,_frame_cnt,_check_sum;
	uint8_t _frame_err_cnt;
	uint8_t _ack_valid;

	/// data type conversion
	Byte2Float _data_temp,_speed_data_temp;
};


#endif /* TERMINAL_H_ */

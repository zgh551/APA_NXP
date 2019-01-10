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
//#include "SystemWork.h"


class Terminal
{
public:
	Terminal();
	virtual ~Terminal();

	// CAN Module:Vehicle information receive
	void Parse(vuint32_t id,vuint8_t dat[],VehicleController *ctl,Ultrasonic *u,MessageManager *msg);

	// Terminal Control
//	void Push(MessageManager *msg);
	void Push(ChangAnMessage *msg);

	void Push(VehicleController *msg);

	void Push(VehicleState *msg);

	void Push(Ultrasonic *u);

	void UltrasonicSend(uint8_t id,LIN_RAM *msg);
	void Ack(void);
	/*** Property ***/
	// AckValid
	uint8_t getAckValid();
	void setAckValid(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> AckValid;

private:
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

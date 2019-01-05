/*
 * Terminal.h
 *
 *  Created on: 2018Äê12ÔÂ6ÈÕ
 *      Author: zhuguohua
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include "property.h"
#include "derivative.h"
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
	void Parse(vuint32_t id,vuint8_t dat[],VehicleController *ctl);

	// Terminal Control
	void Push(MessageManager *msg);

	void Push(VehicleState *msg);

	void Push(Ultrasonic *u);

	void UltrasonicSend(uint8_t id,LIN_RAM *msg);
//	void TerminalControlCommandSend(void);
//	void TerminalControlSpeedSend(void);
//	void TerminalSystemStateSend(void);
//
//	void TerminalControlAckSend(void);

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

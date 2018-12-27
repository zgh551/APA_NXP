/*
 * Terminal.h
 *
 *  Created on: 2018Äê12ÔÂ6ÈÕ
 *      Author: zhuguohua
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_
//#include "Property.h"
//#include "derivative.h"
//#include "project.h"
#include "SystemWork.h"


class Terminal : public SystemWork
{
public:
	Terminal();
	Terminal(float dt,float kp,float ki,float kd,float i_lim,float out_lim);
	Terminal(float dt,float kp,float ki,float kd,float i_lim,float out_lim,float threshold);

	virtual ~Terminal();

	// CAN Module:Vehicle information receive
	void VehicleInformation(CAN_MB_tag mb_msg);
	void VehicleInformation(vuint32_t id,vuint8_t dat[]);

	// Terminal Control
	void TerminalControlCommandReceive(uint8_t data);

	void TerminalControlCommandSend(void);
	void TerminalControlSpeedSend(void);
	void TerminalSystemStateSend(void);

	void TerminalControlAckSend(void);

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

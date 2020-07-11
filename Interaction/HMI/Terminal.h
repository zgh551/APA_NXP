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

#include "../../Common/Utils/Inc/property.h"
#include "../../Driver/System/derivative.h"
#include "../CANBUS/DongFengE70/dong_feng_e70_message.h"

#include "../CANBUS/Interface/vehicle_controller.h"
#include "../CANBUS/Interface/message_manager.h"

#include "../../Common/VehicleState/Interface/vehicle_state.h"
#include "../../Common/VehicleState/GeometricTrack/geometric_track.h"

#include "../../Control/Common/pid.h"

#include "../Ultrasonic/Ultrasonic.h"

#include "../../Control/LonControl/lon_control.h"
#include "../../Control/LatControl/lat_control.h"

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

typedef union _Byte2Int32
{
	uint8_t b[4];
	uint32_t u32;
	int32_t  i32;
}Byte2Int32;

class Terminal
{
public:
	Terminal();
	virtual ~Terminal();

	// CAN Module:Vehicle information receive
	void Parse(vuint32_t id,vuint8_t dat[],VehicleController &ctl);
	void Parse(vuint32_t id,vuint8_t dat[],MessageManager &msg);
	void Parse(vuint32_t id,vuint8_t dat[],Ultrasonic &u);
	void Parse(vuint32_t id,vuint8_t dat[],PID &msg);
	void Parse(vuint32_t id,vuint8_t dat[],LatControl &lat_ctl);
	void Parse(vuint32_t id,vuint8_t dat[]);

	// Terminal Control
	void Push(MessageManager &msg);
	void Push(VehicleController &msg);
	void Push(VehicleState &msg);

	void Push(Ultrasonic &u);
	void Push(Ultrasonic &u, uint8_t f);

	void Push(LonControl &lon_control);
	void Push(LatControl &lat_control);
///////////////////////////////////////////////////////////////////////////
	/*
	 * 鐩存帴娴嬮噺瓒呭０娉㈠師濮嬩俊鍙�
	 * */
	void UltrasonicSend(const uint8_t id,const LIN_RAM *msg);
	void UltrasonicSend(const uint8_t id,const Ultrasonic_Packet *msg_pk);
	/*
	 * 涓夎瀹氫綅鐨勭煭璺濈瓒呭０娉俊鍙�
	 * */
	void UltrasonicLocationSend(const uint8_t id,const LIN_RAM *msg);
	void UltrasonicLocationSend(const uint8_t id,const Ultrasonic_Packet *msg_pk);
	/*
	 * 杞戒綋鍧愭爣绯荤殑鏁版嵁鍙戦��
	 * */
	void UltrasonicBodyLocationSend(const uint8_t id,ObstacleLocationPacket &packet);
	/*
	 * 鍦伴潰鍧愭爣绯荤殑鏁版嵁鍙戦��
	 * */
	void UltrasonicGroundLocationSend(const uint8_t id,ObstacleLocationPacket &packet);

	void Ack(void);
	void ParkingCenterPointSend(Vector2d v);

	/*** Property ***/
	// AckValid
	uint8_t getAckValid();
	void setAckValid(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> AckValid;

	uint8_t getAckEcho();
	void    setAckEcho(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> AckEcho;

	// Commond
	uint8_t getCommand();
	void    setCommand(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> Command;

	// Commond
	uint8_t getWorkMode(void) { return _work_mode; }
	void    setWorkMode(uint8_t value){ _work_mode = value; }

	uint8_t getFunctionState(void) { return _function_state; }
	void    setFunctionState(uint8_t value){ _function_state = value; }

	// Commond
	uint8_t getPushActive();
	void    setPushActive(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> PushActive;

private:
	/// terminal command
	uint8_t _command;

	uint8_t _work_mode;
	uint8_t _function_state;

	/// push state
	uint8_t _push_active;
	/// terminal receive frame state machine
	uint8_t _data_buffer[32];
	uint8_t _send_data_buffer[32];
	uint8_t _frame_id,_frame_length,_frame_cnt,_check_sum;
	uint8_t _frame_err_cnt;
	uint8_t _ack_valid;
	uint8_t _ack_echo;
	/// data type conversion
	Byte2Float _data_temp,_speed_data_temp;
};


#endif /* TERMINAL_H_ */

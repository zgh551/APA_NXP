/*
 * terminal.cpp
 *
 *  Created on: January 8 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: terminal.cpp                        COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: Interaction terminal            					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 8 2019      Initial Version                  */
/*****************************************************************************/

#include "Terminal.h"

Terminal::Terminal() {
	// TODO Auto-generated constructor stub
	_terminal_frame = FirstHead1;
	_frame_err_cnt = 0;

	// ACK Valid
	AckValid.setContainer(this);
	AckValid.getter(&Terminal::getAckValid);
	AckValid.setter(&Terminal::setAckValid);
}

Terminal::~Terminal() {
	// TODO Auto-generated destructor stub
}

/// AckValid
uint8_t Terminal::getAckValid()
{
	return _ack_valid;
}
void Terminal::setAckValid(uint8_t value)
{
	_ack_valid = value;
}

void Terminal::Parse(vuint32_t id,vuint8_t dat[],VehicleController *ctl,Ultrasonic *u,MessageManager *msg)
{
	uint8_t i,check_sum;
	switch(id)
	{
		case 0x516://eps status
			check_sum =0 ;
			for(i=0;i<7;i++){
				check_sum += dat[i];
			}
			check_sum = check_sum ^ 0xFF;
			if(check_sum == dat[7])
			{
				ctl->GearEnable 		=  dat[0]       & 0x01;
				ctl->AccelerationEnable = (dat[0] >> 2) & 0x01;
				ctl->DecelerationEnable = (dat[0] >> 4) & 0x01;
				ctl->TorqueEnable       = (dat[0] >> 5) & 0x01;
				ctl->VelocityEnable     = (dat[0] >> 3) & 0x01;
				if( (0 == ctl->SteeringEnable) || (0 == ((dat[0] >> 1) & 0x01)))
				{
					ctl->SteeringEnable 	= (dat[0] >> 1) & 0x01;
				}
				ctl->Gear 				= (uint8_t)dat[1];
				ctl->SteeringAngle 		= (float)(((int16_t)((dat[3] << 8) | dat[2])) * 0.1);
				ctl->SteeringAngleRate 	= (float)(((uint16_t)((dat[5] << 8) | dat[4])) * 0.01);
				AckValid = 0xa5;
			}
			break;

		case 0x517://eps status
			check_sum =0 ;
			for(i=0;i<7;i++){
				check_sum += dat[i];
			}
			check_sum = check_sum ^ 0xFF;
			if(check_sum == dat[7])
			{
				ctl->Acceleration	= (float)(((int16_t )((dat[1] << 8) | dat[0])) * 0.001);
				ctl->Deceleration	= (float)(((int16_t )((dat[3] << 8) | dat[2])) * 0.001);
				ctl->Velocity		= (float)(((uint16_t)((dat[5] << 8) | dat[4])) * 0.001);
				ctl->Torque			= (float)(dat[6] * 0.5);
			}
			break;

        case 0x508://传感器9
        case 0x509://传感器10
        case 0x50A://传感器11
        case 0x50B://传感器12
        	Ultrasonic_Packet ultrasonic_packet;
        	ultrasonic_packet.Distance1 = (float)(((uint16_t )((dat[1] << 8) | dat[0])) * u->Compensation(0));
        	ultrasonic_packet.Distance2 = (float)(((uint16_t )((dat[3] << 8) | dat[2])) * u->Compensation(0));
        	ultrasonic_packet.Level = dat[4] * LEVEL_RATIO;
        	ultrasonic_packet.Width = dat[5] * WIDTH_RATIO;
        	ultrasonic_packet.status = dat[6];
        	u->setUltrasonicPacket(id & 0x00f,ultrasonic_packet);
			break;

        case 0x510:
        	msg->SteeringAngle = ((int16_t)((dat[3] << 8) | dat[2])) * 0.1;
        	break;

        case 0x520:
        	msg->WheelSpeedRearLeft  = ((uint16_t)((dat[5] << 8) | dat[4])) * 0.001;
        	msg->WheelSpeedRearRight = ((uint16_t)((dat[7] << 8) | dat[6])) * 0.001;
        	break;
		default:

			break;
	}
}

void Terminal::Push(ChangAnMessage *msg)
{
	CAN_Packet m_CAN_Packet;
	int16_t temp_int16;
	uint16_t temp_uint16;

	m_CAN_Packet.id = 0x410;
	m_CAN_Packet.length = 8;

	m_CAN_Packet.data[0] = ( (msg->EMS_QEC_ACC & 0x01) << 2) | ( (msg->ESP_QDC_ACC & 0x01) << 1) | (msg->APA_EPAS_Failed & 0x01);
	m_CAN_Packet.data[1] = 0;

	temp_int16 = (int16_t)(msg->SteeringAngle * 10);
	m_CAN_Packet.data[2] = temp_int16 & 0xff;
	m_CAN_Packet.data[3] = (temp_int16 >> 8) & 0xff;

	temp_uint16 = (uint16_t)(msg->SteeringAngleRate * 100);
	m_CAN_Packet.data[4] = temp_uint16 & 0xff;
	m_CAN_Packet.data[5] = (temp_uint16 >> 8) & 0xff;

	temp_int16 = (int16_t)(msg->SteeringTorque * 100);
	m_CAN_Packet.data[6] = temp_int16 & 0xff;
	m_CAN_Packet.data[7] = (temp_int16 >> 8) & 0xff;

	CAN2_TransmitMsg(m_CAN_Packet);
}

void Terminal::Push(VehicleController *msg)
{
	CAN_Packet m_CAN_Packet;
	int16_t temp_int16;
	uint16_t temp_uint16;

	m_CAN_Packet.id = 0x411;
	m_CAN_Packet.length = 8;

	m_CAN_Packet.data[0] = 	 msg->AccelerationEnable 	   |
							(msg->DecelerationEnable << 1) |
							(msg->TorqueEnable       << 2) |
							(msg->VelocityEnable     << 3) |
							(msg->SteeringEnable     << 4) |
							(msg->GearEnable         << 6) ;
	m_CAN_Packet.data[1] = 0 ;

	temp_int16 = (int16_t)(msg->Acceleration * 100);
	m_CAN_Packet.data[2] = temp_int16 & 0xff ;
	m_CAN_Packet.data[3] = (temp_int16 >> 8) & 0xff ;

	temp_int16 = (int16_t)(msg->Deceleration * 100);
	m_CAN_Packet.data[4] = temp_int16 & 0xff ;
	m_CAN_Packet.data[5] = (temp_int16 >> 8) & 0xff ;

	temp_uint16 = (uint16_t)(msg->Torque * 100);
	m_CAN_Packet.data[6] =  temp_uint16 & 0xff ;
	m_CAN_Packet.data[7] = (temp_uint16 >> 8) & 0xff ;

	CAN2_TransmitMsg(m_CAN_Packet);
}

void Terminal::Push(VehicleState *msg)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x440;
	m_CAN_Packet.length = 8;
	m_CAN_Packet.data[0] = ((int16_t) (msg->X * 100)) & 0xff;
	m_CAN_Packet.data[1] = (((int16_t)(msg->X * 100)) >> 8) & 0xff;
	m_CAN_Packet.data[2] = ((int16_t) (msg->Y * 100)) & 0xff;
	m_CAN_Packet.data[3] = (((int16_t)(msg->Y * 100)) >> 8) & 0xff;
	m_CAN_Packet.data[4] = ((int16_t) (msg->Yaw * 100)) & 0xff;
	m_CAN_Packet.data[5] = (((int16_t)(msg->Yaw * 100)) >> 8) & 0xff;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);
}


void Terminal::Push(Ultrasonic *u)
{
	switch(u->ReadStage)
	{
		case 0:
			UltrasonicSend(1,u->UltrasonicDatas);
			UltrasonicSend(7,u->UltrasonicDatas);
			break;

		case 1:
		case 5:
			UltrasonicSend(8,u->UltrasonicDatas);
			UltrasonicSend(11,u->UltrasonicDatas);
			break;

		case 2:
		case 6:
			UltrasonicSend(9,u->UltrasonicDatas);
			UltrasonicSend(10,u->UltrasonicDatas);
			break;

		case 3:
			UltrasonicSend(3,u->UltrasonicDatas);
			UltrasonicSend(5,u->UltrasonicDatas);
			break;

		case 4:
			UltrasonicSend(0,u->UltrasonicDatas);
			UltrasonicSend(6,u->UltrasonicDatas);
			break;

		case 7:
			UltrasonicSend(2,u->UltrasonicDatas);
			UltrasonicSend(4,u->UltrasonicDatas);
			break;

		default:
			break;
	}
}

void Terminal::UltrasonicSend(uint8_t id,LIN_RAM *msg)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x400 | id;
	m_CAN_Packet.length = 8;
	if(id < 8)
	{

		m_CAN_Packet.data[0] =  msg[id].STP318.TOF       & 0xff;
		m_CAN_Packet.data[1] = (msg[id].STP318.TOF >> 8) & 0xff;
		m_CAN_Packet.data[2] = 0;
		m_CAN_Packet.data[3] = 0;
		m_CAN_Packet.data[4] = 0;
		m_CAN_Packet.data[5] = 0;
		m_CAN_Packet.data[6] =  msg[id].STP318.Status;
		m_CAN_Packet.data[7] = 0;
	}
	else
	{
		m_CAN_Packet.data[0] =  msg[id].STP313.TOF1       & 0xff;
		m_CAN_Packet.data[1] = (msg[id].STP313.TOF1 >> 8) & 0xff;
		m_CAN_Packet.data[2] =  msg[id].STP313.TOF2       & 0xff;
		m_CAN_Packet.data[3] = (msg[id].STP313.TOF2 >> 8) & 0xff;
		m_CAN_Packet.data[4] =  msg[id].STP313.Level;
		m_CAN_Packet.data[5] =  msg[id].STP313.Width;
		m_CAN_Packet.data[6] =  msg[id].STP313.Status;
		m_CAN_Packet.data[7] =  0;
	}
	CAN2_TransmitMsg(m_CAN_Packet);
}

void Terminal::Ack(void)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x416;
	m_CAN_Packet.length = 8;
	m_CAN_Packet.data[0] = 0x5A;
	m_CAN_Packet.data[1] = 0xA5;
	m_CAN_Packet.data[2] = 0;
	m_CAN_Packet.data[3] = 0;
	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);
}




// Terminal control UART Interface
//void Terminal::TerminalControlCommandReceive(uint8_t data)
//{
//	uint8_t i;
//	switch(_terminal_frame)
//	{
//		case FirstHead1:
//			if(data == 0xAA)
//			{
//				_check_sum = data;
//				_terminal_frame = FirstHead2;
//			}
//			break;
//		case FirstHead2:
//			if(data == 0x55)
//			{
//				_check_sum += data;
//				_terminal_frame = ID;
//			}
//			else
//			{
//				_terminal_frame = FirstHead1;
//			}
//			break;
//
//		case ID:
//			_frame_id = data;
//			_check_sum += data;
//			_terminal_frame = Length;
//			break;
//
//		case Length:
//			_frame_length = data;
//			_check_sum += data;
//			_frame_cnt = 0;
//			_terminal_frame = Data;
//		break;
//
//		case Data:
//			_data_buffer[_frame_cnt] = data;
//			_check_sum += data;
//			_frame_cnt++;
//			if(_frame_cnt >= _frame_length)
//			{
//				_terminal_frame = CheckSum;
//			}
//			break;
//
//		case CheckSum:
//			if(_check_sum == data)
//			{
//				switch(_frame_id)
//				{
//				case 0x1A:
//					GearShiftEnable = _data_buffer[0] & 0x01;
//					GearShiftValid = (_data_buffer[0] >> 1) & 0x01;
//
//					if( (((_data_buffer[0] >> 2) & 0x01) == 0) || (SteeringAngleTargetActive == 0))
//					{
//						SteeringAngleTargetActive = (_data_buffer[0] >> 2) & 0x01;
//					}
//
//					TorqueEnable = (_data_buffer[0] >> 3) & 0x01;
//					TargetDecelerationEnable = (_data_buffer[0] >> 4) & 0x01;
//					TargetAccelerationEnable = (_data_buffer[0] >> 5) & 0x01;
//
//					GearShift = _data_buffer[1];
//
//					Torque = ((uint16_t)((_data_buffer[3] << 8) | _data_buffer[2]))*0.1;
//
//					SteeringAngleTarget = ((int16_t)((_data_buffer[5] << 8) | _data_buffer[4])) * 0.1;
//
//					SteeringAngleSpeedTarget = ((uint16_t)((_data_buffer[7] << 8) | _data_buffer[6])) * 0.01;
//
//					for(i = 0;i<4;i++)
//					{
//						_data_temp.b[3-i] = _data_buffer[i + 8];
//					}
//					TargetDecelerationAEB = _data_temp.f;
//					for(i = 0;i<4;i++)
//					{
//						_data_temp.b[3-i] = _data_buffer[i + 12];
//					}
//					TargetAccelerationACC = _data_temp.f;
//					break;
//
//				case 0x2A:
//					for(i = 0;i<4;i++)
//					{
//						_data_temp.b[3-i] = _data_buffer[i];
//					}
//					KP = _data_temp.f;
//
//					for(i = 0;i<4;i++)
//					{
//						_data_temp.b[3-i] = _data_buffer[i + 4];
//					}
//					KI = _data_temp.f;
//
//					for(i = 0;i<4;i++)
//					{
//						_data_temp.b[3-i] = _data_buffer[i + 8];
//					}
//					KD = _data_temp.f;
//					break;
//
//				case 0x2B:// Threshold
//					for(i = 0;i<4;i++)
//					{
//						_data_temp.b[3-i] = _data_buffer[i];
//					}
//					Threshold = _data_temp.f;
//					break;
//
//				case 0x3A:// Target Vehicle Set
//					for(i = 0;i<4;i++)
//					{
//						_data_temp.b[3-i] = _data_buffer[i];
//					}
//					VehicleSpeedTarget= _data_temp.f;
//					Desired = VehicleSpeedTarget;
//					break;
//
//				case 0x4A:
//					WorkingModule = _data_buffer[0];
//					FunctionStatus = _data_buffer[1];
//					break;
//
//				case 0x5A:
//					//Gear Enable Single
//					GearShiftEnable = _data_buffer[0] & 0x01;
//					GearShiftValid  = _data_buffer[0] & 0x01;
//
//					/// Steering Angle Active
//					if( (((_data_buffer[0] >> 1) & 0x01) == 0) || (SteeringAngleTargetActive == 0))
//					{
//						SteeringAngleTargetActive = (_data_buffer[0] >> 1) & 0x01;
//					}
//
//					/// ACC Enable Single
//					TargetAccelerationEnable = (_data_buffer[0] >> 2) & 0x01;
//
//					/// Speed Control Single
//					VehicleSpeedControlEnable = (_data_buffer[0] >> 3) & 0x01;
//
//					/// Gear Value
//					GearShift = _data_buffer[1];
//
//					/// Steering Angle Value
//					SteeringAngleTarget = ((int16_t)((_data_buffer[3] << 8) | _data_buffer[2])) * 0.1;
//
//					/// Steering Angle Value
//					SteeringAngleSpeedTarget = ((uint16_t)((_data_buffer[5] << 8) | _data_buffer[4])) * 0.01;
//
//					/// ACC Value
//					if(!VehicleSpeedControlEnable)
//					{
//						for(i = 0;i<4;i++)
//						{
//							_data_temp.b[3-i] = _data_buffer[i + 6];
//						}
//						TargetAccelerationACC = _data_temp.f;
//					}
//					/// Target Vehicle Speed Value
//					VehicleSpeedTarget = _data_buffer[10]*0.01;
//					Desired = VehicleSpeedTarget;
//					break;
//
//				default:
//
//					break;
//				}
				// ACK Single
//				AckValid = 0x5A;
//				TerminalControlAckSend();
//			}
//			else
//			{
//				_frame_err_cnt++;
//			}
//			_terminal_frame = FirstHead1;
//			break;
//	}
//}

//void Terminal::TerminalControlCommandSend(void)
//{
//	uint8_t i,check_sum;
//	Byte2Float data_temp;
//	Byte2Int data_temp_int;
//
//	_send_data_buffer[0] = 0x7F;
//	_send_data_buffer[1] = 0x80;
//	_send_data_buffer[2] = 0x85;
//	_send_data_buffer[3] = 19;
//	_send_data_buffer[4] = ((EMS_QEC_ACC << 2) & 0x04) | ((ESP_QDC_ACC << 1) & 0x02) | APA_EPAS_Failed;
//	_send_data_buffer[5] = 	(  TargetAccelerationEnable         & 0x01) |
//							( (TargetDecelerationEnable   << 1) & 0x02) |
//							( (TorqueEnable               << 2) & 0x04) |
//							( (GearShiftEnable            << 3) & 0x08) |
//							( (SteeringAngleTargetActive  << 4) & 0x30) ;
//
//	_send_data_buffer[6] = (uint8_t)(SteeringAngleActual & 0xff);
//	_send_data_buffer[7] = (uint8_t)((SteeringAngleActual >> 8) & 0xff);
//
//	_send_data_buffer[8] = (uint8_t)(SteeringAngleSpeed & 0xff);
//	_send_data_buffer[9] = (uint8_t)((SteeringAngleSpeed >> 8) & 0xff);
//
//	data_temp.f = SteeringTorque;
//	for(i = 0; i < 4; i++)
//	{
//		_send_data_buffer[i + 10] = data_temp.b[3-i] ;
//	}
//
//	data_temp.f = TargetAccelerationACC;
//	for(i = 0; i < 4; i++)
//	{
//		_send_data_buffer[i + 14] = data_temp.b[3-i] ;
//	}
//
//	data_temp_int.u16 = (uint16_t)(Torque * 10);
//	_send_data_buffer[18] = data_temp_int.b[1];
//	_send_data_buffer[19] = data_temp_int.b[0];
//
//
//	_send_data_buffer[20] = 0;
//	_send_data_buffer[21] = 0;
//	_send_data_buffer[22] = 0;
//
//	check_sum = 0;
//	for(i=0;i<23;i++)
//	{
//		check_sum += _send_data_buffer[i];
//	}
//	_send_data_buffer[23] = check_sum;
//
//	for(i=0;i<24;i++)
//	{
//		TransmitData(_send_data_buffer[i]);
//	}
//}
//
//void Terminal::TerminalControlSpeedSend(void)
//{
//	uint8_t i,check_sum;
//	_send_data_buffer[0] = 0x7F;
//	_send_data_buffer[1] = 0x80;
//	_send_data_buffer[2] = 0x95;
//	_send_data_buffer[3] = 19;
//	_speed_data_temp.f = VehicleSpeed;
//	for(i = 0;i<4;i++)
//	{
//		_send_data_buffer[i + 4] = _speed_data_temp.b[3-i] ;
//	}
//
//	_send_data_buffer[8] = (uint8_t)(( (uint16_t)(WheelSpeedFrontLeftData*10)    ) & 0xff);
//	_send_data_buffer[9] = (uint8_t)((((uint16_t)(WheelSpeedFrontLeftData*10))>>8) & 0xff);
//
//	_send_data_buffer[10] = (uint8_t)(( (uint16_t)(WheelSpeedFrontRightData*10)    ) & 0xff);
//	_send_data_buffer[11] = (uint8_t)((((uint16_t)(WheelSpeedFrontRightData*10))>>8) & 0xff);
//
//	_send_data_buffer[12] = (uint8_t)(( (uint16_t)(WheelSpeedRearLeftData*10)    ) & 0xff);
//	_send_data_buffer[13] = (uint8_t)((((uint16_t)(WheelSpeedRearLeftData*10))>>8) & 0xff);
//
//	_send_data_buffer[14] = (uint8_t)(( (uint16_t)(WheelSpeedRearRightData*10)    ) & 0xff);
//	_send_data_buffer[15] = (uint8_t)((((uint16_t)(WheelSpeedRearRightData*10))>>8) & 0xff);
//
//	_send_data_buffer[16] = WheelSpeedFrontLeftPulse;
//	_send_data_buffer[17] = WheelSpeedFrontRightPulse;
//	_send_data_buffer[18] = WheelSpeedRearLeftPulse;
//	_send_data_buffer[19] = WheelSpeedRearRightPulse;
//
//	_send_data_buffer[20] = WheelSpeedDirection;
//	_send_data_buffer[21] = 0;
//	_send_data_buffer[22] = 0;
//
//	check_sum = 0;
//	for(i=0;i<23;i++)
//	{
//		check_sum += _send_data_buffer[i];
//	}
//	_send_data_buffer[23] = check_sum;
//	for(i=0;i<24;i++)
//	{
//		TransmitData(_send_data_buffer[i]);
//	}
//}
//
//void Terminal::TerminalSystemStateSend(void)
//{
//	uint8_t i,check_sum;
//
//	_send_data_buffer[0] = 0x7F;
//	_send_data_buffer[1] = 0x80;
//	_send_data_buffer[2] = 0xA5;
//	_send_data_buffer[3] = 3;
//
//	_send_data_buffer[4] = WorkingModule;
//	_send_data_buffer[5] = FunctionStatus;
//
//	_send_data_buffer[6] = 0;
//
//	check_sum = 0;
//	for(i=0;i<7;i++)
//	{
//		check_sum += _send_data_buffer[i];
//	}
//	_send_data_buffer[7] = check_sum;
//	for(i=0;i<8;i++)
//	{
//		TransmitData(_send_data_buffer[i]);
//	}
//}
//
//void Terminal::TerminalControlAckSend(void)
//{
//	uint8_t i,check_sum;
//	_send_data_buffer[0] = 0x7F;
//	_send_data_buffer[1] = 0x80;
//	_send_data_buffer[2] = _frame_id;
//	_send_data_buffer[3] = 1;
//	_send_data_buffer[4] = 0x5A;
//
//	check_sum = 0;
//	for(i=0;i<5;i++)
//	{
//		check_sum += _send_data_buffer[i];
//	}
//	_send_data_buffer[5] = check_sum;
//	for(i=0;i<6;i++)
//	{
//		TransmitData(_send_data_buffer[i]);
//	}
//}

/*
 * Terminal.cpp
 *
 *  Created on: 2018年12月6日
 *      Author: zhuguohua
 */

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

Terminal::Terminal(float dt,float kp,float ki,float kd,float i_lim,float out_lim) : SystemWork(dt,kp,ki,kd,i_lim,out_lim)
{
	// TODO Auto-generated constructor stub
	_terminal_frame = FirstHead1;
	_frame_err_cnt = 0;

	// ACK Valid
	AckValid.setContainer(this);
	AckValid.getter(&Terminal::getAckValid);
	AckValid.setter(&Terminal::setAckValid);
}

Terminal::Terminal(float dt,float kp,float ki,float kd,float i_lim,float out_lim,float threshold) : SystemWork(dt,kp,ki,kd,i_lim,out_lim,threshold)
{
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

void Terminal::VehicleInformation(CAN_MB_tag mb_msg)
{
	switch(mb_msg.ID.B.ID_STD)
	{
		case 0x2A0://eps status
			EPS_Failed = (uint8_t)((mb_msg.DATA.B[1] >> 7) & 0x01);
			APA_EPAS_Failed = (uint8_t)((mb_msg.DATA.B[1] >> 1) & 0x01);
			TorqueSensorStatus = (uint8_t)( mb_msg.DATA.B[1] & 0x01 );
			APA_ControlFeedback = (uint8_t)((mb_msg.DATA.B[3] >> 5) & 0x01);
			SteeringTorque = (float)(mb_msg.DATA.B[2] * 0.1794 - 22.78);
			break;

		case 0x208:// wheel speed
			WheelSpeedRearRightDirection = (uint8_t)(mb_msg.DATA.B[0] >> 5) & 0x03;
			WheelSpeedRearRightValid = (uint8_t)(  mb_msg.DATA.B[0] >> 7) & 0x01;
			WheelSpeedRearRightData = ((uint16_t)(((mb_msg.DATA.B[0] & 0x1F) << 8) | mb_msg.DATA.B[1]))*0.05625;

			WheelSpeedRearLeftDirection = (uint8_t)(mb_msg.DATA.B[2] >> 5) & 0x03;
			WheelSpeedRearLeftValid = (uint8_t)(  mb_msg.DATA.B[2] >> 7) & 0x01;
			WheelSpeedRearLeftData = ((uint16_t)(((mb_msg.DATA.B[2] & 0x1F) << 8) | mb_msg.DATA.B[3]))*0.05625;

			WheelSpeedFrontRightDirection = (uint8_t)(mb_msg.DATA.B[4] >> 5) & 0x03;
			WheelSpeedFrontRightValid = (uint8_t)(  mb_msg.DATA.B[4] >> 7) & 0x01;
			WheelSpeedFrontRightData = ((uint16_t)(((mb_msg.DATA.B[4] & 0x1F) << 8) | mb_msg.DATA.B[5]))*0.05625;

			WheelSpeedFrontLeftDirection = (uint8_t)(mb_msg.DATA.B[6] >> 5) & 0x03;
			WheelSpeedFrontLeftValid = (uint8_t)(  mb_msg.DATA.B[6] >> 7) & 0x01;
			WheelSpeedFrontLeftData = ((uint16_t)(((mb_msg.DATA.B[6] & 0x1F) << 8) | mb_msg.DATA.B[7]))*0.05625;
			break;

		case 0x218://vehicle speed
			VehicleSpeedValid = (uint8_t)( mb_msg.DATA.B[4] >> 5 ) & 0x01;
			VehicleSpeed = (float)(((uint16_t)(((mb_msg.DATA.B[4] & 0x1F) << 8) | mb_msg.DATA.B[5])) * 0.05625);
			break;

		case 0x258://Wheel speed pulse
			WheelSpeedDirection = (uint8_t)(mb_msg.DATA.B[2] & 0x03);
			WheelSpeedRearRightPulse  = (uint8_t)mb_msg.DATA.B[4];
			WheelSpeedRearLeftPulse   = (uint8_t)mb_msg.DATA.B[5];
			WheelSpeedFrontRightPulse = (uint8_t)mb_msg.DATA.B[6];
			WheelSpeedFrontLeftPulse  = (uint8_t)mb_msg.DATA.B[7];
			break;

		case 0x277://ESP
			ESP_QDC_ACC = (uint8_t)( (mb_msg.DATA.B[3] >> 1) & 0x03);
			break;

		case 0x26A://EMS
			EMS_QEC_ACC = (uint8_t)( (mb_msg.DATA.B[0] >> 1) & 0x03);
			break;

		case 0x180://actual steering angle
			SteeringAngleActual = (int16_t)(((int16_t)((mb_msg.DATA.B[0] << 8) | mb_msg.DATA.B[1])) * 0.1);
			SteeringAngleSpeed = (uint16_t)(mb_msg.DATA.B[2] * 4);
			SteeringAngleValid = (uint8_t)( mb_msg.DATA.B[3] >> 7 ) & 0x01;
			SAS_Failure = (uint8_t)( mb_msg.DATA.B[3] >> 6 ) & 0x01;
			break;

		default:

			break;
	}
}

void Terminal::VehicleInformation(vuint32_t id,vuint8_t dat[])
{
	switch(id)
	{
		case 0x2A0://eps status
			EPS_Failed = (uint8_t)((dat[1] >> 7) & 0x01);
			APA_EPAS_Failed = (uint8_t)((dat[1] >> 1) & 0x01);
			TorqueSensorStatus = (uint8_t)( dat[1] & 0x01 );
			APA_ControlFeedback = (uint8_t)((dat[3] >> 5) & 0x01);
			SteeringTorque = (float)(dat[2] * 0.1794 - 22.78);
			SteeringAngleControlStateMachine();
			break;

		case 0x208:// wheel speed
			WheelSpeedRearRightDirection = (uint8_t)(dat[0] >> 5) & 0x03;
			WheelSpeedRearRightValid = (uint8_t)(  dat[0] >> 7) & 0x01;
			WheelSpeedRearRightData = ((uint16_t)(((dat[0] & 0x1F) << 8) | dat[1])) * V_M_S;

			WheelSpeedRearLeftDirection = (uint8_t)(dat[2] >> 5) & 0x03;
			WheelSpeedRearLeftValid = (uint8_t)(  dat[2] >> 7) & 0x01;
			WheelSpeedRearLeftData = ((uint16_t)(((dat[2] & 0x1F) << 8) | dat[3])) * V_M_S;

			WheelSpeedFrontRightDirection = (uint8_t)(dat[4] >> 5) & 0x03;
			WheelSpeedFrontRightValid = (uint8_t)(  dat[4] >> 7) & 0x01;
			WheelSpeedFrontRightData = ((uint16_t)(((dat[4] & 0x1F) << 8) | dat[5])) * V_M_S;

			WheelSpeedFrontLeftDirection = (uint8_t)(dat[6] >> 5) & 0x03;
			WheelSpeedFrontLeftValid = (uint8_t)(  dat[6] >> 7) & 0x01;
			WheelSpeedFrontLeftData = ((uint16_t)(((dat[6] & 0x1F) << 8) | dat[7])) * V_M_S;
			break;

		case 0x218://vehicle speed
			VehicleSpeedValid = (uint8_t)( dat[4] >> 5 ) & 0x01;
			VehicleSpeed = (float)(((uint16_t)(((dat[4] & 0x1F) << 8) | dat[5])) * V_M_S);
			break;

		case 0x258://Wheel speed pulse
			WheelSpeedDirection = (uint8_t)(dat[2] & 0x03);
			WheelSpeedRearRightPulse  = (uint8_t)dat[4];
			WheelSpeedRearLeftPulse   = (uint8_t)dat[5];
			WheelSpeedFrontRightPulse = (uint8_t)dat[6];
			WheelSpeedFrontLeftPulse  = (uint8_t)dat[7];
			break;

		case 0x277://ESP
			ESP_QDC_ACC = (uint8_t)( (dat[3] >> 1) & 0x03);
			break;

		case 0x26A://EMS
			EMS_QEC_ACC = (uint8_t)( (dat[0] >> 1) & 0x03);
			break;

		case 0x180://actual steering angle
			SteeringAngleActual = (float)(((int16_t)((dat[0] << 8) | dat[1])) * 0.1);
			SteeringAngleSpeed = (uint16_t)(dat[2] * 4);
			SteeringAngleValid = (uint8_t)( dat[3] >> 7 ) & 0x01;
			SAS_Failure = (uint8_t)( dat[3] >> 6 ) & 0x01;
			break;

		default:

			break;
	}
}

// Terminal control
void Terminal::TerminalControlCommandReceive(uint8_t data)
{
	uint8_t i;
	switch(_terminal_frame)
	{
		case FirstHead1:
			if(data == 0xAA)
			{
				_check_sum = data;
				_terminal_frame = FirstHead2;
			}
			break;
		case FirstHead2:
			if(data == 0x55)
			{
				_check_sum += data;
				_terminal_frame = ID;
			}
			else
			{
				_terminal_frame = FirstHead1;
			}
			break;

		case ID:
			_frame_id = data;
			_check_sum += data;
			_terminal_frame = Length;
			break;

		case Length:
			_frame_length = data;
			_check_sum += data;
			_frame_cnt = 0;
			_terminal_frame = Data;
		break;

		case Data:
			_data_buffer[_frame_cnt] = data;
			_check_sum += data;
			_frame_cnt++;
			if(_frame_cnt >= _frame_length)
			{
				_terminal_frame = CheckSum;
			}
			break;

		case CheckSum:
			if(_check_sum == data)
			{
				switch(_frame_id)
				{
				case 0x1A:
					GearShiftEnable = _data_buffer[0] & 0x01;
					GearShiftValid = (_data_buffer[0] >> 1) & 0x01;

					if( (((_data_buffer[0] >> 2) & 0x01) == 0) || (SteeringAngleTargetActive == 0))
					{
						SteeringAngleTargetActive = (_data_buffer[0] >> 2) & 0x01;
					}

					TorqueEnable = (_data_buffer[0] >> 3) & 0x01;
					TargetDecelerationEnable = (_data_buffer[0] >> 4) & 0x01;
					TargetAccelerationEnable = (_data_buffer[0] >> 5) & 0x01;

					GearShift = _data_buffer[1];

					Torque = ((uint16_t)((_data_buffer[3] << 8) | _data_buffer[2]))*0.1;

					SteeringAngleTarget = ((int16_t)((_data_buffer[5] << 8) | _data_buffer[4])) * 0.1;

					SteeringAngleSpeedTarget = ((uint16_t)((_data_buffer[7] << 8) | _data_buffer[6])) * 0.01;

					for(i = 0;i<4;i++)
					{
						_data_temp.b[3-i] = _data_buffer[i + 8];
					}
					TargetDecelerationAEB = _data_temp.f;
					for(i = 0;i<4;i++)
					{
						_data_temp.b[3-i] = _data_buffer[i + 12];
					}
					TargetAccelerationACC = _data_temp.f;
					break;

				case 0x2A:
					for(i = 0;i<4;i++)
					{
						_data_temp.b[3-i] = _data_buffer[i];
					}
					KP = _data_temp.f;

					for(i = 0;i<4;i++)
					{
						_data_temp.b[3-i] = _data_buffer[i + 4];
					}
					KI = _data_temp.f;

					for(i = 0;i<4;i++)
					{
						_data_temp.b[3-i] = _data_buffer[i + 8];
					}
					KD = _data_temp.f;
					break;

				case 0x2B:// Threshold
					for(i = 0;i<4;i++)
					{
						_data_temp.b[3-i] = _data_buffer[i];
					}
					Threshold = _data_temp.f;
					break;

				case 0x3A:// Target Vehicle Set
					for(i = 0;i<4;i++)
					{
						_data_temp.b[3-i] = _data_buffer[i];
					}
					VehicleSpeedTarget= _data_temp.f;
					Desired = VehicleSpeedTarget;
					break;

				case 0x4A:
					// TODO 临时修改
					WorkingModule = _data_buffer[0];
					FunctionStatus = _data_buffer[1];
					break;

				case 0x5A:
					//Gear Enable Single
					GearShiftEnable = _data_buffer[0] & 0x01;
					GearShiftValid  = _data_buffer[0] & 0x01;

					/// Steering Angle Active
					if( (((_data_buffer[0] >> 1) & 0x01) == 0) || (SteeringAngleTargetActive == 0))
					{
						SteeringAngleTargetActive = (_data_buffer[0] >> 1) & 0x01;
					}

					/// ACC Enable Single
					TargetAccelerationEnable = (_data_buffer[0] >> 2) & 0x01;

					/// Speed Control Single
					VehicleSpeedControlEnable = (_data_buffer[0] >> 3) & 0x01;

					/// Gear Value
					GearShift = _data_buffer[1];

					/// Steering Angle Value
					SteeringAngleTarget = ((int16_t)((_data_buffer[3] << 8) | _data_buffer[2])) * 0.1;

					/// Steering Angle Value
					SteeringAngleSpeedTarget = ((uint16_t)((_data_buffer[5] << 8) | _data_buffer[4])) * 0.01;

					/// ACC Value
					if(!VehicleSpeedControlEnable)
					{
						for(i = 0;i<4;i++)
						{
							_data_temp.b[3-i] = _data_buffer[i + 6];
						}
						TargetAccelerationACC = _data_temp.f;						
					}
					/// Target Vehicle Speed Value
					VehicleSpeedTarget = _data_buffer[10]*0.01;
					Desired = VehicleSpeedTarget;
					break;

				default:

					break;
				}
				// ACK Single
				AckValid = 0x5A;
				TerminalControlAckSend();
			}
			else
			{
				_frame_err_cnt++;
			}
			_terminal_frame = FirstHead1;
			break;
	}
}

void Terminal::TerminalControlCommandSend(void)
{
	uint8_t i,check_sum;
	Byte2Float data_temp;
	Byte2Int data_temp_int;

	_send_data_buffer[0] = 0x7F;
	_send_data_buffer[1] = 0x80;
	_send_data_buffer[2] = 0x85;
	_send_data_buffer[3] = 19;
	_send_data_buffer[4] = ((EMS_QEC_ACC << 2) & 0x04) | ((ESP_QDC_ACC << 1) & 0x02) | APA_EPAS_Failed;
	_send_data_buffer[5] = 	(  TargetAccelerationEnable         & 0x01) |
							( (TargetDecelerationEnable   << 1) & 0x02) |
							( (TorqueEnable               << 2) & 0x04) |
							( (GearShiftEnable            << 3) & 0x08) |
							( (SteeringAngleTargetActive  << 4) & 0x30) ;

	_send_data_buffer[6] = (uint8_t)(SteeringAngleActual & 0xff);
	_send_data_buffer[7] = (uint8_t)((SteeringAngleActual >> 8) & 0xff);

	_send_data_buffer[8] = (uint8_t)(SteeringAngleSpeed & 0xff);
	_send_data_buffer[9] = (uint8_t)((SteeringAngleSpeed >> 8) & 0xff);

	data_temp.f = SteeringTorque;
	for(i = 0; i < 4; i++)
	{
		_send_data_buffer[i + 10] = data_temp.b[3-i] ;
	}

	data_temp.f = TargetAccelerationACC;
	for(i = 0; i < 4; i++)
	{
		_send_data_buffer[i + 14] = data_temp.b[3-i] ;
	}

	data_temp_int.u16 = (uint16_t)(Torque * 10);
	_send_data_buffer[18] = data_temp_int.b[1];
	_send_data_buffer[19] = data_temp_int.b[0];


	_send_data_buffer[20] = 0;
	_send_data_buffer[21] = 0;
	_send_data_buffer[22] = 0;

	check_sum = 0;
	for(i=0;i<23;i++)
	{
		check_sum += _send_data_buffer[i];
	}
	_send_data_buffer[23] = check_sum;

	for(i=0;i<24;i++)
	{
		TransmitData(_send_data_buffer[i]);
	}
}

void Terminal::TerminalControlSpeedSend(void)
{
	uint8_t i,check_sum;
	_send_data_buffer[0] = 0x7F;
	_send_data_buffer[1] = 0x80;
	_send_data_buffer[2] = 0x95;
	_send_data_buffer[3] = 19;
	_speed_data_temp.f = VehicleSpeed;
	for(i = 0;i<4;i++)
	{
		_send_data_buffer[i + 4] = _speed_data_temp.b[3-i] ;
	}

	_send_data_buffer[8] = (uint8_t)(( (uint16_t)(WheelSpeedFrontLeftData*10)    ) & 0xff);
	_send_data_buffer[9] = (uint8_t)((((uint16_t)(WheelSpeedFrontLeftData*10))>>8) & 0xff);

	_send_data_buffer[10] = (uint8_t)(( (uint16_t)(WheelSpeedFrontRightData*10)    ) & 0xff);
	_send_data_buffer[11] = (uint8_t)((((uint16_t)(WheelSpeedFrontRightData*10))>>8) & 0xff);

	_send_data_buffer[12] = (uint8_t)(( (uint16_t)(WheelSpeedRearLeftData*10)    ) & 0xff);
	_send_data_buffer[13] = (uint8_t)((((uint16_t)(WheelSpeedRearLeftData*10))>>8) & 0xff);

	_send_data_buffer[14] = (uint8_t)(( (uint16_t)(WheelSpeedRearRightData*10)    ) & 0xff);
	_send_data_buffer[15] = (uint8_t)((((uint16_t)(WheelSpeedRearRightData*10))>>8) & 0xff);

	_send_data_buffer[16] = WheelSpeedFrontLeftPulse;
	_send_data_buffer[17] = WheelSpeedFrontRightPulse;
	_send_data_buffer[18] = WheelSpeedRearLeftPulse;
	_send_data_buffer[19] = WheelSpeedRearRightPulse;

	_send_data_buffer[20] = WheelSpeedDirection;
	_send_data_buffer[21] = 0;
	_send_data_buffer[22] = 0;

	check_sum = 0;
	for(i=0;i<23;i++)
	{
		check_sum += _send_data_buffer[i];
	}
	_send_data_buffer[23] = check_sum;
	for(i=0;i<24;i++)
	{
		TransmitData(_send_data_buffer[i]);
	}
}

void Terminal::TerminalSystemStateSend(void)
{
	uint8_t i,check_sum;

	_send_data_buffer[0] = 0x7F;
	_send_data_buffer[1] = 0x80;
	_send_data_buffer[2] = 0xA5;
	_send_data_buffer[3] = 3;
	// TODO 临时修改
	_send_data_buffer[4] = WorkingModule;
	_send_data_buffer[5] = FunctionStatus;

	_send_data_buffer[6] = 0;

	check_sum = 0;
	for(i=0;i<7;i++)
	{
		check_sum += _send_data_buffer[i];
	}
	_send_data_buffer[7] = check_sum;
	for(i=0;i<8;i++)
	{
		TransmitData(_send_data_buffer[i]);
	}
}

void Terminal::TerminalControlAckSend(void)
{
	uint8_t i,check_sum;
	_send_data_buffer[0] = 0x7F;
	_send_data_buffer[1] = 0x80;
	_send_data_buffer[2] = _frame_id;
	_send_data_buffer[3] = 1;
	_send_data_buffer[4] = 0x5A;

	check_sum = 0;
	for(i=0;i<5;i++)
	{
		check_sum += _send_data_buffer[i];
	}
	_send_data_buffer[5] = check_sum;
	for(i=0;i<6;i++)
	{
		TransmitData(_send_data_buffer[i]);
	}
}

/*
 * dong_feng_e70_controller.cpp
 *
 *  Created on: 2019年6月20日
 *      Author: zhuguohua
 */

#include <DongFengE70/dong_feng_e70_controller.h>
uint8_t time_cnt;
DongFengE70Controller::DongFengE70Controller() {
	// TODO Auto-generated constructor stub

}

DongFengE70Controller::~DongFengE70Controller() {
	// TODO Auto-generated destructor stub
}

void DongFengE70Controller::Init()
{
	_apa_torque_control_state = TorqueWaitEnable;
	_apa_turn_control_state   = TurnWaitHand;
}

void DongFengE70Controller::Start()
{

}

void DongFengE70Controller::Stop()
{

}

void DongFengE70Controller::Update(ControlCommand cmd)
{
	SteeringEnable  	= cmd.ControlEnable.B.SteeringEnable;

	AccelerationEnable  = cmd.ControlEnable.B.AccelerationEnable;
	DecelerationEnable  = cmd.ControlEnable.B.DecelerationEnable;
	TorqueEnable 	    = cmd.ControlEnable.B.TorqueEnable;
	VelocityEnable      = cmd.ControlEnable.B.VelocityEnable;


	Gear			  = cmd.Gear;
	SteeringAngle 	  = cmd.SteeringAngle;
	SteeringAngleRate = cmd.SteeringAngleRate;

	if(0 == VelocityEnable)
	{
		Acceleration 	  = cmd.Acceleration;
	}
	Deceleration 	  = cmd.Deceleration;
	Torque 			  = cmd.Torque;
	Velocity          = cmd.Velocity;
}

void DongFengE70Controller::Update(APAControlCommand cmd)
{


}

void DongFengE70Controller::VehicleContorl_20ms(void)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.length = 8;
	uint8_t i,crc_temp;

	/*
	 * 	VCU
	 * */
	m_CAN_Packet.id = 0x175;
	_apa_troque  = (uint16_t)Torque;
	_apa_max_troque = 1000;
	/// Data Mapping
	m_CAN_Packet.data[0] = (uint8_t)( ((_apa_vcu_control << 7 ) & 0x80) |
			                          ((Gear             << 4 ) & 0x70) |
									  ((_apa_max_troque  >> 8 ) & 0x0f) );

	m_CAN_Packet.data[1] = (uint8_t)(_apa_max_troque & 0xff);

	m_CAN_Packet.data[2] = (uint8_t)( ((_apa_epb_request << 4 ) & 0x30) |
			                          ((_apa_troque      >> 8 ) & 0x0f) );
	m_CAN_Packet.data[3] = (uint8_t)(_apa_troque & 0xff);

	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = (uint8_t)(_apa_work_status & 0x03);
	m_CAN_Packet.data[6] = (uint8_t)(_apa_rolling_counter & 0x0f);
	// TODO 校验方式待确认
	crc_temp = 0;
	for(i=0;i<7;i++)
	{
		crc_temp = crc_temp ^ m_CAN_Packet.data[i];
	}
	m_CAN_Packet.data[7] = crc_temp & 0xff;
	CAN0_TransmitMsg(m_CAN_Packet);
	_apa_rolling_counter++;
}

void DongFengE70Controller::VehicleContorl_10ms(void)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.length = 8;
	uint8_t i,crc_temp;

	/*
	 * ESP
	 * */
	m_CAN_Packet.id = 0x80;
	_apa_angle_wheel       = (int16_t)( SteeringAngle * 10);
	_apa_anglular_velocity = (uint8_t )(SteeringAngleRate * 0.5);
	/// Data Mapping
	m_CAN_Packet.data[0] = (uint8_t)((_apa_auto_drive_enable << 4) & 0x70);
	m_CAN_Packet.data[1] = 0;
	m_CAN_Packet.data[2] = (uint8_t)( (_apa_angle_wheel >> 8) & 0xff );
	m_CAN_Packet.data[3] = (uint8_t)(  _apa_angle_wheel       & 0xff );

	m_CAN_Packet.data[4] = _apa_anglular_velocity;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = (uint8_t)(_apa_rolling_count_had1 & 0x0f);
	// TODO 校验方式待确认
	crc_temp = 0;
	for(i=0;i<7;i++)
	{
		crc_temp = crc_temp ^ m_CAN_Packet.data[i];
	}
	m_CAN_Packet.data[7] = crc_temp & 0xff;
	CAN0_TransmitMsg(m_CAN_Packet);
	_apa_rolling_count_had1++;
	/*
	 * ESC
	 * */
	m_CAN_Packet.id = 0x8B;
	_apa_esc_brake_req = (uint16_t)( (Acceleration + 27) * 100);
	/// Data Mapping
	m_CAN_Packet.data[0] = (uint8_t)( ((_apa_esc_control_req               ) & 0x01 ) |
									  ((_apa_esc_brake_req_valid_flag << 1 ) & 0x02 ) |
									  ((_apa_esc_manuver_state        << 2 ) & 0x04 ) |
									  ((_apa_esc_prefill_req          << 3 ) & 0x08 ) );
	m_CAN_Packet.data[1] = (uint8_t)( (_apa_esc_brake_req >> 8) & 0x0f );
	m_CAN_Packet.data[2] = (uint8_t)(  _apa_esc_brake_req       & 0xff );
	m_CAN_Packet.data[3] = 0;

	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = (uint8_t)(_apa_esc_rolling_count & 0x0f);
	// TODO 校验方式待确认
	crc_temp = 0;
	for(i=0;i<7;i++)
	{
		crc_temp = crc_temp ^ m_CAN_Packet.data[i];
	}
	m_CAN_Packet.data[7] = crc_temp & 0xff;
	m_CAN_Packet.data[7] = 0;
	CAN0_TransmitMsg(m_CAN_Packet);
	_apa_esc_rolling_count++;
}

void DongFengE70Controller::APA_ControlStateMachine(uint8_t apa_ctl_sts,uint8_t esp_availab_sts)
{
	switch(_apa_torque_control_state)
	{
		case TorqueWaitEnable:
			if(TorqueEnable)
			{
				_apa_work_status = 1;
			}
			if(3 == apa_ctl_sts)
			{
				_apa_work_status = 2;
				_apa_vcu_control = 1;
				_apa_torque_control_state = TorqueWaitActive;
			}
			break;

		case TorqueWaitActive:
			if(2 == apa_ctl_sts)
			{
				_apa_torque_control_state = TorqueWaitDisable;
			}
			break;

		case TorqueWaitDisable:
			if(0 == TorqueEnable)
			{
				_apa_work_status = 0;
				_apa_vcu_control = 0;
				_apa_torque_control_state = TorqueWaitEnable;
			}
			break;

		default:

			break;
	}
//	if(TorqueEnable)
//	{
//		if(2 != apa_ctl_sts)
//		{
//			_apa_work_status = 2;
//			_apa_vcu_control = 1;
//		}
//	}
//	else
//	{
//		if(0 != apa_ctl_sts)
//		{
//			_apa_work_status = 0;
//			_apa_vcu_control = 0;
//		}
//	}

	switch(_apa_turn_control_state)
	{
		case TurnWaitHand:
			if(1 == esp_availab_sts)
			{
				if(SteeringEnable)
				{
					_apa_auto_drive_enable = 6;
					_apa_turn_control_state = TurnWiatActive;
				}
			}
			break;

		case TurnWiatActive:
			if(2 == esp_availab_sts)
			{
				_apa_turn_control_state = TurnWaitDisable;
			}
			break;

		case TurnWaitDisable:
			if(0 == SteeringEnable)
			{
				_apa_auto_drive_enable = 0;
				_apa_turn_control_state = TurnWaitHand;
			}
			break;

		default:
			break;

	}

//	if(SteeringEnable)
//	{
//		if(1 == esp_availab_sts)
//		{
//			_apa_auto_drive_enable = 6;
//		}
//	}
//	else
//	{
//		if(2 == esp_availab_sts)
//		{
//			_apa_auto_drive_enable = 0;
//		}
//	}

	if(AccelerationEnable)
	{
		_apa_esc_brake_req_valid_flag = 1;
	}
	else
	{
		_apa_esc_brake_req_valid_flag = 0;
	}

}

void DongFengE70Controller::Push(float dt)
{
	time_cnt = (time_cnt + 1)%2;
	if(1 == time_cnt)
	{
		VehicleContorl_20ms();
	}
	VehicleContorl_10ms();
}

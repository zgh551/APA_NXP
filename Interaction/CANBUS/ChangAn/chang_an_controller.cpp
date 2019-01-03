/*
 * chang_an_controller.cpp
 *
 *  Created on: December 28 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: chang_an_controller.cpp             COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: chang an vehicle control class     					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 28 2018    Initial Version                  */
/*****************************************************************************/

#include <ChangAn/chang_an_controller.h>

ChangAnController::ChangAnController() {
	// TODO Auto-generated constructor stub
	Init();
}

ChangAnController::~ChangAnController() {
	// TODO Auto-generated destructor stub
}

void ChangAnController::Init()
{
	_steerig_angle_active_control_state = WaitSteeringAngleControlSingleState;
	_rolling_counter_torque_AEB 		= 0;
	_rolling_counter_brake_ACC 			= 0;
	_rolling_counter_steering_control 	= 0;
	_rolling_counter_gear_control 		= 0;

	_target_acceleration_acc 	= 0;
	_target_acceleration_enable = 0;
	// current value
	_current_target_acceleration_ACC 			= 0;
	_current_target_acceleration_enable_single 	= 0;

	/* AEB */
	// actual value
	_target_deceleration_aeb 	= 0;
	_target_deceleration_enable = 0;
	// current value
	_current_target_deceleration_AEB 			= 0;
	_current_target_deceleration_enable_single 	= 0;

	/* Torque */
	// actual value
	_torque 		= 0;
	_torque_enable 	= 0;
	// current value
	_current_torque = 0;
	_current_torque_enable_single = 0;

	/* SteeringAngle */
	// actual value
	_steering_angle_set 			= 0;
	_steering_angle_target 			= 0;
	_steering_angle_rate_target 	= 0;
	_steering_angle_target_active 	= 0;
	// current value
	_current_steering_angle_target 					= 0;
	_current_steering_angle_target_active_single 	= 0;

	/* Gear */
	// actual value
	_gear 			= 0;
	_gear_enable 	= 0;
	_gear_valid 	= 0;
	// current value
	_current_gear 				= 0;
	_current_gear_enable_single = 0;
	_current_gear_valid_single 	= 0;
}




void ChangAnController::Start()
{
	_target_acceleration_enable = 1;
	_target_deceleration_enable  = 1;
	_torque_enable = 1;
	_gear_enable = 1;
	_gear_valid = 1;
	_steering_angle_target_active = 1;
}

void ChangAnController::Stop()
{
	_target_acceleration_enable  = 0;
	_target_deceleration_enable  = 0;
	_torque_enable 	= 0;
	_gear_enable 	= 0;
	_gear_valid 	= 0;
	_steering_angle_target_active = 0;
}

void ChangAnController::Update(ControlCommand cmd)
{
	_torque 					= cmd.Torque;
	_gear						= cmd.Gear;
	_target_acceleration_acc 	= cmd.Acc;
	_target_deceleration_aeb 	= cmd.Aeb;

	_steering_angle_target 		= cmd.SteeringAngle;
	_steering_angle_rate_target = cmd.SteeringAngleRate;
}

void ChangAnController::Push(float dt)
{
	this->SteeringAngleControl(dt);
	this->VehicleContorl();
}

void ChangAnController::VehicleContorlStep1()
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x6fe;
	m_CAN_Packet.length = 8;
	/// data buffer
	/* ACC */
	_current_target_acceleration_ACC = (uint8_t)((_target_acceleration_acc + 5.0)*20);
	_current_target_acceleration_enable_single = _target_acceleration_enable;
	/* AEB */
	_current_target_deceleration_AEB = (uint16_t)((_target_deceleration_aeb + 16.0) * 2000);
	_current_target_deceleration_enable_single = _target_deceleration_enable;
	/* Torque */
	_current_torque = (uint16_t)(_torque * 10.23);
	_current_torque_enable_single = _torque_enable;
	/* Steering Angle */
	_current_steering_angle_target = (int16_t)(_steering_angle_set * 10);
	_current_steering_angle_target_active_single = _steering_angle_target_active;

	/// Data Mapping
	m_CAN_Packet.data[0] = _current_target_acceleration_ACC;
	m_CAN_Packet.data[1] = (uint8_t)((_current_target_deceleration_AEB >> 8) & 0xFF);
	m_CAN_Packet.data[2] = (uint8_t)((_current_target_deceleration_AEB     ) & 0xFF);
	m_CAN_Packet.data[3] = (uint8_t)( _rolling_counter_torque_AEB & 0x0F);
	m_CAN_Packet.data[3] = _current_target_acceleration_enable_single ?
						   (uint8_t) ( m_CAN_Packet.data[3] | 0x80 ) :
						   (uint8_t) ( m_CAN_Packet.data[3] & 0x7F ) ;
	m_CAN_Packet.data[3] = _current_target_deceleration_enable_single ?
						   (uint8_t) ( m_CAN_Packet.data[3] | 0x40 ) :
						   (uint8_t) ( m_CAN_Packet.data[3] & 0xBF ) ;
	m_CAN_Packet.data[4] = (uint8_t)((_current_torque >> 2) & 0xFF);
	m_CAN_Packet.data[5] = (uint8_t)((_current_torque << 6) & 0xC0);
	m_CAN_Packet.data[5] = _current_torque_enable_single              ?
						   (uint8_t) ( m_CAN_Packet.data[5] | 0x20 ) :
						   (uint8_t) ( m_CAN_Packet.data[5] & 0xDF ) ;
	m_CAN_Packet.data[5] = (uint8_t) ((m_CAN_Packet.data[5] & 0xFC ) |
									  ( _current_steering_angle_target_active_single & 0x03));
	m_CAN_Packet.data[6] = (uint8_t)((_current_steering_angle_target >> 8) & 0xFF);
	m_CAN_Packet.data[7] = (uint8_t)((_current_steering_angle_target     ) & 0xFF);
	CAN0_TransmitMsg(m_CAN_Packet);
}

void ChangAnController::VehicleContorlStep2()
{
	uint8_t i;
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x6FF;
	m_CAN_Packet.length = 8;
	/// data buffer
	_current_gear = _gear;
	_current_gear_enable_single = _gear_enable;
	_current_gear_valid_single  = _gear_valid;

	/// data mapping
	for(i=0;i<4;i++){m_CAN_Packet.data[i] = 0;}
	m_CAN_Packet.data[4] = (uint8_t)(_rolling_counter_brake_ACC & 0x0f);
	m_CAN_Packet.data[5] = (uint8_t)((_current_gear << 4 ) & 0x70);
	m_CAN_Packet.data[5] = _current_gear_enable_single          ?
						   (uint8_t) ( m_CAN_Packet.data[5] | 0x80 ) :
						   (uint8_t) ( m_CAN_Packet.data[5] & 0x7F ) ;
	m_CAN_Packet.data[5] = _current_gear_valid_single          ?
						   (uint8_t) ( m_CAN_Packet.data[5] | 0x08 ) :
						   (uint8_t) ( m_CAN_Packet.data[5] & 0xF7 ) ;

	m_CAN_Packet.data[6] = (uint8_t)(
								(
									( _rolling_counter_brake_ACC                 & 0x0f )
								+ 	((_current_target_acceleration_enable_single & 0x01 ) << 4)
								+ 	  _current_target_acceleration_ACC
								)^0xFF
							);
	m_CAN_Packet.data[7] = (uint8_t)(
								(
									( _rolling_counter_torque_AEB   & 0x0f)
								+	((_current_torque_enable_single & 0x01) << 2)
								+	((_current_torque & 0x001F) << 3)
								+	((_current_torque & 0x03E0) >> 5)
								+	((_current_target_deceleration_enable_single & 0x01) << 7)
								+	((_current_target_deceleration_AEB >> 8) & 0xFF)
								+	((_current_target_deceleration_AEB     ) & 0xFF)
								) ^ 0xFF
							);
	CAN0_TransmitMsg(m_CAN_Packet);
}

void ChangAnController::VehicleContorlStep3()
{
	uint8_t i;
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x6E9;
	m_CAN_Packet.length = 8;

	/// data mapping
	m_CAN_Packet.data[0] = 0;
	m_CAN_Packet.data[1] = (uint8_t)(_rolling_counter_steering_control & 0x0f);
	m_CAN_Packet.data[2] = (uint8_t)	(
											(
												(_rolling_counter_steering_control & 0x0f)
											+	((_current_steering_angle_target >> 8) & 0xFF)
											+	((_current_steering_angle_target     ) & 0xFF)
											+	_current_steering_angle_target_active_single
											) ^ 0xFF
										);
	m_CAN_Packet.data[3] = (uint8_t)	(
											(
												(((_current_gear_enable_single & 0x01) << 2)
												^(((_current_gear_valid_single & 0x01) << 3) + _current_gear)
												^(_rolling_counter_gear_control & 0x0F)) << 4
											)|(_rolling_counter_gear_control & 0x0F)
										);
	for( i = 4 ;i < 8 ; i++ ){m_CAN_Packet.data[i] = 0;}
	CAN0_TransmitMsg(m_CAN_Packet);
}

void ChangAnController::VehicleContorl()
{
	VehicleContorlStep1();
	VehicleContorlStep2();
	VehicleContorlStep3();
	_rolling_counter_torque_AEB++;
	_rolling_counter_brake_ACC++;
	_rolling_counter_steering_control++;
	_rolling_counter_gear_control++;
}

void ChangAnController::SteeringAngleControl(float dt)
{
    float da = _steering_angle_rate_target * dt;
    float left_target_angle = _steering_angle_target - da;
    float right_target_angle = _steering_angle_target + da;

    if(_steering_angle_set < left_target_angle)
    {
    	_steering_angle_set += da;
    }
    else if(_steering_angle_set > right_target_angle)
    {
    	_steering_angle_set -= da;
    }
    else
    {
    	_steering_angle_set = _steering_angle_target;
    }
}

void ChangAnController::SteeringAngleControlStateMachine(uint8_t fd)
{
	switch(_steerig_angle_active_control_state)
	{
		case WaitSteeringAngleControlSingleState:
			if(1 == _steering_angle_target_active)
			{
				_steerig_angle_active_control_state = WaitFeedbackSingleState;
			}
			break;

		case WaitFeedbackSingleState:
			if(fd)
			{
				_steering_angle_target_active = 2;
				_steerig_angle_active_control_state = WaitExistState;
			}
			break;

		case WaitExistState:
			if( (!fd) | ( 0 == _steering_angle_target_active ))
			{
				_steerig_angle_active_control_state = WaitSteeringAngleControlSingleState;
			}
			break;

		default:
			_steerig_angle_active_control_state = WaitSteeringAngleControlSingleState;
			break;
	}
}

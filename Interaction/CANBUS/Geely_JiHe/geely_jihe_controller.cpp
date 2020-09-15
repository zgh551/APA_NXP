/*
 * bo_rui_controller.cpp
 *
 *  Created on: 2019楠烇拷3閺堬拷15閺冿拷
 *      Author: zhuguohua
 */

#include "geely_jihe_controller.h"

GeelyJiHeController::GeelyJiHeController() {
	// TODO Auto-generated constructor stub

}

GeelyJiHeController::~GeelyJiHeController() {
	// TODO Auto-generated destructor stub
}

void GeelyJiHeController::Init()
{

}
/**
 * @brief start the vehicle controller.
 * @return true if successfully started.
 */
void GeelyJiHeController::Start()
{

}

/**
 * @brief stop the vehicle controller.
 */
void GeelyJiHeController::Stop()
{

}
  /**
   * @brief update the vehicle controller.
   * @param command the control command
   * @return error_code
   */
void GeelyJiHeController::Update(ControlCommand cmd)
{

}

void GeelyJiHeController::Update(APAControlCommand cmd)
{
	APAEnable         = cmd.ControlEnable.B.APAEnable;

	Gear              = cmd.Gear;

	SteeringAngle 	  = cmd.SteeringAngle;
	SteeringAngleRate = cmd.SteeringAngleRate;

	Velocity          = cmd.Velocity;
	Distance          = cmd.Distance;
}


void GeelyJiHeController::VehicleContorl()
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x190;
	m_CAN_Packet.length = 8;

	_current_steering_angle_target = (int16_t)(this->getSteeringAngleSet() * 10);
	_current_distance = (uint16_t)(this->getDistanceSet() * 100);
	/// Data Mapping
	m_CAN_Packet.data[0] = 0;
	m_CAN_Packet.data[1] = 0;
	m_CAN_Packet.data[2] = 0;
	m_CAN_Packet.data[3] = (uint8_t)((this->getEpsRequest() & 0x03) << 3 );

	m_CAN_Packet.data[4] = (uint8_t)((_current_steering_angle_target >> 8) & 0xFF);
	m_CAN_Packet.data[5] = (uint8_t)((_current_steering_angle_target     ) & 0xFF);
	m_CAN_Packet.data[6] = (uint8_t)(this->getRollingCounter() & 0x0f);
	m_CAN_Packet.data[7] = 0;
	CAN0_TransmitMsg(m_CAN_Packet);
}

void GeelyJiHeController::VehicleContorlPri()
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x412;
	m_CAN_Packet.length = 8;

	_current_turn_torque_value = (uint16_t)(TurnTorqueVal * 100);
	_current_acceleration  = Acceleration * 20;
	/// Data Mapping
	m_CAN_Packet.data[0] = (uint8_t)((_current_turn_torque_value >> 2) & 0xFF);
	m_CAN_Packet.data[1] = (uint8_t)(((_current_turn_torque_value << 6) & 0xC0) | ((TurnTorqueDir << 5) & 0x20));

	m_CAN_Packet.data[2] = (uint8_t)((TurnTorqueAct << 7) & 0x80);
	m_CAN_Packet.data[3] = _current_acceleration;

	m_CAN_Packet.data[4] = (uint8_t)((AccelerationEnable << 4) & 0xf0);
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN0_TransmitMsg(m_CAN_Packet);
}

void GeelyJiHeController::SteeringAngleControl(float dt)
{
    float da = this->getSteeringAngleRate() * dt;
    float left_target_angle  = this->getSteeringAngle() - da;
    float right_target_angle = this->getSteeringAngle() + da;

    if(this->getSteeringAngleSet() < left_target_angle)
    {
    	this->setSteeringAngleSet(this->getSteeringAngleSet() + da);
    }
    else if(this->getSteeringAngleSet() > right_target_angle)
    {
    	this->setSteeringAngleSet(this->getSteeringAngleSet() - da);
    }
    else
    {
    	this->setSteeringAngleSet(this->getSteeringAngle());
    }
}

void GeelyJiHeController::SteeringAngleControl(float dt,float actual_steering)
{
	if(getSteeringAngleRate() > 400.0f)
	{
		setSteeringAngleRate(400.0f);
	}
//	float delta_theta = getSteeringAngleRate() * dt;
    float da = getSteeringAngleRate() * (dt + 0.1);
    float left_target_angle  = getSteeringAngle() - da;
    float right_target_angle = getSteeringAngle() + da;

    // 瀵倸鐖舵潏鎾冲弳娑撳秴鎼锋惔锟�
    if( (fabs(getSteeringAngleRate()) < 501.0f) &&
    	(fabs(getSteeringAngle()) < 501.0f)
	  )
    {
		if(getSteeringAngleSet() < left_target_angle)
		{
			setSteeringAngleSet(actual_steering + da);
		}
		else if(getSteeringAngleSet() > right_target_angle)
		{
			setSteeringAngleSet(actual_steering - da);
		}
		else
		{
			if(actual_steering < left_target_angle)
			{
				setSteeringAngleSet(actual_steering + da);
			}
			else if(actual_steering > right_target_angle)
			{
				setSteeringAngleSet(actual_steering - da);
			}
			else
			{
				setSteeringAngleSet(getSteeringAngle());
			}
		}
    }
}

void GeelyJiHeController::Push(float dt)
{
//	SteeringAngleControl(dt);
//	VehicleContorl();
}

void GeelyJiHeController::Push(float dt,float actual_steering)
{
	SteeringAngleControl(dt,actual_steering);
	VehicleContorl();
}

void GeelyJiHeController::WorkStateMachine(MessageManager& msg)
{
	switch(_eps_control_state)
	{
	case EpsWaitRequest:
		if(1 == this->getAPAEnable())
		{
			this->setEpsRequest(1);
			_eps_control_state = EpsWiatEnable;
		}
		else
		{
			this->setEpsRequest(0);
			_eps_control_state = EpsWaitRequest;
		}
		break;

	case EpsWiatEnable:
		if(Enable == msg.getEPS_RequestFeedback())
		{
			this->setEpsRequest(2);
			_eps_control_state = EpsWaitDisable;
		}
		else
		{
			this->setEpsRequest(1);
			_eps_control_state = EpsWiatEnable;
		}
		break;

	case EpsWaitDisable:
		if((0 == this->getAPAEnable()) ||
		   (0 == msg.getEPS_RequestFeedback()))
		{
			this->setEpsRequest(0);
			_eps_control_state = EpsWaitRequest;
		}
		else
		{
			_eps_control_state = EpsWaitDisable;
		}
		break;

	default:
		break;
	}
}

void GeelyJiHeController::DataPush(void)
{
	SteeringAngleControl(0.02);
	VehicleContorl();
}

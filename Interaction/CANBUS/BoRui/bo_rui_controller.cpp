/*
 * bo_rui_controller.cpp
 *
 *  Created on: 2019年3月15日
 *      Author: zhuguohua
 */

#include "bo_rui_controller.h"

BoRuiController::BoRuiController() {
	// TODO Auto-generated constructor stub

}

BoRuiController::~BoRuiController() {
	// TODO Auto-generated destructor stub
}

void BoRuiController::Init()
{

}
/**
 * @brief start the vehicle controller.
 * @return true if successfully started.
 */
void BoRuiController::Start()
{

}

/**
 * @brief stop the vehicle controller.
 */
void BoRuiController::Stop()
{

}
  /**
   * @brief update the vehicle controller.
   * @param command the control command
   * @return error_code
   */
void BoRuiController::Update(ControlCommand cmd)
{

}

void BoRuiController::Update(APAControlCommand cmd)
{
	APAEnable         = cmd.ControlEnable.B.APAEnable;

	Gear              = cmd.Gear;

	SteeringAngle 	  = cmd.SteeringAngle;
	SteeringAngleRate = cmd.SteeringAngleRate;

	Velocity          = cmd.Velocity;
	Distance          = cmd.Distance;
}


void BoRuiController::VehicleContorl()
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x411;
	m_CAN_Packet.length = 8;

	_current_steering_angle_target = (int16_t)(this->getSteeringAngleSet() * 10);
	_current_distance = (uint16_t)(this->getDistanceSet() * 100);
	/// Data Mapping
	m_CAN_Packet.data[0] = (uint8_t)((_current_steering_angle_target >> 8) & 0xFF);
	m_CAN_Packet.data[1] = (uint8_t)((_current_steering_angle_target     ) & 0xFF);
	m_CAN_Packet.data[2] = (uint8_t)((_current_distance >> 8) & 0xFF);
	m_CAN_Packet.data[3] = (uint8_t)((_current_distance     ) & 0xFF);

	m_CAN_Packet.data[4] = (uint8_t)(this->getVelocity() * 36);
	m_CAN_Packet.data[5] = this->getGear();
	m_CAN_Packet.data[6] = (uint8_t)(this->getAPAEnable() << 6);
	m_CAN_Packet.data[7] = 0;
	CAN0_TransmitMsg(m_CAN_Packet);
}

void BoRuiController::VehicleContorlPri()
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

void BoRuiController::SteeringAngleControl(float dt)
{
//    float da = SteeringAngleRate * dt;
//    float left_target_angle = SteeringAngle - da;
//    float right_target_angle = SteeringAngle + da;
//
//    if(SteeringAngleSet < left_target_angle)
//    {
//    	SteeringAngleSet = SteeringAngleSet + da;
//    }
//    else if(SteeringAngleSet > right_target_angle)
//    {
//    	SteeringAngleSet = SteeringAngleSet - da;
//    }
//    else
//    {
//    	SteeringAngleSet = SteeringAngle;
//    }
}

void BoRuiController::SteeringAngleControl(float dt,float actual_steering)
{
	if(getSteeringAngleRate() > 400.0f)
	{
		setSteeringAngleRate(400.0f);
	}
//	float delta_theta = getSteeringAngleRate() * dt;
    float da = getSteeringAngleRate() * (dt + 0.1);
    float left_target_angle  = getSteeringAngle() - da;
    float right_target_angle = getSteeringAngle() + da;

    // 异常输入不响应
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

void BoRuiController::Push(float dt)
{
//	SteeringAngleControl(dt);
//	VehicleContorl();
}

void BoRuiController::Push(float dt,float actual_steering)
{
	SteeringAngleControl(dt,actual_steering);
	VehicleContorl();
}

void BoRuiController::WorkStateMachine(MessageManager& msg)
{

}

void BoRuiController::DataPush(void)
{
	VehicleContorl();
}
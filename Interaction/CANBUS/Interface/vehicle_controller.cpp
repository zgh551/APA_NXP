/*
 * VehicleControllercontroller.cpp
 *
 *  Created on: December 28 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: VehicleControllercontroller.cpp               COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: VehicleController control base class       					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 28 2018    Initial Version                  */
/*****************************************************************************/
#include "vehicle_controller.h"

VehicleController::VehicleController() {
	////// ACC //////
	TargetAcceleration.setContainer(this);
	TargetAcceleration.getter(&VehicleController::getTargetAcceleration);
	TargetAcceleration.setter(&VehicleController::setTargetAcceleration);

	Acceleration.setContainer(this);
	Acceleration.getter(&VehicleController::getAcceleration);
	Acceleration.setter(&VehicleController::setAcceleration);

	AccelerationEnable.setContainer(this);
	AccelerationEnable.getter(&VehicleController::getAccelerationEnable);
	AccelerationEnable.setter(&VehicleController::setAccelerationEnable);

	////// AEB //////
	Deceleration.setContainer(this);
	Deceleration.getter(&VehicleController::getDeceleration);
	Deceleration.setter(&VehicleController::setDeceleration);

	////// Torque //////
	Torque.setContainer(this);
	Torque.getter(&VehicleController::getTorque);
	Torque.setter(&VehicleController::setTorque);

	TorqueEnable.setContainer(this);
	TorqueEnable.getter(&VehicleController::getTorqueEnable);
	TorqueEnable.setter(&VehicleController::setTorqueEnable);
	////// Turnning Torque Control Single //////
	TurnTorqueVal.setContainer(this);
	TurnTorqueVal.getter(&VehicleController::getTurnTorqueVal);
	TurnTorqueVal.setter(&VehicleController::setTurnTorqueVal);

	TurnTorqueDir.setContainer(this);
	TurnTorqueDir.getter(&VehicleController::getTurnTorqueDir);
	TurnTorqueDir.setter(&VehicleController::setTurnTorqueDir);

	TurnTorqueAct.setContainer(this);
	TurnTorqueAct.getter(&VehicleController::getTurnTorqueAct);
	TurnTorqueAct.setter(&VehicleController::setTurnTorqueAct);
	////// Velocity //////
	Velocity.setContainer(this);
	Velocity.getter(&VehicleController::getVelocity);
	Velocity.setter(&VehicleController::setVelocity);

	Distance.setContainer(this);
	Distance.getter(&VehicleController::getDistance);
	Distance.setter(&VehicleController::setDistance);

	VelocityEnable.setContainer(this);
	VelocityEnable.getter(&VehicleController::getVelocityEnable);
	VelocityEnable.setter(&VehicleController::setVelocityEnable);

	////// Steering Angle //////
	SteeringAngle.setContainer(this);
	SteeringAngle.getter(&VehicleController::getSteeringAngle);
	SteeringAngle.setter(&VehicleController::setSteeringAngle);

	SteeringAngleRate.setContainer(this);
	SteeringAngleRate.getter(&VehicleController::getSteeringAngleRate);
	SteeringAngleRate.setter(&VehicleController::setSteeringAngleRate);

	SteeringEnable.setContainer(this);
	SteeringEnable.getter(&VehicleController::getSteeringEnable);
	SteeringEnable.setter(&VehicleController::setSteeringEnable);

	////// Gear //////
	Gear.setContainer(this);
	Gear.getter(&VehicleController::getGear);
	Gear.setter(&VehicleController::setGear);

	APAEnable.setContainer(this);
	APAEnable.getter(&VehicleController::getAPAEnable);
	APAEnable.setter(&VehicleController::setAPAEnable);

	EPBEnable.setContainer(this);
	EPBEnable.getter(&VehicleController::getEPBEnable);
	EPBEnable.setter(&VehicleController::setEPBEnable);
}

VehicleController::~VehicleController() {

}

void VehicleController::SteeringAngleControl(void)
{
	steering_angle_valid_request_  = this->getSteeringAngle();
}

void VehicleController::SteeringAngleControl(float dt)
{
	// limit the steering angle rate
	if (fabs(this->getSteeringAngleRate()) > MAX_STEERING_ANGLE_RATE)
	{
		this->setSteeringAngleRate(MAX_STEERING_ANGLE_RATE);
	}
	else
	{
		// do nothing
	}

	// limit the steering angle
	if (this->getSteeringAngle() > MAX_STEERING_ANGLE)
	{
		this->setSteeringAngle(MAX_STEERING_ANGLE);
	}
	else if (this->getSteeringAngle() < -MAX_STEERING_ANGLE)
	{
		this->setSteeringAngle(-MAX_STEERING_ANGLE);
	}
	else
	{
		// Do Nothing
	}

	float delta_angle      = fabs(this->getSteeringAngleRate()) * dt;
	float left_band_angle  = this->getSteeringAngle() - delta_angle;
	float right_band_angle = this->getSteeringAngle() + delta_angle;

	if (steering_angle_valid_request_ < left_band_angle)
	{
		steering_angle_valid_request_ = steering_angle_valid_request_ + delta_angle;
	}
	else if (steering_angle_valid_request_ > right_band_angle)
	{
		steering_angle_valid_request_ = steering_angle_valid_request_ - delta_angle;
	}
	else
	{
		steering_angle_valid_request_  = this->getSteeringAngle();
	}
}

//void VehicleController::SteeringAngleControl(float dt,float actual_steering)
//{
//	// limit the steering angle rate
//	if (fabs(this->getSteeringAngleRate()) > MAX_STEERING_ANGLE_RATE)
//	{
//		this->setSteeringAngleRate(MAX_STEERING_ANGLE_RATE);
//	}
//	else
//	{
//		// do nothing
//	}
//
//	// limit the steering angle
//	if (this->getSteeringAngle() > MAX_STEERING_ANGLE)
//	{
//		this->setSteeringAngle(MAX_STEERING_ANGLE);
//	}
//	else if (this->getSteeringAngle() < -MAX_STEERING_ANGLE)
//	{
//		this->setSteeringAngle(-MAX_STEERING_ANGLE);
//	}
//	else
//	{
//		// Do Nothing
//	}
//
//	float delta_angle      = fabs(this->getSteeringAngleRate()) * (0.1 + dt);
//	float left_band_angle  = this->getSteeringAngle() - delta_angle;
//	float right_band_angle = this->getSteeringAngle() + delta_angle;
//
//	if (actual_steering < left_band_angle)
//	{
//		steering_angle_valid_request_ = actual_steering + delta_angle;
//	}
//	else if (actual_steering > right_band_angle)
//	{
//		steering_angle_valid_request_ = actual_steering - delta_angle;
//	}
//	else
//	{
//		steering_angle_valid_request_  = this->getSteeringAngle();
//	}
//
//	if (steering_angle_valid_request_ > MAX_STEERING_ANGLE)
//	{
//		steering_angle_valid_request_ = MAX_STEERING_ANGLE;
//	}
//	else if (steering_angle_valid_request_ < -MAX_STEERING_ANGLE)
//	{
//		steering_angle_valid_request_ = -MAX_STEERING_ANGLE;
//	}
//	else
//	{
//		// Do Nothing
//	}
//}


void VehicleController::SteeringAngleControl(float dt,float actual_steering)
{
	// limit the steering angle rate
	if (fabs(this->getSteeringAngleRate()) > MAX_STEERING_ANGLE_RATE)
	{
		this->setSteeringAngleRate(MAX_STEERING_ANGLE_RATE);
	}
	else if (this->getSteeringAngleRate() < 0.0f)
	{
		this->setSteeringAngleRate(-this->getSteeringAngleRate());
	}
	else
	{
		// do nothing
	}

	// limit the steering angle
	if (this->getSteeringAngle() > MAX_STEERING_ANGLE)
	{
		this->setSteeringAngle(MAX_STEERING_ANGLE);
	}
	else if (this->getSteeringAngle() < -MAX_STEERING_ANGLE)
	{
		this->setSteeringAngle(-MAX_STEERING_ANGLE);
	}
	else
	{
		// Do Nothing
	}

	float delta_angle      = this->getSteeringAngleRate() * dt;
	float left_band_angle  = this->getSteeringAngle() - delta_angle;
	float right_band_angle = this->getSteeringAngle() + delta_angle;

	if (steering_angle_valid_request_ < left_band_angle)
	{
		steering_angle_valid_request_ = steering_angle_valid_request_ + delta_angle;
	}
	else if (steering_angle_valid_request_ > right_band_angle)
	{
		steering_angle_valid_request_ = steering_angle_valid_request_ - delta_angle;
	}
	else
	{
		steering_angle_valid_request_  = this->getSteeringAngle();
	}

	// actual steering angle restriction
	if ((steering_angle_valid_request_ - actual_steering) > 100)
	{
		steering_angle_valid_request_ = actual_steering + 100;
	}
	else if ((steering_angle_valid_request_ - actual_steering) < -100)
	{
		steering_angle_valid_request_ = actual_steering - 100;
	}
	else
	{
		// do nothing;
	}

	if (steering_angle_valid_request_ > MAX_STEERING_ANGLE)
	{
		steering_angle_valid_request_ = MAX_STEERING_ANGLE;
	}
	else if (steering_angle_valid_request_ < -MAX_STEERING_ANGLE)
	{
		steering_angle_valid_request_ = -MAX_STEERING_ANGLE;
	}
	else
	{
		// Do Nothing
	}
}


/// ACC
float VehicleController::getTargetAcceleration()           { return _target_acceleration;}
void  VehicleController::setTargetAcceleration(float value){_target_acceleration = value;}

float VehicleController::getAcceleration()           { return _acceleration;}
void  VehicleController::setAcceleration(float value){_acceleration = value;}

uint8_t VehicleController::getAccelerationEnable()             { return _acceleration_enable;}
void    VehicleController::setAccelerationEnable(uint8_t value){_acceleration_enable = value;}

/// AEB
float VehicleController::getDeceleration()           { return _deceleration;}
void  VehicleController::setDeceleration(float value){_deceleration = value;}

/// Torque
float VehicleController::getTorque()           { return _torque;}
void  VehicleController::setTorque(float value){_torque = value;}

uint8_t VehicleController::getTorqueEnable()             { return _torque_enable;}
void    VehicleController::setTorqueEnable(uint8_t value){_torque_enable = value;}

/// Turnning Torque Control Single
float VehicleController::getTurnTorqueVal()           { return _turn_torque_val;}
void  VehicleController::setTurnTorqueVal(float value){_turn_torque_val = value;}

uint8_t VehicleController::getTurnTorqueDir()             { return _turn_torque_dir;}
void    VehicleController::setTurnTorqueDir(uint8_t value){_turn_torque_dir = value;}

uint8_t VehicleController::getTurnTorqueAct()             { return _turn_torque_act;}
void    VehicleController::setTurnTorqueAct(uint8_t value){_turn_torque_act = value;}
/// Velocity
float VehicleController::getVelocity()           { return _velocity;}
void  VehicleController::setVelocity(float value){_velocity = value;}
/// Distance
float VehicleController::getDistance()           { return _distance;}
void  VehicleController::setDistance(float value){_distance = value;}

uint8_t VehicleController::getVelocityEnable()             { return _velocity_enable;}
void    VehicleController::setVelocityEnable(uint8_t value){_velocity_enable = value;}

/// Steering Angle
float VehicleController::getSteeringAngle()           { return _steering_angle;}
void  VehicleController::setSteeringAngle(float value){_steering_angle = value;}

float VehicleController::getSteeringAngleRate()           { return _steering_angle_rate;}
void  VehicleController::setSteeringAngleRate(float value){_steering_angle_rate = value;}

uint8_t VehicleController::getSteeringEnable()             { return _steering_enable;}
void    VehicleController::setSteeringEnable(uint8_t value){_steering_enable = value;}
/// Gear
GearStatus VehicleController::getGear()                { return _gear;}
void       VehicleController::setGear(GearStatus value){_gear = value;}

/// APA Enable
uint8_t VehicleController::getAPAEnable()             { return _apa_enable;}
void    VehicleController::setAPAEnable(uint8_t value){_apa_enable = value;}

/// EPB Enable
uint8_t VehicleController::getEPBEnable()             { return _epb_enable;}
void    VehicleController::setEPBEnable(uint8_t value){_epb_enable = value;}

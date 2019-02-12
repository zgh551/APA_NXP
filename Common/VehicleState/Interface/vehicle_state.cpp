/*
 * vehicle_state.c
 *
 *  Created on: December 29 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vehicle_state.cp                    COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: return the vehicle position ans attitude state		         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 29 2018    Initial Version                  */
/*****************************************************************************/

#include "../Interface/vehicle_state.h"

VehicleState::VehicleState() {
	// TODO Auto-generated constructor stub
	Init();
}

VehicleState::~VehicleState() {
	// TODO Auto-generated destructor stub
}

void VehicleState::Init()
{
	Position.setContainer(this);
	Position.getter(&VehicleState::getPosition);
	Position.setter(&VehicleState::setPosition);

	Yaw.setContainer(this);
	Yaw.getter(&VehicleState::getYaw);
	Yaw.setter(&VehicleState::setYaw);

	LinearVelocity.setContainer(this);
	LinearVelocity.getter(&VehicleState::getLinearVelocity);
	LinearVelocity.setter(&VehicleState::setLinearVelocity);

	LinearRate.setContainer(this);
	LinearRate.getter(&VehicleState::getLinearRate);
	LinearRate.setter(&VehicleState::setLinearRate);
}

float VehicleState::SteeringAngle2TurnningRadius(float steer,float a,float b)
{
	return WHEEL_BASE / tanf((a * steer + b) * PI / 180);
}

float VehicleState::SteeringAngle2TurnningRadiusExp(float steer,float a,float b)
{
	return steer < 0 ? -a * expf(b * -steer) : a * expf(b * steer);
}

float VehicleState::TurnningRadius2SteeringAngleExp(float radius,float a,float b)
{
	return radius < 0 ? -logf(-radius/a)/b : logf(radius/a)/b;
}

float VehicleState::TurnningRadius2SteeringAngle(float radius,float a,float b)
{
	return (atanf(WHEEL_BASE/radius) * 180 / PI - b)/a;
}

float VehicleState::TurnRadiusCalculate(float steering_angle)
{
	return 	steering_angle >  400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A1,FIT_RADIUS_B1) :
			steering_angle >  300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A2,FIT_RADIUS_B2) :
			steering_angle >  200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A3,FIT_RADIUS_B3) :
			steering_angle >  100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A4,FIT_RADIUS_B4) :
			steering_angle >   50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A5,FIT_RADIUS_B5) :
			steering_angle >    0 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A6,FIT_RADIUS_B6) :
			steering_angle >  -50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A7,FIT_RADIUS_B7) :
			steering_angle > -100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A8,FIT_RADIUS_B8) :
			steering_angle > -200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A9,FIT_RADIUS_B9) :
			steering_angle > -300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A10,FIT_RADIUS_B10) :
			steering_angle > -400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A11,FIT_RADIUS_B11) :
			                        SteeringAngle2TurnningRadiusExp(steering_angle,FIT_RADIUS_A12,FIT_RADIUS_B12);
}

float VehicleState::SteeringAngleCalculate(float radius)
{
	return
		   radius >   48.308 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A6,FIT_RADIUS_B6) :
		   radius >   24.018 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A5,FIT_RADIUS_B5) :
		   radius >   11.910 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A4,FIT_RADIUS_B4) :
		   radius >   7.6736 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A3,FIT_RADIUS_B3) :
		   radius >    5.463 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A2,FIT_RADIUS_B2) :
		   radius >        0 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A1,FIT_RADIUS_B1) :
		   radius > - 5.4745 ? TurnningRadius2SteeringAngleExp(radius,FIT_RADIUS_A12,FIT_RADIUS_B12) :
		   radius > - 7.7214 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A11,FIT_RADIUS_B11):
		   radius > - 11.975 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A10,FIT_RADIUS_B10):
		   radius > - 24.975 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A9,FIT_RADIUS_B9):
		   radius > - 49.975 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A8,FIT_RADIUS_B8):
		                       TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A7,FIT_RADIUS_B7);
}

Vector2d VehicleState::getPosition()              { return  _position;}
void     VehicleState::setPosition(Vector2d value){ _position = value;}

float VehicleState::getYaw()           { return _yaw;}
void  VehicleState::setYaw(float value){_yaw = value;}

float VehicleState::getLinearVelocity()           { return _linear_velocity;}
void  VehicleState::setLinearVelocity(float value){_linear_velocity = value;}

float VehicleState::getLinearRate()           { return _linear_rate;}
void  VehicleState::setLinearRate(float value){_linear_rate = value;}

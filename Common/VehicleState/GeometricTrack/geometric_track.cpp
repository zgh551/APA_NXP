/*
 * geometric_track.cpp
 *
 *  Created on: January 9 2019
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: geometric_track.cpp                 COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: track the vehilce position        					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 9 2019      Initial Version                  */
/*****************************************************************************/
#include "../GeometricTrack/geometric_track.h"

GeometricTrack::GeometricTrack() {
	// TODO Auto-generated constructor stub
	Init();
}

GeometricTrack::~GeometricTrack() {
	// TODO Auto-generated destructor stub
}

void GeometricTrack::Init(void)
{
	X = 0.0f;
	Y = 0.0f;
	Z = 0.0f;
	Yaw = 0.0f;
	LinearVelocity = 0.0f;
}

void GeometricTrack::Init(float x,float y,float yaw)
{
	X = x;
	Y = y;
	Z = 0.0f;
	Yaw = yaw;
	LinearVelocity = 0.0f;
}
void GeometricTrack::Init(VehicleBody v)
{
	Vector2d v_temp;
	v_temp = v.Center;
	X = v_temp.getX();
	Y = v_temp.getY();
	Z = 0.0f;
	Yaw = v.AttitudeYaw;
	LinearVelocity = 0.0f;
}

void GeometricTrack::VelocityUpdate(MessageManager *msg,float dt)
{
	float v;
	v = (msg->WheelSpeedRearRight + msg->WheelSpeedRearLeft)*0.5;
	LinearVelocity = msg->WheelSpeedDirection == 0 ?  v :
					 msg->WheelSpeedDirection == 1 ? -v : 0;

	if(msg->SteeringAngle != 0)
	{
		Yaw  = _last_yaw + LinearVelocity * dt / TurnRadiusCalculate(msg->SteeringAngle);
		X = X - TurnRadiusCalculate(msg->SteeringAngle) * (sinf(_last_yaw) - sinf(Yaw));
		Y = Y - TurnRadiusCalculate(msg->SteeringAngle) * (cosf(Yaw) - cosf(_last_yaw));
	}
	else
	{
		X = X + LinearVelocity * cosf(Yaw) * dt;
		Y = Y + LinearVelocity * sinf(Yaw) * dt;
	}
	_last_yaw = Yaw;
}

void GeometricTrack::PulseUpdate(MessageManager *msg,float dt)
{
	float rear_left_displacement,rear_right_displacement;
	float displacement;
	float radius;

	rear_left_displacement = msg->WheelPulseDirection == 0 ?
							(msg->WheelPulseRearLeft - _last_rear_left_pulse) * WHEEL_PUSLE_RATIO :
							 msg->WheelPulseDirection == 1 ?
						   -(msg->WheelPulseRearLeft - _last_rear_left_pulse) * WHEEL_PUSLE_RATIO : 0;


	rear_right_displacement = msg->WheelPulseDirection == 0 ?
							 (msg->WheelPulseRearRight - _last_rear_right_pulse) * WHEEL_PUSLE_RATIO :
							  msg->WheelPulseDirection == 1 ?
							-(msg->WheelPulseRearRight - _last_rear_right_pulse) * WHEEL_PUSLE_RATIO : 0;

	displacement = (rear_left_displacement + rear_right_displacement) * 0.5;

	if(msg->SteeringAngle != 0)
	{
		radius = TurnRadiusCalculate(msg->SteeringAngle);
		Yaw  = _last_yaw + displacement / radius;
		X = X - radius * (sinf(_last_yaw) - sinf(Yaw));
		Y = Y - radius * (cosf(Yaw) - cosf(_last_yaw));
	}
	else
	{
		X = X + displacement * cosf(Yaw);
		Y = Y + displacement * sinf(Yaw);
	}
	_last_rear_left_pulse  = msg->WheelPulseRearLeft;
	_last_rear_right_pulse = msg->WheelPulseRearRight;
	_last_yaw = Yaw;
}

float GeometricTrack::SteeringAngle2TurnningRadius(float steer,float a,float b)
{
	return WHEEL_BASE / tanf((a * steer + b) * PI / 180);
}

float GeometricTrack::SteeringAngle2TurnningRadiusExp(float steer,float a,float b)
{
	return a * expf(b * steer);
}

float GeometricTrack::TurnRadiusCalculate(float steering_angle)
{
	return 	steering_angle >  400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A1,FIT_RADIUS_B1) :
			steering_angle >  300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A2,FIT_RADIUS_B2) :
			steering_angle >  200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A3,FIT_RADIUS_B3) :
			steering_angle >  100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A4,FIT_RADIUS_B4) :
			steering_angle >   50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A5,FIT_RADIUS_B5) :
			steering_angle >    0 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A6,FIT_RADIUS_B6) :
			steering_angle <    0 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A7,FIT_RADIUS_B7) :
			steering_angle < - 50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A8,FIT_RADIUS_B8) :
			steering_angle < -100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A9,FIT_RADIUS_B9) :
			steering_angle < -200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A10,FIT_RADIUS_B10) :
			steering_angle < -300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A11,FIT_RADIUS_B11) :
			steering_angle < -400 ? SteeringAngle2TurnningRadiusExp(steering_angle,FIT_RADIUS_A12,FIT_RADIUS_B12) : 0;
}

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

VehilceConfig m_GeometricVehicleConfig;

GeometricTrack::GeometricTrack() {
	Init();
}

GeometricTrack::~GeometricTrack() {

}

void GeometricTrack::Init(void)
{
	_position.X = 0;
	_position.Y = 0;
	Yaw = 0.0f;
	LinearVelocity = 0.0f;
}

void GeometricTrack::Init(float x,float y,float yaw)
{
	_position.setX(x);
	_position.Y = y;
	Yaw = yaw;
	_last_yaw = Yaw;
	LinearVelocity = 0.0f;
}
void GeometricTrack::Init(VehicleBody v)
{
	_position.X = v.getCenter().getX();
	_position.Y = v.getCenter().getY();
	Yaw = v.AttitudeYaw;
	_last_yaw = Yaw;
	LinearVelocity = 0.0f;
}
void GeometricTrack::Init(Line l)
{
	_position.X = l.Point.getX();
	_position.Y = l.Point.getY();
	Yaw = l.Angle;
	_last_yaw = Yaw;
	LinearVelocity = 0.0f;
}

void GeometricTrack::Init(Percaption *p)
{
	_position.X = p->PositionX;
	_position.Y = p->PositionY;
	Yaw = p->AttitudeYaw;
	_last_yaw = Yaw;
	LinearVelocity = 0.0f;
}

void GeometricTrack::VelocityUpdate(MessageManager *msg,float dt)
{
	float radius;
	LinearRate = (msg->WheelSpeedRearRight + msg->WheelSpeedRearLeft)*0.5;
	LinearVelocity = msg->WheelSpeedDirection == 0 ?  LinearRate :
					 msg->WheelSpeedDirection == 1 ? -LinearRate : 0;

	if(fabs(msg->SteeringAngle) > 1)
	{
		radius = m_GeometricVehicleConfig.TurnRadiusCalculate(msg->SteeringAngle);
		Yaw  = _last_yaw + LinearVelocity * dt / radius;
		_position.X = _position.X + radius * (sinf(Yaw) - sinf(_last_yaw));
		_position.Y = _position.Y + radius * (cosf(_last_yaw) - cosf(Yaw));
	}
	else
	{
		_position.X = _position.X + LinearVelocity * cosf(Yaw) * dt;
		_position.Y = _position.Y + LinearVelocity * sinf(Yaw) * dt;
	}
	_last_yaw = Yaw;
}

void GeometricTrack::PulseUpdate(MessageManager *msg,float dt)
{
	float rear_left_displacement,rear_right_displacement;
	float displacement;
	float radius;

	LinearRate = (msg->WheelSpeedRearRight + msg->WheelSpeedRearLeft)*0.5;
	LinearVelocity = msg->WheelSpeedDirection == 0 ?  LinearRate :
					 msg->WheelSpeedDirection == 1 ? -LinearRate : 0;

	if(msg->WheelPulseRearLeft >= _last_rear_left_pulse)
	{
		rear_left_displacement = msg->WheelPulseDirection == 0 ?
								(msg->WheelPulseRearLeft - _last_rear_left_pulse) * WHEEL_PUSLE_RATIO :
								 msg->WheelPulseDirection == 1 ?
							   -(msg->WheelPulseRearLeft - _last_rear_left_pulse) * WHEEL_PUSLE_RATIO : 0;
	}
	else
	{
		rear_left_displacement = msg->WheelPulseDirection == 0 ?
								(254 + msg->WheelPulseRearLeft - _last_rear_left_pulse) * WHEEL_PUSLE_RATIO :
								 msg->WheelPulseDirection == 1 ?
							   -(254 + msg->WheelPulseRearLeft - _last_rear_left_pulse) * WHEEL_PUSLE_RATIO : 0;
	}

	if(msg->WheelPulseRearRight >= _last_rear_right_pulse)
	{
		rear_right_displacement = msg->WheelPulseDirection == 0 ?
								 (msg->WheelPulseRearRight - _last_rear_right_pulse) * WHEEL_PUSLE_RATIO :
								  msg->WheelPulseDirection == 1 ?
								-(msg->WheelPulseRearRight - _last_rear_right_pulse) * WHEEL_PUSLE_RATIO : 0;
	}
	else
	{
		rear_right_displacement = msg->WheelPulseDirection == 0 ?
								 (254 + msg->WheelPulseRearRight - _last_rear_right_pulse) * WHEEL_PUSLE_RATIO :
								  msg->WheelPulseDirection == 1 ?
								-(254 + msg->WheelPulseRearRight - _last_rear_right_pulse) * WHEEL_PUSLE_RATIO : 0;
	}
	displacement = (rear_left_displacement + rear_right_displacement) * 0.5;

	if(msg->SteeringAngle != 0)
	{
		radius = m_GeometricVehicleConfig.TurnRadiusCalculate(msg->SteeringAngle);
		Yaw  = _last_yaw + displacement / radius;
		_position.X = _position.X + radius * (sinf(Yaw) - sinf(_last_yaw));
		_position.Y = _position.Y + radius * (cosf(_last_yaw) - cosf(Yaw));
	}
	else
	{
		_position.X = _position.X + displacement * cosf(Yaw);
		_position.Y = _position.Y + displacement * sinf(Yaw);
	}
	_last_rear_left_pulse  = msg->WheelPulseRearLeft;
	_last_rear_right_pulse = msg->WheelPulseRearRight;
	_last_yaw = Yaw;
}

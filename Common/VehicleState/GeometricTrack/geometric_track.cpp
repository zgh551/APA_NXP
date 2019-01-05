/*
 * geometric_track.cpp
 *
 *  Created on: 2019Äê1ÔÂ2ÈÕ
 *      Author: zhuguohua
 */

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

void GeometricTrack::Update(MessageManager *msg,float dt)
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

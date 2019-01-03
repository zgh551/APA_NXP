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
	return 	steering_angle >  400 ? SteeringAngle2TurnningRadius(steering_angle,a1,b1) :
			steering_angle >  300 ? SteeringAngle2TurnningRadius(steering_angle,a2,b2) :
			steering_angle >  200 ? SteeringAngle2TurnningRadius(steering_angle,a3,b3) :
			steering_angle >  100 ? SteeringAngle2TurnningRadius(steering_angle,a4,b4) :
			steering_angle >   50 ? SteeringAngle2TurnningRadius(steering_angle,a5,b5) :
			steering_angle >    0 ? SteeringAngle2TurnningRadius(steering_angle,a6,b6) :
			steering_angle <    0 ? SteeringAngle2TurnningRadius(steering_angle,a7,b7) :
			steering_angle < - 50 ? SteeringAngle2TurnningRadius(steering_angle,a8,b8) :
			steering_angle < -100 ? SteeringAngle2TurnningRadius(steering_angle,a9,b9) :
			steering_angle < -200 ? SteeringAngle2TurnningRadius(steering_angle,a10,b10) :
			steering_angle < -300 ? SteeringAngle2TurnningRadius(steering_angle,a11,b11) :
			steering_angle < -400 ? SteeringAngle2TurnningRadiusExp(steering_angle,a12,b12) : 0;
}

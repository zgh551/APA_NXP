/*
 * lat_control.cpp
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */

#include "lat_control.h"

LatControl::LatControl() {
	// TODO Auto-generated constructor stub

}

LatControl::~LatControl() {
	// TODO Auto-generated destructor stub
}

void LatControl::Init()
{

}

void LatControl::Proc(MessageManager *msg,VehicleController *ctl,PID *pid)
{
	if(ctl->SteeringEnable)
	{
		ctl->SteeringAngle = 0;
	}
}

float LatControl::TargetLine(float x)
{
	return cosf(COEFFICIENT_TLS*x)-1;
}

float LatControl::TargetLineFirstDerivative(float x)
{
	return -COEFFICIENT_TLS*sinf(COEFFICIENT_TLS*x);
}

float LatControl::TargetLineSecondDerivative(float x)
{
	return -COEFFICIENT_TLS*COEFFICIENT_TLS*cosf(COEFFICIENT_TLS*x);
}

float LatControl::SatFunction(float x)
{
	if(x > COEFFICIENT_DELTA)
	{
		return  1.0f;
	}
	else if(x < -COEFFICIENT_DELTA)
	{
		return -1.0f;
	}
	else
	{
		return x/COEFFICIENT_DELTA;
	}
}

void LatControl::Proc(MessageManager *msg,VehicleController *ctl,GeometricTrack *track)
{
	float psi_r,delta_r;
	float cos_psi,cos_psi_r;
	float tan_psi,tan_psi_r;

	float delta_ctl;
	float temp_a,temp_b;

	if(ctl->SteeringEnable)
	{
		_target_track.setX(track->getPosition().getX());
		_target_track.setY(TargetLine(_target_track.getX()));

		_x1 = _target_track.getY() - track->getPosition().getY();
		psi_r = atanf(TargetLineFirstDerivative(track->getPosition().getX()));

		cos_psi_r = cosf(psi_r);
		tan_psi_r = tanf(psi_r);

		cos_psi   = cosf(track->getYaw());
		tan_psi   = tanf(track->getYaw());

		delta_r = atanf(WHEEL_BASE*cos_psi_r*cos_psi_r*cos_psi_r*TargetLineSecondDerivative(track->getPosition().getX()));

		if(Drive == msg->getGear())
		{
			_x2 = tan_psi_r - tan_psi;
		}
		else if(Reverse == msg->getGear())
		{
			_x2 = tan_psi - tan_psi_r;
		}
		else
		{

		}
		_sliding_variable = COEFFICIENT_SMV * _x1 + _x2;

		temp_a = WHEEL_BASE * cos_psi * cos_psi * cos_psi;
		temp_b = tanf(delta_r)/(WHEEL_BASE * cos_psi_r * cos_psi_r * cos_psi_r) +
				COEFFICIENT_SMV * _x2 +
				COEFFICIENT_RHO * SatFunction(_sliding_variable) +
				COEFFICIENT_K * _sliding_variable;

		delta_ctl = atanf(temp_a * temp_b);

		ctl->SteeringAngle 		= delta_ctl*16*57.3f;
		ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
	}
}

Vector2d LatControl::getTargetTrack()               { return  _target_track;}
void     LatControl::setTargetTrack(Vector2d value) { _target_track = value;}

float LatControl::getX1()            { return  _x1;}
void  LatControl::setX1(float value) { _x1 = value;}

float LatControl::getX2()            { return  _x2;}
void  LatControl::setX2(float value) { _x2 = value;}

float LatControl::getSlidingVariable()            { return  _sliding_variable;}
void  LatControl::setSlidingVariable(float value) { _sliding_variable = value;}

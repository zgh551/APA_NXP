/*
 * lon_control.cpp
 *
 *  Created on: 2019年1月3日
 *      Author: zhuguohua
 */

#include <lon_control.h>

LonControl::LonControl() {
	Init();
}

LonControl::~LonControl() {

}

void LonControl::Init()
{
	_max_position = MAX_POSITION;// 速度控制上限点
	_min_position = MIN_POSITION;// 速度控制下限点
	_max_velocity = MAX_VELOCITY;// 直线段的速度
	_min_velocity = MIN_VELOCITY;// 曲线段的速度
}

//根据距离规划速度
float LonControl::VelocityPlanningControl(float distance)
{
	if(distance > _max_position)
	{
		return _max_velocity;
	}
	else if(distance > _min_position)
	{
		return _min_velocity + (_max_velocity - _min_velocity)*(distance - _min_position)/(_max_position - _min_position);
	}
	else if(distance > 0.05)
	{
		return distance * _min_velocity / _min_position ;
	}
	else
	{
		return 0;
	}
}


float LonControl::VelocityControl(float distance,float velocity)
{
	float temp_v;
	temp_v = VelocityPlanningControl(distance);
	return velocity < temp_v ? velocity :temp_v;
}

void LonControl::Proc(MessageManager *msg,VehicleController *ctl,PID *pid)
{
	if(ctl->VelocityEnable)
	{
		float v = (msg->WheelSpeedRearLeft + msg->WheelSpeedRearRight) * 0.5;
		pid->Desired = ctl->Velocity;
//		pid->Desired = VelocityControl(ctl->Velocity,ctl->Distance);
		ctl->Acceleration = pid->pidUpdateIntegralSeparation(v);
	}
}

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

void LonControl::Proc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid)
{
	if(ctl->VelocityEnable)
	{
		float v = (msg->WheelSpeedRearLeft + msg->WheelSpeedRearRight) * 0.5;
		velocity_pid->Desired = ctl->Velocity;
//		pid->Desired = VelocityControl(ctl->Velocity,ctl->Distance);
		ctl->Acceleration = velocity_pid->pidUpdateIntegralSeparation(v);
//		ctl->Torque = velocity_pid->pidUpdate(v);
	}
}

void LonControl::Proc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid,PID *acc_pid)
{
	if(ctl->VelocityEnable)
	{
		float v = (msg->WheelSpeedRearLeft + msg->WheelSpeedRearRight) * 0.5;
		velocity_pid->Desired = ctl->Velocity;
		acc_pid->Desired = velocity_pid->pidUpdateIntegralSeparation(v);
		if(acc_pid->Desired > 0)
		{
			if(msg->Gear == Drive)
			{
				ctl->Acceleration = acc_pid->pidUpdateIntegralSeparation(msg->LonAcc);
			}
			else if(msg->Gear == Reverse)
			{
				ctl->Acceleration = acc_pid->pidUpdateIntegralSeparation(-msg->LonAcc);
			}
		}
		else
		{
			ctl->Acceleration = acc_pid->Desired;
		}

	}
}

void LonControl::AccProc(MessageManager *msg,VehicleController *ctl,PID *acc_pid)
{
	if(ctl->AccelerationEnable)
	{
		acc_pid->Desired = ctl->TargetAcceleration;
		ctl->Acceleration = acc_pid->pidUpdateIntegralSeparation(msg->LonAcc);
	}
}

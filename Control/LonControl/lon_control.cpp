/*
 * lon_control.cpp
 *
 *  Created on: 2019年1月3日
 *      Author: zhuguohua
 */

#include <lon_control.h>

LonControl::LonControl() {

}

LonControl::~LonControl() {

}

void LonControl::Init()
{

}

void LonControl::Proc(MessageManager *msg,VehicleController *ctl,PID *pid)
{
	if(ctl->VelocityEnable)
	{
		float v = (msg->WheelSpeedRearLeft + msg->WheelSpeedRearRight) * 0.5;
		pid->Desired = ctl->Velocity;
		ctl->Acceleration = pid->pidUpdateIntegralSeparation(v);
	}
}

/*
 * lon_control.cpp
 *
 *  Created on: 2019Äê1ÔÂ3ÈÕ
 *      Author: zhuguohua
 */

#include <lon_control.h>

LonControl::LonControl() {
	// TODO Auto-generated constructor stub

}

LonControl::~LonControl() {
	// TODO Auto-generated destructor stub
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

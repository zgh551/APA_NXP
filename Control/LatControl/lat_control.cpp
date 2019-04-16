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
	float pid_val;
	if(ctl->SteeringEnable)
	{
		pid->Desired = ctl->SteeringAngle;
		pid_val = pid->pidUpdateIntegralSeparation(msg->SteeringAngle);
		ctl->TurnTorqueVal = fabs(pid_val);
		ctl->TurnTorqueDir = pid_val > 0 ? 0:1;
	}
}

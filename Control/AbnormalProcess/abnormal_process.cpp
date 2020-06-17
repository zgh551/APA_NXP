/*
 * abnormal_process.cpp
 *
 *  Created on: 2020年6月2日
 *      Author: zhuguohua
 */

#include "abnormal_process.h"

AbnormalProcess::AbnormalProcess() {
	// TODO Auto-generated constructor stub
	_err_velocity_cnt = 0;
}

AbnormalProcess::~AbnormalProcess() {
	// TODO Auto-generated destructor stub
}

void AbnormalProcess::work(MessageManager *msg,VehicleController *ctl)
{
	// 异常处理
	// (1)速度反馈异常
	if(0 == msg->getBrakePressure())
	{
		if(SpeedAbnormal == msg->getVehicleMiddleSpeedAbnormal())/*速度值异常*/
		{
			_err_velocity_cnt++;
		}
		else
		{
			_err_velocity_cnt = 0;
		}
	}
	if(_err_velocity_cnt > 100)
	{
		// 策略：制动减速度设置为最大减速度，强制车辆停止
		ctl->setAccelerationEnable(1);
		ctl->setTorqueEnable(1);
		ctl->setAcceleration(MAX_DECELERATION);
		ctl->setTorque(0);
	}

	// (2)ESC异常
	if(ActuatorErr == msg->getESC_Status())
	{
		// 策略：失效扭矩控制
		ctl->setAccelerationEnable(0);
		ctl->setTorqueEnable(0);
		ctl->setAcceleration(0);
		ctl->setTorque(0);
	}
}

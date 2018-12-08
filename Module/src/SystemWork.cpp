/*
 * SystemWork.cpp
 *
 *  Created on: 2018Äê12ÔÂ5ÈÕ
 *      Author: zhuguohua
 */

#include "SystemWork.h"


SystemWork::SystemWork() {
	// TODO Auto-generated constructor stub
	_system_working_state = Debug;

	_working_module = 0;
	_function_status = 2;
	//// System Status //////
	WorkingModule.setContainer(this);
	WorkingModule.getter(&SystemWork::getWorkingModule);
	WorkingModule.setter(&SystemWork::setWorkingModule);

	FunctionStatus.setContainer(this);
	FunctionStatus.getter(&SystemWork::getFunctionStatus);
	FunctionStatus.setter(&SystemWork::setFunctionStatus);
}

SystemWork::SystemWork(float dt,float kp,float ki,float kd,float i_lim,float out_lim):Vehicle(dt,kp,ki,kd,i_lim,out_lim)
{
	// TODO Auto-generated constructor stub
	_system_working_state = Debug;

	_working_module = 0;
	_function_status = 2;

	////// System Status //////
	WorkingModule.setContainer(this);
	WorkingModule.getter(&SystemWork::getWorkingModule);
	WorkingModule.setter(&SystemWork::setWorkingModule);

	FunctionStatus.setContainer(this);
	FunctionStatus.getter(&SystemWork::getFunctionStatus);
	FunctionStatus.setter(&SystemWork::setFunctionStatus);
}

SystemWork::SystemWork(float dt,float kp,float ki,float kd,float i_lim,float out_lim,float threshold):Vehicle(dt,kp,ki,kd,i_lim,out_lim,threshold)
{
	// TODO Auto-generated constructor stub
	_system_working_state = Debug;

	_working_module = 0;
	_function_status = 2;
	////// System Status //////
	WorkingModule.setContainer(this);
	WorkingModule.getter(&SystemWork::getWorkingModule);
	WorkingModule.setter(&SystemWork::setWorkingModule);

	FunctionStatus.setContainer(this);
	FunctionStatus.getter(&SystemWork::getFunctionStatus);
	FunctionStatus.setter(&SystemWork::setFunctionStatus);
}

SystemWork::~SystemWork() {
	// TODO Auto-generated destructor stub
}

uint8_t SystemWork::getWorkingModule()
{
	return _working_module;
}
void SystemWork::setWorkingModule(uint8_t value)
{
	_working_module = value;
}

uint8_t SystemWork::getFunctionStatus()
{
	return _function_status;
}
void SystemWork::setFunctionStatus(uint8_t value)
{
	_function_status = value;
}


void SystemWork::SystemWorkState(void)
{
	switch((WorkingState)_working_module)
	{
	case Debug:
		DubugStataMahine();
		break;

	case Calibration:

		break;

	case Test:

		break;

	case Normal:

		break;

	default:

		break;
	}
}

void SystemWork::DubugStataMahine(void)
{
	VehicleContorl();//vehicle control commond
	SteeringAngleControl(0.02);// steering angle control
	switch((DubugState)_function_status)
	{
	case DirectControl:

		break;

	case SpeedControl:
		// VehicleSpeedControl(pidUpdate(VehicleSpeed));
		VehicleSpeedControl(pidUpdateIntegralSeparation(VehicleSpeed));
		break;

	case ChangAnControl:
		if(VehicleSpeedControlEnable)
		{
			// VehicleSpeedControl(pidUpdate(VehicleSpeed));
			VehicleSpeedControl(pidUpdateIntegralSeparation(VehicleSpeed));
		}
		break;

	case UltrasonicSR:

		break;

	default:

		break;
	}
}

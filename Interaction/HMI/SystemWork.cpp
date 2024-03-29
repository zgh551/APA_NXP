/*
 * SystemWork.cpp
 *
 *  Created on: 2018��12��5��
 *      Author: zhuguohua
 */

#include "SystemWork.h"


SystemWork::SystemWork() {
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



SystemWork::~SystemWork() {

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

	switch((DubugState)_function_status)
	{
	case DirectControl:

		break;

	case SpeedControl:

		break;

	case ChangAnControl:

		break;

	case UltrasonicSR:

		break;

	default:

		break;
	}
}

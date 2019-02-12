/*
 * SystemWork.h
 *
 *  Created on: 2018年12月5日
 *      Author: zhuguohua
 */

#ifndef SYSTEMWORK_H_
#define SYSTEMWORK_H_

#include "property.h"
#include "derivative.h"
#include "project.h"
#include "Vehicle.h"

typedef enum _WorkingState
{
	Debug = 0,
	Calibration,
	Test,
	Normal
}WorkingState;

typedef enum _DubugState
{
	DirectControl = 0,
	SpeedControl,
	ChangAnControl,
	UltrasonicSR
}DubugState;

class SystemWork : public Vehicle
{

public:
	SystemWork();
	SystemWork(float dt,float kp,float ki,float kd,float i_lim,float out_lim);
	SystemWork(float dt,float kp,float ki,float kd,float i_lim,float out_lim,float threshold);
	virtual ~SystemWork();

	void SystemWorkState(void);
	void DubugStataMahine(void);
//	/*** Variabel Property ***/
//	/* System State */
	uint8_t getWorkingModule();
	void setWorkingModule(uint8_t value);
	Property<SystemWork,uint8_t,READ_WRITE> WorkingModule;

	uint8_t getFunctionStatus();
	void setFunctionStatus(uint8_t value);
	Property<SystemWork,uint8_t,READ_WRITE> FunctionStatus;

private:
	/*** System state ***/
	uint8_t _working_module;
	uint8_t _function_status;

	/*** System Working State Machine ***/
	WorkingState _system_working_state;
};


#endif /* SYSTEMWORK_H_ */

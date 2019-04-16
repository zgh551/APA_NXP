/*
 * lat_control.h
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */

#ifndef LATCONTROL_LAT_CONTROL_H_
#define LATCONTROL_LAT_CONTROL_H_

#include "derivative.h"
#include "property.h"
#include "controller.h"
#include "pid.h"

class LatControl :public Controller
{
public:
	LatControl();
	virtual ~LatControl();

	void Init() override;

	void Proc(MessageManager *msg,VehicleController *ctl,PID *pid) override;
};

#endif /* LATCONTROL_LAT_CONTROL_H_ */

/*
 * lon_control.h
 *
 *  Created on: 2019Äê1ÔÂ3ÈÕ
 *      Author: zhuguohua
 */

#ifndef LONCONTROL_LON_CONTROL_H_
#define LONCONTROL_LON_CONTROL_H_

#include "derivative.h"
#include "property.h"
#include "controller.h"
#include "pid.h"

class LonControl:public Controller
{
public:
	LonControl();
	virtual ~LonControl();

	void Init() override;

	void Proc(MessageManager *msg,VehicleController *ctl,PID *pid) override;
};

#endif /* LONCONTROL_LON_CONTROL_H_ */

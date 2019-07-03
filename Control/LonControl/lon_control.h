/*
 * lon_control.h
 *
 *  Created on: 2019年1月3日
 *      Author: zhuguohua
 */

#ifndef LONCONTROL_LON_CONTROL_H_
#define LONCONTROL_LON_CONTROL_H_

#include "derivative.h"
#include "property.h"
#include "controller.h"
#include "pid.h"

/**************************速度控制******************************/
#define MAX_POSITION            ( 0.8 ) // 速度控制上限点
#define MIN_POSITION            ( 0.4 ) // 速度控制下限点
#define MAX_VELOCITY	  		( 0.5 ) // 直线段的速度
#define MIN_VELOCITY	      	( 0.3 ) // 曲线段的速度

class LonControl:public Controller
{
public:
	LonControl();
	virtual ~LonControl();

	void Init() override;

	void Proc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid) override;

	void Proc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid,PID *acc_pid);

	void AccProc(MessageManager *msg,VehicleController *ctl,PID *acc_pid);

	float VelocityPlanningControl(float distance);
	float VelocityControl(float distance,float velocity);
private:
	float _max_position,_min_position;
	float _max_velocity,_min_velocity;
};

#endif /* LONCONTROL_LON_CONTROL_H_ */

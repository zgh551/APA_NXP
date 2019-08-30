/*****************************************************************************/
/* FILE NAME: lon_control.h                       COPYRIGHT (c) Motovis 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this class using to interpolation data  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 3 2019      Initial Version                  */
/* 1.0	 Guohua Zhu     July   30 2019      Add Dongfeng Control Function    */
/*****************************************************************************/

#ifndef LONCONTROL_LON_CONTROL_H_
#define LONCONTROL_LON_CONTROL_H_

#include "derivative.h"
#include "property.h"
#include "controller.h"
#include "pid.h"
#include "math.h"
#include "vehilce_config.h"
#include "interpolation.h"

/**************************速度控制******************************/
#define MAX_POSITION            ( 0.9 ) // 速度控制上限点
#define MIN_POSITION            ( 0.4 ) // 速度控制下限点
#define MAX_VELOCITY	  		( 1.0 ) // 直线段的速度
#define MIN_VELOCITY	      	( 0.3 ) // 曲线段的速度

class LonControl:public Controller
{
public:
	LonControl();
	virtual ~LonControl();

	void Init() override;

	void Proc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid) override;

	void VelocityProc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid);

	void AccProc(MessageManager *msg,VehicleController *ctl,PID *acc_pid);

	void VelocityLookupProc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid);

	float VelocityPlanningControl(float distance);
	float VelocityControl(float distance,float velocity);
private:
	float _max_position,_min_position;
	float _max_velocity,_min_velocity;

	float _vehicle_velocity;	// 车辆速度
	float _throttle_lowerbound; // 油门最低边界
	float _brake_lowerbound;    // 制动最低边界
	float _calibration_value;   // 校准值

	VehilceConfig _lon_VehilceConfig;
	Interpolation _lon_Interpolation;
};

#endif /* LONCONTROL_LON_CONTROL_H_ */

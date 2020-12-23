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

#include "math.h"
#include "../Interface/controller.h"
#include "../../Common/Math/interpolation.h"

/**************************速度控制******************************/
#define MAX_POSITION            ( 1.0  ) // 速度控制上限位置
#define MIN_POSITION            ( 0.05 ) // 速度控制下限位置
#define MAX_VELOCITY	  		( 1.0  ) // 直线段的速度
#define MIN_VELOCITY	      	( 0.1  ) // 曲线段的速度
/**************************加速度控制******************************/
#define START_ACC               ( 0.2f  ) // 车辆起步时的正向加速度
#define DT                      ( 0.02f ) // 控制时间间隔

typedef enum _Lon_VelocityControlState
{
	VelocityStartStatus = 0,
	WaitVelocityStableStatus
}Lon_VelocityControlState;

typedef enum _LonControlState
{
	LON_WaitStart = 0,
	LON_ShortStart,
	LON_Starting,
	LON_Running,
	LON_EmergencyBrake,
	LON_ComfortBrake
}LonControlState;

class LonControl:public Controller
{
public:
	LonControl();
	virtual ~LonControl();

	void Init() override;

	void Proc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid) override;
	void AccProc(MessageManager *msg,VehicleController *ctl,PID *acc_pid);
	/*
	 * @brief the velocity control base on the accelerate interface
	 */
	void VelocityProc(MessageManager &msg, VehicleController &ctl, PID &velocity_pid);
	void VelocityLookupProc(MessageManager *msg, VehicleController *ctl,PID *velocity_pid);
	void VelocityLookupProc(MessageManager *msg, VehicleController *ctl,PID *start_velocity_pid,PID *velocity_pid);

	void DistanceProc(MessageManager *msg,VehicleController *ctl);

	float VelocityPlanningControl(float distance);
	float VelocityControl(float distance,float velocity);
	float AcceleratePlanningControl(float cur_velocity,float stop_distance);

	float AccAccelerateControlStop(float cur_velocity,float stop_distance);
	float AccAccelerateControlStart(float cur_velocity,float stop_distance);

	float getControlStateFlag();
	void  setControlStateFlag(float value);
	Property<LonControl,float,READ_WRITE> ControlStateFlag;
private:
	float _max_position,_min_position;
	float _max_velocity,_min_velocity;

	float _vehicle_velocity;	// 车辆速度
	float _throttle_lowerbound; // 油门最低边界
	float _brake_lowerbound;    // 制动最低边界
	float _calibration_value;   // 校准值

	VehilceConfig _lon_VehilceConfig;
	Interpolation _lon_Interpolation;

	Lon_VelocityControlState _lon_velocity_control_state;
	LonControlState _lon_control_state;
	GearStatus _current_gear,_last_gear;
	float _current_velocity,_last_velocity;
	uint8_t _control_state_flag;
	float _target_velocity;
	float _actual_velocity;
	float _delta_velocity;
	float _distance_update_distance_value;
	float _variable_distance_value;
	float _distance_update_pulse_value;

	// Start
	float _pid_acc;
	float _pid_slow_start_acc;

	// braking,update the current distance or pulse
	float _brake_distance = 0.0f;
	uint16_t _last_pulse_rear_left;
	uint16_t _last_pulse_rear_right;

    float _init_stop_velocity;
    float _teory_velocity;
    uint16_t _init_time_count = 0;

	float _delta_distance ; // the delta distance
	float _change_distance; // the change distance
	float _update_distance; // the update distance
	float _remain_distance; // the remain distance

	GearStatus _last_update_gear;

	uint16_t _delta_pulse_rear_left; // the change number of left rear pulse
	uint16_t _delta_pulse_rear_right;// the change number of right rear pulse

	// the accelerate of vehicle stop
	float _vehicle_start_acc;
	float _vehicle_start_acc_acc;

	float _vehicle_stop_acc;
	float _vehicle_stop_acc_acc;
	float _vehicle_slow_down_acc;
};

#endif /* LONCONTROL_LON_CONTROL_H_ */

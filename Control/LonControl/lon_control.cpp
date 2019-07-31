/*****************************************************************************/
/* FILE NAME: lon_control.cpp                     COPYRIGHT (c) Motovis 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this class using to interpolation data  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 3 2019      Initial Version                  */
/* 1.0	 Guohua Zhu     July   30 2019      Add Dongfeng Control Function    */
/*****************************************************************************/

#include <lon_control.h>

LonControl::LonControl() {
	Init();
}

LonControl::~LonControl() {

}

void LonControl::Init()
{
	_max_position = MAX_POSITION;// 速度控制上限点
	_min_position = MIN_POSITION;// 速度控制下限点
	_max_velocity = MAX_VELOCITY;// 直线段的速度
	_min_velocity = MIN_VELOCITY;// 曲线段的速度

	_throttle_lowerbound = 50;//Nm
	_brake_lowerbound = 0;// m/s2

}

//根据距离规划速度
float LonControl::VelocityPlanningControl(float distance)
{
	if(distance > _max_position)
	{
		return _max_velocity;
	}
	else if(distance > _min_position)
	{
		return _min_velocity + (_max_velocity - _min_velocity)*(distance - _min_position)/(_max_position - _min_position);
	}
	else if(distance > 0.05)
	{
		return distance * _min_velocity / _min_position ;
	}
	else
	{
		return 0;
	}
}


float LonControl::VelocityControl(float distance,float velocity)
{
	float temp_v;
	temp_v = VelocityPlanningControl(distance);
	return velocity < temp_v ? velocity :temp_v;
}

void LonControl::Proc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid)
{
	if(ctl->VelocityEnable)
	{
		_vehicle_velocity = (msg->WheelSpeedRearLeft + msg->WheelSpeedRearRight) * 0.5;
//		velocity_pid->Desired = ctl->Velocity;
		velocity_pid->Desired = VelocityControl(ctl->Velocity,ctl->Distance);
		ctl->Acceleration = velocity_pid->pidUpdateIntegralSeparation(_vehicle_velocity);
		if((ctl->getAcceleration() < 1.0e-6) && (velocity_pid->Desired < 1.0e-6))
		{
			ctl->Acceleration = -0.5;
		}
	}
}

void LonControl::VelocityProc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid)
{
	if(ctl->VelocityEnable)
	{
		_vehicle_velocity = (msg->WheelSpeedRearLeft + msg->WheelSpeedRearRight) * 0.5;
		velocity_pid->Desired = ctl->Velocity;
		ctl->TargetAcceleration = velocity_pid->pidUpdateIntegralSeparation(_vehicle_velocity);
	}
}

void LonControl::AccProc(MessageManager *msg,VehicleController *ctl,PID *acc_pid)
{
	if(ctl->TorqueEnable)
	{
		if(ctl->TargetAcceleration < 0)
		{
			ctl->Torque = 0;
			ctl->Acceleration = ctl->TargetAcceleration;
			ctl->AccelerationEnable = 1;
		}
		else
		{
			acc_pid->Desired = ctl->TargetAcceleration;
			if(msg->Gear == Drive)
			{
				ctl->Torque = acc_pid->pidUpdateIntegralSeparation(msg->LonAcc);
			}
			else if(msg->Gear == Reverse)
			{
				ctl->Torque = acc_pid->pidUpdateIntegralSeparation(-msg->LonAcc);
			}
			ctl->Acceleration = 0;
			ctl->AccelerationEnable = 0;
		}
	}
}

void LonControl::VelocityLookupProc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid)
{
	if(ctl->VelocityEnable)
	{
		_vehicle_velocity = (msg->WheelSpeedRearLeft + msg->WheelSpeedRearRight) * 0.5;
//		velocity_pid->Desired = ctl->Velocity;
		velocity_pid->Desired   = VelocityControl(ctl->Velocity,ctl->Distance);
		ctl->TargetAcceleration = velocity_pid->pidUpdateIntegralSeparation(_vehicle_velocity);

		if(ctl->TargetAcceleration >= 0)
		{
			ctl->TorqueEnable       = 1;
			ctl->AccelerationEnable = 0;
			_calibration_value = _lon_Interpolation.Interpolation2D(ctl->TargetAcceleration, _vehicle_velocity,
																	_lon_VehilceConfig.AccelerateTable, _lon_VehilceConfig.AccNum,
																	_lon_VehilceConfig.VelocityTable, _lon_VehilceConfig.VlcNum,
																	_lon_VehilceConfig.TorqueTable);
			if(_calibration_value >= 0)
			{
				ctl->Torque = fmax(_calibration_value,_throttle_lowerbound);//添加最小死区的判断
			}
			else
			{
				ctl->Torque = _throttle_lowerbound;
			}
			ctl->Acceleration = 0;
		}
		else
		{
			ctl->AccelerationEnable = 1;
			ctl->TorqueEnable       = 1;
			_calibration_value = ctl->TargetAcceleration;
			if(_calibration_value >= 0)
			{
				ctl->Acceleration = _brake_lowerbound;
			}
			else
			{
				ctl->Acceleration = _calibration_value;
			}
			ctl->Torque = 0;
		}
	}
}

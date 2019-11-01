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
	ControlStateFlag.setContainer(this);
	ControlStateFlag.getter(&LonControl::getControlStateFlag);
	ControlStateFlag.setter(&LonControl::setControlStateFlag);

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

	_throttle_lowerbound = 30;//Nm
	_brake_lowerbound = -0.123;// m/s2

	_control_state_flag = 0;
	_lon_velocity_control_state = VelocityStartStatus;

	_delta_velocity = 0.3 * 0.02f;
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
		velocity_pid->Desired = VelocityControl(ctl->Distance,ctl->Velocity);
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

void LonControl::VelocityLookupProc(MessageManager *msg,VehicleController *ctl,PID *start_velocity_pid,PID *velocity_pid)
{
	_current_gear = msg->getGear();

//	_target_velocity = ctl->Velocity;//纯速度控制
	_target_velocity = VelocityControl(ctl->Distance,ctl->Velocity);//距离速度控制

	_vehicle_velocity = msg->VehicleMiddleSpeed;
	_current_velocity = _vehicle_velocity;

	switch(_lon_velocity_control_state)
	{
		case VelocityStartStatus:
			if(_current_gear != _last_gear)//换挡
			{
				_control_state_flag = 0xAA;
				_lon_velocity_control_state = WaitVelocityStableStatus;
			}
			else if(_target_velocity < 1.0e-6f)//目标速度接近0
			{
				_control_state_flag = 0xAA;
				_lon_velocity_control_state = WaitVelocityStableStatus;
			}
			break;

		case WaitVelocityStableStatus:
			if( (fabs(_vehicle_velocity - _target_velocity) < 0.1f) && (_target_velocity > 0.1F))//接近目标速度
			{
				_control_state_flag = 0x55;
				_lon_velocity_control_state = VelocityStartStatus;
			}
			else if(_current_velocity < _last_velocity)//速度变慢
			{
				_control_state_flag = 0x55;
				_lon_velocity_control_state = VelocityStartStatus;
			}
			break;

		default:

			break;
	}
	_last_gear = _current_gear;
	_last_velocity = _current_velocity;

	if(ctl->VelocityEnable)
	{
//		if(0x55 == _control_state_flag)//启动后速度控制
//		{
			velocity_pid->Desired = _target_velocity;
			ctl->TargetAcceleration = velocity_pid->pidUpdate(_vehicle_velocity);
//		}
//		else if(0xAA == _control_state_flag)//启动前
//		{
//			start_velocity_pid->Desired = _target_velocity;
//			ctl->TargetAcceleration = start_velocity_pid->pidUpdate(_vehicle_velocity);
//		}

		if(ctl->TargetAcceleration > 0)
		{
			ctl->TorqueEnable       = 1;
			_calibration_value = _lon_Interpolation.Interpolation2D(ctl->TargetAcceleration, _vehicle_velocity,
																	_lon_VehilceConfig.AccelerateTable, _lon_VehilceConfig.AccNum,
																	_lon_VehilceConfig.VelocityTable, _lon_VehilceConfig.VlcNum,
																	_lon_VehilceConfig.TorqueTable);

			if(velocity_pid->Desired < 1.0e-6)//速度为0时，直接刹住
			{
				ctl->Torque = 0;
				ctl->Acceleration = -0.5;
				ctl->AccelerationEnable = 1;
			}
			else
			{
				if(ctl->TargetAcceleration >= 1.0e-2)
				{
					ctl->AccelerationEnable = 0;
					ctl->Torque = fmax(_calibration_value,_throttle_lowerbound);//添加最小死区的判断
					ctl->Acceleration = 0;
				}
				else
				{
					ctl->Torque = _throttle_lowerbound;
					ctl->Acceleration = 0;
					ctl->AccelerationEnable = 0;
				}
			}
		}
		else
		{
			ctl->TorqueEnable  = 1;
			_calibration_value = ctl->TargetAcceleration;

			if(velocity_pid->Desired < 1.0e-6)
			{
				ctl->AccelerationEnable = 1;
				ctl->Acceleration = -0.6;
			}
			else
			{
				ctl->Torque = _throttle_lowerbound;
//				if(_calibration_value >= -1.0e-6)
//				{
//					ctl->Acceleration = _brake_lowerbound;
//				}
//				else
//				{
//					ctl->Acceleration = _calibration_value;
//				}
			}
		}
	}
	else
	{
		ctl->AccelerationEnable = 0;
		ctl->TorqueEnable       = 0;
		ctl->Acceleration = 0;
		ctl->Torque       = 0;
	}
}

void LonControl::VelocityLookupProc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid)
{
//	_target_velocity = ctl->Velocity;//纯速度控制
	_target_velocity = VelocityControl(ctl->Distance,ctl->Velocity);//距离速度控制
	_vehicle_velocity = msg->VehicleMiddleSpeed;

	if(ctl->VelocityEnable)
	{
		if(msg->BrakePressure > 0)
		{
			velocity_pid->Desired = -0.1f;
		}
		else
		{
			if(velocity_pid->Desired <= (_target_velocity - _delta_velocity))
			{
				velocity_pid->Desired =  velocity_pid->Desired + _delta_velocity;//_target_velocity;
			}
			else
			{
				velocity_pid->Desired = _target_velocity;
			}
		}

		ctl->TargetAcceleration = velocity_pid->pidUpdate(_vehicle_velocity);
		ctl->TorqueEnable       = 1;

		if(ctl->TargetAcceleration > 0)
		{
			_calibration_value = _lon_Interpolation.Interpolation2D(ctl->TargetAcceleration, _vehicle_velocity,
																	_lon_VehilceConfig.AccelerateTable, _lon_VehilceConfig.AccNum,
																	_lon_VehilceConfig.VelocityTable, _lon_VehilceConfig.VlcNum,
																	_lon_VehilceConfig.TorqueTable);

			if(velocity_pid->Desired < 1.0e-6)//速度为0时，直接刹住
			{
				ctl->Torque = 0;
				ctl->Acceleration = -0.5;
				ctl->AccelerationEnable = 1;
			}
			else
			{
				if(ctl->TargetAcceleration >= 1.0e-2)
				{
					ctl->AccelerationEnable = 0;
					ctl->Torque = fmax(_calibration_value,_throttle_lowerbound);//添加最小死区的判断
					ctl->Acceleration = 0;
				}
				else
				{
					ctl->Torque = _throttle_lowerbound;
					ctl->Acceleration = 0;
					ctl->AccelerationEnable = 0;
				}
			}
		}
		else
		{
			_calibration_value = ctl->TargetAcceleration;

			if((velocity_pid->Desired < 1.0e-6) && (velocity_pid->Desired > -1.0e-6))
			{
				ctl->AccelerationEnable = 1;
				ctl->Acceleration = -0.5;
				ctl->Torque = 0;
			}
			else
			{
				ctl->AccelerationEnable = 0;
				ctl->Torque = _throttle_lowerbound;
			}
		}
	}
	else
	{
		ctl->AccelerationEnable = 0;
		ctl->TorqueEnable       = 0;
		ctl->Acceleration = 0;
		ctl->Torque       = 0;
	}
}

float LonControl::getControlStateFlag()           { return _control_state_flag;}
void  LonControl::setControlStateFlag(float value){_control_state_flag = value;}

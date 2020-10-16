/*
 * geometric_track.cpp
 *
 *  Created on: January 9 2019
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: geometric_track.cpp                 COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: track the vehilce position        					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 9 2019      Initial Version                  */
/*****************************************************************************/
#include "geometric_track.h"

VehilceConfig m_GeometricVehicleConfig;

GeometricTrack::GeometricTrack() {
	Init();
}

GeometricTrack::~GeometricTrack() {

}

void GeometricTrack::Init(void)
{
	_position.setX(0);
	_position.setY(0);
	Yaw = 0.0f;
	_last_yaw = Yaw;
	LinearVelocity = 0.0f;

	_last_rear_left_pulse   = 0;
	_last_rear_right_pulse  = 0;
	_delta_rear_left_pulse  = 0;
	_delta_rear_right_pulse = 0;

	 _cumulation_rear_left_pulse  = 0;
	 _cumulation_rear_right_pulse = 0;

	_wait_time_cnt = 0;
}

void GeometricTrack::Init(float x,float y,float yaw)
{
	_position.setX(x);
	_position.setY(y);
	Yaw = yaw;
	_last_yaw = Yaw;
	LinearVelocity = 0.0f;

	_last_rear_left_pulse   = 0;
	_last_rear_right_pulse  = 0;
	_delta_rear_left_pulse  = 0;
	_delta_rear_right_pulse = 0;

	 _cumulation_rear_left_pulse  = 0;
	 _cumulation_rear_right_pulse = 0;

	_wait_time_cnt = 0;
}

float GeometricTrack::pi2pi(float angle)
{
	if(angle > M_PI)
	{
		return angle - 2 * M_PI;
	}
	else if(angle <= -M_PI)
	{
		return angle + 2 * M_PI;
	}
	else
	{
		return angle;
	}
}

void GeometricTrack::VelocityUpdate(MessageManager &msg,float dt)
{
	float radius;
	LinearRate = (msg.WheelSpeedRearRight + msg.WheelSpeedRearLeft)*0.5;
	LinearVelocity = msg.WheelSpeedDirection == Forward ?  LinearRate :
					 msg.WheelSpeedDirection == Backward ? -LinearRate : 0;

	if( ((int16_t)fabs(msg.SteeringAngle) != 0) && ((int16_t)fabs(msg.SteeringAngle) < 520))
	{
		radius = m_GeometricVehicleConfig.TurnRadiusCalculate(msg.SteeringAngle);
		Yaw  = _last_yaw + LinearVelocity * dt / radius;
		_position.setX(_position.getX() + radius * (sinf(Yaw) - sinf(_last_yaw)));
		_position.setY(_position.getY() + radius * (cosf(_last_yaw) - cosf(Yaw)));
	}
	else
	{
		_position.setX(_position.getX() + LinearVelocity * cosf(Yaw) * dt);
		_position.setY(_position.getY() + LinearVelocity * sinf(Yaw) * dt);
	}
	_last_yaw = Yaw;
}

void GeometricTrack::PulseUpdate(MessageManager &msg)
{
	float displacement,radius;

	if( (0 == _delta_rear_left_pulse) && (0 == _last_rear_left_pulse) )
	{
		_delta_rear_left_pulse = 0;
	}
	else
	{
		_delta_rear_left_pulse =  msg.getWheelSpeedDirection() == Forward ?
								((msg.getWheelPulseRearLeft() >= _last_rear_left_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								  msg.getWheelPulseRearLeft()  - _last_rear_left_pulse) :
								  msg.getWheelSpeedDirection() == Backward ?
							   -((msg.getWheelPulseRearLeft() >= _last_rear_left_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								  msg.getWheelPulseRearLeft()  - _last_rear_left_pulse) : 0;
	}

	if( (0 == _delta_rear_right_pulse) && (0 == _last_rear_right_pulse) )
	{
		_delta_rear_right_pulse = 0;
	}
	else
	{
		_delta_rear_right_pulse =  msg.getWheelSpeedDirection() == Forward ?
								 ((msg.getWheelPulseRearRight() >= _last_rear_right_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								   msg.getWheelPulseRearRight()  - _last_rear_right_pulse) :
								   msg.getWheelSpeedDirection() == Backward ?
								-((msg.getWheelPulseRearRight() >= _last_rear_right_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								   msg.getWheelPulseRearRight()  - _last_rear_right_pulse) : 0;
	}
	displacement = (_delta_rear_left_pulse + _delta_rear_right_pulse) * 0.5f * WHEEL_PUSLE_RATIO;

	LinearVelocity = msg.getWheelSpeedDirection() == Forward ?  LinearRate :
					 msg.getWheelSpeedDirection() == Backward ? -LinearRate : 0;

	if( ((int16_t)fabs(msg.getSteeringAngle()) != 0) && ((uint16_t)fabs(msg.getSteeringAngle()) < 520))
	{
		radius = m_GeometricVehicleConfig.TurnRadiusCalculate(msg.getSteeringAngle());
		Yaw  = _last_yaw + displacement / radius;
		_position.setX(_position.getX() + radius * (sinf(Yaw) - sinf(_last_yaw)));
		_position.setY(_position.getY() + radius * (cosf(_last_yaw) - cosf(Yaw)));
	}
	else
	{
		_position.setX(_position.getX() + displacement * cosf(Yaw));
		_position.setY(_position.getY() + displacement * sinf(Yaw));
	}

	msg.setRearLeftSumPulse (msg.getRearLeftSumPulse()  + _delta_rear_left_pulse );
	msg.setRearRightSumPulse(msg.getRearRightSumPulse() + _delta_rear_right_pulse);

	_last_rear_left_pulse  = msg.getWheelPulseRearLeft();
	_last_rear_right_pulse = msg.getWheelPulseRearRight();
	_last_yaw = Yaw;
}

void GeometricTrack::PulseTrackUpdate(MessageManager &msg)
{
	float displacement;

	if( (0 == _delta_rear_left_pulse) && (0 == _last_rear_left_pulse) )
	{
		_delta_rear_left_pulse = 0;
	}
	else
	{
		_delta_rear_left_pulse =  msg.getWheelSpeedDirection() == Forward ?
								((msg.getWheelPulseRearLeft() >= _last_rear_left_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								  msg.getWheelPulseRearLeft()  - _last_rear_left_pulse) :
								  msg.getWheelSpeedDirection() == Backward ?
							   -((msg.getWheelPulseRearLeft() >= _last_rear_left_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								  msg.getWheelPulseRearLeft()  - _last_rear_left_pulse) : 0;
	}

	if( (0 == _delta_rear_right_pulse) && (0 == _last_rear_right_pulse) )
	{
		_delta_rear_right_pulse = 0;
	}
	else
	{
		_delta_rear_right_pulse =  msg.getWheelSpeedDirection() == Forward ?
								 ((msg.getWheelPulseRearRight() >= _last_rear_right_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								   msg.getWheelPulseRearRight()  - _last_rear_right_pulse) :
								   msg.getWheelSpeedDirection() == Backward ?
								-((msg.getWheelPulseRearRight() >= _last_rear_right_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								   msg.getWheelPulseRearRight()  - _last_rear_right_pulse) : 0;
	}

	displacement = (_delta_rear_left_pulse + _delta_rear_right_pulse) * 0.5f * WHEEL_PUSLE_RATIO;
	if(_delta_rear_right_pulse == _delta_rear_left_pulse)
	{
		_position.setX(_position.getX() + displacement * cosf(Yaw));
		_position.setY(_position.getY() + displacement * sinf(Yaw));
	}
	else
	{
		_delta_yaw = (_delta_rear_right_pulse - _delta_rear_left_pulse)/WIDTH;
		Yaw  = _last_yaw + _delta_yaw;
		_position.setX(_position.getX() + displacement * cosf(_last_yaw + _delta_yaw*0.5));
		_position.setY(_position.getY() + displacement * sinf(_last_yaw + _delta_yaw*0.5));
	}
	_last_yaw = Yaw;
}

void GeometricTrack::VelocityPulseUpdate(MessageManager &msg)
{
	////// pulse update
	if((0 == _delta_rear_left_pulse) && (0 == _last_rear_left_pulse))// first running,use to update the last_pulse
	{
		_delta_rear_left_pulse = 0;
	}
	else
	{
		_delta_rear_left_pulse =  msg.getWheelSpeedDirection() == Forward ?
								((msg.getWheelPulseRearLeft() >= _last_rear_left_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								  msg.getWheelPulseRearLeft()  - _last_rear_left_pulse) :
								  msg.getWheelSpeedDirection() == Backward ?
							   -((msg.getWheelPulseRearLeft() >= _last_rear_left_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								  msg.getWheelPulseRearLeft()  - _last_rear_left_pulse) : 0;
	}

	if((0 == _delta_rear_right_pulse) && (0 == _last_rear_right_pulse))
	{
		_delta_rear_right_pulse = 0;
	}
	else
	{
		_delta_rear_right_pulse =  msg.getWheelSpeedDirection() == Forward ?
								 ((msg.getWheelPulseRearRight() >= _last_rear_right_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								   msg.getWheelPulseRearRight()  - _last_rear_right_pulse) :
								   msg.getWheelSpeedDirection() == Backward ?
								-((msg.getWheelPulseRearRight() >= _last_rear_right_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								   msg.getWheelPulseRearRight()  - _last_rear_right_pulse) : 0;
	}

	// select the vehicle speed update source : wheel speed or pulse speed
	if((msg.getWheelSpeedRearRight() > 2.0f) && (msg.getWheelSpeedRearLeft() > 2.0f))
	{
		_cumulation_rear_left_pulse  = 0;
		_cumulation_rear_right_pulse = 0;
		_wait_time_cnt               = 0;
		msg.setVehicleMiddleSpeed((msg.getWheelSpeedRearRight() + msg.getWheelSpeedRearLeft()) * 0.5f);
	}
	else
	{
		_cumulation_rear_left_pulse  += _delta_rear_left_pulse;
		_cumulation_rear_right_pulse += _delta_rear_right_pulse;
		_cumulation_middle_displacement = fabs((_cumulation_rear_left_pulse + _cumulation_rear_right_pulse) * 0.5f * WHEEL_PUSLE_RATIO);
		_wait_time_cnt++;

		if (_cumulation_middle_displacement > WHEEL_PUSLE_RATIO)
		{
			// Step1: calculate the velocity update by pulse
			_pul_update_velocity = _cumulation_middle_displacement  * 50.0f / _wait_time_cnt;

			// Step2: calculate the accelerate which update by the pulse velocity
			_pul_update_acc = (_pul_update_velocity - _last_pul_update_velocity) * 5.1f / _wait_time_cnt;

			// Step3: calculate the velocity error between pulse velocity and the accelerate velocity
			_err_update_velocity = (_pul_update_velocity - _acc_update_velocity) * 0.1f;

			// Step4: calculate the longitudinal gravity accelerate
			_lon_gravity_acc = msg.getLonAcc()
					         -(msg.getWheelSpeedDirection() == Forward  ?  _pul_update_acc :
					           msg.getWheelSpeedDirection() == Backward ? -_pul_update_acc : 0.0f);

			// initialize the pulse variable and the time count
			_cumulation_rear_left_pulse  = 0;
			_cumulation_rear_right_pulse = 0;
			_wait_time_cnt               = 0;

			// save current pulse velocity for last
			_last_pul_update_velocity = _pul_update_velocity;
		}
		else
		{
			if (_wait_time_cnt > 80)
			{
				_cumulation_rear_left_pulse  = 0;
				_cumulation_rear_right_pulse = 0;
				_wait_time_cnt               = 0;
				_acc_update_velocity         = 0.0f;
				_err_update_velocity         = 0.0f;
				_pul_update_velocity         = 0.0f;
				_pul_update_acc              = 0.0f;
				_lon_gravity_acc             = 0.0f;
			}
			else
			{
				_vehicle_velocity_acc = msg.getWheelSpeedDirection() == Forward  ?   msg.getLonAcc() - _lon_gravity_acc  :
									    msg.getWheelSpeedDirection() == Backward ? -(msg.getLonAcc() - _lon_gravity_acc) : 0.0f;
				// TODO 加速计补偿
				_acc_update_velocity += (_vehicle_velocity_acc + _err_update_velocity) * 0.196; // acc * g(9.8 [m/s^2]) * dt(0.02 [s])

				_acc_update_velocity = _acc_update_velocity < 1.0e-6f ? 0.0f : _acc_update_velocity;
			}
		}
		msg.setVehicleMiddleSpeed(_acc_update_velocity);
	}
	/**************************************************Velocity Checker****************************************/

	/************************************************** Track Calculate ****************************************/
	float displacement = (_delta_rear_left_pulse + _delta_rear_right_pulse) * 0.5f * WHEEL_PUSLE_RATIO;
	////////////////////////////Delta Displace//////////////////////////////////
	if(((int16_t)fabs(msg.getSteeringAngle()) != 0) && ((uint16_t)fabs(msg.getSteeringAngle()) < 520))
	{
		float radius = m_GeometricVehicleConfig.TurnRadiusCalculate(msg.getSteeringAngle());
		_yaw  = pi2pi(_last_yaw + displacement / radius);
		_position.setX(_position.getX() + radius * (sinf(_yaw) - sinf(_last_yaw)));
		_position.setY(_position.getY() + radius * (cosf(_last_yaw) - cosf(_yaw)));
	}
	else
	{
		_position.setX(_position.getX() + displacement * cosf(_yaw));
		_position.setY(_position.getY() + displacement * sinf(_yaw));
	}
	/////////////////////////////Pulse Track/////////////////////////////////
//	if(_delta_rear_right_pulse == _delta_rear_left_pulse)
//	{
//		_position.X = _position.X + displacement * cosf(Yaw);
//		_position.Y = _position.Y + displacement * sinf(Yaw);
//	}
//	else
//	{
//		_delta_yaw = (_delta_rear_right_pulse - _delta_rear_left_pulse)/WIDTH;
//		Yaw  = _last_yaw + _delta_yaw;
//		_position.X = _position.X + displacement * cosf(_last_yaw + _delta_yaw*0.5);
//		_position.Y = _position.Y + displacement * sinf(_last_yaw + _delta_yaw*0.5);
//	}
	/////////////////////////////////////////////////////////////
	_last_yaw = _yaw;
	_last_rear_left_pulse  = msg.getWheelPulseRearLeft();
	_last_rear_right_pulse = msg.getWheelPulseRearRight();

	msg.setRearLeftSumPulse (msg.getRearLeftSumPulse()  + _delta_rear_left_pulse );
	msg.setRearRightSumPulse(msg.getRearRightSumPulse() + _delta_rear_right_pulse);
	msg.setWheelSumPulse((int32_t)((msg.getRearRightSumPulse() + msg.getRearLeftSumPulse()) * 0.5f));
}
/***************************************** End *********************************************/

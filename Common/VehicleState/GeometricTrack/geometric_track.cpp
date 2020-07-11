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

	// 閫熷害鍒ゅ畾
	if((msg.getWheelSpeedRearRight() > 0.2f) && (msg.getWheelSpeedRearLeft() > 0.2f))
	{
		msg.setVehicleMiddleSpeed((msg.getWheelSpeedRearRight() + msg.getWheelSpeedRearLeft()) * 0.5f);
	}
	else
	{
		msg.setVehicleMiddleSpeed(0);
	}
	/*************************************************鑴夊啿璁＄畻閫熷害********************************************************/
	 if((_delta_rear_left_pulse > 0) || (_delta_rear_right_pulse > 0))//鑴夊啿鏇存柊瑙ｉ攣
	 {
		 _velocity_lock = 0;
	 }
	// 绱Н鑴夊啿杈惧埌鎸囧畾
	 _cumulation_rear_left_pulse  += _delta_rear_left_pulse;
	 _cumulation_rear_right_pulse += _delta_rear_right_pulse;
	 _cumulation_middle_displacement = (_cumulation_rear_left_pulse + _cumulation_rear_right_pulse) * 0.5f * WHEEL_PUSLE_RATIO;
	 if(fabs(_cumulation_middle_displacement) >= 0.04f)
	 {
		 _velocity_lock = 0;
		 _wait_time_cnt++;
		 setPulseUpdateVelocity(_cumulation_middle_displacement  * 50 /_wait_time_cnt);
		 _cumulation_rear_left_pulse  = 0;
		 _cumulation_rear_right_pulse = 0;
		 _wait_time_cnt               = 0;
	 }
	 else if(_wait_time_cnt >= 25)
	 {
		 if((msg.getWheelSpeedRearRight() < 1.0e-6) && (msg.getWheelSpeedRearLeft() < 1.0e-6))
		 {
			 _velocity_lock               = 0xff;
			 _cumulation_rear_left_pulse  = 0;
			 _cumulation_rear_right_pulse = 0;
			 _wait_time_cnt               = 0;
			 setPulseUpdateVelocity(0);
		 }
	 }
	 else
	 {
		 if(0xff != _velocity_lock)
		 {
			 _wait_time_cnt++;
		 }
	 }
	 /**************************************************鑴夊啿璁＄畻缁撴潫****************************************/
	 // 杞剦鍐蹭笌杞�熸瘮杈冿紝鍒ゆ柇閫熷害鏄惁寮傚父
	 if(msg.getVehicleMiddleSpeed() < 1.0e-6)
	 {
		 if( getPulseUpdateVelocity() < 0.3f )
		 {
			 msg.setVehicleMiddleSpeedAbnormal(SpeedNormal);
		 }
		 else
		 {
			 msg.setVehicleMiddleSpeedAbnormal(SpeedAbnormal);
		 }
		 msg.setVehicleMiddleSpeed(fabs(getPulseUpdateVelocity()));
	 }
	 else
	 {
		 if(fabs(msg.getVehicleMiddleSpeed() - getPulseUpdateVelocity()) < 0.2f)
		 {
			 msg.setVehicleMiddleSpeedAbnormal(SpeedNormal);
		 }
		 else
		 {
			 msg.setVehicleMiddleSpeedAbnormal(SpeedAbnormal);
		 }
	 }

	displacement = (_delta_rear_left_pulse + _delta_rear_right_pulse) * 0.5f * WHEEL_PUSLE_RATIO;
	//////////////////////////////////////////////////////////////
	if( ((int16_t)fabs(msg.getSteeringAngle()) != 0) && ((uint16_t)fabs(msg.getSteeringAngle()) < 520))
	{
		radius = m_GeometricVehicleConfig.TurnRadiusCalculate(msg.getSteeringAngle());
		_yaw  = pi2pi(_last_yaw + displacement / radius);
		_position.setX(_position.getX() + radius * (sinf(_yaw) - sinf(_last_yaw)));
		_position.setY(_position.getY() + radius * (cosf(_last_yaw) - cosf(_yaw)));
	}
	else
	{
		_position.setX(_position.getX() + displacement * cosf(_yaw));
		_position.setY(_position.getY() + displacement * sinf(_yaw));
	}
	//////////////////////////////////////////////////////////////
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
	msg.setRearLeftSumPulse (msg.getRearLeftSumPulse()  + _delta_rear_left_pulse );
	msg.setRearRightSumPulse(msg.getRearRightSumPulse() + _delta_rear_right_pulse);
	msg.setWheelSumPulse((int32_t)((msg.getRearRightSumPulse() + msg.getRearLeftSumPulse()) * 0.5f));

	_last_rear_left_pulse  = msg.getWheelPulseRearLeft();
	_last_rear_right_pulse = msg.getWheelPulseRearRight();
}
/**************************************************************************************/
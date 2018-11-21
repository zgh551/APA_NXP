/*
 * Vehicle.cpp
 *
 *  Created on: 2018Äê11ÔÂ21ÈÕ
 *      Author: zhuguohua
 */

#include "Vehicle.h"

Vehicle::Vehicle() {
	// TODO Auto-generated constructor stub
	SteeringWheelTargetAngle.setContainer(this);
	SteeringWheelTargetAngle.setter(&Vehicle::setSteeringWheelTargetAngle);
	SteeringWheelTargetAngle.getter(&Vehicle::getSteeringWheelTargetAngle);
}

Vehicle::~Vehicle() {
	// TODO Auto-generated destructor stub
}

int Vehicle::getSteeringWheelTargetAngle()
{
	return _steering_wheel_target_angle;
}

void Vehicle::setSteeringWheelTargetAngle(int value)
{
	_steering_wheel_target_angle = value;
}

/*
 * Vehicle.h
 *
 *  Created on: 2018Äê11ÔÂ21ÈÕ
 *      Author: zhuguohua
 */

#ifndef SRC_VEHICLE_H_
#define SRC_VEHICLE_H_
#include "Property.h"


class Vehicle {
public:
	Vehicle();
	virtual ~Vehicle();
	//
	int getSteeringWheelTargetAngle();
	void setSteeringWheelTargetAngle(int value);
	Property<Vehicle,int,READ_WRITE> SteeringWheelTargetAngle;
private:
	int _steering_wheel_target_angle;
};


#endif /* SRC_VEHICLE_H_ */

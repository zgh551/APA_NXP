/*
 * vehicle_body.h
 *
 *  Created on: 2019Äê1ÔÂ10ÈÕ
 *      Author: zhuguohua
 */

#ifndef MATH_VEHICLE_BODY_H_
#define MATH_VEHICLE_BODY_H_

#include "derivative.h"
#include "property.h"
#include <vector_2d.h>

class VehicleBody {
public:
	VehicleBody();
	virtual ~VehicleBody();

	void Rotation(float angle);

	Vector2d getCenter();
	void     setCenter(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> Center;

	Vector2d getFrontLeft();
	void     setFrontLeft(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> FrontLeft;

	Vector2d getFrontRight();
	void     setFrontRight(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> FrontRight;

	Vector2d getRearLeft();
	void     setRearLeft(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> RearLeft;

	Vector2d getRearRight();
	void     setRearRight(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> RearRight;

private:
	Vector2d _center;
	Vector2d _front_left;
	Vector2d _front_right;
	Vector2d _rear_left;
	Vector2d _rear_right;

	float _attitude_yaw;

};

#endif /* MATH_VEHICLE_BODY_H_ */

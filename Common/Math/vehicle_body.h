/*
 * vehicle_body.h
 *
 *  Created on: January 10 2019
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vehicle_body.h                      COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the vector property             					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 10 2019      Initial Version                 */
/*****************************************************************************/

#ifndef MATH_VEHICLE_BODY_H_
#define MATH_VEHICLE_BODY_H_

#include "derivative.h"
#include "property.h"
#include <vector_2d.h>
#include "math.h"
#include "chang_an_configure.h"
#include "vehilce_config.h"

class VehicleBody {
public:
	VehicleBody();
	virtual ~VehicleBody();

	float RotateAngle(Vector2d vehicle_edge,Vector2d boundary);
	void RotationCenter(float radius);
	void Rotate(float angle);
	void EdgePoint(void);


	void OneTrial(float radius,Vector2d r);



	float getAttitudeYaw();
	void  setAttitudeYaw(float value);
	Property<VehicleBody,float,READ_WRITE> AttitudeYaw;

	Vector2d getCenter();
	void     setCenter(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> Center;

	Vector2d getRotation();
	void     setRotation(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> Rotation;

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
	// the rotation radisu of the four edge
	float _attitude_yaw;
	Vector2d _center;
	Vector2d _rotation;
	Vector2d _front_left;
	Vector2d _front_right;
	Vector2d _rear_left;
	Vector2d _rear_right;
};

#endif /* MATH_VEHICLE_BODY_H_ */

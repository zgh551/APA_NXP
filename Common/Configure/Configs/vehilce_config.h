/*
 * vehilce_config.h
 *
 *  Created on: 2019Äê1ÔÂ10ÈÕ
 *      Author: zhuguohua
 */

#ifndef CONFIGURE_CONFIGS_VEHILCE_CONFIG_H_
#define CONFIGURE_CONFIGS_VEHILCE_CONFIG_H_

#include "derivative.h"
#include "property.h"
#include "math.h"
#include "chang_an_configure.h"

class VehilceConfig {
public:
	VehilceConfig();
	virtual ~VehilceConfig();

	void Init();
	void EdgeRadius(float r);

	float getFrontDiagonalAxis();
	void  setFrontDiagonalAxis(float value);
	Property<VehilceConfig,float,READ_WRITE> FrontDiagonalAxis;

	float getFrontDiagonalAngle();
	void  setFrontDiagonalAngle(float value);
	Property<VehilceConfig,float,READ_WRITE> FrontDiagonalAngle;

	float getRearDiagonalAxis();
	void  setRearDiagonalAxis(float value);
	Property<VehilceConfig,float,READ_WRITE> RearDiagonalAxis;

	float getRearDiagonalAngle();
	void  setRearDiagonalAngle(float value);
	Property<VehilceConfig,float,READ_WRITE> RearDiagonalAngle;

	float getRadiusFrontLeft();
	void  setRadiusFrontLeft(float value);
	Property<VehilceConfig,float,READ_WRITE> RadiusFrontLeft;

	float getRadiusFrontRight();
	void  setRadiusFrontRight(float value);
	Property<VehilceConfig,float,READ_WRITE> RadiusFrontRight;

	float getRadiusRearLeft();
	void  setRadiusRearLeft(float value);
	Property<VehilceConfig,float,READ_WRITE> RadiusRearLeft;

	float getRadiusRearRight();
	void  setRadiusRearRight(float value);
	Property<VehilceConfig,float,READ_WRITE> RadiusRearRight;
private:
	// the diagonal axis of the four edge to the center point and the angle
	float _front_diagonal_axis;
	float _front_diagonal_angle;
	float _rear_diagonal_axis;
	float _rear_diagonal_angle;

	float _radius_front_left;
	float _radius_front_right;
	float _radius_rear_left;
	float _radius_rear_right;
};

#endif /* CONFIGURE_CONFIGS_VEHILCE_CONFIG_H_ */

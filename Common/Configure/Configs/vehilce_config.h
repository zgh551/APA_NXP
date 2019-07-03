/*
 * vehilce_config.h
 *
 *  Created on: 2019年1月10日
 *      Author: zhuguohua
 */

#ifndef CONFIGURE_CONFIGS_VEHILCE_CONFIG_H_
#define CONFIGURE_CONFIGS_VEHILCE_CONFIG_H_

#include "derivative.h"
#include "property.h"
#include "math.h"
#include "vector_2d.h"
#include "system_config.h"

#ifdef CHANGAN
#include "chang_an_configure.h"
#endif
#ifdef BORUI
#include "bo_rui_configure.h"
#endif
#ifdef DONG_FENG_E70
#include "dong_feng_configure.h"
#endif

typedef struct _Location
{
	Vector2d Point;
	float Angle;
}Location;

class VehilceConfig {
public:
	VehilceConfig();
	virtual ~VehilceConfig();

	void Init();
	void EdgeRadius(float r);
	/**********************************************************************/
	float SteeringAngle2TurnningRadiusExp(float steer,float a,float b);
	float TurnningRadius2SteeringAngleExp(float radius,float a,float b);

	float SteeringAngle2TurnningRadius(float steer,float a,float b);
	float TurnningRadius2SteeringAngle(float radius,float a,float b);

	float TurnRadiusCurveFitting(float steering_angle);
	float TurnRadiusFindingTable(float steering_angle);

	float SteeringAngleCurveFitting(float radius);
	float SteeringAngleFindingCallback(uint16_t left_id,uint16_t right_id,float radius);
	float SteeringAngleFindingLoop(uint16_t left_id,uint16_t right_id,float radius);

	float TurnRadiusCalculate(float steering_angle);
	float SteeringAngleCalculate(float radius);
	/**********************************************************************/
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

	Location* getUltrasonicLocationArray();
	Property<VehilceConfig,Location*,READ_ONLY> UltrasonicLocationArray;
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

	Location _ultrasonic_location_array[12];
};

#endif /* CONFIGURE_CONFIGS_VEHILCE_CONFIG_H_ */

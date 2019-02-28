/*
 * vehilce_config.cpp
 *
 *  Created on: 2019年1月10日
 *      Author: zhuguohua
 */

#include <vehilce_config.h>

VehilceConfig::VehilceConfig() {
	FrontDiagonalAxis.setContainer(this);
	FrontDiagonalAxis.getter(&VehilceConfig::getFrontDiagonalAxis);
	FrontDiagonalAxis.setter(&VehilceConfig::setFrontDiagonalAxis);

	FrontDiagonalAngle.setContainer(this);
	FrontDiagonalAngle.getter(&VehilceConfig::getFrontDiagonalAngle);
	FrontDiagonalAngle.setter(&VehilceConfig::setFrontDiagonalAngle);

	RearDiagonalAxis.setContainer(this);
	RearDiagonalAxis.getter(&VehilceConfig::getRearDiagonalAxis);
	RearDiagonalAxis.setter(&VehilceConfig::setRearDiagonalAxis);

	RearDiagonalAngle.setContainer(this);
	RearDiagonalAngle.getter(&VehilceConfig::getRearDiagonalAngle);
	RearDiagonalAngle.setter(&VehilceConfig::setRearDiagonalAngle);

	RadiusFrontLeft.setContainer(this);
	RadiusFrontLeft.getter(&VehilceConfig::getRadiusFrontLeft);
	RadiusFrontLeft.setter(&VehilceConfig::setRadiusFrontLeft);

	RadiusFrontRight.setContainer(this);
	RadiusFrontRight.getter(&VehilceConfig::getRadiusFrontRight);
	RadiusFrontRight.setter(&VehilceConfig::setRadiusFrontRight);

	RadiusRearLeft.setContainer(this);
	RadiusRearLeft.getter(&VehilceConfig::getRadiusRearLeft);
	RadiusRearLeft.setter(&VehilceConfig::setRadiusRearLeft);

	RadiusRearRight.setContainer(this);
	RadiusRearRight.getter(&VehilceConfig::getRadiusRearRight);
	RadiusRearRight.setter(&VehilceConfig::setRadiusRearRight);

	Init();
}

VehilceConfig::~VehilceConfig() {

}

void VehilceConfig::Init()
{
	FrontDiagonalAxis = sqrtf( powf(LEFT_EDGE_TO_CENTER,2) + powf(FRONT_EDGE_TO_CENTER,2));
	RearDiagonalAxis  = sqrtf( powf(LEFT_EDGE_TO_CENTER,2) + powf(REAR_EDGE_TO_CENTER,2));

	FrontDiagonalAngle = atanf(LEFT_EDGE_TO_CENTER/FRONT_EDGE_TO_CENTER);
	RearDiagonalAngle  = atanf(LEFT_EDGE_TO_CENTER/REAR_EDGE_TO_CENTER);
}

// r is + and -
void VehilceConfig::EdgeRadius(float r)
{
	RadiusFrontLeft = sqrtf( powf( r - LEFT_EDGE_TO_CENTER , 2 ) +
			                 powf( FRONT_EDGE_TO_CENTER , 2 ) );

	RadiusFrontRight = sqrtf( powf( r + RIGHT_EDGE_TO_CENTER , 2 ) +
                              powf( FRONT_EDGE_TO_CENTER , 2 ) );

	RadiusRearLeft = sqrtf( powf( r -  LEFT_EDGE_TO_CENTER , 2 ) +
			                powf( REAR_EDGE_TO_CENTER , 2 ) );

	RadiusRearRight = sqrtf( powf( r + RIGHT_EDGE_TO_CENTER , 2 ) +
                             powf( REAR_EDGE_TO_CENTER , 2 ) );
}

float VehilceConfig::SteeringAngle2TurnningRadius(float steer,float a,float b)
{
	return WHEEL_BASE / tanf((a * steer + b) * PI / 180);
}

float VehilceConfig::SteeringAngle2TurnningRadiusExp(float steer,float a,float b)
{
	return steer < 0 ? -a * expf(b * -steer) : a * expf(b * steer);
}

float VehilceConfig::TurnningRadius2SteeringAngleExp(float radius,float a,float b)
{
	return radius < 0 ? -logf(-radius/a)/b : logf(radius/a)/b;
}

float VehilceConfig::TurnningRadius2SteeringAngle(float radius,float a,float b)
{
	return (atanf(WHEEL_BASE/radius) * 180 / PI - b)/a;
}

float VehilceConfig::TurnRadiusCalculate(float steering_angle)
{
	return 	steering_angle >  400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A1,FIT_RADIUS_B1) :
			steering_angle >  300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A2,FIT_RADIUS_B2) :
			steering_angle >  200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A3,FIT_RADIUS_B3) :
			steering_angle >  100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A4,FIT_RADIUS_B4) :
			steering_angle >   50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A5,FIT_RADIUS_B5) :
			steering_angle >    0 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A6,FIT_RADIUS_B6) :
			steering_angle >  -50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A7,FIT_RADIUS_B7) :
			steering_angle > -100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A8,FIT_RADIUS_B8) :
			steering_angle > -200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A9,FIT_RADIUS_B9) :
			steering_angle > -300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A10,FIT_RADIUS_B10) :
			steering_angle > -400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A11,FIT_RADIUS_B11) :
					                SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A12,FIT_RADIUS_B12);
}

float VehilceConfig::SteeringAngleCalculate(float radius)
{
	return
		   radius >   48.308 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A6,FIT_RADIUS_B6) :
		   radius >   24.018 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A5,FIT_RADIUS_B5) :
		   radius >   11.910 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A4,FIT_RADIUS_B4) :
		   radius >   7.6736 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A3,FIT_RADIUS_B3) :
		   radius >    5.463 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A2,FIT_RADIUS_B2) :
		   radius >        0 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A1,FIT_RADIUS_B1) :
		   radius > - 5.4745 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A12,FIT_RADIUS_B12) :
		   radius > - 7.7214 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A11,FIT_RADIUS_B11):
		   radius > - 11.975 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A10,FIT_RADIUS_B10):
		   radius > - 24.975 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A9,FIT_RADIUS_B9):
		   radius > - 49.975 ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A8,FIT_RADIUS_B8):
		                       TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A7,FIT_RADIUS_B7);
}

float VehilceConfig::getRadiusFrontLeft()           { return  _radius_front_left;}
void  VehilceConfig::setRadiusFrontLeft(float value){ _radius_front_left = value;}
float VehilceConfig::getRadiusFrontRight()           { return  _radius_front_right;}
void  VehilceConfig::setRadiusFrontRight(float value){ _radius_front_right = value;}
float VehilceConfig::getRadiusRearLeft()           { return  _radius_rear_left;}
void  VehilceConfig::setRadiusRearLeft(float value){ _radius_rear_left = value;}
float VehilceConfig::getRadiusRearRight()           { return  _radius_rear_right;}
void  VehilceConfig::setRadiusRearRight(float value){ _radius_rear_right = value;}

float VehilceConfig::getFrontDiagonalAxis()           { return  _front_diagonal_axis;}
void  VehilceConfig::setFrontDiagonalAxis(float value){ _front_diagonal_axis = value;}
float VehilceConfig::getFrontDiagonalAngle()           { return  _front_diagonal_angle;}
void  VehilceConfig::setFrontDiagonalAngle(float value){ _front_diagonal_angle = value;}
float VehilceConfig::getRearDiagonalAxis()           { return  _rear_diagonal_axis;}
void  VehilceConfig::setRearDiagonalAxis(float value){ _rear_diagonal_axis = value;}
float VehilceConfig::getRearDiagonalAngle()           { return  _rear_diagonal_angle;}
void  VehilceConfig::setRearDiagonalAngle(float value){ _rear_diagonal_angle = value;}

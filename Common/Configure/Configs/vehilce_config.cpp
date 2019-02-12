/*
 * vehilce_config.cpp
 *
 *  Created on: 2019年1月10日
 *      Author: zhuguohua
 */

#include <vehilce_config.h>
#include "common_configure.h"

VehilceConfig::VehilceConfig() {
	// TODO Auto-generated constructor stub
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
}

VehilceConfig::~VehilceConfig() {
	// TODO Auto-generated destructor stub
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

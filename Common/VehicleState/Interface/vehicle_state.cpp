/*
 * vehicle_state.c
 *
 *  Created on: December 29 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vehicle_state.cp                    COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: return the vehicle position ans attitude state		         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 29 2018    Initial Version                  */
/*****************************************************************************/

#include "../Interface/vehicle_state.h"

VehicleState::VehicleState() {
	// TODO Auto-generated constructor stub
	Init();
}

VehicleState::~VehicleState() {
	// TODO Auto-generated destructor stub
}

void VehicleState::Init()
{
	X.setContainer(this);
	X.getter(&VehicleState::getX);
	X.setter(&VehicleState::setX);

	Y.setContainer(this);
	Y.getter(&VehicleState::getY);
	Y.setter(&VehicleState::setY);

	Z.setContainer(this);
	Z.getter(&VehicleState::getZ);
	Z.setter(&VehicleState::setZ);

	Yaw.setContainer(this);
	Yaw.getter(&VehicleState::getYaw);
	Yaw.setter(&VehicleState::setYaw);

	LinearVelocity.setContainer(this);
	LinearVelocity.getter(&VehicleState::getLinearVelocity);
	LinearVelocity.setter(&VehicleState::setLinearVelocity);
}

float VehicleState::getX()           { return _x;}
void  VehicleState::setX(float value){_x = value;}

float VehicleState::getY()           { return _y;}
void  VehicleState::setY(float value){_y = value;}

float VehicleState::getZ()           { return _z;}
void  VehicleState::setZ(float value){_z = value;}

float VehicleState::getYaw()           { return _yaw;}
void  VehicleState::setYaw(float value){_yaw = value;}

float VehicleState::getLinearVelocity()           { return _linear_velocity;}
void  VehicleState::setLinearVelocity(float value){_linear_velocity = value;}

/*
 * vehicle_state.h
 *
 *  Created on: December 29 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vehicle_state.h                     COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: return the vehicle position ans attitude state		         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 29 2018    Initial Version                  */
/*****************************************************************************/

#ifndef VEHICLESTATE_VEHICLESTATE_H_
#define VEHICLESTATE_VEHICLESTATE_H_

#include "derivative.h"
#include "property.h"
#include "Interface/message_manager.h"

class VehicleState
{
public:
	VehicleState();
	virtual ~VehicleState();

	void Init();

	virtual void VelocityUpdate(MessageManager *msg,float dt) = 0;
	virtual void PulseUpdate(MessageManager *msg,float dt) = 0;

	float getX();
	void  setX(float value);
	Property<VehicleState,float,READ_WRITE> X;

	float getY();
	void  setY(float value);
	Property<VehicleState,float,READ_WRITE> Y;

	float getZ();
	void  setZ(float value);
	Property<VehicleState,float,READ_WRITE> Z;

	float getYaw();
	void  setYaw(float value);
	Property<VehicleState,float,READ_WRITE> Yaw;

	float getLinearVelocity();
	void  setLinearVelocity(float value);
	Property<VehicleState,float,READ_WRITE> LinearVelocity;
private:
	float _x;
	float _y;
	float _z;
	float _yaw;

	float _linear_velocity;
};

#endif /* VEHICLESTATE_VEHICLESTATE_H_ */

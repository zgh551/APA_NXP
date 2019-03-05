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
#include "math.h"
#include "vector_2d.h"
#include "chang_an_configure.h"
#include "common_configure.h"
#include "Interface/message_manager.h"
#include "vehilce_config.h"

class VehicleState
{
public:
	VehicleState();
	virtual ~VehicleState();

	void Init(void);

	virtual void VelocityUpdate(MessageManager *msg,float dt) = 0;
	virtual void PulseUpdate(MessageManager *msg) = 0;



	Vector2d getPosition();
	void     setPosition(Vector2d value);
	Property<VehicleState,Vector2d,READ_WRITE> Position;

	float getYaw();
	void  setYaw(float value);
	Property<VehicleState,float,READ_WRITE> Yaw;

	float getLinearVelocity();
	void  setLinearVelocity(float value);
	Property<VehicleState,float,READ_WRITE> LinearVelocity;

	float getLinearRate();
	void  setLinearRate(float value);
	Property<VehicleState,float,READ_WRITE> LinearRate;

protected:
	Vector2d _position;
	float    _yaw;
	float _linear_velocity;
	float _linear_rate;
};

#endif /* VEHICLESTATE_VEHICLESTATE_H_ */

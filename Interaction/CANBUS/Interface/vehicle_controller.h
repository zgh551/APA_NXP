/*
 * vehiclecontroller.h
 *
 *  Created on: December 28 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vehiclecontroller.h                 COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: vehicle control base class       					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 28 2018    Initial Version                  */
/*****************************************************************************/

#ifndef CANBUS_INTERFACE_VEHICLECONTROLLER_H_
#define CANBUS_INTERFACE_VEHICLECONTROLLER_H_

#include "derivative.h"

typedef enum _GearPosition
{
	Praking = 1,
	Reverse,
	Neutral,
	Drive
}GearPosition;

typedef struct ControlCommand
{
	GearPosition Gear;
	float Torque;
	float Acc;
	float Aeb;
	float SteeringAngle;
	float SteeringAngleRate;
}ControlCommand;


class VehicleController {
public:
	VehicleController();
	virtual ~VehicleController();

	/**
	 * @brief initialize the vehicle controller.
	 * @return none
	 */
	virtual void Init() = 0;
	/**
	 * @brief start the vehicle controller.
	 * @return true if successfully started.
	 */
	virtual void Start() = 0;

	/**
	 * @brief stop the vehicle controller.
	 */
	virtual void Stop() = 0;
	  /**
	   * @brief update the vehicle controller.
	   * @param command the control command
	   * @return error_code
	   */
	virtual void Update(ControlCommand cmd) = 0;

private:
	/*
	* @brief NEUTRAL, REVERSE, DRIVE
	*/
//	virtual void Gear() = 0;
	/*
	* @brief detail function for auto driving brake with new acceleration
	* acceleration:0.00~99.99, unit:%
	*/
//	virtual void Brake(float acceleration) = 0;

	/*
	* @brief drive with old acceleration gas:0.00~99.99 unit:%
	*/
//	virtual void Throttle(float throttle) = 0;

	/*
	* @brief steering with new angle speed angle:-99.99~0.00~99.99, unit:%,
	* left:+, right:- angle_spd:0.00~99.99, unit:deg/s
	*/
//	virtual void Steer(float angle, float angle_spd) = 0;
};

#endif /* CANBUS_INTERFACE_VEHICLECONTROLLER_H_ */

/*
 * vehicle_controller.h
 *
 *  Created on: December 28 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vehicle_controller.h                COPYRIGHT (c) Motovis 2018 */
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
#include "property.h"

typedef enum _GearPosition
{
	Praking = 1,
	Reverse,
	Neutral,
	Drive
}GearPosition;

typedef struct ControlCommand
{
	union{
		struct
		{
			uint8_t GearEnable 			: 1;
			uint8_t SteeringEnable 		: 1;
			uint8_t AccelerationEnable 	: 1;
			uint8_t DecelerationEnable 	: 1;
			uint8_t TorqueEnable 		: 1;
			uint8_t VelocityEnable 		: 1;
			uint8_t Invalid 			: 2;
		}B;
		uint8_t R;
	}ControlEnable;
	GearPosition Gear;
	float SteeringAngle;
	float SteeringAngleRate;
	float Acceleration;
	float Deceleration;
	float Velocity;
	float Torque;
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


	/* ACC */
	float getAcceleration();
	void  setAcceleration(float value);
	Property<VehicleController,float,READ_WRITE> Acceleration;

	uint8_t getAccelerationEnable();
	void    setAccelerationEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> AccelerationEnable;

	/* AEB */
	float getDeceleration();
	void  setDeceleration(float value);
	Property<VehicleController,float,READ_WRITE> Deceleration;

	uint8_t getDecelerationEnable();
	void    setDecelerationEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> DecelerationEnable;

	/* Torque */
	float getTorque();
	void  setTorque(float value);
	Property<VehicleController,float,READ_WRITE> Torque;

	uint8_t getTorqueEnable();
	void    setTorqueEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> TorqueEnable;

	/* Velocity */
	float getVelocity();
	void  setVelocity(float value);
	Property<VehicleController,float,READ_WRITE> Velocity;

	uint8_t getVelocityEnable();
	void    setVelocityEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> VelocityEnable;

	/* Steering Angle */
	float getSteeringAngle();
	void  setSteeringAngle(float value);
	Property<VehicleController,float,READ_WRITE> SteeringAngle;

	float getSteeringAngleRate();
	void  setSteeringAngleRate(float value);
	Property<VehicleController,float,READ_WRITE> SteeringAngleRate;

	uint8_t getSteeringEnable();
	void    setSteeringEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> SteeringEnable;

	/* Gear */
	uint8_t getGear();
	void    setGear(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> Gear;

	uint8_t getGearEnable();
	void    setGearEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> GearEnable;
private:
	/* ACC */
	float _acceleration;
	uint8_t _acceleration_enable;

	/* AEB */
	float _deceleration;
	uint8_t _deceleration_enable;

	/* Torque */
	float _torque;
	uint8_t _torque_enable;

	/* Velocity */
	float _velocity;
	uint8_t _velocity_enable;

	/* Steering System */
	float _steering_angle;
	float _steering_angle_rate;
	uint8_t _steering_enable;

	/* Gear */
	uint8_t _gear;
	uint8_t _gear_enable;
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

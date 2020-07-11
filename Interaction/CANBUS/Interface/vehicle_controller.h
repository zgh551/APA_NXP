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

#include "../../../Driver/System/derivative.h"
#include "../../../Common/Utils/Inc/property.h"
#include "message_manager.h"

typedef struct _ControlCommand
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
	GearStatus Gear;
	float SteeringAngle;
	float SteeringAngleRate;
	float Acceleration;
	float Deceleration;
	float Velocity;
	float Torque;
}ControlCommand;

typedef struct _APAControlCommand
{
	union{
		struct
		{
			uint8_t Invalid   : 6;
			uint8_t APAEnable : 2;
		}B;
		uint8_t R;
	}ControlEnable;
	GearStatus Gear;
	float SteeringAngle;
	float SteeringAngleRate;
	float Velocity;
	float Distance;
}APAControlCommand;

typedef enum _ActiveStatus
{
	Inactive = 0,
	Active
}ActiveStatus;

typedef enum _HandeBrakeRequestStatus
{
	ReleaseRequest = 0,
	AppliedRequest
}HandeBrakeRequestStatus;

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

	virtual void Update(APAControlCommand cmd) = 0;

	// control state machine
	virtual void WorkStateMachine(MessageManager& msg) = 0;

	// push the command to the vehicle
	virtual void DataPush(void) = 0;

	/* ACC */
	float getTargetAcceleration();
	void  setTargetAcceleration(float value);
	Property<VehicleController,float,READ_WRITE> TargetAcceleration;

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

	uint8_t getDecelerationReq(void)			{ return _deceleration_req; }
	void    setDecelerationReq(uint8_t value) 	{ _deceleration_req = value;}

	float getBrakeDegree(void) 			{ return _brake_degree; }
	void  setBrakeDegree(float value)	{ _brake_degree = value;}

	/* Torque */
	float getTorque();
	void  setTorque(float value);
	Property<VehicleController,float,READ_WRITE> Torque;

	uint8_t getTorqueEnable();
	void    setTorqueEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> TorqueEnable;

	/* Turnning Torque Control Single */
	float getTurnTorqueVal();
	void  setTurnTorqueVal(float value);
	Property<VehicleController,float,READ_WRITE> TurnTorqueVal;

	uint8_t getTurnTorqueDir();
	void    setTurnTorqueDir(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> TurnTorqueDir;

	uint8_t getTurnTorqueAct();
	void    setTurnTorqueAct(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> TurnTorqueAct;

	/* Velocity */
	float getVelocity();
	void  setVelocity(float value);
	Property<VehicleController,float,READ_WRITE> Velocity;

	float getDistance();
	void  setDistance(float value);
	Property<VehicleController,float,READ_WRITE> Distance;

	float getDistanceSet();
	void  setDistanceSet(float value);

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

	float getSteeringAngleSet();
	void  setSteeringAngleSet(float value);
	Property<VehicleController,float,READ_WRITE> SteeringAngleSet;

	uint8_t getSteeringEnable();
	void    setSteeringEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> SteeringEnable;

	/* Gear */
	GearStatus getGear();
	void       setGear(GearStatus value);
	Property<VehicleController,GearStatus,READ_WRITE> Gear;

	uint8_t getGearEnable();
	void    setGearEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> GearEnable;

	uint8_t getAPAEnable();
	void    setAPAEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> APAEnable;

	uint8_t getEPBEnable();
	void    setEPBEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> EPBEnable;

	uint16_t getShakeHandsCnt() 			  { return _shake_hands_cnt ; }
	void     setShakeHandsCnt(uint16_t value) { _shake_hands_cnt = value; }

	HandeBrakeRequestStatus getEpbReq() 								{ return _epb_req; }
	void                    setEpbReq(HandeBrakeRequestStatus value) 	{ _epb_req = value;}

	ActiveStatus getTurnLightLeftReq() 						{ return _turn_light_left_req; }
	void         setTurnLightLeftReq(ActiveStatus value) 	{ _turn_light_left_req = value;}

	ActiveStatus getTurnLightRightReq() 					{ return _turn_light_right_req; }
	void         setTurnLightRightReq(ActiveStatus value) 	{ _turn_light_right_req = value;}
private:
	/* APA */
	uint8_t _apa_enable;

	/* ACC */
	float _target_acceleration;
	float _acceleration;
	uint8_t _acceleration_enable;

	/* AEB */
	float _deceleration;
	uint8_t _deceleration_req;

	/* Brake */
	float _brake_degree;

	/* Torque */
	float _torque;
	uint8_t _torque_enable;

	/* Turnning Torque Control Single */
	float   _turn_torque_val;
	uint8_t _turn_torque_dir;
	uint8_t _turn_torque_act;

	/* Velocity */
	float _velocity;
	float _distance;
	float _distance_set;
	uint8_t _velocity_enable;

	/* Steering System */
	float _steering_angle;
	float _steering_angle_rate;
	float _steering_angle_set;
	uint8_t _steering_enable;

	/* Gear */
	GearStatus _gear;
	uint8_t _gear_enable;

	/* EPB */
	uint8_t _epb_enable;
	HandeBrakeRequestStatus _epb_req;
	// ICM
	ActiveStatus _turn_light_left_req;
	ActiveStatus _turn_light_right_req;

	// shake hands
	uint16_t _shake_hands_cnt;
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

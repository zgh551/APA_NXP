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

typedef enum _HandeBrakeRequestStatus
{
	ReleaseRequest = 0,
	AppliedRequest
}HandeBrakeRequestStatus;

typedef enum _EPS_ControlState
{
	EPS_WaitRequest = 0,
	EPS_WaitEnable,
	EPS_Active,
	EPS_WaitDisable,
	EPS_ResumableInterrupt,
	EPS_UnresumableInterrupt,
	EPS_Err
}EPS_ControlState;

typedef enum _VCU_ControlState
{
	VCU_WaitRequest = 0,
	VCU_WaitActive,
	VCU_Active,
	VCU_Arrival,
	VCU_ParkingArrival,
	VCU_Err
}VCU_ControlState;

typedef enum _ESC_ControlState
{
	ESC_WaitRequest = 0,
	ESC_WaitBrake,
	ESC_WaitActive,
	ESC_Active,
	ESC_WaitAlodActive,
	ESC_AlodActive,
	ESC_CloseRequest,
	ESC_EPB_Inactive,
	ESC_Err
}ESC_ControlState;

typedef enum _ControlMode
{
	ControlNormal = 0,
	ControlBrake
}ControlMode;

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

	/* APA */
	uint8_t getAPAEnable();
	void    setAPAEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> APAEnable;

	ControlMode getControlMode()                  { return  _control_mode; }
	void        setControlMode(ControlMode value) { _control_mode = value; }

	/* Lon ACC */
	float getTargetAcceleration();
	void  setTargetAcceleration(float value);
	Property<VehicleController,float,READ_WRITE> TargetAcceleration;

	float getAcceleration();
	void  setAcceleration(float value);
	Property<VehicleController,float,READ_WRITE> Acceleration;

	uint8_t getAccelerationEnable();
	void    setAccelerationEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> AccelerationEnable;

	/* Lon AEB */
	float getDeceleration();
	void  setDeceleration(float value);
	Property<VehicleController,float,READ_WRITE> Deceleration;

	uint8_t getDecelerationReq(void)			{ return _deceleration_req; }
	void    setDecelerationReq(uint8_t value) 	{ _deceleration_req = value;}

	float getBrakeDegree(void) 			{ return _brake_degree; }
	void  setBrakeDegree(float value)	{ _brake_degree = value;}

	/* Lon Torque */
	float getTorque();
	void  setTorque(float value);
	Property<VehicleController,float,READ_WRITE> Torque;

	uint8_t getTorqueEnable();
	void    setTorqueEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> TorqueEnable;

	/* ESC */
	float getJerkMin() 			  { return  _jerk_min; }
	void  setJerkMin(float value) { _jerk_min = value; }

	float getJerkMax() 			  { return  _jerk_max; }
	void  setJerkMax(float value) { _jerk_max = value; }

	float getAccLowerLimit() 			{ return  _acc_lower_limit; }
	void  setAccLowerLimit(float value) { _acc_lower_limit = value; }

	float getAccUpperLimit() 			{ return  _acc_upper_limit; }
	void  setAccUpperLimit(float value) { _acc_upper_limit = value; }

	float getVelocity();
	void  setVelocity(float value);
	Property<VehicleController,float,READ_WRITE> Velocity;

	float getDistance();
	void  setDistance(float value);
	Property<VehicleController,float,READ_WRITE> Distance;

	float getRemainMoveDistance(void)        { return  _remain_move_distance; }
	void  setRemainMoveDistance(float value) { _remain_move_distance = value; }

	uint8_t getVelocityEnable();
	void    setVelocityEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> VelocityEnable;

	/* Lat EPS */
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
	GearStatus getGear();
	void       setGear(GearStatus value);
	Property<VehicleController,GearStatus,READ_WRITE> Gear;

	/* EPB */
	uint8_t getEPBEnable();
	void    setEPBEnable(uint8_t value);
	Property<VehicleController,uint8_t,READ_WRITE> EPBEnable;

	HandeBrakeRequestStatus getEpbReq() 								{ return _epb_req; }
	void                    setEpbReq(HandeBrakeRequestStatus value) 	{ _epb_req = value;}

	/* BCM */
	ActiveStatus getTurnLightLeftReq() 						{ return _turn_light_left_req; }
	void         setTurnLightLeftReq(ActiveStatus value) 	{ _turn_light_left_req = value;}

	ActiveStatus getTurnLightRightReq() 					{ return _turn_light_right_req; }
	void         setTurnLightRightReq(ActiveStatus value) 	{ _turn_light_right_req = value;}

	/* Other */
	uint16_t getShakeHandsCnt() 			  { return _shake_hands_cnt ; }
	void     setShakeHandsCnt(uint16_t value) { _shake_hands_cnt = value; }
protected:
	// steering angle rate control
	/*
	 * @brief direct control
	 */
	void SteeringAngleControl(void);
	/*
	 * @brief the target steering angle add at every step
	 */
	void SteeringAngleControl(float dt);
	/*
	 * @brief base on the actual steering angle, control the steering angle
	 */
	void SteeringAngleControl(float dt, float actual_steering);

	// EPS
	uint8_t eps_request_;
	int16_t steering_angle_request_;
	float steering_angle_valid_request_; // eps can request max steer angle

	// VCU
	uint8_t 	gear_enable_;
	GearStatus  gear_request_;

	// ESC
	float distance_set_;

	uint8_t stop_distance_;
	uint8_t esc_prefill_request_;
	uint8_t driver_off_request_;
	uint8_t brake_preferred_request_;

	uint8_t vlc_shutdown_request_;
	uint8_t standstill_request_;
	uint8_t emergency_brake_request_;

	// lower speed control single
	uint8_t vlc_mode_request_;
	uint8_t esc_target_acceleration_request_;

	uint8_t esc_acc_lower_limit_;
	uint8_t esc_acc_upper_limit_;

	uint8_t esc_jerk_min_;
	uint8_t esc_jerk_max_;

	// high speed control single
	uint8_t alod_mode_request_;
	uint8_t alod_target_acceleration_request_;

	uint8_t alod_acc_lower_limit_;
	uint8_t alod_acc_upper_limit_;

	uint8_t alod_jerk_min_;
	uint8_t alod_jerk_max_;

	uint8_t rolling_counter_;
private:
	/* APA */
	uint8_t _apa_enable;
	ControlMode _control_mode;

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

	/* ESC */
	// the jerk of target acceleration
	float _jerk_min;
	float _jerk_max;

	float _acc_lower_limit;
	float _acc_upper_limit;

	float _velocity;
	float _distance;
	float _remain_move_distance;
	uint8_t _velocity_enable;

	/* EPS */
	/* the source Turn Torque Control Single */
	float   _turn_torque_val;
	uint8_t _turn_torque_dir;
	uint8_t _turn_torque_act;

	// the steer angle control
	float _steering_angle;
	float _steering_angle_rate;
	uint8_t _steering_enable;

	/* Gear TCU */
	GearStatus _gear;

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

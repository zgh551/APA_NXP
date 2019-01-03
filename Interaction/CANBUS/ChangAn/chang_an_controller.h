/*
 * chang_an_controller.h
 *
 *  Created on: December 28 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: chang_an_controller.h               COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: chang an vehicle control class     					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 28 2018    Initial Version                  */
/*****************************************************************************/

#ifndef CANBUS_CHANGAN_CHANGANCONTROLLER_H_
#define CANBUS_CHANGAN_CHANGANCONTROLLER_H_

#include "derivative.h"
#include "property.h"
#include "Interface/vehicle_controller.h"

typedef enum _SteeringAngleActiveControl
{
	WaitSteeringAngleControlSingleState = 0,
	WaitFeedbackSingleState,
	WaitExistState
}SteeringAngleActiveControl;


class ChangAnController : public VehicleController
{
public:
	ChangAnController();
	virtual ~ChangAnController();
	/**
	 * @brief initialize the vehicle controller.
	 * @return none
	 */
	void Init() override;
	/**
	 * @brief start the vehicle controller.
	 * @return true if successfully started.
	 */
	void Start() override;

	/**
	 * @brief stop the vehicle controller.
	 */
	void Stop() override;
	  /**
	   * @brief update the vehicle controller.
	   * @param command the control command
	   * @return error_code
	   */
	void Update(ControlCommand cmd) override;

	// push the command to the vehicle
	void Push(float dt);

	/*** Function ***/
	// Vehicle control command function
	void VehicleContorlStep1();
	void VehicleContorlStep2();
	void VehicleContorlStep3();
	void VehicleContorl();

	// Steeing angle control base on the angle speed
	void SteeringAngleControl(float dt);

	// Steering Angle control state machine
	void SteeringAngleControlStateMachine(uint8_t fd);

private:
	SteeringAngleActiveControl _steerig_angle_active_control_state;
	/*** Send to Vehicle Messege ***/
	/* Roolling Counter */
	uint8_t _rolling_counter_torque_AEB;
	uint8_t _rolling_counter_brake_ACC;
	uint8_t _rolling_counter_steering_control;
	uint8_t _rolling_counter_gear_control;

	/* ACC */
	// actual value
	float _target_acceleration_acc;
	uint8_t _target_acceleration_enable;
	// current value
	uint8_t _current_target_acceleration_ACC;
	uint8_t _current_target_acceleration_enable_single;

	/* AEB */
	// actual value
	float _target_deceleration_aeb;
	uint8_t _target_deceleration_enable;
	// current value
	uint16_t _current_target_deceleration_AEB;
	uint8_t _current_target_deceleration_enable_single;

	/* Torque */
	// actual value
	float _torque;
	uint8_t _torque_enable;
	// current value
	uint16_t _current_torque;
	uint8_t _current_torque_enable_single;

	/* SteeringAngle */
	// actual value
	float _steering_angle_set;
	float _steering_angle_target;
	float _steering_angle_rate_target;
	uint8_t _steering_angle_target_active;
	// current value
	int16_t _current_steering_angle_target;
	uint8_t _current_steering_angle_target_active_single;

	/* Gear */
	// actual value
	uint8_t _gear;
	uint8_t _gear_enable;
	uint8_t _gear_valid;
	// current value
	uint8_t _current_gear;
	uint8_t _current_gear_enable_single;
	uint8_t _current_gear_valid_single;
};

#endif /* CANBUS_CHANGAN_CHANGANCONTROLLER_H_ */

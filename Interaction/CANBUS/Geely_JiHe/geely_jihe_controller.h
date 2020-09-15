/*
 * bo_rui_controller.h
 *
 *  Created on: 2019骞�3鏈�15鏃�
 *      Author: zhuguohua
 */

#ifndef CANBUS_GEELY_JIHE_CONTROLLER_H_
#define CANBUS_GEELY_JIHE_CONTROLLER_H_

#include "../Interface/vehicle_controller.h"

typedef enum _EpsControlState
{
	EpsWaitRequest = 0,
	EpsWiatEnable,
	EpsWaitDisable
}EpsControlState;

class GeelyJiHeController  : public VehicleController
{
public:
	GeelyJiHeController();
	virtual ~GeelyJiHeController();

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
	void Update(APAControlCommand cmd) override;

	void VehicleContorl();
	void VehicleContorlPri();
	// Steeing angle control base on the angle speed
	void SteeringAngleControl(float dt);
	void SteeringAngleControl(float dt,float actual_steering);

	// push the command to the vehicle
	void Push(float dt);
	void Push(float dt,float actual_steering);

	void WorkStateMachine(MessageManager& msg) override;

	// push the command to the vehicle
	void DataPush(void) override;

private:
	EpsControlState _eps_control_state;
	/* SteeringAngle */
	// actual value

	// current value
	int16_t _current_steering_angle_target;

	uint16_t _current_distance;

	uint16_t _current_turn_torque_value;

	int8_t _current_acceleration;
};

#endif /* CANBUS_BORUI_BO_RUI_CONTROLLER_H_ */
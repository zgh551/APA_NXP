/*
 * bo_rui_controller.h
 *
 *  Created on: 2019年3月15日
 *      Author: zhuguohua
 */

#ifndef CANBUS_BORUI_BO_RUI_CONTROLLER_H_
#define CANBUS_BORUI_BO_RUI_CONTROLLER_H_

#include "derivative.h"
#include "property.h"
#include "Interface/vehicle_controller.h"

class BoRuiController  : public VehicleController
{
public:
	BoRuiController();
	virtual ~BoRuiController();

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

	// push the command to the vehicle
	void Push(float dt);

private:
	/* SteeringAngle */
	// actual value
	float _steering_angle_set;
	// current value
	int16_t _current_steering_angle_target;

	uint16_t _current_distance;

	uint16_t _current_turn_torque_value;

	int8_t _current_acceleration;
};

#endif /* CANBUS_BORUI_BO_RUI_CONTROLLER_H_ */

/*
 * bo_rui_controller.h
 *
 *  Created on: 2019骞�3鏈�15鏃�
 *      Author: zhuguohua
 */

#ifndef CANBUS_GEELY_JIHE_CONTROLLER_H_
#define CANBUS_GEELY_JIHE_CONTROLLER_H_

#include "../Interface/vehicle_controller.h"
#include "../../../Common/Math/crc_compute.h"

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

	// the control state machine
	void WorkStateMachine(MessageManager& msg) override;

	// push the command to the vehicle
	void DataPush(void) override;
protected:
	// CAN data send
	void VehicleControl(void);

	void EPS_StateMachine(MessageManager& msg);
	void VCU_StateMachine(MessageManager& msg);
	void ESC_StateMachine(MessageManager& msg);
private:
	// EPS State
	EPS_ControlState _eps_control_state;
	uint8_t _eps_timeout;

	// VCU State
	VCU_ControlState _vcu_control_state;
	uint8_t _vcu_timeout;

	// ESC State
	ESC_ControlState _esc_control_state;
	uint8_t _esc_timeout;
};

#endif /* CANBUS_BORUI_BO_RUI_CONTROLLER_H_ */

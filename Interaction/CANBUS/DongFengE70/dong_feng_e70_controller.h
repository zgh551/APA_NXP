/*
 * dong_feng_e70_controller.h
 *
 *  Created on: 2019年6月20日
 *      Author: zhuguohua
 */

#ifndef CANBUS_DONGFENGE70_DONG_FENG_E70_CONTROLLER_H_
#define CANBUS_DONGFENGE70_DONG_FENG_E70_CONTROLLER_H_

#include "derivative.h"
#include "property.h"
#include "Interface/vehicle_controller.h"

typedef enum _APA_TorqueControlState
{
	TorqueWaitEnable = 0,
	TorqueWaitActive,
	TorqueWaitDisable
}APA_TorqueControlState;

typedef enum APA_TurnControlState
{
	TurnWaitHand = 0,
	TurnWiatActive,
	TurnWaitDisable
}APA_TurnControlState;

class DongFengE70Controller  : public VehicleController
{
public:
	DongFengE70Controller();
	virtual ~DongFengE70Controller();
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

	void VehicleContorl_20ms(void);
	void VehicleContorl_10ms(void);
	void APA_ControlStateMachine(uint8_t apa_ctl_sts,uint8_t esp_availab_sts);

	// push the command to the vehicle
	void Push(float dt);
private:
	APA_TorqueControlState _apa_torque_control_state;
	APA_TurnControlState   _apa_turn_control_state;
	/*
	 * VCU
	 * */
	uint8_t _apa_work_status;
	uint8_t _apa_vcu_control;
	uint16_t _apa_max_troque;
	uint16_t _apa_troque;
	uint8_t _apa_epb_request;
	//rolling counter
	uint8_t _apa_rolling_counter;

	/*
	 * ESP
	 * */
	uint8_t _apa_auto_drive_enable;
	int16_t _apa_angle_wheel;
	uint8_t _apa_anglular_velocity;
	uint8_t _apa_rolling_count_had1;

	/*
	 * ESC
	 * */
	uint8_t _apa_esc_prefill_req;
	uint8_t _apa_esc_manuver_state;
	uint8_t _apa_esc_brake_req_valid_flag;
	uint8_t _apa_esc_control_req;

	uint16_t _apa_esc_brake_req;
	uint8_t _apa_esc_rolling_count;
};

#endif /* CANBUS_DONGFENGE70_DONG_FENG_E70_CONTROLLER_H_ */

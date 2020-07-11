/*
 * chery_s51ev_controller.h
 *
 *  Created on: 2020年6月29日
 *      Author: zhuguohua
 */

#ifndef CANBUS_CHERYS51EV_CHERY_S51EV_CONTROLLER_H_
#define CANBUS_CHERYS51EV_CHERY_S51EV_CONTROLLER_H_

#include "../Interface/vehicle_controller.h"
#include "../Interface/message_manager.h"

typedef enum _Chery_ControlState
{
	CheryReady = 0,
	CheryAutoDriverActive,
	CheryGearConfigure,
	CheryExceptionHandling,
	CheryAbnormalExit
}Chery_ControlState;

class CheryS51EV_Controller : public VehicleController
{
public:
	CheryS51EV_Controller();
	virtual ~CheryS51EV_Controller();
	
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

	// Steeing angle control base on the angle speed
	void SteeringAngleControl(float dt);

	void WorkStateMachine(MessageManager& msg) override;

	// push the command to the vehicle
	void DataPush(void) override;
private:
	Chery_ControlState _control_state;
	uint8_t push_time_cnt;
};

#endif /* CANBUS_CHERYS51EV_CHERY_S51EV_CONTROLLER_H_ */

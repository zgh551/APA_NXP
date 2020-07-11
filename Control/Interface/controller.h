/*
 * controller.h
 *
 *  Created on: 2019骞�1鏈�3鏃�
 *      Author: zhuguohua
 */

#ifndef INTERFACE_CONTROLLER_H_
#define INTERFACE_CONTROLLER_H_

#include "../../Driver/System/derivative.h"
#include "../../Common/Utils/Inc/property.h"
#include "../../Common/Configure/Configs/vehicle_config.h"

#include "../../Interaction/CANBUS/Interface/vehicle_controller.h"
#include "../../Interaction/CANBUS/Interface/message_manager.h"

#include "../Common/pid.h"

class Controller {
public:
	Controller();
	virtual ~Controller();

	virtual void Init() = 0;

	virtual void Proc(MessageManager *msg,VehicleController *ctl,PID *pid) = 0;
};

#endif /* INTERFACE_CONTROLLER_H_ */

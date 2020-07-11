/*
 * abnormal_process.h
 *
 *  Created on: 2020骞�6鏈�2鏃�
 *      Author: zhuguohua
 */

#ifndef ABNORMAL_PROCESS_H_
#define ABNORMAL_PROCESS_H_

#include "math.h"
#include "../../Driver/System/derivative.h"
#include "../../Common/Utils/Inc/property.h"
#include "../Common/pid.h"
#include "../../Common/Configure/Configs/vehicle_config.h"
#include "../../Interaction/CANBUS/Interface/vehicle_controller.h"
#include "../../Interaction/CANBUS/Interface/message_manager.h"

class AbnormalProcess {
public:
	AbnormalProcess();
	virtual ~AbnormalProcess();

	void work(MessageManager *msg,VehicleController *ctl);

private:
	uint16_t _err_velocity_cnt;
};

#endif /* ABNORMAL_PROCESS_H_ */

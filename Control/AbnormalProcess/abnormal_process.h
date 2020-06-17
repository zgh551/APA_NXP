/*
 * abnormal_process.h
 *
 *  Created on: 2020年6月2日
 *      Author: zhuguohua
 */

#ifndef ABNORMAL_PROCESS_H_
#define ABNORMAL_PROCESS_H_

#include "derivative.h"
#include "property.h"
#include "controller.h"
#include "pid.h"
#include "math.h"
#include "vehicle_config.h"
#include "interpolation.h"

class AbnormalProcess {
public:
	AbnormalProcess();
	virtual ~AbnormalProcess();

	void work(MessageManager *msg,VehicleController *ctl);

private:
	uint16_t _err_velocity_cnt;
};

#endif /* ABNORMAL_PROCESS_H_ */

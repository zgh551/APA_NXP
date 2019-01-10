/*
 * parallel_parking_path_planning.h
 *
 *  Created on: 2019Äê1ÔÂ9ÈÕ
 *      Author: zhuguohua
 */
#include "derivative.h"
#include "property.h"
#include "chang_an_configure.h"
#include "path_planning.h"
#include <vehilce_config.h>

#ifndef PARALLELPARKING_PARALLEL_PARKING_PATH_PLANNING_H_
#define PARALLELPARKING_PARALLEL_PARKING_PATH_PLANNING_H_

class ParallelParkingPathPlanning : public PathPlannig
{
public:
	ParallelParkingPathPlanning();
	virtual ~ParallelParkingPathPlanning();

	void Init() override;
	void Work() override;
};

#endif /* PARALLELPARKING_PARALLEL_PARKING_PATH_PLANNING_H_ */

/*
 * parallel_parking_path_planning.cpp
 *
 *  Created on: 2019Äê1ÔÂ9ÈÕ
 *      Author: zhuguohua
 */

#include <parallel_parking_path_planning.h>
#include "math.h"

//VehilceConfig m_VehilceConfig;

ParallelParkingPathPlanning::ParallelParkingPathPlanning() {
	// TODO Auto-generated constructor stub

}

ParallelParkingPathPlanning::~ParallelParkingPathPlanning() {
	// TODO Auto-generated destructor stub
}


void ParallelParkingPathPlanning::Init()
{
	VehilceConfig *m_VehilceConfig = new VehilceConfig();
	m_VehilceConfig->EdgeRadiusUpdate(1,MIN_TURN_RADIUS);
	MinParkingLength = REAR_EDGE_TO_CENTER + sqrtf(powf(m_VehilceConfig->RadiusFrontRight,2) - powf(MIN_TURN_RADIUS - LEFT_EDGE_TO_CENTER,2));
	MinParkingWidth  = LEFT_EDGE_TO_CENTER + m_VehilceConfig->RadiusRearRight - MIN_TURN_RADIUS;
	delete m_VehilceConfig;
}

void ParallelParkingPathPlanning::Work()
{

}

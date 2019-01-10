/*
 * path_plannig.cpp
 *
 *  Created on: 2019Äê1ÔÂ9ÈÕ
 *      Author: zhuguohua
 */

#include <path_planning.h>

PathPlannig::PathPlannig() {
	// TODO Auto-generated constructor stub
	MinParkingLength.setContainer(this);
	MinParkingLength.getter(&PathPlannig::getMinParkingLength);
	MinParkingLength.setter(&PathPlannig::setMinParkingLength);

	MinParkingWidth.setContainer(this);
	MinParkingWidth.getter(&PathPlannig::getMinParkingWidth);
	MinParkingWidth.setter(&PathPlannig::setMinParkingWidth);
}

PathPlannig::~PathPlannig() {
	// TODO Auto-generated destructor stub
}

float PathPlannig::getMinParkingLength()           { return  _min_parking_length;}
void  PathPlannig::setMinParkingLength(float value){ _min_parking_length = value;}

float PathPlannig::getMinParkingWidth()           { return  _min_parking_width;}
void  PathPlannig::setMinParkingWidth(float value){ _min_parking_width = value;}


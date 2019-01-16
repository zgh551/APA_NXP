/*
 * path_plannig.cpp
 *
 *  Created on: 2019Äê1ÔÂ9ÈÕ
 *      Author: zhuguohua
 */

#include <planning.h>

Planning::Planning() {
	// TODO Auto-generated constructor stub
	MinParkingLength.setContainer(this);
	MinParkingLength.getter(&Planning::getMinParkingLength);
	MinParkingLength.setter(&Planning::setMinParkingLength);

	MinParkingWidth.setContainer(this);
	MinParkingWidth.getter(&Planning::getMinParkingWidth);
	MinParkingWidth.setter(&Planning::setMinParkingWidth);
}

Planning::~Planning() {
	// TODO Auto-generated destructor stub
}

float Planning::getMinParkingLength()           { return  _min_parking_length;}
void  Planning::setMinParkingLength(float value){ _min_parking_length = value;}

float Planning::getMinParkingWidth()           { return  _min_parking_width;}
void  Planning::setMinParkingWidth(float value){ _min_parking_width = value;}


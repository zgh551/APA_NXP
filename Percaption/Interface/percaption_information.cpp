/*
 * percaption_information.cpp
 *
 *  Created on: 2019Äê1ÔÂ9ÈÕ
 *      Author: zhuguohua
 */

#include <percaption_information.h>

PercaptionInformation::PercaptionInformation() {
	// TODO Auto-generated constructor stub
	PositionX.setContainer(this);
	PositionX.getter(&PercaptionInformation::getPositionX);
	PositionX.setter(&PercaptionInformation::setPositionX);

	PositionY.setContainer(this);
	PositionY.getter(&PercaptionInformation::getPositionY);
	PositionY.setter(&PercaptionInformation::setPositionY);

	AttitudeYaw.setContainer(this);
	AttitudeYaw.getter(&PercaptionInformation::getAttitudeYaw);
	AttitudeYaw.setter(&PercaptionInformation::setAttitudeYaw);

	ParkingLength.setContainer(this);
	ParkingLength.getter(&PercaptionInformation::getParkingLength);
	ParkingLength.setter(&PercaptionInformation::setParkingLength);

	ParkingWidth.setContainer(this);
	ParkingWidth.getter(&PercaptionInformation::getParkingWidth);
	ParkingWidth.setter(&PercaptionInformation::setParkingWidth);

	DetectParkingStatus.setContainer(this);
	DetectParkingStatus.getter(&PercaptionInformation::getDetectParkingStatus);
	DetectParkingStatus.setter(&PercaptionInformation::setDetectParkingStatus);
}

PercaptionInformation::~PercaptionInformation() {
	// TODO Auto-generated destructor stub
}

float PercaptionInformation::getPositionX()           { return  _position_x;}
void  PercaptionInformation::setPositionX(float value){ _position_x = value;}

float PercaptionInformation::getPositionY()           { return  _position_y;}
void  PercaptionInformation::setPositionY(float value){ _position_y = value;}

float PercaptionInformation::getAttitudeYaw()           { return  _attitude_yaw;}
void  PercaptionInformation::setAttitudeYaw(float value){ _attitude_yaw = value;}

float PercaptionInformation::getParkingLength()           { return  _parking_length;}
void  PercaptionInformation::setParkingLength(float value){ _parking_length = value;}

float PercaptionInformation::getParkingWidth()           { return  _parking_width;}
void  PercaptionInformation::setParkingWidth(float value){ _parking_width = value;}


bool PercaptionInformation::getDetectParkingStatus()           { return  _detect_parking_status;}
void PercaptionInformation::setDetectParkingStatus(bool value) { _detect_parking_status = value;}


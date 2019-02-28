/*
 * percaption_information.cpp
 *
 *  Created on: 2019年1月9日
 *      Author: zhuguohua
 */

#include <percaption.h>

Percaption::Percaption() {
	PositionX.setContainer(this);
	PositionX.getter(&Percaption::getPositionX);
	PositionX.setter(&Percaption::setPositionX);

	PositionY.setContainer(this);
	PositionY.getter(&Percaption::getPositionY);
	PositionY.setter(&Percaption::setPositionY);

	AttitudeYaw.setContainer(this);
	AttitudeYaw.getter(&Percaption::getAttitudeYaw);
	AttitudeYaw.setter(&Percaption::setAttitudeYaw);

	ParkingLength.setContainer(this);
	ParkingLength.getter(&Percaption::getParkingLength);
	ParkingLength.setter(&Percaption::setParkingLength);

	ParkingWidth.setContainer(this);
	ParkingWidth.getter(&Percaption::getParkingWidth);
	ParkingWidth.setter(&Percaption::setParkingWidth);

	DetectParkingStatus.setContainer(this);
	DetectParkingStatus.getter(&Percaption::getDetectParkingStatus);
	DetectParkingStatus.setter(&Percaption::setDetectParkingStatus);
}

Percaption::~Percaption() {

}

float Percaption::getPositionX()           { return  _position_x;}
void  Percaption::setPositionX(float value){ _position_x = value;}

float Percaption::getPositionY()           { return  _position_y;}
void  Percaption::setPositionY(float value){ _position_y = value;}

float Percaption::getAttitudeYaw()           { return  _attitude_yaw;}
void  Percaption::setAttitudeYaw(float value){ _attitude_yaw = value;}

float Percaption::getParkingLength()           { return  _parking_length;}
void  Percaption::setParkingLength(float value){ _parking_length = value;}

float Percaption::getParkingWidth()           { return  _parking_width;}
void  Percaption::setParkingWidth(float value){ _parking_width = value;}

bool Percaption::getDetectParkingStatus()           { return  _detect_parking_status;}
void Percaption::setDetectParkingStatus(bool value) { _detect_parking_status = value;}


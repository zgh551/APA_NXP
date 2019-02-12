/*
 * percaption_information.h
 *
 *  Created on: 2019年1月9日
 *      Author: zhuguohua
 */

#ifndef INTERFACE_PERCAPTION_INFORMATION_H_
#define INTERFACE_PERCAPTION_INFORMATION_H_

#include "derivative.h"
#include "property.h"
#include "Ultrasonic.h"
#include "Interface/vehicle_state.h"

class Percaption {
public:
	Percaption();
	virtual ~Percaption();

//	virtual uint8_t Work(Ultrasonic *u,VehicleState *v) = 0;
	// wheel speed
	float getPositionX();
	void  setPositionX(float value);
	Property<Percaption,float,READ_WRITE> PositionX;

	float getPositionY();
	void  setPositionY(float value);
	Property<Percaption,float,READ_WRITE> PositionY;

	float getAttitudeYaw();
	void  setAttitudeYaw(float value);
	Property<Percaption,float,READ_WRITE> AttitudeYaw;

	float getParkingLength();
	void  setParkingLength(float value);
	Property<Percaption,float,READ_WRITE> ParkingLength;

	float getParkingWidth();
	void  setParkingWidth(float value);
	Property<Percaption,float,READ_WRITE> ParkingWidth;

	bool getDetectParkingStatus();
	void setDetectParkingStatus(bool value);
	Property<Percaption,bool,READ_WRITE> DetectParkingStatus;

private:
	float _position_x;
	float _position_y;
	float _attitude_yaw;
	float _parking_length;
	float _parking_width;
	bool  _detect_parking_status;
};

#endif /* INTERFACE_PERCAPTION_INFORMATION_H_ */

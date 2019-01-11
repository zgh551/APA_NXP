/*
 * vehicle_body.cpp
 *
 *  Created on: 2019Äê1ÔÂ10ÈÕ
 *      Author: zhuguohua
 */

#include <vehicle_body.h>

VehicleBody::VehicleBody() {
	// TODO Auto-generated constructor stub

	Center.setContainer(this);
	Center.getter(&VehicleBody::getCenter);
	Center.setter(&VehicleBody::setCenter);

	FrontLeft.setContainer(this);
	FrontLeft.getter(&VehicleBody::getFrontLeft);
	FrontLeft.setter(&VehicleBody::setFrontLeft);

	FrontRight.setContainer(this);
	FrontRight.getter(&VehicleBody::getFrontRight);
	FrontRight.setter(&VehicleBody::setFrontRight);

	RearLeft.setContainer(this);
	RearLeft.getter(&VehicleBody::getRearLeft);
	RearLeft.setter(&VehicleBody::setRearLeft);

	RearRight.setContainer(this);
	RearRight.getter(&VehicleBody::getRearRight);
	RearRight.setter(&VehicleBody::setRearRight);
}

VehicleBody::~VehicleBody() {
	// TODO Auto-generated destructor stub
}

void Rotation(float angle)
{

}

Vector2d VehicleBody::getCenter()              { return  _center;}
void     VehicleBody::setCenter(Vector2d value){ _center = value;}

Vector2d VehicleBody::getFrontLeft()              { return  _front_left;}
void     VehicleBody::setFrontLeft(Vector2d value){ _front_left = value;}

Vector2d VehicleBody::getFrontRight()              { return  _front_right;}
void     VehicleBody::setFrontRight(Vector2d value){ _front_right = value;}

Vector2d VehicleBody::getRearLeft()              { return  _rear_left;}
void     VehicleBody::setRearLeft(Vector2d value){ _rear_left = value;}

Vector2d VehicleBody::getRearRight()              { return  _rear_right;}
void     VehicleBody::setRearRight(Vector2d value){ _rear_right = value;}

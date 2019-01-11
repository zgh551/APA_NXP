/*
 * vector_2d.cpp
 *
 *  Created on: 2019Äê1ÔÂ2ÈÕ
 *      Author: zhuguohua
 */

#include <vector_2d.h>

Vector2d::Vector2d() {
	// TODO Auto-generated constructor stub
	X.setContainer(this);
	X.getter(&Vector2d::getX);
	X.setter(&Vector2d::setX);

	Y.setContainer(this);
	Y.getter(&Vector2d::getY);
	Y.setter(&Vector2d::setY);
}

Vector2d::~Vector2d() {
	// TODO Auto-generated destructor stub
}

float Vector2d::getX()           { return  _x;}
void  Vector2d::setX(float value){ _x = value;}

float Vector2d::getY()           { return  _y;}
void  Vector2d::setY(float value){ _y = value;}

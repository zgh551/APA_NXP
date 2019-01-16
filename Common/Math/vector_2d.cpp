/*
 * vector_2d.cpp
 *
 *  Created on: January 2 2019
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vector_2d.cpp                       COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the vector property             					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 2 2019      Initial Version                  */
/*****************************************************************************/
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

float Vector2d::Length(void)const
{
	return hypotf(this->_x,this->_y);
}

float Vector2d::LengthSquare(void)const
{
	return this->_x * this->_x + this->_y * this->_y;
}

float Vector2d::Angle(void)const
{
	return atan2f(this->_y , this->_x);
}

float Vector2d::DistanceTo(const Vector2d &other)const
{
	return hypotf(this->_x - other._x , this->_y - other._y);
}


Vector2d Vector2d::rotate(const float angle) const
{
	return Vector2d(this->_x * cosf(angle) - this->_y * sinf(angle),
			        this->_x * sinf(angle) + this->_y * cosf(angle));
}

Vector2d Vector2d::operator+(const Vector2d& other) const
{
	return Vector2d(this->_x + other._x,this->_y + other._y);
}

Vector2d Vector2d::operator-(const Vector2d& other) const
{
	return Vector2d(this->_x - other._x,this->_y - other._y);
}

float Vector2d::getX()           { return  _x;}
void  Vector2d::setX(float value){ _x = value;}

float Vector2d::getY()           { return  _y;}
void  Vector2d::setY(float value){ _y = value;}

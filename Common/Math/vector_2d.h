/*
 * vector_2d.h
 *
 *  Created on: January 2 2019
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vector_2d.h                         COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the vector property             					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 2 2019      Initial Version                  */
/*****************************************************************************/

#ifndef MATH_VECTOR_2D_H_
#define MATH_VECTOR_2D_H_

#include "derivative.h"
#include "property.h"
#include "math.h"

class Vector2d {
public:
	Vector2d();
	Vector2d(const float x,const float y) noexcept:_x(x),_y(y){}
	virtual ~Vector2d();

	float Length(void)const;
	float LengthSquare(void)const;

	float Angle(void)const;

	float DistanceTo(const Vector2d &other)const;

	Vector2d rotate(const float angle) const;

	Vector2d operator+(const Vector2d &other) const;
	Vector2d operator-(const Vector2d &other) const;



	float getX();
	void  setX(float value);
	Property<Vector2d,float,READ_WRITE> X;

	float getY();
	void  setY(float value);
	Property<Vector2d,float,READ_WRITE> Y;

protected:
	float _x;
	float _y;
};

#endif /* MATH_VECTOR_2D_H_ */

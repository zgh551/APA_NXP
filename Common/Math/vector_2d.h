/*
 * vector_2d.h
 *
 *  Created on: 2019Äê1ÔÂ2ÈÕ
 *      Author: zhuguohua
 */

#ifndef MATH_VECTOR_2D_H_
#define MATH_VECTOR_2D_H_

#include "derivative.h"
#include "property.h"

class Vector2d {
public:
	Vector2d();
	virtual ~Vector2d();

	float getX();
	void  setX(float value);
	Property<Vector2d,float,READ_WRITE> X;

	float getY();
	void  setY(float value);
	Property<Vector2d,float,READ_WRITE> Y;

private:
	float _x;
	float _y;
};

#endif /* MATH_VECTOR_2D_H_ */

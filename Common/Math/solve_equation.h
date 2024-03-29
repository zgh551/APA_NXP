/*
 * solve_equation.h
 *
 *  Created on: 2019年3月18日
 *      Author: zhuguohua
 */

#ifndef MATH_SOLVE_EQUATION_H_
#define MATH_SOLVE_EQUATION_H_

#include "derivative.h"
#include "property.h"
#include "vector_2d.h"
#include "math.h"
#include "vehilce_config.h"

class SolveEquation {
public:
	SolveEquation();
	virtual ~SolveEquation();

	// A*cos + B*sin = C
	int8_t TrigonometricEquation(float a,float b,float c,float *theta1,float *theta2);

	// a*x^2 + b*x + c = 0
	int8_t QuadraticEquation(float a,float b,float c,float *x1,float *x2);

};

#endif /* MATH_SOLVE_EQUATION_H_ */

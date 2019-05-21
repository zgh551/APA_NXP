/*
 * curve_fitting.h
 *
 *  Created on: 2019年5月20日
 *      Author: zhuguohua
 */

#ifndef MATH_CURVE_FITTING_H_
#define MATH_CURVE_FITTING_H_

#include "link_list.h"


class CurveFitting {
public:
	CurveFitting();
	virtual ~CurveFitting();

	void LineFitting(LinkList *l,float *a,float *b);
};

#endif /* MATH_CURVE_FITTING_H_ */

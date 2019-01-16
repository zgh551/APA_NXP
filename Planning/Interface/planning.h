/*
 * path_plannig.h
 *
 *  Created on: 2019Äê1ÔÂ9ÈÕ
 *      Author: zhuguohua
 */

#ifndef INTERFACE_PATH_PLANNING_H_
#define INTERFACE_PATH_PLANNING_H_

#include "derivative.h"
#include "property.h"
#include <percaption_information.h>
//#include "Terminal.h"

class Planning {
public:
	Planning();
	virtual ~Planning();

	virtual void Init() = 0;
	virtual void Work(PercaptionInformation *p) = 0;

	float getMinParkingLength();
	void  setMinParkingLength(float value);
	Property<Planning,float,READ_WRITE> MinParkingLength;

	float getMinParkingWidth();
	void  setMinParkingWidth(float value);
	Property<Planning,float,READ_WRITE> MinParkingWidth;
private:
	float _min_parking_length;
	float _min_parking_width;
};

#endif /* INTERFACE_PATH_PLANNIG_H_ */

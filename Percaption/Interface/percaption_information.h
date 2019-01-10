/*
 * percaption_information.h
 *
 *  Created on: 2019Äê1ÔÂ9ÈÕ
 *      Author: zhuguohua
 */

#ifndef INTERFACE_PERCAPTION_INFORMATION_H_
#define INTERFACE_PERCAPTION_INFORMATION_H_

#include "derivative.h"
#include "property.h"

class PercaptionInformation {
public:
	PercaptionInformation();
	virtual ~PercaptionInformation();

	// wheel speed
	float getPositionX();
	void  setPositionX(float value);
	Property<PercaptionInformation,float,READ_WRITE> PositionX;

	float getPositionY();
	void  setPositionY(float value);
	Property<PercaptionInformation,float,READ_WRITE> PositionY;

	float getPositionYaw();
	void  setPositionYaw(float value);
	Property<PercaptionInformation,float,READ_WRITE> PositionYaw;
private:
	bool _detect_parking_status;
	float _position_x;
	float _position_y;
	float _position_yaw;
	float _parking_lenght;
	float _parking_width;
};

#endif /* INTERFACE_PERCAPTION_INFORMATION_H_ */

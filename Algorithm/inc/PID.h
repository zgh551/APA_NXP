/*
 * PID.h
 *
 *  Created on: 2018Äê11ÔÂ28ÈÕ
 *      Author: zhuguohua
 */

#ifndef PID_H_
#define PID_H_

#include "math.h"
//namespace APA {

class PID {
public:
	PID();
	PID(float dt,float kp,float ki,float kd,float i_lim,float out_lim);
	virtual ~PID();
	float pidUpdate(float measured);

private:
    float _desired;      //< set point
    float _error;        //< error
    float _prevError;    //< previous error
    float _integ;        //< integral
    float _deriv;        //< derivative
    float _kp;           //< proportional gain
    float _ki;           //< integral gain
    float _kd;           //< derivative gain
    float _outP;         //< proportional output (debugging)
    float _outI;         //< integral output (debugging)
    float _outD;         //< derivative output (debugging)
    float _iLimit;       //< integral limit, absolute value. '0' means no limit.
    float _outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
    float _dt;           //< delta-time dt
};

//} /* namespace APA */

#endif /* PID_H_ */

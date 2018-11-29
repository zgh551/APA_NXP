/*
 * PID.cpp
 *
 *  Created on: 2018Äê11ÔÂ28ÈÕ
 *      Author: zhuguohua
 */

#include "PID.h"

PID::PID() {
	// TODO Auto-generated constructor stub
    _desired = 0.0f;      //< set point
    _error = 0.0f;        //< error
    _prevError = 0.0f;    //< previous error
    _integ = 0.0f;        //< integral
    _deriv = 0.0f;        //< derivative
    _kp = 0.0f;           //< proportional gain
    _ki = 0.0f;           //< integral gain
    _kd = 0.0f;           //< derivative gain
    _iLimit = 3;       //< integral limit, absolute value. '0' means no limit.
    _outputLimit = 5;  //< total PID output limit, absolute value. '0' means no limit.
    _dt = 0.0f;           //< delta-time dt
}

PID::PID(float dt,float kp,float ki,float kd,float i_lim,float out_lim) {
	// TODO Auto-generated constructor stub
    _desired = 0.0f;      //< set point
    _error = 0.0f;        //< error
    _prevError = 0.0f;    //< previous error
    _integ = 0.0f;        //< integral
    _deriv = 0.0f;        //< derivative
    _kp = kp;           //< proportional gain
    _ki = ki;           //< integral gain
    _kd = kd;           //< derivative gain
    _iLimit = i_lim;       //< integral limit, absolute value. '0' means no limit.
    _outputLimit = out_lim;  //< total PID output limit, absolute value. '0' means no limit.
    _dt = dt;           //< delta-time dt
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

float PID::pidUpdate(float measured)
{
    float output = 0.0f;
    _error = _desired - measured;

    _outP = _kp * _error;
    output += _outP;

    _deriv = (_error - _prevError) / _dt;
    _outD = _kd * _deriv;
    output += _outD;

    _integ += _error * _dt;

}




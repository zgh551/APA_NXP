/*
 * lat_control.h
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */

#ifndef LATCONTROL_LAT_CONTROL_H_
#define LATCONTROL_LAT_CONTROL_H_

#include "derivative.h"
#include "property.h"
#include "controller.h"
#include "pid.h"
#include "math.h"
#include "vector_2d.h"
#include "GeometricTrack/geometric_track.h"
#include "vehilce_config.h"

#define COEFFICIENT_TLS 	( 0.4f  )	// 目标曲线曲率因子
#define COEFFICIENT_SMV 	( 12.0f )	// 滑模变量系数
#define COEFFICIENT_RHO 	( 0.02f )	// 补偿噪声系数
#define COEFFICIENT_K   	( 1.1f  )	// 指数趋近率系数
#define COEFFICIENT_DELTA	( 0.1f  ) // Sat函数系数

class LatControl :public Controller
{
public:
	LatControl();
	virtual ~LatControl();

	void Init() override;

	float TargetLine(float x);
	float TargetLineFirstDerivative(float x);
	float TargetLineSecondDerivative(float x);

	float SatFunction(float x);

	void Proc(MessageManager *msg,VehicleController *ctl,PID *pid) override;
	void Proc(MessageManager *msg,VehicleController *ctl,GeometricTrack *track);

	Vector2d getTargetTrack();
	void setTargetTrack(Vector2d value);

	float getX1();
	void  setX1(float value);

	float getX2();
	void  setX2(float value);

	float getSlidingVariable();
	void  setSlidingVariable(float value);

private:
	Vector2d _target_track;

	float _x1;
	float _x2;
	float _sliding_variable;
};

#endif /* LATCONTROL_LAT_CONTROL_H_ */

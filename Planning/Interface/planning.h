/*****************************************************************************/
/* FILE NAME: path_plannig.h                      COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the trajectory planning interface  					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 9 2019      Initial Version                  */
/*****************************************************************************/

#ifndef INTERFACE_PATH_PLANNING_H_
#define INTERFACE_PATH_PLANNING_H_

#include "derivative.h"
#include "property.h"
#include <percaption.h>
#include "Interface/vehicle_controller.h"
#include <vector_2d.h>
#include "../../VehicleState/GeometricTrack/geometric_track.h"
#include "ChangAn/chang_an_controller.h"
#include "Ultrasonic.h"
//#include "Terminal.h"
//state
#define PARKING_FINISH        ( 1   ) // 泊车完成的状态标志
//
#define STEERING_RATE         ( 300 ) // 转向角速度
#define EMERGENCY_BRAKING	  (-5 ) // 紧急制动的减速度
#define PLANNING_BRAKING	  (-1  ) // 规划减速度
#define PLANNING_BRAKING_R	  ( 1 ) // 规划减速度的倒数
#define STRAIGHT_VELOCITY	  ( 0.6 ) // 直线段的速度
#define CURVE_VELOCITY	      ( 0.3 ) // 曲线段的速度
#define TURN_FEEDFORWARD_TIME ( 0.01) // 转向角前向反馈的补偿时间
#define PARKING_CENTER_MARGIN ( 0.1 ) // 泊车中心点余量

//extern GeometricTrack    m_GeometricTrack;
//extern ChangAnController m_ChangAnController;

typedef struct _Turn
{
	Vector2d Point;
	float SteeringAngle;
}Turn;

class Planning {
public:
	Planning();
	virtual ~Planning();

	virtual void Init() = 0;
	virtual void Work(Percaption *p,VehicleState *s) = 0;
	virtual void Control(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u) = 0;

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

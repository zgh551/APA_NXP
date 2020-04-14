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
#include "vehicle_config.h"

#define COEFFICIENT_TLS 	( 1.2f  )	// 目标曲线曲率因子
#define COEFFICIENT_SMV 	( 12.0f )	// 滑模变量系数
#define COEFFICIENT_RHO 	( 0.02f )	// 补偿噪声系数
#define COEFFICIENT_K   	( 1.1f  )	// 指数趋近率系数
#define COEFFICIENT_DELTA	( 0.1f  )   // Sat函数系数

#define COEFFICIENT_KPSI	( 1.0f  )   // K_psi
#define COEFFICIENT_KE		( 0.5f  )   // K_e

typedef enum _LatControlStatus
{
	init_status = 0,
	process_status
}LatControlStatus;


typedef struct _TargetTrack
{
	Vector2d point;
	float    yaw;
	float    curvature;
}TargetTrack;

/**
 * @brief The TrackLinkList class：曲线跟踪目标曲线链表
 */
class TrackLinkList : public LinkList<TargetTrack>{};

class LatControl :public Controller
{
public:
	LatControl();
	virtual ~LatControl();

	void Init() override;

	float TargetLine(float x);
	float TargetLineFirstDerivative(float x);
	float TargetLineSecondDerivative(float x);
	TargetTrack TrackingCurve(float x);

    /**
     * @brief GenerateCurvatureSets：生成目标曲线数据集
     * @param list：目标曲线数据集
     */
    void GenerateCurvatureSets(TrackLinkList *list);

    /**
     * @brief CalculateNearestPoint：计算最近的目标点
     * @param list：目标曲线数据集
     * @param a_track：当前车辆跟踪位置
     * @return 返回离车辆后轴最近的目标曲线点
     */
    TargetTrack CalculateNearestPoint(TrackLinkList *list,GeometricTrack *a_track);

	float pi2pi(float angle);

	float SatFunction(float x);

	void Proc(MessageManager *msg,VehicleController *ctl,PID *pid) override;

    /**
     * @brief LatControl::ProcV1_0 基于非时间参考的滑模控制算法
     * @param msg：车辆信号反馈，主要获取车辆速度信号
     * @param ctl：车辆控制信号量，控制转向角和角速度信号
     * @param a_track       :实际跟踪变量
     * @param a_track.point :实时跟踪坐标
     * @param a_track.yaw   :实时航向角@(-pi,pi)
     * @param t_track           :目标路径参数变量
     * @param t_track.point     :目标路径与车辆后轴最近的点
     * @param t_track.yaw       :目标路径的航向角 in(-pi,pi)
     * @param t_track.curvature :目标路径处的曲率 左正右负
     */
	void ProcV1_0(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track);

    /**
     * @brief LatControl::RearWheelFeedback 基于车辆后轴位置反馈的路径跟踪算法
     * @param msg：车辆信号反馈，主要获取车辆速度信号
     * @param ctl：车辆控制信号量，控制转向角和角速度信号
     * @param a_track       :实际跟踪变量
     * @param a_track.point :实时跟踪坐标
     * @param a_track.yaw   :实时航向角@(-pi,pi)
     * @param t_track           :目标路径参数变量
     * @param t_track.point     :目标路径与车辆后轴最近的点
     * @param t_track.yaw       :目标路径的航向角 in(-pi,pi)
     * @param t_track.curvature :目标路径处的曲率 左正右负
     */
	void RearWheelFeedback(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track);

	/**
     * @brief LatControl::Work 横向控制状态切换控制
     * @param msg：车辆信息->挡位、车速
     * @param ctl：车辆控制->转向角、转向角速度、车速、挡位
     * @param a_track：当前跟踪位置信息
     * @param t_track：目标曲线信息
     * @param last_track：车辆终点信息
     */
    void Work(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track,TargetTrack last_track);

	TargetTrack getTargetTrack();
	void setTargetTrack(TargetTrack value);

	float getX1();
	void  setX1(float value);

	float getX2();
	void  setX2(float value);

	float getSlidingVariable();
	void  setSlidingVariable(float value);

	uint8_t getCommand();
	void    setCommand(uint8_t value);

	uint8_t getCurvatureValid();
	void    setCurvatureValid(uint8_t value);

private:
	TargetTrack _target_track;
	float _x1;
	float _x2;
	float _sliding_variable;

	LatControlStatus _lat_control_status;

	uint8_t _command;
	uint8_t _curvature_valid;
};

#endif /* LATCONTROL_LAT_CONTROL_H_ */

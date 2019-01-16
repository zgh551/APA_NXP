/*
 * chang_an_configure.h
 *
 *  Created on: December 27 2018
 *      Author: Guohua Zhu
 */

#ifndef _CHANG_AN_CONFIGURE_H_
#define _CHANG_AN_CONFIGURE_H_

/****************** 车辆信息 ******************/
// 长度(m)
#define LENGHT                           (4.445)
// 宽度(m)
#define WIDTH                            (1.594)
// 高度(m)
#define HEIGHT                           (1.35)
// 轴距(m)
#define WHEEL_BASE                        (2.64)
// 车辆边沿到车辆中心点的距离(m)
#define FRONT_EDGE_TO_CENTER              (3.54)
#define REAR_EDGE_TO_CENTER               (0.905)
#define LEFT_EDGE_TO_CENTER               (0.797)
#define RIGHT_EDGE_TO_CENTER              (0.797)

// 最小转弯半径(m)
#define MIN_TURN_RADIUS                   (5.1)
// 转向传动比
#define STEERING_RATIO                   (15.3)

// 最大加速度(m/s2)
#define MAX_ACCELERATION                    (3)
// 最大减速度(m/s2)
#define MAX_DECELERATION                   (-5)

// 最大转向角(°)
#define MAX_STEERING_ANGLE                (450)
// 最大转向加速度(°/s)
#define MAX_STEERING_ANGLE_RATE           (500)

// 速度修正比例系数
#define SPEED_REVISE_RATIO               (1.01)
// 轮脉冲比例系数
#define WHEEL_PUSLE_RATIO                (0.02540618132391610878926842900878)

// Math ratio
#define PI 								 (3.1415926)

/*** turn radius and the steering angle relationship ***/
// (400,470]
#define FIT_RADIUS_A1 		( 0.07466)
#define FIT_RADIUS_B1 		(-4.072)

// (300,400]
#define FIT_RADIUS_A2 		( 0.06758)
#define FIT_RADIUS_B2 		(-1.289)

// (200,300]
#define FIT_RADIUS_A3 		( 0.0653)
#define FIT_RADIUS_B3 		(-0.5621)

// (100,200]
#define FIT_RADIUS_A4 		( 0.06206)
#define FIT_RADIUS_B4 		( 0.06662)

// (50,100]
#define FIT_RADIUS_A5 		( 0.06312)
#define FIT_RADIUS_B5 		(-0.02795)

// (0,50]
#define FIT_RADIUS_A6 		( 0.06706)
#define FIT_RADIUS_B6 		(-0.1501)

// (-50,0)
#define FIT_RADIUS_A7 		( 0.06442)
#define FIT_RADIUS_B7 		(-0.07814)

// (-100,-50]
#define FIT_RADIUS_A8 		( 0.06123)
#define FIT_RADIUS_B8 		(-0.02889)

// (-200,-100]
#define FIT_RADIUS_A9 		( 0.06356)
#define FIT_RADIUS_B9 		(-0.2489)

// (-300,-200]
#define FIT_RADIUS_A10 		( 0.06393)
#define FIT_RADIUS_B10 		(-0.3534)

// (-400,-300]
#define FIT_RADIUS_A11 		( 0.06846)
#define FIT_RADIUS_B11 		(-1.662)

// (-510,-400]
#define FIT_RADIUS_A12 		( 18.3)
#define FIT_RADIUS_B12 		(-0.003017)

#endif

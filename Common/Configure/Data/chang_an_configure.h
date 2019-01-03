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
#define LENGHT                           (4.89)
// 宽度(m)
#define WIDTH                            (1.78)
// 高度(m)
#define HEIGHT                           (1.35)
// 轴距(m)
#define WHEEL_BASE                        (2.8)
// 车辆边沿到车辆中心点的距离(m)
#define FRONT_EDGE_TO_CENTER              (3.2)
#define REAR_EDGE_TO_CENTER               (1.2)
#define LEFT_EDGE_TO_CENTER               (1.0)
#define RIGHT_EDGE_TO_CENTER              (1.0)

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
#define WHEEL_PUSLE_RATIO                (0.13)

// Math ratio
#define PI 								 (3.1415926)

/*** turn radius and the steering angle relationship ***/
// (400,470]
#define a1 		( 0.07466)
#define b1 		(-4.072)

// (300,400]
#define a2 		( 0.06758)
#define b2 		(-1.289)

// (200,300]
#define a3 		( 0.0653)
#define b3 		(-0.5621)

// (100,200]
#define a4 		( 0.06206)
#define b4 		( 0.06662)

// (50,100]
#define a5 		( 0.06312)
#define b5 		(-0.02795)

// (0,50]
#define a6 		( 0.06706)
#define b6 		(-0.1501)

// (-50,0)
#define a7 		( 0.06442)
#define b7 		(-0.07814)

// (-100,-50]
#define a8 		( 0.06123)
#define b8 		(-0.02889)

// (-200,-100]
#define a9 		( 0.06356)
#define b9 		(-0.2489)

// (-300,-200]
#define a10 	( 0.06393)
#define b10 	(-0.3534)

// (-400,-300]
#define a11 	( 0.06846)
#define b11 	(-1.662)

// (-510,-400]
#define a12 	( 18.3)
#define b12 	(-0.003017)

#endif

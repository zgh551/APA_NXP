/*
 * bo_rui_configure.h
 *
 *  Created on: 2019年3月26日
 *      Author: zhuguohua
 */

#ifndef _BO_RUI_CONFIGURE_H_
#define _BO_RUI_CONFIGURE_H_

#include "common_configure.h"
/****************** 车辆信息 ******************/
// 长度(m)
#define LENGHT                            (4.986)
// 宽度(m)
#define WIDTH                             (1.861)
// 高度(m)
#define HEIGHT                            (1.513)
// 轴距(m)
#define WHEEL_BASE                        (2.850)
// 车辆边沿到车辆中心点的距离(m)
#define FRONT_EDGE_TO_CENTER              (3.54)
#define REAR_EDGE_TO_CENTER               (0.905)
#define LEFT_EDGE_TO_CENTER               (0.9275)
#define RIGHT_EDGE_TO_CENTER              (0.9275)

// 最小转弯半径(m)
#define MIN_RIGHT_TURN_RADIUS             (4.7079)
#define MIN_LEFT_TURN_RADIUS              (4.6614)
// 转向传动比
#define STEERING_RATIO                    (16)

// 最大加速度(m/s2)
#define MAX_ACCELERATION                    (3)
// 最大减速度(m/s2)
#define MAX_DECELERATION                   (-5)

// 最大转向角(°)
#define MAX_STEERING_ANGLE                (500)
// 最大转向加速度(°/s)
#define MAX_STEERING_ANGLE_RATE           (500)

// 速度修正比例系数
#define SPEED_REVISE_RATIO               (1.01)
// 轮脉冲比例系数
#define WHEEL_PUSLE_RATIO                (0.021)

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
#define FIT_RADIUS_A6 		( 0.062  )
#define FIT_RADIUS_B6 		( 0.0    )

// (-50,0)
#define FIT_RADIUS_A7 		( 0.062  )
#define FIT_RADIUS_B7 		( 0.0    )

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
#define FIT_RADIUS_A12 		( 0.06652)
#define FIT_RADIUS_B12 		(-1.586)

/**********************************Ultrasonic Location Information***********************************************/
//#define SENSOR1_X     (-0.20f )
//#define SENSOR1_Y     ( 0.68f )
//#define SENSOR1_ANGLE ( 0.00f )
//
//#define SENSOR2_X     ( 0.00f )
//#define SENSOR2_Y     ( 0.32f )
//#define SENSOR2_ANGLE ( 0.00f )
//
//#define SENSOR3_X     ( 0.00f )
//#define SENSOR3_Y     (-0.32f )
//#define SENSOR3_ANGLE ( 0.00f )
//
//#define SENSOR4_X     (-0.20f )
//#define SENSOR4_Y     (-0.68f )
//#define SENSOR4_ANGLE ( 0.00f )
//实验室测试
//#define SENSOR1_X     ( 0.00f )
//#define SENSOR1_Y     ( 0.45f )
//#define SENSOR1_ANGLE ( 0.00f )
//
//#define SENSOR2_X     ( 0.00f )
//#define SENSOR2_Y     ( 0.18f )
//#define SENSOR2_ANGLE ( 0.00f )
//
//#define SENSOR3_X     ( 0.00f )
//#define SENSOR3_Y     (-0.18f )
//#define SENSOR3_ANGLE ( 0.00f )
//
//#define SENSOR4_X     ( 0.00f )
//#define SENSOR4_Y     (-0.50f )
//#define SENSOR4_ANGLE ( 0.00f )

//实际车辆的超声安装位置
#define SENSOR1_X     ( 3.56f )
#define SENSOR1_Y     ( 0.68f )
#define SENSOR1_ANGLE ( 0.00f )

#define SENSOR2_X     ( 3.76f )
#define SENSOR2_Y     ( 0.32f )
#define SENSOR2_ANGLE ( 0.00f )

#define SENSOR3_X     ( 3.76f )
#define SENSOR3_Y     (-0.32f )
#define SENSOR3_ANGLE ( 0.00f )

#define SENSOR4_X     ( 3.56f )
#define SENSOR4_Y     (-0.68f )
#define SENSOR4_ANGLE ( 0.00f )

#define SENSOR5_X     (-1.02f )
#define SENSOR5_Y     ( 0.69f )
#define SENSOR5_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR6_X     (-1.08f )
#define SENSOR6_Y     ( 0.32f )
#define SENSOR6_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR7_X     (-1.08f )
#define SENSOR7_Y     (-0.32f )
#define SENSOR7_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR8_X     (-1.02f )
#define SENSOR8_Y     (-0.69f )
#define SENSOR8_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR9_X     ( 3.15f )
#define SENSOR9_Y     ( 0.90f )
#define SENSOR9_ANGLE (1.5707963267948966192313216916398 )

#define SENSOR10_X     ( 3.15f )
#define SENSOR10_Y     (-0.90f )
#define SENSOR10_ANGLE (-1.5707963267948966192313216916398 )

#define SENSOR11_X     (-0.32f)
#define SENSOR11_Y     ( 0.90f)
#define SENSOR11_ANGLE (1.5707963267948966192313216916398)

#define SENSOR12_X     (-0.32f)
#define SENSOR12_Y     (-0.90f)
#define SENSOR12_ANGLE (-1.5707963267948966192313216916398)

#endif /* CONFIGURE_DATA_BO_RUI_CONFIGURE_H_ */

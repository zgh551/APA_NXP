/*
 * chang_an_configure.h
 *
 *  Created on: December 27 2018
 *      Author: Guohua Zhu
 */

#ifndef _CHANG_AN_CONFIGURE_H_
#define _CHANG_AN_CONFIGURE_H_

/****************** 车辆信息 ******************/
// 车辆长度(m)
#define LENGHT                           (4.89)
// 车辆宽度(m)
#define WIDTH                            (1.78)
// 车辆高度(m)
#define HEIGHT                           (1.35)
// 车辆轴距(m)
#define WHEEL_BASE                        (2.8)
// 车辆边沿到车辆中心的距离(m)
#define FRONT_EDGE_TO_CENTER              (3.2)
#define REAR_EDGE_TO_CENTER               (1.2)
#define LEFT_EDGE_TO_CENTER               (1.0)
#define RIGHT_EDGE_TO_CENTER              (1.0)

// 车辆最小转弯半径(m)
#define MIN_TURN_RADIUS                   (5.1)
// 车辆转向比
#define STEERING_RATIO                   (15.3)

// 最大加速度(m/s2)
#define MAX_ACCELERATION                    (3)
// 最大减速度(m/s2)
#define MAX_DECELERATION                   (-5)

// 最大加速度(m/s2)
#define MAX_ACCELERATION                    (3)
// 最大减速度(m/s2)
#define MAX_DECELERATION                   (-5)

// 最大转向角度(°)
#define MAX_STEERING_ANGLE                (450)
// 最大角速度(°/s)
#define MAX_STEERING_ANGLE_RATE           (500)

// 速度比例系数
#define SPEED_REVISE_RATIO               (1.01)
// 轮脉冲系数
#define WHEEL_PUSLE_RATIO                (0.13)

#endif

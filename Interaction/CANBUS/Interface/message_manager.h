/*
 * message_manager.h
 *
 *  Created on: December 29 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: message_manager.h                   COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: Messege manage module 								         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 29 2018    Initial Version                  */
/*****************************************************************************/

#ifndef CANBUS_INTERFACE_MESSAGE_MANAGER_H_
#define CANBUS_INTERFACE_MESSAGE_MANAGER_H_

#include "derivative.h"
#include "property.h"

class MessageManager {
public:
	MessageManager();
	virtual ~MessageManager();

	virtual void Parse(const uint32_t id,const vuint8_t *data,const vuint32_t lenght) = 0;

	// wheel speed
	float getWheelSpeedFrontLeft();
	void  setWheelSpeedFrontLeft(float value);
	Property<MessageManager,float,READ_WRITE> WheelSpeedFrontLeft;

	float getWheelSpeedFrontRight();
	void  setWheelSpeedFrontRight(float value);
	Property<MessageManager,float,READ_WRITE> WheelSpeedFrontRight;

	float getWheelSpeedRearRight();
	void  setWheelSpeedRearRight(float value);
	Property<MessageManager,float,READ_WRITE> WheelSpeedRearRight;

	float getWheelSpeedRearLeft();
	void  setWheelSpeedRearLeft(float value);
	Property<MessageManager,float,READ_WRITE> WheelSpeedRearLeft;

	// wheel speed direction
	uint8_t getWheelSpeedDirection();
	void    setWheelSpeedDirection(uint8_t value);
	Property<MessageManager,uint8_t,READ_WRITE> WheelSpeedDirection;

	// wheel pulse
	uint8_t getWheelPulseFrontLeft();
	void    setWheelPulseFrontLeft(uint8_t value);
	Property<MessageManager,uint8_t,READ_WRITE> WheelPulseFrontLeft;

	uint8_t getWheelPulseFrontRight();
	void    setWheelPulseFrontRight(uint8_t value);
	Property<MessageManager,uint8_t,READ_WRITE> WheelPulseFrontRight;

	uint8_t getWheelPulseRearRight();
	void    setWheelPulseRearRight(uint8_t value);
	Property<MessageManager,uint8_t,READ_WRITE> WheelPulseRearRight;

	uint8_t getWheelPulseRearLeft();
	void    setWheelPulseRearLeft(uint8_t value);
	Property<MessageManager,uint8_t,READ_WRITE> WheelPulseRearLeft;

	// wheel pulse dirction
	uint8_t getWheelPulseDirection();
	void    setWheelPulseDirection(uint8_t value);
	Property<MessageManager,uint8_t,READ_WRITE> WheelPulseDirection;

	// SAS Steering angle
	float getSteeringAngle();
	void  setSteeringAngle(float value);
	Property<MessageManager,float,READ_WRITE> SteeringAngle;

	float getSteeringAngleRate();
	void  setSteeringAngleRate(float value);
	Property<MessageManager,float,READ_WRITE> SteeringAngleRate;

	// TCU
	float getGear();
	void  setGear(float value);
	Property<MessageManager,float,READ_WRITE> Gear;

	// ESP Sensor
	float getYawRate();
	void  setYawRate(float value);
	Property<MessageManager,float,READ_WRITE> YawRate;

	float getLonAcc();
	void  setLonAcc(float value);
	Property<MessageManager,float,READ_WRITE> LonAcc;

	float getLatAcc();
	void  setLatAcc(float value);
	Property<MessageManager,float,READ_WRITE> LatAcc;
private:
	// wheel speed
	float _wheel_speed_front_left ;
	float _wheel_speed_front_right;
	float _wheel_speed_rear_right ;
	float _wheel_speed_rear_left  ;
	// wheel speed direction
	uint8_t _wheel_speed_direction;

	// wheel pulse
	uint8_t _wheel_pulse_front_left ;
	uint8_t _wheel_pulse_front_right;
	uint8_t _wheel_pulse_rear_right ;
	uint8_t _wheel_pulse_rear_left  ;
	// wheel pulse dirction
	uint8_t _wheel_pulse_direction;

	// SAS Steering angle
	float _steering_angle;
	float _steering_angle_rate;

	// TCU
	float _gear;

	// ESP Sensor
	float _yaw_rate;
	float _lon_acc;
	float _lat_acc;
};

#endif /* CANBUS_INTERFACE_MESSAGE_MANAGER_H_ */

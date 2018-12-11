/*
 * Vehicle.cpp
 *
 *  Created on: 2018��11��21��
 *      Author: zhuguohua
 */

#include "Vehicle.h"

Vehicle::Vehicle()
{
	// TODO Auto-generated constructor stub
	_steering_angle_control_state = 0;
	/*** the vehicle body information ***/
	// Lenght
	_wheelbase_lenght = 2.65;
	_front_overhang_distance = 0.952;
	_rear_overhang_distance = 1;

	// width
	_wheel_axis_width = 1.794;
	_wheel_axis_width_half = 0.765;
	_wheel_edge_distance = 0.132;

	// the vehice four edge point calculate
	_front_axis_lenght = sqrtf(powf(_wheel_axis_width_half + _wheel_edge_distance , 2) + powf(_wheelbase_lenght + _front_overhang_distance ,2));
	_rear_axis_lenght  = sqrtf(powf(_wheel_axis_width_half + _wheel_edge_distance , 2) + powf(_rear_overhang_distance ,2));
	_beta_front        = atanf( (_wheel_axis_width_half + _wheel_edge_distance) / (_wheelbase_lenght + _front_overhang_distance));
	_beta_rear         = atanf( (_wheel_axis_width_half + _wheel_edge_distance) / _rear_overhang_distance );

	////// Vehicle Body Information //////
	/// Lenght
	WheelBaseLenght.setContainer(this);
	WheelBaseLenght.getter(&Vehicle::getWheelBaseLenght);
	WheelBaseLenght.setter(&Vehicle::setWheelBaseLenght);

	FrontOverhangDistance.setContainer(this);
	FrontOverhangDistance.getter(&Vehicle::getFrontOverhangDistance);
	FrontOverhangDistance.setter(&Vehicle::setFrontOverhangDistance);

	RearOverhangDistance.setContainer(this);
	RearOverhangDistance.getter(&Vehicle::getRearOverhangDistance);
	RearOverhangDistance.setter(&Vehicle::setRearOverhangDistance);
	/// Width
	WheelAxisWidth.setContainer(this);
	WheelAxisWidth.getter(&Vehicle::getWheelAxisWidth);
	WheelAxisWidth.setter(&Vehicle::setWheelAxisWidth);

	WheelAxisWidthHalf.setContainer(this);
	WheelAxisWidthHalf.getter(&Vehicle::getWheelAxisWidthHalf);
	WheelAxisWidthHalf.setter(&Vehicle::setWheelAxisWidthHalf);

	WheelEdgeDistance.setContainer(this);
	WheelEdgeDistance.getter(&Vehicle::getWheelEdgeDistance);
	WheelEdgeDistance.setter(&Vehicle::setWheelEdgeDistance);
	/// the vehicle edge point calculate information
	FrontAxisLenght.setContainer(this);
	FrontAxisLenght.getter(&Vehicle::getFrontAxisLenght);
	FrontAxisLenght.setter(&Vehicle::setFrontAxisLenght);

	RearAxisLenght.setContainer(this);
	RearAxisLenght.getter(&Vehicle::getRearAxisLenght);
	RearAxisLenght.setter(&Vehicle::setRearAxisLenght);

	BetaFront.setContainer(this);
	BetaFront.getter(&Vehicle::getBetaFront);
	BetaFront.setter(&Vehicle::setBetaFront);

	BetaRear.setContainer(this);
	BetaRear.getter(&Vehicle::getBetaRear);
	BetaRear.setter(&Vehicle::setBetaRear);
	////// ACC //////
	TargetAccelerationACC.setContainer(this);
	TargetAccelerationACC.getter(&Vehicle::getTargetAccelerationACC);
	TargetAccelerationACC.setter(&Vehicle::setTargetAccelerationACC);

	TargetAccelerationEnable.setContainer(this);
	TargetAccelerationEnable.getter(&Vehicle::getTargetAccelerationEnable);
	TargetAccelerationEnable.setter(&Vehicle::setTargetAccelerationEnable);

	////// AEB //////
	TargetDecelerationAEB.setContainer(this);
	TargetDecelerationAEB.getter(&Vehicle::getTargetDecelerationAEB);
	TargetDecelerationAEB.setter(&Vehicle::setTargetDecelerationAEB);

	TargetDecelerationEnable.setContainer(this);
	TargetDecelerationEnable.getter(&Vehicle::getTargetDecelerationEnable);
	TargetDecelerationEnable.setter(&Vehicle::setTargetDecelerationEnable);

	////// Torque //////
	Torque.setContainer(this);
	Torque.getter(&Vehicle::getTorque);
	Torque.setter(&Vehicle::setTorque);

	TorqueEnable.setContainer(this);
	TorqueEnable.getter(&Vehicle::getTorqueEnable);
	TorqueEnable.setter(&Vehicle::setTorqueEnable);

	////// Steering Angle //////
	SteeringAngleTarget.setContainer(this);
	SteeringAngleTarget.getter(&Vehicle::getSteeringAngleTarget);
	SteeringAngleTarget.setter(&Vehicle::setSteeringAngleTarget);

	SteeringAngleSpeedTarget.setContainer(this);
	SteeringAngleSpeedTarget.getter(&Vehicle::getSteeringAngleSpeedTarget);
	SteeringAngleSpeedTarget.setter(&Vehicle::setSteeringAngleSpeedTarget);

	SteeringAngleTargetActive.setContainer(this);
	SteeringAngleTargetActive.getter(&Vehicle::getSteeringAngleTargetActive);
	SteeringAngleTargetActive.setter(&Vehicle::setSteeringAngleTargetActive);

	////// Gear //////
	GearShift.setContainer(this);
	GearShift.getter(&Vehicle::getGearShift);
	GearShift.setter(&Vehicle::setGearShift);

	GearShiftEnable.setContainer(this);
	GearShiftEnable.getter(&Vehicle::getGearShiftEnable);
	GearShiftEnable.setter(&Vehicle::setGearShiftEnable);

	GearShiftValid.setContainer(this);
	GearShiftValid.getter(&Vehicle::getGearShiftValid);
	GearShiftValid.setter(&Vehicle::setGearShiftValid);

	/// Read Only
	// EPS
	EPS_Failed.setContainer(this);
	EPS_Failed.getter(&Vehicle::getEPS_Failed);
	EPS_Failed.setter(&Vehicle::setEPS_Failed);

	APA_EPAS_Failed.setContainer(this);
	APA_EPAS_Failed.getter(&Vehicle::getAPA_EPAS_Failed);
	APA_EPAS_Failed.setter(&Vehicle::setAPA_EPAS_Failed);

	APA_ControlFeedback.setContainer(this);
	APA_ControlFeedback.getter(&Vehicle::getAPA_ControlFeedback);
	APA_ControlFeedback.setter(&Vehicle::setAPA_ControlFeedback);

	TorqueSensorStatus.setContainer(this);
	TorqueSensorStatus.getter(&Vehicle::getTorqueSensorStatus);
	TorqueSensorStatus.setter(&Vehicle::setTorqueSensorStatus);

	SteeringTorque.setContainer(this);
	SteeringTorque.getter(&Vehicle::getSteeringTorque);
	SteeringTorque.setter(&Vehicle::setSteeringTorque);

	// Wheel Speed
	WheelSpeedRearLeftDirection.setContainer(this);
	WheelSpeedRearLeftDirection.getter(&Vehicle::getWheelSpeedRearLeftDirection);
	WheelSpeedRearLeftDirection.setter(&Vehicle::setWheelSpeedRearLeftDirection);

	WheelSpeedRearLeftValid.setContainer(this);
	WheelSpeedRearLeftValid.getter(&Vehicle::getWheelSpeedRearLeftValid);
	WheelSpeedRearLeftValid.setter(&Vehicle::setWheelSpeedRearLeftValid);

	WheelSpeedRearLeftData.setContainer(this);
	WheelSpeedRearLeftData.getter(&Vehicle::getWheelSpeedRearLeftData);
	WheelSpeedRearLeftData.setter(&Vehicle::setWheelSpeedRearLeftData);

	WheelSpeedRearRightDirection.setContainer(this);
	WheelSpeedRearRightDirection.getter(&Vehicle::getWheelSpeedRearRightDirection);
	WheelSpeedRearRightDirection.setter(&Vehicle::setWheelSpeedRearRightDirection);

	WheelSpeedRearRightValid.setContainer(this);
	WheelSpeedRearRightValid.getter(&Vehicle::getWheelSpeedRearRightValid);
	WheelSpeedRearRightValid.setter(&Vehicle::setWheelSpeedRearRightValid);

	WheelSpeedRearRightData.setContainer(this);
	WheelSpeedRearRightData.getter(&Vehicle::getWheelSpeedRearRightData);
	WheelSpeedRearRightData.setter(&Vehicle::setWheelSpeedRearRightData);

	WheelSpeedFrontLeftDirection.setContainer(this);
	WheelSpeedFrontLeftDirection.getter(&Vehicle::getWheelSpeedFrontLeftDirection);
	WheelSpeedFrontLeftDirection.setter(&Vehicle::setWheelSpeedFrontLeftDirection);

	WheelSpeedFrontLeftValid.setContainer(this);
	WheelSpeedFrontLeftValid.getter(&Vehicle::getWheelSpeedFrontLeftValid);
	WheelSpeedFrontLeftValid.setter(&Vehicle::setWheelSpeedFrontLeftValid);

	WheelSpeedFrontLeftData.setContainer(this);
	WheelSpeedFrontLeftData.getter(&Vehicle::getWheelSpeedFrontLeftData);
	WheelSpeedFrontLeftData.setter(&Vehicle::setWheelSpeedFrontLeftData);

	WheelSpeedFrontRightDirection.setContainer(this);
	WheelSpeedFrontRightDirection.getter(&Vehicle::getWheelSpeedFrontRightDirection);
	WheelSpeedFrontRightDirection.setter(&Vehicle::setWheelSpeedFrontRightDirection);

	WheelSpeedFrontRightValid.setContainer(this);
	WheelSpeedFrontRightValid.getter(&Vehicle::getWheelSpeedFrontRightValid);
	WheelSpeedFrontRightValid.setter(&Vehicle::setWheelSpeedFrontRightValid);

	WheelSpeedFrontRightData.setContainer(this);
	WheelSpeedFrontRightData.getter(&Vehicle::getWheelSpeedFrontRightData);
	WheelSpeedFrontRightData.setter(&Vehicle::setWheelSpeedFrontRightData);
	// Wheel Pusle
	WheelSpeedDirection.setContainer(this);
	WheelSpeedDirection.getter(&Vehicle::getWheelSpeedDirection);
	WheelSpeedDirection.setter(&Vehicle::setWheelSpeedDirection);

	WheelSpeedRearRightPulse.setContainer(this);
	WheelSpeedRearRightPulse.getter(&Vehicle::getWheelSpeedRearRightPulse);
	WheelSpeedRearRightPulse.setter(&Vehicle::setWheelSpeedRearRightPulse);

	WheelSpeedRearLeftPulse.setContainer(this);
	WheelSpeedRearLeftPulse.getter(&Vehicle::getWheelSpeedRearLeftPulse);
	WheelSpeedRearLeftPulse.setter(&Vehicle::setWheelSpeedRearLeftPulse);

	WheelSpeedFrontRightPulse.setContainer(this);
	WheelSpeedFrontRightPulse.getter(&Vehicle::getWheelSpeedFrontRightPulse);
	WheelSpeedFrontRightPulse.setter(&Vehicle::setWheelSpeedFrontRightPulse);

	WheelSpeedFrontLeftPulse.setContainer(this);
	WheelSpeedFrontLeftPulse.getter(&Vehicle::getWheelSpeedFrontLeftPulse);
	WheelSpeedFrontLeftPulse.setter(&Vehicle::setWheelSpeedFrontLeftPulse);
	/////////////vehicle speed
	VehicleSpeedValid.setContainer(this);
	VehicleSpeedValid.getter(&Vehicle::getVehicleSpeedValid);
	VehicleSpeedValid.setter(&Vehicle::setVehicleSpeedValid);

	VehicleSpeed.setContainer(this);
	VehicleSpeed.getter(&Vehicle::getVehicleSpeed);
	VehicleSpeed.setter(&Vehicle::setVehicleSpeed);

	////// Target Speed //////
	VehicleSpeedTarget.setContainer(this);
	VehicleSpeedTarget.getter(&Vehicle::getVehicleSpeedTarget);
	VehicleSpeedTarget.setter(&Vehicle::setVehicleSpeedTarget);

	VehicleSpeedControlEnable.setContainer(this);
	VehicleSpeedControlEnable.getter(&Vehicle::getVehicleSpeedControlEnable);
	VehicleSpeedControlEnable.setter(&Vehicle::setVehicleSpeedControlEnable);

	// SAS Steering Angle
	SteeringAngleActual.setContainer(this);
	SteeringAngleActual.getter(&Vehicle::getSteeringAngleActual);
	SteeringAngleActual.setter(&Vehicle::setSteeringAngleActual);

	SteeringAngleSpeed.setContainer(this);
	SteeringAngleSpeed.getter(&Vehicle::getSteeringAngleSpeed);
	SteeringAngleSpeed.setter(&Vehicle::setSteeringAngleSpeed);

	SteeringAngleValid.setContainer(this);
	SteeringAngleValid.getter(&Vehicle::getSteeringAngleValid);
	SteeringAngleValid.setter(&Vehicle::setSteeringAngleValid);

	SAS_Failure.setContainer(this);
	SAS_Failure.getter(&Vehicle::getSAS_Failure);
	SAS_Failure.setter(&Vehicle::setSAS_Failure);
	//ESP
	ESP_QDC_ACC.setContainer(this);
	ESP_QDC_ACC.getter(&Vehicle::getESP_QDC_ACC);
	ESP_QDC_ACC.setter(&Vehicle::setESP_QDC_ACC);
	//EMS
	EMS_QEC_ACC.setContainer(this);
	EMS_QEC_ACC.getter(&Vehicle::getEMS_QEC_ACC);
	EMS_QEC_ACC.setter(&Vehicle::setEMS_QEC_ACC);
}

Vehicle::Vehicle(float dt,float kp,float ki,float kd,float i_lim,float out_lim):PID(dt,kp,ki,kd,i_lim,out_lim)
{
	// TODO Auto-generated constructor stub
	_steering_angle_control_state = 0;
	/*** the vehicle body information ***/
	// Lenght
	_wheelbase_lenght = 2.65;
	_front_overhang_distance = 0.952;
	_rear_overhang_distance = 1;

	// width
	_wheel_axis_width = 1.794;
	_wheel_axis_width_half = 0.765;
	_wheel_edge_distance = 0.132;

	// the vehice four edge point calculate
	_front_axis_lenght = sqrtf(powf(_wheel_axis_width_half + _wheel_edge_distance , 2) + powf(_wheelbase_lenght + _front_overhang_distance ,2));
	_rear_axis_lenght  = sqrtf(powf(_wheel_axis_width_half + _wheel_edge_distance , 2) + powf(_rear_overhang_distance ,2));
	_beta_front        = atanf( (_wheel_axis_width_half + _wheel_edge_distance) / (_wheelbase_lenght + _front_overhang_distance));
	_beta_rear         = atanf( (_wheel_axis_width_half + _wheel_edge_distance) / _rear_overhang_distance );

	////// Vehicle Body Information //////
	/// Lenght
	WheelBaseLenght.setContainer(this);
	WheelBaseLenght.getter(&Vehicle::getWheelBaseLenght);
	WheelBaseLenght.setter(&Vehicle::setWheelBaseLenght);

	FrontOverhangDistance.setContainer(this);
	FrontOverhangDistance.getter(&Vehicle::getFrontOverhangDistance);
	FrontOverhangDistance.setter(&Vehicle::setFrontOverhangDistance);

	RearOverhangDistance.setContainer(this);
	RearOverhangDistance.getter(&Vehicle::getRearOverhangDistance);
	RearOverhangDistance.setter(&Vehicle::setRearOverhangDistance);
	/// Width
	WheelAxisWidth.setContainer(this);
	WheelAxisWidth.getter(&Vehicle::getWheelAxisWidth);
	WheelAxisWidth.setter(&Vehicle::setWheelAxisWidth);

	WheelAxisWidthHalf.setContainer(this);
	WheelAxisWidthHalf.getter(&Vehicle::getWheelAxisWidthHalf);
	WheelAxisWidthHalf.setter(&Vehicle::setWheelAxisWidthHalf);

	WheelEdgeDistance.setContainer(this);
	WheelEdgeDistance.getter(&Vehicle::getWheelEdgeDistance);
	WheelEdgeDistance.setter(&Vehicle::setWheelEdgeDistance);
	/// the vehicle edge point calculate information
	FrontAxisLenght.setContainer(this);
	FrontAxisLenght.getter(&Vehicle::getFrontAxisLenght);
	FrontAxisLenght.setter(&Vehicle::setFrontAxisLenght);

	RearAxisLenght.setContainer(this);
	RearAxisLenght.getter(&Vehicle::getRearAxisLenght);
	RearAxisLenght.setter(&Vehicle::setRearAxisLenght);

	BetaFront.setContainer(this);
	BetaFront.getter(&Vehicle::getBetaFront);
	BetaFront.setter(&Vehicle::setBetaFront);

	BetaRear.setContainer(this);
	BetaRear.getter(&Vehicle::getBetaRear);
	BetaRear.setter(&Vehicle::setBetaRear);
	////// ACC //////
	TargetAccelerationACC.setContainer(this);
	TargetAccelerationACC.getter(&Vehicle::getTargetAccelerationACC);
	TargetAccelerationACC.setter(&Vehicle::setTargetAccelerationACC);

	TargetAccelerationEnable.setContainer(this);
	TargetAccelerationEnable.getter(&Vehicle::getTargetAccelerationEnable);
	TargetAccelerationEnable.setter(&Vehicle::setTargetAccelerationEnable);

	////// AEB //////
	TargetDecelerationAEB.setContainer(this);
	TargetDecelerationAEB.getter(&Vehicle::getTargetDecelerationAEB);
	TargetDecelerationAEB.setter(&Vehicle::setTargetDecelerationAEB);

	TargetDecelerationEnable.setContainer(this);
	TargetDecelerationEnable.getter(&Vehicle::getTargetDecelerationEnable);
	TargetDecelerationEnable.setter(&Vehicle::setTargetDecelerationEnable);

	////// Torque //////
	Torque.setContainer(this);
	Torque.getter(&Vehicle::getTorque);
	Torque.setter(&Vehicle::setTorque);

	TorqueEnable.setContainer(this);
	TorqueEnable.getter(&Vehicle::getTorqueEnable);
	TorqueEnable.setter(&Vehicle::setTorqueEnable);

	////// Steering Angle //////
	SteeringAngleTarget.setContainer(this);
	SteeringAngleTarget.getter(&Vehicle::getSteeringAngleTarget);
	SteeringAngleTarget.setter(&Vehicle::setSteeringAngleTarget);

	SteeringAngleSpeedTarget.setContainer(this);
	SteeringAngleSpeedTarget.getter(&Vehicle::getSteeringAngleSpeedTarget);
	SteeringAngleSpeedTarget.setter(&Vehicle::setSteeringAngleSpeedTarget);

	SteeringAngleTargetActive.setContainer(this);
	SteeringAngleTargetActive.getter(&Vehicle::getSteeringAngleTargetActive);
	SteeringAngleTargetActive.setter(&Vehicle::setSteeringAngleTargetActive);

	////// Gear //////
	GearShift.setContainer(this);
	GearShift.getter(&Vehicle::getGearShift);
	GearShift.setter(&Vehicle::setGearShift);

	GearShiftEnable.setContainer(this);
	GearShiftEnable.getter(&Vehicle::getGearShiftEnable);
	GearShiftEnable.setter(&Vehicle::setGearShiftEnable);

	GearShiftValid.setContainer(this);
	GearShiftValid.getter(&Vehicle::getGearShiftValid);
	GearShiftValid.setter(&Vehicle::setGearShiftValid);

	/// Read Only
	// EPS
	EPS_Failed.setContainer(this);
	EPS_Failed.getter(&Vehicle::getEPS_Failed);
	EPS_Failed.setter(&Vehicle::setEPS_Failed);

	APA_EPAS_Failed.setContainer(this);
	APA_EPAS_Failed.getter(&Vehicle::getAPA_EPAS_Failed);
	APA_EPAS_Failed.setter(&Vehicle::setAPA_EPAS_Failed);

	APA_ControlFeedback.setContainer(this);
	APA_ControlFeedback.getter(&Vehicle::getAPA_ControlFeedback);
	APA_ControlFeedback.setter(&Vehicle::setAPA_ControlFeedback);

	TorqueSensorStatus.setContainer(this);
	TorqueSensorStatus.getter(&Vehicle::getTorqueSensorStatus);
	TorqueSensorStatus.setter(&Vehicle::setTorqueSensorStatus);

	SteeringTorque.setContainer(this);
	SteeringTorque.getter(&Vehicle::getSteeringTorque);
	SteeringTorque.setter(&Vehicle::setSteeringTorque);

	// Wheel Speed
	WheelSpeedRearLeftDirection.setContainer(this);
	WheelSpeedRearLeftDirection.getter(&Vehicle::getWheelSpeedRearLeftDirection);
	WheelSpeedRearLeftDirection.setter(&Vehicle::setWheelSpeedRearLeftDirection);

	WheelSpeedRearLeftValid.setContainer(this);
	WheelSpeedRearLeftValid.getter(&Vehicle::getWheelSpeedRearLeftValid);
	WheelSpeedRearLeftValid.setter(&Vehicle::setWheelSpeedRearLeftValid);

	WheelSpeedRearLeftData.setContainer(this);
	WheelSpeedRearLeftData.getter(&Vehicle::getWheelSpeedRearLeftData);
	WheelSpeedRearLeftData.setter(&Vehicle::setWheelSpeedRearLeftData);

	WheelSpeedRearRightDirection.setContainer(this);
	WheelSpeedRearRightDirection.getter(&Vehicle::getWheelSpeedRearRightDirection);
	WheelSpeedRearRightDirection.setter(&Vehicle::setWheelSpeedRearRightDirection);

	WheelSpeedRearRightValid.setContainer(this);
	WheelSpeedRearRightValid.getter(&Vehicle::getWheelSpeedRearRightValid);
	WheelSpeedRearRightValid.setter(&Vehicle::setWheelSpeedRearRightValid);

	WheelSpeedRearRightData.setContainer(this);
	WheelSpeedRearRightData.getter(&Vehicle::getWheelSpeedRearRightData);
	WheelSpeedRearRightData.setter(&Vehicle::setWheelSpeedRearRightData);

	WheelSpeedFrontLeftDirection.setContainer(this);
	WheelSpeedFrontLeftDirection.getter(&Vehicle::getWheelSpeedFrontLeftDirection);
	WheelSpeedFrontLeftDirection.setter(&Vehicle::setWheelSpeedFrontLeftDirection);

	WheelSpeedFrontLeftValid.setContainer(this);
	WheelSpeedFrontLeftValid.getter(&Vehicle::getWheelSpeedFrontLeftValid);
	WheelSpeedFrontLeftValid.setter(&Vehicle::setWheelSpeedFrontLeftValid);

	WheelSpeedFrontLeftData.setContainer(this);
	WheelSpeedFrontLeftData.getter(&Vehicle::getWheelSpeedFrontLeftData);
	WheelSpeedFrontLeftData.setter(&Vehicle::setWheelSpeedFrontLeftData);

	WheelSpeedFrontRightDirection.setContainer(this);
	WheelSpeedFrontRightDirection.getter(&Vehicle::getWheelSpeedFrontRightDirection);
	WheelSpeedFrontRightDirection.setter(&Vehicle::setWheelSpeedFrontRightDirection);

	WheelSpeedFrontRightValid.setContainer(this);
	WheelSpeedFrontRightValid.getter(&Vehicle::getWheelSpeedFrontRightValid);
	WheelSpeedFrontRightValid.setter(&Vehicle::setWheelSpeedFrontRightValid);

	WheelSpeedFrontRightData.setContainer(this);
	WheelSpeedFrontRightData.getter(&Vehicle::getWheelSpeedFrontRightData);
	WheelSpeedFrontRightData.setter(&Vehicle::setWheelSpeedFrontRightData);
	// Wheel Pusle
	WheelSpeedDirection.setContainer(this);
	WheelSpeedDirection.getter(&Vehicle::getWheelSpeedDirection);
	WheelSpeedDirection.setter(&Vehicle::setWheelSpeedDirection);

	WheelSpeedRearRightPulse.setContainer(this);
	WheelSpeedRearRightPulse.getter(&Vehicle::getWheelSpeedRearRightPulse);
	WheelSpeedRearRightPulse.setter(&Vehicle::setWheelSpeedRearRightPulse);

	WheelSpeedRearLeftPulse.setContainer(this);
	WheelSpeedRearLeftPulse.getter(&Vehicle::getWheelSpeedRearLeftPulse);
	WheelSpeedRearLeftPulse.setter(&Vehicle::setWheelSpeedRearLeftPulse);

	WheelSpeedFrontRightPulse.setContainer(this);
	WheelSpeedFrontRightPulse.getter(&Vehicle::getWheelSpeedFrontRightPulse);
	WheelSpeedFrontRightPulse.setter(&Vehicle::setWheelSpeedFrontRightPulse);

	WheelSpeedFrontLeftPulse.setContainer(this);
	WheelSpeedFrontLeftPulse.getter(&Vehicle::getWheelSpeedFrontLeftPulse);
	WheelSpeedFrontLeftPulse.setter(&Vehicle::setWheelSpeedFrontLeftPulse);
	/////////////vehicle speed
	VehicleSpeedValid.setContainer(this);
	VehicleSpeedValid.getter(&Vehicle::getVehicleSpeedValid);
	VehicleSpeedValid.setter(&Vehicle::setVehicleSpeedValid);

	VehicleSpeed.setContainer(this);
	VehicleSpeed.getter(&Vehicle::getVehicleSpeed);
	VehicleSpeed.setter(&Vehicle::setVehicleSpeed);

	////// Target Speed //////
	VehicleSpeedTarget.setContainer(this);
	VehicleSpeedTarget.getter(&Vehicle::getVehicleSpeedTarget);
	VehicleSpeedTarget.setter(&Vehicle::setVehicleSpeedTarget);

	VehicleSpeedControlEnable.setContainer(this);
	VehicleSpeedControlEnable.getter(&Vehicle::getVehicleSpeedControlEnable);
	VehicleSpeedControlEnable.setter(&Vehicle::setVehicleSpeedControlEnable);

	// SAS Steering Angle
	SteeringAngleActual.setContainer(this);
	SteeringAngleActual.getter(&Vehicle::getSteeringAngleActual);
	SteeringAngleActual.setter(&Vehicle::setSteeringAngleActual);

	SteeringAngleSpeed.setContainer(this);
	SteeringAngleSpeed.getter(&Vehicle::getSteeringAngleSpeed);
	SteeringAngleSpeed.setter(&Vehicle::setSteeringAngleSpeed);

	SteeringAngleValid.setContainer(this);
	SteeringAngleValid.getter(&Vehicle::getSteeringAngleValid);
	SteeringAngleValid.setter(&Vehicle::setSteeringAngleValid);

	SAS_Failure.setContainer(this);
	SAS_Failure.getter(&Vehicle::getSAS_Failure);
	SAS_Failure.setter(&Vehicle::setSAS_Failure);
	//ESP
	ESP_QDC_ACC.setContainer(this);
	ESP_QDC_ACC.getter(&Vehicle::getESP_QDC_ACC);
	ESP_QDC_ACC.setter(&Vehicle::setESP_QDC_ACC);
	//EMS
	EMS_QEC_ACC.setContainer(this);
	EMS_QEC_ACC.getter(&Vehicle::getEMS_QEC_ACC);
	EMS_QEC_ACC.setter(&Vehicle::setEMS_QEC_ACC);
}

Vehicle::Vehicle(float dt,float kp,float ki,float kd,float i_lim,float out_lim,float threshold):PID(dt,kp,ki,kd,i_lim,out_lim,threshold)
{
 	// TODO Auto-generated constructor stub
	_steering_angle_control_state = 0;
	/*** the vehicle body information ***/
	// Lenght
	_wheelbase_lenght = 2.65;
	_front_overhang_distance = 0.952;
	_rear_overhang_distance = 1;

	// width
	_wheel_axis_width = 1.794;
	_wheel_axis_width_half = 0.765;
	_wheel_edge_distance = 0.132;

	// the vehice four edge point calculate
	_front_axis_lenght = sqrtf(powf(_wheel_axis_width_half + _wheel_edge_distance , 2) + powf(_wheelbase_lenght + _front_overhang_distance ,2));
	_rear_axis_lenght  = sqrtf(powf(_wheel_axis_width_half + _wheel_edge_distance , 2) + powf(_rear_overhang_distance ,2));
	_beta_front        = atanf( (_wheel_axis_width_half + _wheel_edge_distance) / (_wheelbase_lenght + _front_overhang_distance));
	_beta_rear         = atanf( (_wheel_axis_width_half + _wheel_edge_distance) / _rear_overhang_distance );

	////// Vehicle Body Information //////
	/// Lenght
	WheelBaseLenght.setContainer(this);
	WheelBaseLenght.getter(&Vehicle::getWheelBaseLenght);
	WheelBaseLenght.setter(&Vehicle::setWheelBaseLenght);

	FrontOverhangDistance.setContainer(this);
	FrontOverhangDistance.getter(&Vehicle::getFrontOverhangDistance);
	FrontOverhangDistance.setter(&Vehicle::setFrontOverhangDistance);

	RearOverhangDistance.setContainer(this);
	RearOverhangDistance.getter(&Vehicle::getRearOverhangDistance);
	RearOverhangDistance.setter(&Vehicle::setRearOverhangDistance);
	/// Width
	WheelAxisWidth.setContainer(this);
	WheelAxisWidth.getter(&Vehicle::getWheelAxisWidth);
	WheelAxisWidth.setter(&Vehicle::setWheelAxisWidth);

	WheelAxisWidthHalf.setContainer(this);
	WheelAxisWidthHalf.getter(&Vehicle::getWheelAxisWidthHalf);
	WheelAxisWidthHalf.setter(&Vehicle::setWheelAxisWidthHalf);

	WheelEdgeDistance.setContainer(this);
	WheelEdgeDistance.getter(&Vehicle::getWheelEdgeDistance);
	WheelEdgeDistance.setter(&Vehicle::setWheelEdgeDistance);
	/// the vehicle edge point calculate information
	FrontAxisLenght.setContainer(this);
	FrontAxisLenght.getter(&Vehicle::getFrontAxisLenght);
	FrontAxisLenght.setter(&Vehicle::setFrontAxisLenght);

	RearAxisLenght.setContainer(this);
	RearAxisLenght.getter(&Vehicle::getRearAxisLenght);
	RearAxisLenght.setter(&Vehicle::setRearAxisLenght);

	BetaFront.setContainer(this);
	BetaFront.getter(&Vehicle::getBetaFront);
	BetaFront.setter(&Vehicle::setBetaFront);

	BetaRear.setContainer(this);
	BetaRear.getter(&Vehicle::getBetaRear);
	BetaRear.setter(&Vehicle::setBetaRear);
	////// ACC //////
	TargetAccelerationACC.setContainer(this);
	TargetAccelerationACC.getter(&Vehicle::getTargetAccelerationACC);
	TargetAccelerationACC.setter(&Vehicle::setTargetAccelerationACC);

	TargetAccelerationEnable.setContainer(this);
	TargetAccelerationEnable.getter(&Vehicle::getTargetAccelerationEnable);
	TargetAccelerationEnable.setter(&Vehicle::setTargetAccelerationEnable);

	////// AEB //////
	TargetDecelerationAEB.setContainer(this);
	TargetDecelerationAEB.getter(&Vehicle::getTargetDecelerationAEB);
	TargetDecelerationAEB.setter(&Vehicle::setTargetDecelerationAEB);

	TargetDecelerationEnable.setContainer(this);
	TargetDecelerationEnable.getter(&Vehicle::getTargetDecelerationEnable);
	TargetDecelerationEnable.setter(&Vehicle::setTargetDecelerationEnable);

	////// Torque //////
	Torque.setContainer(this);
	Torque.getter(&Vehicle::getTorque);
	Torque.setter(&Vehicle::setTorque);

	TorqueEnable.setContainer(this);
	TorqueEnable.getter(&Vehicle::getTorqueEnable);
	TorqueEnable.setter(&Vehicle::setTorqueEnable);

	////// Steering Angle //////
	SteeringAngleTarget.setContainer(this);
	SteeringAngleTarget.getter(&Vehicle::getSteeringAngleTarget);
	SteeringAngleTarget.setter(&Vehicle::setSteeringAngleTarget);

	SteeringAngleSpeedTarget.setContainer(this);
	SteeringAngleSpeedTarget.getter(&Vehicle::getSteeringAngleSpeedTarget);
	SteeringAngleSpeedTarget.setter(&Vehicle::setSteeringAngleSpeedTarget);

	SteeringAngleTargetActive.setContainer(this);
	SteeringAngleTargetActive.getter(&Vehicle::getSteeringAngleTargetActive);
	SteeringAngleTargetActive.setter(&Vehicle::setSteeringAngleTargetActive);

	////// Gear //////
	GearShift.setContainer(this);
	GearShift.getter(&Vehicle::getGearShift);
	GearShift.setter(&Vehicle::setGearShift);

	GearShiftEnable.setContainer(this);
	GearShiftEnable.getter(&Vehicle::getGearShiftEnable);
	GearShiftEnable.setter(&Vehicle::setGearShiftEnable);

	GearShiftValid.setContainer(this);
	GearShiftValid.getter(&Vehicle::getGearShiftValid);
	GearShiftValid.setter(&Vehicle::setGearShiftValid);

	/// Read Only
	// EPS
	EPS_Failed.setContainer(this);
	EPS_Failed.getter(&Vehicle::getEPS_Failed);
	EPS_Failed.setter(&Vehicle::setEPS_Failed);

	APA_EPAS_Failed.setContainer(this);
	APA_EPAS_Failed.getter(&Vehicle::getAPA_EPAS_Failed);
	APA_EPAS_Failed.setter(&Vehicle::setAPA_EPAS_Failed);

	APA_ControlFeedback.setContainer(this);
	APA_ControlFeedback.getter(&Vehicle::getAPA_ControlFeedback);
	APA_ControlFeedback.setter(&Vehicle::setAPA_ControlFeedback);

	TorqueSensorStatus.setContainer(this);
	TorqueSensorStatus.getter(&Vehicle::getTorqueSensorStatus);
	TorqueSensorStatus.setter(&Vehicle::setTorqueSensorStatus);

	SteeringTorque.setContainer(this);
	SteeringTorque.getter(&Vehicle::getSteeringTorque);
	SteeringTorque.setter(&Vehicle::setSteeringTorque);

	// Wheel Speed
	WheelSpeedRearLeftDirection.setContainer(this);
	WheelSpeedRearLeftDirection.getter(&Vehicle::getWheelSpeedRearLeftDirection);
	WheelSpeedRearLeftDirection.setter(&Vehicle::setWheelSpeedRearLeftDirection);

	WheelSpeedRearLeftValid.setContainer(this);
	WheelSpeedRearLeftValid.getter(&Vehicle::getWheelSpeedRearLeftValid);
	WheelSpeedRearLeftValid.setter(&Vehicle::setWheelSpeedRearLeftValid);

	WheelSpeedRearLeftData.setContainer(this);
	WheelSpeedRearLeftData.getter(&Vehicle::getWheelSpeedRearLeftData);
	WheelSpeedRearLeftData.setter(&Vehicle::setWheelSpeedRearLeftData);

	WheelSpeedRearRightDirection.setContainer(this);
	WheelSpeedRearRightDirection.getter(&Vehicle::getWheelSpeedRearRightDirection);
	WheelSpeedRearRightDirection.setter(&Vehicle::setWheelSpeedRearRightDirection);

	WheelSpeedRearRightValid.setContainer(this);
	WheelSpeedRearRightValid.getter(&Vehicle::getWheelSpeedRearRightValid);
	WheelSpeedRearRightValid.setter(&Vehicle::setWheelSpeedRearRightValid);

	WheelSpeedRearRightData.setContainer(this);
	WheelSpeedRearRightData.getter(&Vehicle::getWheelSpeedRearRightData);
	WheelSpeedRearRightData.setter(&Vehicle::setWheelSpeedRearRightData);

	WheelSpeedFrontLeftDirection.setContainer(this);
	WheelSpeedFrontLeftDirection.getter(&Vehicle::getWheelSpeedFrontLeftDirection);
	WheelSpeedFrontLeftDirection.setter(&Vehicle::setWheelSpeedFrontLeftDirection);

	WheelSpeedFrontLeftValid.setContainer(this);
	WheelSpeedFrontLeftValid.getter(&Vehicle::getWheelSpeedFrontLeftValid);
	WheelSpeedFrontLeftValid.setter(&Vehicle::setWheelSpeedFrontLeftValid);

	WheelSpeedFrontLeftData.setContainer(this);
	WheelSpeedFrontLeftData.getter(&Vehicle::getWheelSpeedFrontLeftData);
	WheelSpeedFrontLeftData.setter(&Vehicle::setWheelSpeedFrontLeftData);

	WheelSpeedFrontRightDirection.setContainer(this);
	WheelSpeedFrontRightDirection.getter(&Vehicle::getWheelSpeedFrontRightDirection);
	WheelSpeedFrontRightDirection.setter(&Vehicle::setWheelSpeedFrontRightDirection);

	WheelSpeedFrontRightValid.setContainer(this);
	WheelSpeedFrontRightValid.getter(&Vehicle::getWheelSpeedFrontRightValid);
	WheelSpeedFrontRightValid.setter(&Vehicle::setWheelSpeedFrontRightValid);

	WheelSpeedFrontRightData.setContainer(this);
	WheelSpeedFrontRightData.getter(&Vehicle::getWheelSpeedFrontRightData);
	WheelSpeedFrontRightData.setter(&Vehicle::setWheelSpeedFrontRightData);
	// Wheel Pusle
	WheelSpeedDirection.setContainer(this);
	WheelSpeedDirection.getter(&Vehicle::getWheelSpeedDirection);
	WheelSpeedDirection.setter(&Vehicle::setWheelSpeedDirection);

	WheelSpeedRearRightPulse.setContainer(this);
	WheelSpeedRearRightPulse.getter(&Vehicle::getWheelSpeedRearRightPulse);
	WheelSpeedRearRightPulse.setter(&Vehicle::setWheelSpeedRearRightPulse);

	WheelSpeedRearLeftPulse.setContainer(this);
	WheelSpeedRearLeftPulse.getter(&Vehicle::getWheelSpeedRearLeftPulse);
	WheelSpeedRearLeftPulse.setter(&Vehicle::setWheelSpeedRearLeftPulse);

	WheelSpeedFrontRightPulse.setContainer(this);
	WheelSpeedFrontRightPulse.getter(&Vehicle::getWheelSpeedFrontRightPulse);
	WheelSpeedFrontRightPulse.setter(&Vehicle::setWheelSpeedFrontRightPulse);

	WheelSpeedFrontLeftPulse.setContainer(this);
	WheelSpeedFrontLeftPulse.getter(&Vehicle::getWheelSpeedFrontLeftPulse);
	WheelSpeedFrontLeftPulse.setter(&Vehicle::setWheelSpeedFrontLeftPulse);
	/////////////vehicle speed
	VehicleSpeedValid.setContainer(this);
	VehicleSpeedValid.getter(&Vehicle::getVehicleSpeedValid);
	VehicleSpeedValid.setter(&Vehicle::setVehicleSpeedValid);

	VehicleSpeed.setContainer(this);
	VehicleSpeed.getter(&Vehicle::getVehicleSpeed);
	VehicleSpeed.setter(&Vehicle::setVehicleSpeed);

	////// Target Speed //////
	VehicleSpeedTarget.setContainer(this);
	VehicleSpeedTarget.getter(&Vehicle::getVehicleSpeedTarget);
	VehicleSpeedTarget.setter(&Vehicle::setVehicleSpeedTarget);

	VehicleSpeedControlEnable.setContainer(this);
	VehicleSpeedControlEnable.getter(&Vehicle::getVehicleSpeedControlEnable);
	VehicleSpeedControlEnable.setter(&Vehicle::setVehicleSpeedControlEnable);

	// SAS Steering Angle
	SteeringAngleActual.setContainer(this);
	SteeringAngleActual.getter(&Vehicle::getSteeringAngleActual);
	SteeringAngleActual.setter(&Vehicle::setSteeringAngleActual);

	SteeringAngleSpeed.setContainer(this);
	SteeringAngleSpeed.getter(&Vehicle::getSteeringAngleSpeed);
	SteeringAngleSpeed.setter(&Vehicle::setSteeringAngleSpeed);

	SteeringAngleValid.setContainer(this);
	SteeringAngleValid.getter(&Vehicle::getSteeringAngleValid);
	SteeringAngleValid.setter(&Vehicle::setSteeringAngleValid);

	SAS_Failure.setContainer(this);
	SAS_Failure.getter(&Vehicle::getSAS_Failure);
	SAS_Failure.setter(&Vehicle::setSAS_Failure);
	//ESP
	ESP_QDC_ACC.setContainer(this);
	ESP_QDC_ACC.getter(&Vehicle::getESP_QDC_ACC);
	ESP_QDC_ACC.setter(&Vehicle::setESP_QDC_ACC);
	//EMS
	EMS_QEC_ACC.setContainer(this);
	EMS_QEC_ACC.getter(&Vehicle::getEMS_QEC_ACC);
	EMS_QEC_ACC.setter(&Vehicle::setEMS_QEC_ACC);
 }

Vehicle::~Vehicle() {
	// TODO Auto-generated destructor stub
}

/****** Property ******/
/* the vehicle body information */
// Lenght
float Vehicle::getWheelBaseLenght()
{
	return _wheelbase_lenght;
}
void  Vehicle::setWheelBaseLenght(float value)
{
	_wheelbase_lenght = value;
}

float Vehicle::getFrontOverhangDistance()
{
	return _front_overhang_distance;
}
void  Vehicle::setFrontOverhangDistance(float value)
{
	_front_overhang_distance = value;
}

float Vehicle::getRearOverhangDistance()
{
	return _rear_overhang_distance;
}
void  Vehicle::setRearOverhangDistance(float value)
{
	_rear_overhang_distance = value;
}
// width
float Vehicle::getWheelAxisWidth()
{
	return _wheel_axis_width;
}
void  Vehicle::setWheelAxisWidth(float value)
{
	_wheel_axis_width = value;
}

float Vehicle::getWheelAxisWidthHalf()
{
	return _wheel_axis_width_half;
}
void  Vehicle::setWheelAxisWidthHalf(float value)
{
	_wheel_axis_width_half = value;
}

float Vehicle::getWheelEdgeDistance()
{
	return _wheel_edge_distance;
}
void  Vehicle::setWheelEdgeDistance(float value)
{
	_wheel_edge_distance = value;
}
// the vehice four edge point calculate
float Vehicle::getFrontAxisLenght()
{
	return _front_axis_lenght;
}
void  Vehicle::setFrontAxisLenght(float value)
{
	_front_axis_lenght = value;
}

float Vehicle::getRearAxisLenght()
{
	return _rear_axis_lenght;
}
void  Vehicle::setRearAxisLenght(float value)
{
	_rear_axis_lenght = value;
}

float Vehicle::getBetaFront()
{
	return _beta_front;
}
void  Vehicle::setBetaFront(float value)
{
	_beta_front = value;
}

float Vehicle::getBetaRear()
{
	return _beta_rear;
}
void  Vehicle::setBetaRear(float value)
{
	_beta_rear = value;
}
/// ACC
float Vehicle::getTargetAccelerationACC()
{
	return _target_acceleration_acc;
}
void  Vehicle::setTargetAccelerationACC(float value)
{
	_target_acceleration_acc = value;
}

uint8_t Vehicle::getTargetAccelerationEnable()
{
	return _target_acceleration_enable;
}
void    Vehicle::setTargetAccelerationEnable(uint8_t value)
{
	_target_acceleration_enable = value;
}

/// AEB
float Vehicle::getTargetDecelerationAEB()
{
	return _target_deceleration_aeb;
}
void  Vehicle::setTargetDecelerationAEB(float value)
{
	_target_deceleration_aeb = value;
}

uint8_t Vehicle::getTargetDecelerationEnable()
{
	return _target_deceleration_enable;
}
void    Vehicle::setTargetDecelerationEnable(uint8_t value)
{
	_target_deceleration_enable = value;
}

/// Torque
float Vehicle::getTorque()
{
	return _torque;
}
void  Vehicle::setTorque(float value)
{
	_torque = value;
}

uint8_t Vehicle::getTorqueEnable()
{
	return _torque_enable;
}
void    Vehicle::setTorqueEnable(uint8_t value)
{
	_torque_enable = value;
}

/// Steering Angle
float Vehicle::getSteeringAngleTarget()
{
	return _steering_angle_target;
}
void  Vehicle::setSteeringAngleTarget(float value)
{
	_steering_angle_target = value;
}

float Vehicle::getSteeringAngleSpeedTarget()
{
	return _steering_angle_speed_target;
}
void  Vehicle::setSteeringAngleSpeedTarget(float value)
{
	_steering_angle_speed_target = value;
}

uint8_t Vehicle::getSteeringAngleTargetActive()
{
	return _steering_angle_target_active;
}
void    Vehicle::setSteeringAngleTargetActive(uint8_t value)
{
	_steering_angle_target_active = value;
}

/// Gear
uint8_t Vehicle::getGearShift()
{
	return _gear_shift;
}
void    Vehicle::setGearShift(uint8_t value)
{
	_gear_shift = value;
}

uint8_t Vehicle::getGearShiftEnable()
{
	return _gear_shift_enable;
}
void    Vehicle::setGearShiftEnable(uint8_t value)
{
	_gear_shift_enable = value;
}

uint8_t Vehicle::getGearShiftValid()
{
	return _gear_shift_valid;
}
void    Vehicle::setGearShiftValid(uint8_t value)
{
	_gear_shift_valid = value;
}
/* The information from vehicle and read only */
/// EPS
uint8_t Vehicle::getEPS_Failed()
{
	return _eps_failed;
}
void    Vehicle::setEPS_Failed(uint8_t value)
{
	_eps_failed = value;
}

uint8_t Vehicle::getAPA_EPAS_Failed()
{
	return _apa_epas_failed;
}
void    Vehicle::setAPA_EPAS_Failed(uint8_t value)
{
	_apa_epas_failed = value;
}

uint8_t Vehicle::getAPA_ControlFeedback()
{
	return _apa_control_feedback;
}
void    Vehicle::setAPA_ControlFeedback(uint8_t value)
{
	_apa_control_feedback = value;
}

uint8_t Vehicle::getTorqueSensorStatus()
{
	return _torque_sensor_status;
}
void    Vehicle::setTorqueSensorStatus(uint8_t value)
{
	_torque_sensor_status = value;
}

float Vehicle::getSteeringTorque()
{
	return _steering_torque;
}
void  Vehicle::setSteeringTorque(float value)
{
	_steering_torque = value;
}
///////////////////////////Wheel Speed
uint8_t Vehicle::getWheelSpeedRearLeftDirection()
{
	return _wheel_speed_rear_left_direction;
}
void    Vehicle::setWheelSpeedRearLeftDirection(uint8_t value)
{
	_wheel_speed_rear_left_direction = value;
}

uint8_t Vehicle::getWheelSpeedRearLeftValid()
{
	return _wheel_speed_rear_left_valid;
}
void    Vehicle::setWheelSpeedRearLeftValid(uint8_t value)
{
	_wheel_speed_rear_left_valid = value;
}

float Vehicle::getWheelSpeedRearLeftData()
{
	return _wheel_speed_rear_left_data;
}
void  Vehicle::setWheelSpeedRearLeftData(float value)
{
	_wheel_speed_rear_left_data = value;
}

uint8_t Vehicle::getWheelSpeedRearRightDirection()
{
	return _wheel_speed_rear_right_direction;
}
void    Vehicle::setWheelSpeedRearRightDirection(uint8_t value)
{
	_wheel_speed_rear_right_direction = value;
}

uint8_t Vehicle::getWheelSpeedRearRightValid()
{
	return _wheel_speed_rear_right_valid;
}
void    Vehicle::setWheelSpeedRearRightValid(uint8_t value)
{
	_wheel_speed_rear_right_valid = value;
}

float Vehicle::getWheelSpeedRearRightData()
{
	return _wheel_speed_rear_right_data;
}
void  Vehicle::setWheelSpeedRearRightData(float value)
{
	_wheel_speed_rear_right_data = value;
}
//////////////////////////////////
uint8_t Vehicle::getWheelSpeedFrontLeftDirection()
{
	return _wheel_speed_front_left_direction;
}
void    Vehicle::setWheelSpeedFrontLeftDirection(uint8_t value)
{
	_wheel_speed_front_left_direction = value;
}

uint8_t Vehicle::getWheelSpeedFrontLeftValid()
{
	return _wheel_speed_front_left_valid;
}
void    Vehicle::setWheelSpeedFrontLeftValid(uint8_t value)
{
	_wheel_speed_front_left_valid = value;
}

float Vehicle::getWheelSpeedFrontLeftData()
{
	return _wheel_speed_front_left_data;
}
void  Vehicle::setWheelSpeedFrontLeftData(float value)
{
	_wheel_speed_front_left_data = value;
}

uint8_t Vehicle::getWheelSpeedFrontRightDirection()
{
	return _wheel_speed_front_right_direction;
}
void    Vehicle::setWheelSpeedFrontRightDirection(uint8_t value)
{
	_wheel_speed_front_right_direction = value;
}

uint8_t Vehicle::getWheelSpeedFrontRightValid()
{
	return _wheel_speed_front_right_valid;
}
void    Vehicle::setWheelSpeedFrontRightValid(uint8_t value)
{
	_wheel_speed_front_right_valid = value;
}

float Vehicle::getWheelSpeedFrontRightData()
{
	return _wheel_speed_front_right_data;
}
void  Vehicle::setWheelSpeedFrontRightData(float value)
{
	_wheel_speed_front_right_data = value;
}
//////////////////////////////////////////////////////////////////
//Speed vehicle
uint8_t Vehicle::getVehicleSpeedValid()
{
	return _vehicle_speed_valid;
}
void    Vehicle::setVehicleSpeedValid(uint8_t value)
{
	_vehicle_speed_valid = value;
}

float Vehicle::getVehicleSpeed()
{
	return _vehicle_speed;
}
void  Vehicle::setVehicleSpeed(float value)
{
	_vehicle_speed = value;
}

float Vehicle::getVehicleSpeedTarget()
{
	return _vehicle_speed_target;
}
void  Vehicle::setVehicleSpeedTarget(float value)
{
	_vehicle_speed_target = value;
}

uint8_t Vehicle::getVehicleSpeedControlEnable()
{
	return _vehicle_speed_control_enable;
}
void    Vehicle::setVehicleSpeedControlEnable(uint8_t value)
{
	_vehicle_speed_control_enable = value;
}
////////////////////////////////////////////////////////////////////
// Wheel Pusle
uint8_t Vehicle::getWheelSpeedDirection()
{
	return _wheel_speed_direction;
}
void    Vehicle::setWheelSpeedDirection(uint8_t value)
{
	_wheel_speed_direction = value;
}

uint8_t Vehicle::getWheelSpeedRearRightPulse()
{
	return _wheel_speed_rear_right_pulse;
}
void    Vehicle::setWheelSpeedRearRightPulse(uint8_t value)
{
	_wheel_speed_rear_right_pulse = value;
}

uint8_t Vehicle::getWheelSpeedRearLeftPulse()
{
	return _wheel_speed_rear_left_pulse;
}
void    Vehicle::setWheelSpeedRearLeftPulse(uint8_t value)
{
	_wheel_speed_rear_left_pulse = value;
}

uint8_t Vehicle::getWheelSpeedFrontRightPulse()
{
	return _wheel_speed_front_right_pulse;
}
void    Vehicle::setWheelSpeedFrontRightPulse(uint8_t value)
{
	_wheel_speed_front_right_pulse = value;
}

uint8_t Vehicle::getWheelSpeedFrontLeftPulse()
{
	return _wheel_speed_front_left_pulse;
}
void    Vehicle::setWheelSpeedFrontLeftPulse(uint8_t value)
{
	_wheel_speed_front_left_pulse = value;
}
//////////////////////////////////////////////////////////////////
// SAS Steering Angle
int16_t Vehicle::getSteeringAngleActual()
{
	return _steering_angle_actual;
}
void    Vehicle::setSteeringAngleActual(int16_t value)
{
	_steering_angle_actual = value;
}

uint16_t Vehicle::getSteeringAngleSpeed()
{
	return _steering_angle_speed;
}
void     Vehicle::setSteeringAngleSpeed(uint16_t value)
{
	_steering_angle_speed = value;
}

uint8_t Vehicle::getSteeringAngleValid()
{
	return _steering_angle_valid;
}
void    Vehicle::setSteeringAngleValid(uint8_t value)
{
	_steering_angle_valid = value;
}

uint8_t Vehicle::getSAS_Failure()
{
	return _sas_failure;
}
void    Vehicle::setSAS_Failure(uint8_t value)
{
	_sas_failure = value;
}

uint8_t Vehicle::getESP_QDC_ACC()
{
	return esp_qdc_acc;
}
void    Vehicle::setESP_QDC_ACC(uint8_t value)
{
	esp_qdc_acc = value;
}

uint8_t Vehicle::getEMS_QEC_ACC()
{
	return ems_qec_acc;
}
void    Vehicle::setEMS_QEC_ACC(uint8_t value)
{
	ems_qec_acc = value;
}
////////////////////////////////////////////////////////////
void Vehicle::VehicleContorlStep1()
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x6fe;
	m_CAN_Packet.length = 8;
	/// data buffer
	/* ACC */
	_current_target_acceleration_ACC = (uint8_t)((_target_acceleration_acc + 5.0)*20);
	_current_target_acceleration_enable_single = _target_acceleration_enable;
	/* AEB */
	_current_target_deceleration_AEB = (uint16_t)((_target_deceleration_aeb + 16.0) * 2000);
	_current_target_deceleration_enable_single = _target_deceleration_enable;
	/* Torque */
	_current_torque = (uint16_t)(_torque * 10.23);
	_current_torque_enable_single = _torque_enable;
	/* Steering Angle */
	_current_steering_angle_target = (int16_t)(_steering_angle_set * 10);
	_current_steering_angle_target_active_single = _steering_angle_target_active;

	/// Data Mapping
	m_CAN_Packet.data[0] = _current_target_acceleration_ACC;
	m_CAN_Packet.data[1] = (uint8_t)((_current_target_deceleration_AEB >> 8) & 0xFF);
	m_CAN_Packet.data[2] = (uint8_t)((_current_target_deceleration_AEB     ) & 0xFF);
	m_CAN_Packet.data[3] = (uint8_t)( _rolling_counter_torque_AEB & 0x0F);
	m_CAN_Packet.data[3] = _current_target_acceleration_enable_single ?
						   (uint8_t) ( m_CAN_Packet.data[3] | 0x80 ) :
						   (uint8_t) ( m_CAN_Packet.data[3] & 0x7F ) ;
	m_CAN_Packet.data[3] = _current_target_deceleration_enable_single ?
						   (uint8_t) ( m_CAN_Packet.data[3] | 0x40 ) :
						   (uint8_t) ( m_CAN_Packet.data[3] & 0xBF ) ;
	m_CAN_Packet.data[4] = (uint8_t)((_current_torque >> 2) & 0xFF);
	m_CAN_Packet.data[5] = (uint8_t)((_current_torque << 6) & 0xC0);
	m_CAN_Packet.data[5] = _current_torque_enable_single              ?
						   (uint8_t) ( m_CAN_Packet.data[5] | 0x20 ) :
						   (uint8_t) ( m_CAN_Packet.data[5] & 0xDF ) ;
	m_CAN_Packet.data[5] = (uint8_t) ((m_CAN_Packet.data[5] & 0xFC ) |
									  ( _current_steering_angle_target_active_single & 0x03));
	m_CAN_Packet.data[6] = (uint8_t)((_current_steering_angle_target >> 8) & 0xFF);
	m_CAN_Packet.data[7] = (uint8_t)((_current_steering_angle_target     ) & 0xFF);
	CAN0_TransmitMsg(m_CAN_Packet);
}

void Vehicle::VehicleContorlStep2()
{
	uint8_t i;
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x6FF;
	m_CAN_Packet.length = 8;
	/// data buffer
	_current_gear_shift = _gear_shift;
	_current_gear_shift_enable_single = _gear_shift_enable;
	_current_gear_shift_valid_single = _gear_shift_valid;

	/// data mapping
	for(i=0;i<4;i++){m_CAN_Packet.data[i] = 0;}
	m_CAN_Packet.data[4] = (uint8_t)(_rolling_counter_brake_ACC & 0x0f);
	m_CAN_Packet.data[5] = (uint8_t)((_current_gear_shift << 4 ) & 0x70);
	m_CAN_Packet.data[5] = _current_gear_shift_enable_single          ?
						   (uint8_t) ( m_CAN_Packet.data[5] | 0x80 ) :
						   (uint8_t) ( m_CAN_Packet.data[5] & 0x7F ) ;
	m_CAN_Packet.data[5] = _current_gear_shift_valid_single          ?
						   (uint8_t) ( m_CAN_Packet.data[5] | 0x08 ) :
						   (uint8_t) ( m_CAN_Packet.data[5] & 0xF7 ) ;

	m_CAN_Packet.data[6] = (uint8_t)(
								(
									( _rolling_counter_brake_ACC                 & 0x0f )
								+ 	((_current_target_acceleration_enable_single & 0x01 ) << 4)
								+ 	  _current_target_acceleration_ACC
								)^0xFF
							);
	m_CAN_Packet.data[7] = (uint8_t)(
								(
									( _rolling_counter_torque_AEB   & 0x0f)
								+	((_current_torque_enable_single & 0x01) << 2)
								+	((_current_torque & 0x001F) << 3)
								+	((_current_torque & 0x03E0) >> 5)
								+	((_current_target_deceleration_enable_single & 0x01) << 7)
								+	((_current_target_deceleration_AEB >> 8) & 0xFF)
								+	((_current_target_deceleration_AEB     ) & 0xFF)
								) ^ 0xFF
							);
	CAN0_TransmitMsg(m_CAN_Packet);
}

void Vehicle::VehicleContorlStep3()
{
	uint8_t i;
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x6E9;
	m_CAN_Packet.length = 8;

	/// data mapping
	m_CAN_Packet.data[0] = 0;
	m_CAN_Packet.data[1] = (uint8_t)(_rolling_counter_steering_control & 0x0f);
	m_CAN_Packet.data[2] = (uint8_t)	(
											(
												(_rolling_counter_steering_control & 0x0f)
											+	((_current_steering_angle_target >> 8) & 0xFF)
											+	((_current_steering_angle_target     ) & 0xFF)
											+	_current_steering_angle_target_active_single
											) ^ 0xFF
										);
	m_CAN_Packet.data[3] = (uint8_t)	(
											(
												(((_current_gear_shift_enable_single & 0x01) << 2)
												^(((_current_gear_shift_valid_single & 0x01) << 3) + _current_gear_shift)
												^(_rolling_counter_gear_control & 0x0F)) << 4
											)|(_rolling_counter_gear_control & 0x0F)
										);
	for( i = 4 ;i < 8 ; i++ ){m_CAN_Packet.data[i] = 0;}
	CAN0_TransmitMsg(m_CAN_Packet);
}

void Vehicle::VehicleContorl()
{
	VehicleContorlStep1();
	VehicleContorlStep2();
	VehicleContorlStep3();
	_rolling_counter_torque_AEB++;
	_rolling_counter_brake_ACC++;
	_rolling_counter_steering_control++;
	_rolling_counter_gear_control++;
}

void Vehicle::SteeringAngleControl(float dt)
{
    float da = _steering_angle_speed_target * dt;
    float left_target_angle = _steering_angle_target - da;
    float right_target_angle = _steering_angle_target + da;

    if(_steering_angle_set < left_target_angle)
    {
    	_steering_angle_set += da;
    }
    else if(_steering_angle_set > right_target_angle)
    {
    	_steering_angle_set -= da;
    }
    else
    {
    	_steering_angle_set = _steering_angle_target;
    }
}

void Vehicle::VehicleSpeedControl(float pid_output)
{
	_target_acceleration_acc = pid_output;
}

void Vehicle::SteeringAngleControlStateMachine()
{
	switch(_steering_angle_control_state)
	{
		case 0:
			if(!_apa_epas_failed)
			{
				_steering_angle_control_state = 1;
			}
			break;

		case 1:
			if(_apa_control_feedback)
			{
				_steering_angle_control_state = 2;
			}
			if(_apa_epas_failed)
			{
				_steering_angle_control_state = 0;
			}
			break;

		case 2:
			_steering_angle_target_active = 2;
			_steering_angle_control_state = 3;
			break;

		case 3:
			if(!_apa_control_feedback)
			{
				_steering_angle_control_state = 0;
			}
			if(_apa_epas_failed)
			{
				_steering_angle_control_state = 0;
			}
			break;

		default:
			_steering_angle_control_state = 0;
			break;
	}
}

void Vehicle::SteeringAngleControlStateMachineDelay()
{
	switch(_steering_angle_control_state)
	{
		case 0:
			if(_steering_angle_target_active == 1)
			{
				_steering_angle_control_state = 1;
				_steering_angle_active_cnt = 0;
			}
			break;

		case 1:
			if(_steering_angle_active_cnt >= 2)
			{
				_steering_angle_control_state = 2;
			}
			_steering_angle_active_cnt++;
			break;

		case 2:
			_steering_angle_target_active = 2;
			_steering_angle_control_state = 3;
			break;

		case 3:
			if(_steering_angle_target_active == 0)
			{
				_steering_angle_control_state = 0;
			}
			break;

		default:
			_steering_angle_control_state = 0;
			break;
	}
}

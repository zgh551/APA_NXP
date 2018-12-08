/*
 * Vehicle.h
 *
 *  Created on: 2018
 *      Author: zhuguohua
 */

#ifndef SRC_VEHICLE_H_
#define SRC_VEHICLE_H_
#include "Property.h"
#include "derivative.h"
#include "project.h"
#include "math.h"
#include "can.h"
#include "uart.h"
#include "PID.h"

#define V_KM_H 0.05625
#define V_M_S 0.015625

class Vehicle : public PID
{
public:
	/*** Function ***/
	Vehicle();
	Vehicle(float dt,float kp,float ki,float kd,float i_lim,float out_lim);
	Vehicle(float dt,float kp,float ki,float kd,float i_lim,float out_lim,float threshold);
	virtual ~Vehicle();

	/*** Function ***/
	// Vehicle control command function
	void VehicleContorlStep1();
	void VehicleContorlStep2();
	void VehicleContorlStep3();
	void VehicleContorl();

	// Steeing angle control base on the angle speed
	void SteeringAngleControl(float dt);
	// Vehicle Speed Control
	void VehicleSpeedControl(float pid_output);

	// Steering Angle control state machine
	void SteeringAngleControlStateMachine();

	/*** Variabel Property ***/
	/* the vehicle body information */
	// Lenght
	float getWheelBaseLenght();
	void  setWheelBaseLenght(float value);
	Property<Vehicle,float,READ_WRITE> WheelBaseLenght;

	float getFrontOverhangDistance();
	void  setFrontOverhangDistance(float value);
	Property<Vehicle,float,READ_WRITE> FrontOverhangDistance;

	float getRearOverhangDistance();
	void  setRearOverhangDistance(float value);
	Property<Vehicle,float,READ_WRITE> RearOverhangDistance;
	// width
	float getWheelAxisWidth();
	void  setWheelAxisWidth(float value);
	Property<Vehicle,float,READ_WRITE> WheelAxisWidth;

	float getWheelAxisWidthHalf();
	void  setWheelAxisWidthHalf(float value);
	Property<Vehicle,float,READ_WRITE> WheelAxisWidthHalf;

	float getWheelEdgeDistance();
	void  setWheelEdgeDistance(float value);
	Property<Vehicle,float,READ_WRITE> WheelEdgeDistance;
	// the vehice four edge point calculate
	float getFrontAxisLenght();
	void  setFrontAxisLenght(float value);
	Property<Vehicle,float,READ_WRITE> FrontAxisLenght;

	float getRearAxisLenght();
	void  setRearAxisLenght(float value);
	Property<Vehicle,float,READ_WRITE> RearAxisLenght;

	float getBetaFront();
	void  setBetaFront(float value);
	Property<Vehicle,float,READ_WRITE> BetaFront;

	float getBetaRear();
	void  setBetaRear(float value);
	Property<Vehicle,float,READ_WRITE> BetaRear;
	/* ACC */
	float getTargetAccelerationACC();
	void  setTargetAccelerationACC(float value);
	Property<Vehicle,float,READ_WRITE> TargetAccelerationACC;

	uint8_t getTargetAccelerationEnable();
	void    setTargetAccelerationEnable(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> TargetAccelerationEnable;

	/* AEB */
	float getTargetDecelerationAEB();
	void  setTargetDecelerationAEB(float value);
	Property<Vehicle,float,READ_WRITE> TargetDecelerationAEB;

	uint8_t getTargetDecelerationEnable();
	void    setTargetDecelerationEnable(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> TargetDecelerationEnable;

	/* Torque */
	float getTorque();
	void  setTorque(float value);
	Property<Vehicle,float,READ_WRITE> Torque;

	uint8_t getTorqueEnable();
	void    setTorqueEnable(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> TorqueEnable;

	/* Steering Angle */
	float getSteeringAngleTarget();
	void  setSteeringAngleTarget(float value);
	Property<Vehicle,float,READ_WRITE> SteeringAngleTarget;

	float getSteeringAngleSpeedTarget();
	void  setSteeringAngleSpeedTarget(float value);
	Property<Vehicle,float,READ_WRITE> SteeringAngleSpeedTarget;

	uint8_t getSteeringAngleTargetActive();
	void    setSteeringAngleTargetActive(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> SteeringAngleTargetActive;

	/* Gear */
	uint8_t getGearShift();
	void    setGearShift(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> GearShift;

	uint8_t getGearShiftEnable();
	void    setGearShiftEnable(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> GearShiftEnable;

	uint8_t getGearShiftValid();
	void    setGearShiftValid(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> GearShiftValid;
	/// Read only ///
	// EPS
	uint8_t getEPS_Failed();
	void    setEPS_Failed(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> EPS_Failed;

	uint8_t getAPA_EPAS_Failed();
	void    setAPA_EPAS_Failed(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> APA_EPAS_Failed;

	uint8_t getAPA_ControlFeedback();
	void    setAPA_ControlFeedback(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> APA_ControlFeedback;

	uint8_t getTorqueSensorStatus();
	void    setTorqueSensorStatus(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> TorqueSensorStatus;

	float getSteeringTorque();
	void  setSteeringTorque(float value);
	Property<Vehicle,float,READ_WRITE> SteeringTorque;

	// Wheel Speed
	/////////////////////////////////////////////
	uint8_t getWheelSpeedRearLeftDirection();
	void    setWheelSpeedRearLeftDirection(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedRearLeftDirection;

	uint8_t getWheelSpeedRearLeftValid();
	void    setWheelSpeedRearLeftValid(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedRearLeftValid;

	float getWheelSpeedRearLeftData();
	void  setWheelSpeedRearLeftData(float value);
	Property<Vehicle,float,READ_WRITE> WheelSpeedRearLeftData;
///////////////////////////////////////////
	uint8_t getWheelSpeedRearRightDirection();
	void    setWheelSpeedRearRightDirection(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedRearRightDirection;

	uint8_t getWheelSpeedRearRightValid();
	void    setWheelSpeedRearRightValid(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedRearRightValid;

	float getWheelSpeedRearRightData();
	void  setWheelSpeedRearRightData(float value);
	Property<Vehicle,float,READ_WRITE> WheelSpeedRearRightData;
////////////////////////////////////////////////////////////////////
	uint8_t getWheelSpeedFrontLeftDirection();
	void    setWheelSpeedFrontLeftDirection(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedFrontLeftDirection;

	uint8_t getWheelSpeedFrontLeftValid();
	void    setWheelSpeedFrontLeftValid(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedFrontLeftValid;

	float getWheelSpeedFrontLeftData();
	void  setWheelSpeedFrontLeftData(float value);
	Property<Vehicle,float,READ_WRITE> WheelSpeedFrontLeftData;
	//////////////////////////////////////////////////////////////////
	uint8_t getWheelSpeedFrontRightDirection();
	void    setWheelSpeedFrontRightDirection(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedFrontRightDirection;

	uint8_t getWheelSpeedFrontRightValid();
	void    setWheelSpeedFrontRightValid(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedFrontRightValid;

	float getWheelSpeedFrontRightData();
	void  setWheelSpeedFrontRightData(float value);
	Property<Vehicle,float,READ_WRITE> WheelSpeedFrontRightData;

	/// vehicle Speed
	uint8_t getVehicleSpeedValid();
	void    setVehicleSpeedValid(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> VehicleSpeedValid;

	float getVehicleSpeed();
	void  setVehicleSpeed(float value);
	Property<Vehicle,float,READ_WRITE> VehicleSpeed;

	/* Target Vehicle Speed */
	float getVehicleSpeedTarget();
	void  setVehicleSpeedTarget(float value);
	Property<Vehicle,float,READ_WRITE> VehicleSpeedTarget;

	uint8_t getVehicleSpeedControlEnable();
	void    setVehicleSpeedControlEnable(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> VehicleSpeedControlEnable;

	// wheel pulse
	uint8_t getWheelSpeedDirection();
	void    setWheelSpeedDirection(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedDirection;

	uint8_t getWheelSpeedRearRightPulse();
	void    setWheelSpeedRearRightPulse(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedRearRightPulse;

	uint8_t getWheelSpeedRearLeftPulse();
	void    setWheelSpeedRearLeftPulse(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedRearLeftPulse;

	uint8_t getWheelSpeedFrontRightPulse();
	void    setWheelSpeedFrontRightPulse(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedFrontRightPulse;

	uint8_t getWheelSpeedFrontLeftPulse();
	void    setWheelSpeedFrontLeftPulse(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> WheelSpeedFrontLeftPulse;

	// SAS Steering angle
	int16_t getSteeringAngleActual();
	void    setSteeringAngleActual(int16_t value);
	Property<Vehicle,int16_t,READ_WRITE> SteeringAngleActual;

	uint16_t getSteeringAngleSpeed();
	void     setSteeringAngleSpeed(uint16_t value);
	Property<Vehicle,uint16_t,READ_WRITE> SteeringAngleSpeed;

	uint8_t getSteeringAngleValid();
	void    setSteeringAngleValid(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> SteeringAngleValid;

	uint8_t getSAS_Failure();
	void    setSAS_Failure(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> SAS_Failure;

	// ESP
	uint8_t getESP_QDC_ACC();
	void    setESP_QDC_ACC(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> ESP_QDC_ACC;

	// EMS
	uint8_t getEMS_QEC_ACC();
	void    setEMS_QEC_ACC(uint8_t value);
	Property<Vehicle,uint8_t,READ_WRITE> EMS_QEC_ACC;
private:
	/*** the vehicle body information ***/
	// Lenght
	float _wheelbase_lenght;
	float _front_overhang_distance;
	float _rear_overhang_distance;

	// width
	float _wheel_axis_width;
	float _wheel_axis_width_half;
	float _wheel_edge_distance;

	// the vehice four edge point calculate
	float _front_axis_lenght;
	float _rear_axis_lenght;
	float _beta_front;
	float _beta_rear;

	/*** State Machine ***/
	/// steering angle control state machine
	uint8_t _steering_angle_Control_state;

	/*** Send to Vehicle Messege ***/
	/* Roolling Counter */
	uint8_t _rolling_counter_torque_AEB;
	uint8_t _rolling_counter_brake_ACC;
	uint8_t _rolling_counter_steering_control;
	uint8_t _rolling_counter_gear_control;

	/* ACC */
	// actual value
	float _target_acceleration_acc;
	uint8_t _target_acceleration_enable;
	// current value
	uint8_t _current_target_acceleration_ACC;
	uint8_t _current_target_acceleration_enable_single;

	/* AEB */
	// actual value
	float _target_deceleration_aeb;
	uint8_t _target_deceleration_enable;
	// current value
	uint16_t _current_target_deceleration_AEB;
	uint8_t _current_target_deceleration_enable_single;

	/* Torque */
	// actual value
	float _torque;
	uint8_t _torque_enable;
	// current value
	uint16_t _current_torque;
	uint8_t _current_torque_enable_single;

	/* SteeringAngle */
	// actual value
	float _steering_angle_set;
	float _steering_angle_target;
	float _steering_angle_speed_target;
	uint8_t _steering_angle_target_active;
	// current value
	int16_t _current_steering_angle_target;
	uint8_t _current_steering_angle_target_active_single;

	/* Gear */
	// actual value
	uint8_t _gear_shift;
	uint8_t _gear_shift_enable;
	uint8_t _gear_shift_valid;
	// current value
	uint8_t _current_gear_shift;
	uint8_t _current_gear_shift_enable_single;
	uint8_t _current_gear_shift_valid_single;

	/// Target Vehicle Speed
	float _vehicle_speed_target;
	uint8_t _vehicle_speed_control_enable;

	/*** Receive messege form vehicle ***/
	uint8_t _eps_failed;
	uint8_t _apa_epas_failed;
	uint8_t _apa_control_feedback;
	uint8_t _torque_sensor_status;
	float _steering_torque;

	//Wheel Speed
	uint8_t _wheel_speed_rear_left_direction;
	uint8_t _wheel_speed_rear_left_valid;
	float _wheel_speed_rear_left_data;

	uint8_t _wheel_speed_rear_right_direction;
	uint8_t _wheel_speed_rear_right_valid;
	float _wheel_speed_rear_right_data;

	uint8_t _wheel_speed_front_left_direction;
	uint8_t _wheel_speed_front_left_valid;
	float _wheel_speed_front_left_data;

	uint8_t _wheel_speed_front_right_direction;
	uint8_t _wheel_speed_front_right_valid;
	float _wheel_speed_front_right_data;

	// vehicle speed Feedback
	uint8_t _vehicle_speed_valid;
	float _vehicle_speed;

	// wheel pulse
	uint8_t _wheel_speed_direction;
	uint8_t _wheel_speed_rear_right_pulse;
	uint8_t _wheel_speed_rear_left_pulse;
	uint8_t _wheel_speed_front_right_pulse;
	uint8_t _wheel_speed_front_left_pulse;

	// SAS Steering angle
	int16_t _steering_angle_actual;
	uint16_t _steering_angle_speed;
	uint8_t _steering_angle_valid;
	uint8_t _sas_failure;

	// ESP
	uint8_t esp_qdc_acc;
	// EMS
	uint8_t ems_qec_acc;
};


#endif /* SRC_VEHICLE_H_ */

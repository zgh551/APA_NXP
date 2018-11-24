/*
 * Vehicle.h
 *
 *  Created on: 2018Äê11ÔÂ21ÈÕ
 *      Author: zhuguohua
 */

#ifndef SRC_VEHICLE_H_
#define SRC_VEHICLE_H_
#include "Property.h"
#include "derivative.h"
#include "project.h"
#include "can.h"
#include "uart.h"

class Vehicle {
public:
	/*** Function ***/
	Vehicle();
	virtual ~Vehicle();

	/*** Variabel Property ***/
	/* ACC */
	float getTargetAccelerationACC();
	void setTargetAccelerationACC(float value);
	Property<Vehicle,float,READ_WRITE> TargetAccelerationACC;

	vuint8_t getTargetAccelerationEnable();
	void setTargetAccelerationEnable(vuint8_t value);
	Property<Vehicle,vuint8_t,READ_WRITE> TargetAccelerationEnable;

	/* AEB */
	float getTargetDecelerationAEB();
	void setTargetDecelerationAEB(float value);
	Property<Vehicle,float,READ_WRITE> TargetDecelerationAEB;

	vuint8_t getTargetDecelerationEnable();
	void setTargetDecelerationEnable(vuint8_t value);
	Property<Vehicle,vuint8_t,READ_WRITE> TargetDecelerationEnable;

	/* Torque */
	vuint8_t getTorque();
	void setTorque(vuint8_t value);
	Property<Vehicle,vuint8_t,READ_WRITE> Torque;

	vuint8_t getTorqueEnable();
	void setTorqueEnable(vuint8_t value);
	Property<Vehicle,vuint8_t,READ_WRITE> TorqueEnable;

	/* Steering Angle */
	vint16_t getSteeringAngleTarget();
	void setSteeringAngleTarget(vint16_t value);
	Property<Vehicle,vint16_t,READ_WRITE> SteeringAngleTarget;

	vuint8_t getSteeringAngleTargetActive();
	void setSteeringAngleTargetActive(vuint8_t value);
	Property<Vehicle,vuint8_t,READ_WRITE> SteeringAngleTargetActive;

	/* Gear */
	vuint8_t getGearShift();
	void setGearShift(vuint8_t value);
	Property<Vehicle,vuint8_t,READ_WRITE> GearShift;

	vuint8_t getGearShiftEnable();
	void setGearShiftEnable(vuint8_t value);
	Property<Vehicle,vuint8_t,READ_WRITE> GearShiftEnable;

	vuint8_t getGearShiftValid();
	void setGearShiftValid(vuint8_t value);
	Property<Vehicle,vuint8_t,READ_WRITE> GearShiftValid;

	/// Read only ///
	// EPS
	vuint8_t getEPS_Failed();
	Property<Vehicle,vuint8_t,READ_ONLY> EPS_Failed;

	vuint8_t getAPA_EPAS_Failed();
	Property<Vehicle,vuint8_t,READ_ONLY> APA_EPAS_Failed;

	vuint8_t getAPA_ControlFeedback();
	Property<Vehicle,vuint8_t,READ_ONLY> APA_ControlFeedback;

	vuint8_t getTorqueSensorStatus();
	Property<Vehicle,vuint8_t,READ_ONLY> TorqueSensorStatus;

	float getSteeringTorque();
	Property<Vehicle,float,READ_ONLY> SteeringTorque;

	// Wheel Speed
	/////////////////////////////////////////////
	vuint8_t getWheelSpeedRearLeftDirection();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedRearLeftDirection;

	vuint8_t getWheelSpeedRearLeftValid();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedRearLeftValid;

	float getWheelSpeedRearLeftData();
	Property<Vehicle,float,READ_ONLY> WheelSpeedRearLeftData;
///////////////////////////////////////////
	vuint8_t getWheelSpeedRearRightDirection();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedRearRightDirection;

	vuint8_t getWheelSpeedRearRightValid();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedRearRightValid;

	float getWheelSpeedRearRightData();
	Property<Vehicle,float,READ_ONLY> WheelSpeedRearRightData;
////////////////////////////////////////////////////////////////////
	vuint8_t getWheelSpeedFrontLeftDirection();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedFrontLeftDirection;

	vuint8_t getWheelSpeedFrontLeftValid();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedFrontLeftValid;

	float getWheelSpeedFrontLeftData();
	Property<Vehicle,float,READ_ONLY> WheelSpeedFrontLeftData;
	//////////////////////////////////////////////////////////////////
	vuint8_t getWheelSpeedFrontRightDirection();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedFrontRightDirection;

	vuint8_t getWheelSpeedFrontRightValid();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedFrontRightValid;

	float getWheelSpeedFrontRightData();
	Property<Vehicle,float,READ_ONLY> WheelSpeedFrontRightData;

	/// vehicle Speed
	vuint8_t getVehicleSpeedValid();
	Property<Vehicle,vuint8_t,READ_ONLY> VehicleSpeedValid;

	float getVehicleSpeed();
	Property<Vehicle,float,READ_ONLY> VehicleSpeed;

	// wheel pulse
	vuint8_t getWheelSpeedDirection();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedDirection;

	vuint8_t getWheelSpeedRearRightPulse();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedRearRightPulse;

	vuint8_t getWheelSpeedRearLeftPulse();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedRearLeftPulse;

	vuint8_t getWheelSpeedFrontRightPulse();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedFrontRightPulse;

	vuint8_t getWheelSpeedForntLeftPulse();
	Property<Vehicle,vuint8_t,READ_ONLY> WheelSpeedForntLeftPulse;
	// SAS Steering angle
	vint16_t getSteeringAngleActual();
	Property<Vehicle,vint16_t,READ_ONLY> SteeringAngleActual;
	vuint16_t getSteeringAngleSpeed();
	Property<Vehicle,vuint16_t,READ_ONLY> SteeringAngleSpeed;
	vuint8_t getSteeringAngleValid();
	Property<Vehicle,vuint8_t,READ_ONLY> SteeringAngleValid;
	vuint8_t getSAS_Failure();
	Property<Vehicle,vuint8_t,READ_ONLY> SAS_Failure;

	// ESP
	vuint8_t getESP_QDC_ACC();
	Property<Vehicle,vuint8_t,READ_ONLY> ESP_QDC_ACC;	

	// EMS
	vuint8_t getEMS_QEC_ACC();
	Property<Vehicle,vuint8_t,READ_ONLY> EMS_QEC_ACC;

	/*** Function ***/

	// Vehicle control command function
	void VehicleContorlStep1();
	void VehicleContorlStep2();
	void VehicleContorlStep3();
	void VehicleContorl();

	void SteeringAngleControl(float dt);
	// Steering Angle control state machine
	void SteeringAngleControlStateMachine();

	// Vehicle information receive
	void VehicleInformation(CAN_MB_tag mb_msg);
	void VehicleInformation(vuint32_t id,vuint8_t dat[]);

	// Terminal control
	void TerminalControlCommandReceive(vuint8_t data);

	void TerminalControlCommandSend(void);
private:
	/*** State Machine ***/
	vuint8_t _steering_angle_Control_state;
	ReceiveFrame _terminal_frame;
	vuint8_t _data_buffer[16];
	vuint8_t _send_data_buffer[8];
	vuint8_t _frame_cnt,_check_sum;
	Byte2Float _data_temp;
	/*** Send to Vehicle Messege ***/
	/* Roolling Counter */
	vuint8_t _rolling_counter_torque_AEB;
	vuint8_t _rolling_counter_brake_ACC;
	vuint8_t _rolling_counter_steering_control;
	vuint8_t _rolling_counter_gear_control;

	/* ACC */
	// actual value
	float _target_acceleration_acc;
	vuint8_t _target_acceleration_enable;
	// current value
	vuint8_t _current_target_acceleration_ACC;
	vuint8_t _current_target_acceleration_enable_single;

	/* AEB */
	// actual value
	float _target_deceleration_aeb;
	vuint8_t _target_deceleration_enable;
	// current value
	vuint16_t _current_target_deceleration_AEB;
	vuint8_t _current_target_deceleration_enable_single;

	/* Torque */
	// actual value
	vuint8_t _torque;
	vuint8_t _torque_enable;
	// current value
	vuint16_t _current_torque;
	vuint8_t _current_torque_enable_single;

	/* SteeringAngle */
	// actual value
	float _steering_angle_set;
	vint16_t _steering_angle_target;
	vuint16_t _steering_angle_speed_target;
	vuint8_t _steering_angle_target_active;
	// current value
	vint16_t _current_steering_angle_target;
	vuint8_t _current_steering_angle_target_active_single;

	/* Gear */
	// actual value
	vuint8_t _gear_shift;
	vuint8_t _gear_shift_enable;
	vuint8_t _gear_shift_valid;
	// current value
	vuint8_t _current_gear_shift;
	vuint8_t _current_gear_shift_enable_single;
	vuint8_t _current_gear_shift_valid_single;

	/*** Receive messege form vehicle ***/
	vuint8_t _eps_failed;
	vuint8_t _apa_epas_failed;
	vuint8_t _apa_control_feedback;
	vuint8_t _torque_sensor_status;
	float _steering_torque;

	//Wheel Speed
	vuint8_t _wheel_speed_rear_left_direction;
	vuint8_t _wheel_speed_rear_left_valid;
	float _wheel_speed_rear_left_data;

	vuint8_t _wheel_speed_rear_right_direction;
	vuint8_t _wheel_speed_rear_right_valid;
	float _wheel_speed_rear_right_data;

	vuint8_t _wheel_speed_front_left_direction;
	vuint8_t _wheel_speed_front_left_valid;
	float _wheel_speed_front_left_data;

	vuint8_t _wheel_speed_front_right_direction;
	vuint8_t _wheel_speed_front_right_valid;
	float _wheel_speed_front_right_data;

	// vehicle speed
	vuint8_t _vehicle_speed_valid;
	float _vehicle_speed;

	// wheel pulse
	vuint8_t _wheel_speed_direction;
	vuint8_t _wheel_speed_rear_right_pulse;
	vuint8_t _wheel_speed_rear_left_pulse;
	vuint8_t _wheel_speed_front_right_pulse;
	vuint8_t _wheel_speed_front_left_pulse;

	// SAS Steering angle
	vint16_t _steering_angle_actual;
	vuint16_t _steering_angle_speed;
	vuint8_t _steering_angle_valid;
	vuint8_t _sas_failure;

	// ESP
	vuint8_t esp_qdc_acc;
	// EMS
	vuint8_t ems_qec_acc;
};


#endif /* SRC_VEHICLE_H_ */

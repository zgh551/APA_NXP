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

#include "../../../Driver/System/derivative.h"
#include "../../../Common/Utils/Inc/property.h"
#include "../../../Common/Configure/Configs/vehicle_config.h"

typedef enum _GearStatus
{
	None = 0,
	Parking,
	Reverse,
	Neutral,
	Drive
}GearStatus;

typedef enum _DirectStatus
{
	StandStill = 0,
	Forward,
	Backward,
	Invalid
}DirectStatus;

typedef enum _ActuatorStatus
{
	ActuatorNormal = 0,
	ActuatorErr
}ActuatorStatus;

typedef enum _ValidStatus
{
	DataValid = 0,
	DataInvalid
}ValidStatus;

typedef enum _ReadyStatus
{
	NoReady = 0,
	Ready
}ReadyStatus;

typedef enum _SpeedStatus
{
	SpeedNormal = 0,
	SpeedAbnormal
}SpeedStatus;

typedef enum _BeltStatus
{
	Fasten = 0,
	Unfasten
}BeltStatus;

typedef enum _LightStatus
{
	Off = 0,
	On
}LightStatus;

typedef enum _DoorStatus
{
	Close = 0,
	Open
}DoorStatus;

typedef enum _ControlStatus
{
	Disable = 0,
	Enable
}ControlStatus;

typedef enum _EPB_Status
{
	EPB_NoRequst = 0,
	EPB_Lock,
	EPB_Release,
	EPB_InvalidValue
}EPB_Status;

typedef enum _EPB_SystemStatus
{
	EPB_SystemReleased = 0,
	EPB_SystemApplied,
	EPB_SystemReleasing,
	EPB_SystemFault,
	EPB_SystemApplying,
	EPB_SystemDisenaged
}EPB_SystemStatus;


typedef enum _DriverMode
{
	ManualMode = 0,
	AutoMode
}DriverMode;

typedef enum _ManualDetecte
{
	NoManualControlDetected = 0,
	ManualControlDetected
}ManualDetected;

class MessageManager {
public:
	MessageManager();
	virtual ~MessageManager();

	virtual void Init() = 0;
	virtual void Parse(const uint32_t id,const vuint8_t *data,const vuint32_t lenght) = 0;
	
	ReadyStatus getSystemReadyStatus(void) 				{ return _system_ready_sts; }
	void        setSystemReadyStatus(ReadyStatus value) { _system_ready_sts = value;}

	DriverMode getAutoDriverModeStatus(void) 			{ return _auto_driver_mode_sts; }
	void       setAutoDriverModeStatus(DriverMode value){ _auto_driver_mode_sts = value;}

	DriverMode getEpsAutoDriverModeStatus(void) 			{ return _eps_auto_driver_mode_sts; }
	void       setEpsAutoDriverModeStatus(DriverMode value) { _eps_auto_driver_mode_sts = value;}
	/*** ESC ESP ***/
	ActuatorStatus getESC_Status();
	void           setESC_Status(ActuatorStatus value);
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

	ValidStatus getWheelSpeedFrontLeftValid(void)              	{ return _wheel_speed_front_left_valid ; }
	void        setWheelSpeedFrontLeftValid(ValidStatus value) 	{ _wheel_speed_front_left_valid = value; }

	ValidStatus getWheelSpeedFrontRightValid(void)        		{ return _wheel_speed_front_right_valid ; }
	void  		setWheelSpeedFrontRightValid(ValidStatus value) { _wheel_speed_front_right_valid = value; }

	ValidStatus getWheelSpeedRearLeftValid(void)        	  	{ return _wheel_speed_rear_left_valid ; }
	void  		setWheelSpeedRearLeftValid(ValidStatus value) 	{ _wheel_speed_rear_left_valid = value; }

	ValidStatus getWheelSpeedRearRightValid(void)        	   	{ return _wheel_speed_rear_right_valid ; }
	void  		setWheelSpeedRearRightValid(ValidStatus value) 	{ _wheel_speed_rear_right_valid = value; }
	// the middle speed of the vehicle
	float getVehicleMiddleSpeed();
	void  setVehicleMiddleSpeed(float value);
	Property<MessageManager,float,READ_WRITE> VehicleMiddleSpeed;

	// the middle speed valid single
	uint8_t getVehicleMiddleSpeedValid();
	void    setVehicleMiddleSpeedValid(uint8_t value);
	Property<MessageManager,uint8_t,READ_WRITE> VehicleMiddleSpeedValid;

	// the middle speed abnormal single
	SpeedStatus getVehicleMiddleSpeedAbnormal();
	void        setVehicleMiddleSpeedAbnormal(SpeedStatus value);

	// wheel speed direction
	DirectStatus getWheelSpeedDirection();
	void         setWheelSpeedDirection(DirectStatus value);
	Property<MessageManager,DirectStatus,READ_WRITE> WheelSpeedDirection;

	// wheel pulse
	uint16_t getWheelPulseFrontLeft();
	void     setWheelPulseFrontLeft(uint16_t value);
	Property<MessageManager,uint16_t,READ_WRITE> WheelPulseFrontLeft;

	uint16_t getWheelPulseFrontRight();
	void     setWheelPulseFrontRight(uint16_t value);
	Property<MessageManager,uint16_t,READ_WRITE> WheelPulseFrontRight;

	uint16_t getWheelPulseRearRight();
	void     setWheelPulseRearRight(uint16_t value);
	Property<MessageManager,uint16_t,READ_WRITE> WheelPulseRearRight;

	uint16_t getWheelPulseRearLeft();
	void     setWheelPulseRearLeft(uint16_t value);
	Property<MessageManager,uint16_t,READ_WRITE> WheelPulseRearLeft;

	ValidStatus getWheelPulseFrontLeftValid(void)				{ return _wheel_pulse_front_left_valid; }
	void        setWheelPulseFrontLeftValid(ValidStatus value) 	{ _wheel_pulse_front_left_valid = value;}

	ValidStatus getWheelPulseFrontRightValid(void)				{ return _wheel_pulse_front_right_valid; }
	void        setWheelPulseFrontRightValid(ValidStatus value) { _wheel_pulse_front_right_valid = value;}

	ValidStatus getWheelPulseRearLeftValid(void)				{ return _wheel_pulse_rear_left_valid; }
	void        setWheelPulseRearLeftValid(ValidStatus value)   { _wheel_pulse_rear_left_valid = value;}

	ValidStatus getWheelPulseRearRightValid(void)				{ return _wheel_pulse_rear_right_valid; }
	void        setWheelPulseRearRightValid(ValidStatus value)  { _wheel_pulse_rear_right_valid = value;}

	int32_t getRearLeftSumPulse(void)			{ return  _sum_rear_left_pulse;}
	void    setRearLeftSumPulse(int32_t value) 	{ _sum_rear_left_pulse = value;}

	int32_t getRearRightSumPulse(void)			{ return  _sum_rear_right_pulse; }
	void    setRearRightSumPulse(int32_t value)	{ _sum_rear_right_pulse = value; }

	int32_t getWheelSumPulse(void)          { return _wheel_sum_pulse;}
	void    setWheelSumPulse(int32_t value)	{_wheel_sum_pulse = value;}

	// wheel pulse dirction
	DirectStatus getWheelPulseDirection();
	void         setWheelPulseDirection(DirectStatus value);
	Property<MessageManager,DirectStatus,READ_WRITE> WheelPulseDirection;

	// IMU Sensor
	ValidStatus getYawRateValid(void) 				{ return _yaw_rate_valid; }
	void        setYawRateValid(ValidStatus value)	{ _yaw_rate_valid = value;}

	ValidStatus getLonAccValid(void) 				{ return _lon_acc_valid; }
	void        setLonAccValid(ValidStatus value)	{ _lon_acc_valid = value;}

	ValidStatus getLatAccValid(void) 				{ return _lat_acc_valid; }
	void        setLatAccValid(ValidStatus value)	{ _lat_acc_valid = value;}

	float getYawRate();
	void  setYawRate(float value);
	Property<MessageManager,float,READ_WRITE> YawRate;

	float getLonAcc();
	void  setLonAcc(float value);
	Property<MessageManager,float,READ_WRITE> LonAcc;

	float getLatAcc();
	void  setLatAcc(float value);
	Property<MessageManager,float,READ_WRITE> LatAcc;

	/*** EPS ***/
	ActuatorStatus getEPS_Status();
	void           setEPS_Status(ActuatorStatus value);

	ManualDetected getEPS_ManualControlDetectionStatus(void) 				{ return _eps_manual_control_detection_sts; }
	void           setEPS_ManualControlDetectionStatus(ManualDetected value){ _eps_manual_control_detection_sts = value;}

	ControlStatus getEPS_RequestFeedback(void) 				  { return  _eps_request_feedback; }
	void 		  setEPS_RequestFeedback(ControlStatus value) { _eps_request_feedback = value; }

	uint8_t getEPS_AbortFeedback(void) 				  { return  _eps_abort_feedback; }
	void 	setEPS_AbortFeedback(uint8_t value) { _eps_abort_feedback = value; }

	/*** SAS ***/
	ActuatorStatus getSAS_Status() 						{ return  _sas_status;}
	void           setSAS_Status(ActuatorStatus value) 	{ _sas_status = value;}
	// Steering angle
	float getSteeringAngle();
	void  setSteeringAngle(float value);
	Property<MessageManager,float,READ_WRITE> SteeringAngle;

	float getSteeringAngleRate();
	void  setSteeringAngleRate(float value);
	Property<MessageManager,float,READ_WRITE> SteeringAngleRate;

	/*** EMS ***/
	ActuatorStatus getEMS_Status(void)					{ return  _ems_status;}
	void           setEMS_Status(ActuatorStatus value) 	{ _ems_status = value;}
	/*** TCU ***/
	ActuatorStatus getTCU_Status(void)					{ return  _tcu_status;}
	void           setTCU_Status(ActuatorStatus value) 	{ _tcu_status = value;}

	ValidStatus getTargetGearValid(void)             { return _target_gear_valid ; }
	void        setTargetGearValid(ValidStatus value){ _target_gear_valid = value; }

	ValidStatus getActualGearValid(void)             { return _actual_gear_valid ; }
	void        setActualGearValid(ValidStatus value){ _actual_gear_valid = value; }

	GearStatus getTargetGear(void)            { return _target_gear ; }
	void       setTargetGear(GearStatus value){ _target_gear = value; }

	GearStatus getActualGear(void)				{ return  _actual_gear; }
	void       setActualGear(GearStatus value)	{ _actual_gear = value; }
	/*** VCU ***/
	ValidStatus getVCU_Status(void)		         { return _vcu_status ; }
	void        setVCU_Status(ValidStatus value) { _vcu_status = value; }

	float getAccPedalStroke(void)		{ return _acc_pedal_stroke; }
	void  setAccPedalStroke(float value){ _acc_pedal_stroke = value; }

	ValidStatus getAccPedalValid(void)		        { return _acc_pedal_valid ; }
	void        setAccPedalValid(ValidStatus value) { _acc_pedal_valid = value; }

	uint8_t getBrakePedalSts(void) 		    { return _brake_pedal_sts; }
	void    setBrakePedalSts(uint8_t value) { _brake_pedal_sts = value; }

	ValidStatus getBrakePedalValid(void) 		      { return _brake_pedal_valid; }
	void        setBrakePedalValid(ValidStatus value) { _brake_pedal_valid = value; }

	uint8_t getBrakePressure();
	void  setBrakePressure(uint8_t value);
	Property<MessageManager,uint8_t,READ_WRITE> BrakePressure;

	float getVehicleGradient(void)        { return _vehicle_gradient;  }
	void  setVehicleGradient(float value) { _vehicle_gradient = value; }

	/*** AC ***/
	float getAmbientTemperature(void) 	     { return _ambient_temperature;  }
	void  setAmbientTemperature(float value) { _ambient_temperature = value; }

	uint8_t getAmbientTemperatureValid(void)          { return _ambient_temperature_valid; }
	void    setAmbientTemperatureValid(uint8_t value) { _ambient_temperature_valid = value;}

	/*** EPB ***/
	ActuatorStatus getEPB_Status(void)					{ return _epb_status; }
	void           setEPB_Status(ActuatorStatus value) 	{ _epb_status = value;}

	EPB_SystemStatus getEPB_SystemStatus(void)						{ return _epb_system_sts; }
	void             setEPB_SystemStatus(EPB_SystemStatus value) 	{ _epb_system_sts = value;}

	EPB_Status getEPB_SwitchPosition(void)				{ return _epb_switch_position; }
	void       setEPB_SwitchPosition(EPB_Status value) 	{ _epb_switch_position = value;}

	ValidStatus getEPB_SwitchPositionValid(void)			  { return _epb_switch_position_valid; }
	void        setEPB_SwitchPositionValid(ValidStatus value) { _epb_switch_position_valid = value;}

	/*** BCM ***/
	BeltStatus getDriverSeatBeltSwitchSts(void) 		  	{ return _driver_seat_belt_switch_sts; }
	void	   setDriverSeatBeltSwitchSts(BeltStatus value)	{ _driver_seat_belt_switch_sts = value;}

	// light
	LightStatus getTurnLightLeftSts(void) 		  	   	{ return _turn_light_left_sts; }
	void	    setTurnLightLeftSts(LightStatus value)	{ _turn_light_left_sts = value;}

	LightStatus getTurnLightRightSts(void) 		  	   	{ return _turn_light_right_sts; }
	void	    setTurnLightRightSts(LightStatus value)	{ _turn_light_right_sts = value;}
	
	// door
 	DoorStatus getDriverDoorSts(void) 		  	   	{ return _driver_door_sts; }
	void	   setDriverDoorSts(DoorStatus value)	{ _driver_door_sts = value;}

 	DoorStatus getPassangerDoorSts(void) 		  	   	{ return _passanger_door_sts; }
	void	   setPassangerDoorSts(DoorStatus value)	{ _passanger_door_sts = value;}

	DoorStatus getTrunkSts(void) 		  	   	{ return _trunk_sts; }
	void	   setTrunkSts(DoorStatus value)	{ _trunk_sts = value;}
private:
	/*** System State ***/
	ReadyStatus _system_ready_sts;
	DriverMode _auto_driver_mode_sts;
	DriverMode _eps_auto_driver_mode_sts; 
	/*** ESC ESP ***/
	// status
	ActuatorStatus _esc_status;
	// wheel speed
	float _wheel_speed_front_left ;
	float _wheel_speed_front_right;
	float _wheel_speed_rear_left  ;
	float _wheel_speed_rear_right ;
	// wheel speed valid signal
	ValidStatus _wheel_speed_front_left_valid;
	ValidStatus _wheel_speed_front_right_valid;
	ValidStatus _wheel_speed_rear_left_valid;
	ValidStatus _wheel_speed_rear_right_valid;
	// wheel speed direction
	DirectStatus _wheel_speed_direction;
	// wheel pulse signal
	uint16_t _wheel_pulse_front_left ;
	uint16_t _wheel_pulse_front_right;
	uint16_t _wheel_pulse_rear_right ;
	uint16_t _wheel_pulse_rear_left  ;
	// wheel pulse valid signal
	ValidStatus _wheel_pulse_front_left_valid;
	ValidStatus _wheel_pulse_front_right_valid;
	ValidStatus _wheel_pulse_rear_left_valid;
	ValidStatus _wheel_pulse_rear_right_valid;
	// wheel pulse dirction
	DirectStatus _wheel_pulse_direction;
	// vehicle speed base pulse,calculate by self
	int32_t _sum_rear_left_pulse;
	int32_t _sum_rear_right_pulse;
	int32_t _wheel_sum_pulse;
	// calculate the speed with pulse and speed
	float   _vehicle_middle_speed;
	uint8_t _vehicle_middle_speed_valid;
	SpeedStatus _vehicle_middle_speed_abnormal;
	// IMU Sensor Valid Status
	ValidStatus _yaw_rate_valid;
	ValidStatus _lon_acc_valid;
	ValidStatus _lat_acc_valid;
	// IMU Sensor Value
	float _yaw_rate;
	float _lon_acc;
	float _lat_acc;

	/*** EPS ***/
	ActuatorStatus _eps_status;
	ManualDetected _eps_manual_control_detection_sts;
	ControlStatus  _eps_request_feedback;
	uint8_t        _eps_abort_feedback;
	/*** SAS ***/ 
	// Steering angle
	ActuatorStatus _sas_status;
	float _steering_angle;
	float _steering_angle_rate;

	/*** TCU ***/
	ActuatorStatus _tcu_status;
	ValidStatus _target_gear_valid;
	ValidStatus _actual_gear_valid;
	GearStatus _target_gear;
	GearStatus _actual_gear;

	/*** EMS ***/
	ActuatorStatus _ems_status;

	/*** VCU ***/
	ValidStatus _vcu_status;
	// Acc
	float _acc_pedal_stroke;
	ValidStatus _acc_pedal_valid;
	// Brake
	uint8_t _brake_pedal_sts;
	uint8_t _brake_pressure;
	ValidStatus _brake_pedal_valid;
	// 计算坡度
	float _vehicle_gradient;

	/*** AC ***/
	float _ambient_temperature;
	uint8_t _ambient_temperature_valid;

	/*** EPB ***/
	ActuatorStatus _epb_status;
	EPB_SystemStatus _epb_system_sts;
	EPB_Status _epb_switch_position;
	ValidStatus _epb_switch_position_valid;

	/*** BCM ***/
	// belt
	BeltStatus _driver_seat_belt_switch_sts;
	// light
	LightStatus _turn_light_left_sts;
	LightStatus _turn_light_right_sts;
	// door
	DoorStatus _driver_door_sts;
	DoorStatus _passanger_door_sts;
	DoorStatus _trunk_sts;
};

#endif /* CANBUS_INTERFACE_MESSAGE_MANAGER_H_ */

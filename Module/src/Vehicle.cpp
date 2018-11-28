/*
 * Vehicle.cpp
 *
 *  Created on: 2018Äê11ÔÂ21ÈÕ
 *      Author: zhuguohua
 */

#include "Vehicle.h"

Vehicle::Vehicle()
{
	_terminal_frame = FirstHead1;
	// TODO Auto-generated constructor stub
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

	APA_EPAS_Failed.setContainer(this);
	APA_EPAS_Failed.getter(&Vehicle::getAPA_EPAS_Failed);

	APA_ControlFeedback.setContainer(this);
	APA_ControlFeedback.getter(&Vehicle::getAPA_ControlFeedback);

	TorqueSensorStatus.setContainer(this);
	TorqueSensorStatus.getter(&Vehicle::getTorqueSensorStatus);

	SteeringTorque.setContainer(this);
	SteeringTorque.getter(&Vehicle::getSteeringTorque);

	// Wheel Speed
	WheelSpeedRearLeftDirection.setContainer(this);
	WheelSpeedRearLeftDirection.getter(&Vehicle::getWheelSpeedRearLeftDirection);

	WheelSpeedRearLeftValid.setContainer(this);
	WheelSpeedRearLeftValid.getter(&Vehicle::getWheelSpeedRearLeftValid);

	WheelSpeedRearLeftData.setContainer(this);
	WheelSpeedRearLeftData.getter(&Vehicle::getWheelSpeedRearLeftData);

	WheelSpeedRearRightDirection.setContainer(this);
	WheelSpeedRearRightDirection.getter(&Vehicle::getWheelSpeedRearRightDirection);

	WheelSpeedRearRightValid.setContainer(this);
	WheelSpeedRearRightValid.getter(&Vehicle::getWheelSpeedRearRightValid);

	WheelSpeedRearRightData.setContainer(this);
	WheelSpeedRearRightData.getter(&Vehicle::getWheelSpeedRearRightData);

	WheelSpeedFrontLeftDirection.setContainer(this);
	WheelSpeedFrontLeftDirection.getter(&Vehicle::getWheelSpeedFrontLeftDirection);

	WheelSpeedFrontLeftValid.setContainer(this);
	WheelSpeedFrontLeftValid.getter(&Vehicle::getWheelSpeedFrontLeftValid);

	WheelSpeedFrontLeftData.setContainer(this);
	WheelSpeedFrontLeftData.getter(&Vehicle::getWheelSpeedFrontLeftData);

	WheelSpeedFrontRightDirection.setContainer(this);
	WheelSpeedFrontRightDirection.getter(&Vehicle::getWheelSpeedFrontRightDirection);

	WheelSpeedFrontRightValid.setContainer(this);
	WheelSpeedFrontRightValid.getter(&Vehicle::getWheelSpeedFrontRightValid);

	WheelSpeedFrontRightData.setContainer(this);
	WheelSpeedFrontRightData.getter(&Vehicle::getWheelSpeedFrontRightData);
	/////////////vehicle speed
	VehicleSpeedValid.setContainer(this);
	VehicleSpeedValid.getter(&Vehicle::getVehicleSpeedValid);

	VehicleSpeed.setContainer(this);
	VehicleSpeed.getter(&Vehicle::getVehicleSpeed);
	// SAS Steering Angle
	SteeringAngleActual.setContainer(this);
	SteeringAngleActual.getter(&Vehicle::getSteeringAngleActual);

	SteeringAngleSpeed.setContainer(this);
	SteeringAngleSpeed.getter(&Vehicle::getSteeringAngleSpeed);

	SteeringAngleValid.setContainer(this);
	SteeringAngleValid.getter(&Vehicle::getSteeringAngleValid);

	SAS_Failure.setContainer(this);
	SAS_Failure.getter(&Vehicle::getSAS_Failure);
	//ESP
	ESP_QDC_ACC.setContainer(this);
	ESP_QDC_ACC.getter(&Vehicle::getESP_QDC_ACC);
	//EMS
	EMS_QEC_ACC.setContainer(this);
	EMS_QEC_ACC.getter(&Vehicle::getEMS_QEC_ACC);

//	_steering_angle_target = 0.0;
//	this->SteeringAngleTarget = 0.0;
}

Vehicle::~Vehicle() {
	// TODO Auto-generated destructor stub
}

/****** Property ******/
/// ACC
float Vehicle::getTargetAccelerationACC()
{
	return _target_acceleration_acc;
}
void Vehicle::setTargetAccelerationACC(float value)
{
	_target_acceleration_acc = value;
}

vuint8_t Vehicle::getTargetAccelerationEnable()
{
	return _target_acceleration_enable;
}
void Vehicle::setTargetAccelerationEnable(vuint8_t value)
{
	_target_acceleration_enable = value;
}

/// AEB
float Vehicle::getTargetDecelerationAEB()
{
	return _target_deceleration_aeb;
}
void Vehicle::setTargetDecelerationAEB(float value)
{
	_target_deceleration_aeb = value;
}

vuint8_t Vehicle::getTargetDecelerationEnable()
{
	return _target_deceleration_enable;
}
void Vehicle::setTargetDecelerationEnable(vuint8_t value)
{
	_target_deceleration_enable = value;
}

/// Torque
vuint8_t Vehicle::getTorque()
{
	return _torque;
}
void Vehicle::setTorque(vuint8_t value)
{
	_torque = value;
}

vuint8_t Vehicle::getTorqueEnable()
{
	return _torque_enable;
}
void Vehicle::setTorqueEnable(vuint8_t value)
{
	_torque_enable = value;
}

/// Steering Angle
vint16_t Vehicle::getSteeringAngleTarget()
{
	return _steering_angle_target;
}
void Vehicle::setSteeringAngleTarget(vint16_t value)
{
	_steering_angle_target = value;
}

vuint8_t Vehicle::getSteeringAngleTargetActive()
{
	return _steering_angle_target_active;
}
void Vehicle::setSteeringAngleTargetActive(vuint8_t value)
{
	_steering_angle_target_active = value;
}

/// Gear
vuint8_t Vehicle::getGearShift()
{
	return _gear_shift;
}
void Vehicle::setGearShift(vuint8_t value)
{
	_gear_shift = value;
}

vuint8_t Vehicle::getGearShiftEnable()
{
	return _gear_shift_enable;
}
void Vehicle::setGearShiftEnable(vuint8_t value)
{
	_gear_shift_enable = value;
}

vuint8_t Vehicle::getGearShiftValid()
{
	return _gear_shift_valid;
}
void Vehicle::setGearShiftValid(vuint8_t value)
{
	_gear_shift_valid = value;
}


/* The information from vehicle and read only */
/// EPS
vuint8_t Vehicle::getEPS_Failed()
{
	return _eps_failed;
}
vuint8_t Vehicle::getAPA_EPAS_Failed()
{
	return _apa_epas_failed;
}
vuint8_t Vehicle::getAPA_ControlFeedback()
{
	return _apa_control_feedback;
}
vuint8_t Vehicle::getTorqueSensorStatus()
{
	return _torque_sensor_status;
}
float Vehicle::getSteeringTorque()
{
	return _steering_torque;
}

///////////////////////////Wheel Speed
vuint8_t Vehicle::getWheelSpeedRearLeftDirection()
{
	return _wheel_speed_rear_left_direction;
}
vuint8_t Vehicle::getWheelSpeedRearLeftValid()
{
	return _wheel_speed_rear_left_valid;
}
float Vehicle::getWheelSpeedRearLeftData()
{
	return _wheel_speed_rear_left_data;
}

vuint8_t Vehicle::getWheelSpeedRearRightDirection()
{
	return _wheel_speed_rear_right_direction;
}
vuint8_t Vehicle::getWheelSpeedRearRightValid()
{
	return _wheel_speed_rear_right_valid;
}
float Vehicle::getWheelSpeedRearRightData()
{
	return _wheel_speed_rear_right_data;
}
//////////////////////////////////
vuint8_t Vehicle::getWheelSpeedFrontLeftDirection()
{
	return _wheel_speed_front_left_direction;
}
vuint8_t Vehicle::getWheelSpeedFrontLeftValid()
{
	return _wheel_speed_front_left_valid;
}
float Vehicle::getWheelSpeedFrontLeftData()
{
	return _wheel_speed_front_left_data;
}

vuint8_t Vehicle::getWheelSpeedFrontRightDirection()
{
	return _wheel_speed_front_right_direction;
}
vuint8_t Vehicle::getWheelSpeedFrontRightValid()
{
	return _wheel_speed_front_right_valid;
}
float Vehicle::getWheelSpeedFrontRightData()
{
	return _wheel_speed_front_right_data;
}

//Speed vehicle
vuint8_t Vehicle::getVehicleSpeedValid()
{
	return _vehicle_speed_valid;
}
float Vehicle::getVehicleSpeed()
{
	return _vehicle_speed;
}

// Wheel Pusle
vuint8_t Vehicle::getWheelSpeedDirection()
{
	return _wheel_speed_direction;
}
vuint8_t Vehicle::getWheelSpeedRearRightPulse()
{
	return _wheel_speed_rear_right_pulse;
}
vuint8_t Vehicle::getWheelSpeedRearLeftPulse()
{
	return _wheel_speed_rear_left_pulse;
}
vuint8_t Vehicle::getWheelSpeedFrontRightPulse()
{
	return _wheel_speed_front_right_pulse;
}
vuint8_t Vehicle::getWheelSpeedForntLeftPulse()
{
	return _wheel_speed_front_left_pulse;
}

// SAS Steering Angle
vint16_t Vehicle::getSteeringAngleActual()
{
	return _steering_angle_actual;
}

vuint16_t Vehicle::getSteeringAngleSpeed()
{
	return _steering_angle_speed;
}

vuint8_t Vehicle::getSteeringAngleValid()
{
	return _steering_angle_valid;
}

vuint8_t Vehicle::getSAS_Failure()
{
	return _sas_failure;
}

vuint8_t Vehicle::getESP_QDC_ACC()
{
	return esp_qdc_acc;
}

vuint8_t Vehicle::getEMS_QEC_ACC()
{
	return ems_qec_acc;
}
////////////////////////////////////////////////////////////
void Vehicle::VehicleContorlStep1()
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x6fe;
	m_CAN_Packet.length = 8;
	/// data buffer
	/* ACC */
	_current_target_acceleration_ACC = (vuint8_t)((_target_acceleration_acc + 5.0)*20);
	_current_target_acceleration_enable_single = _target_acceleration_enable;
	/* AEB */
	_current_target_deceleration_AEB = (vuint16_t)((_target_deceleration_aeb + 16.0) * 2000);
	_current_target_deceleration_enable_single = _target_deceleration_enable;
	/* Torque */
	_current_torque = (vuint16_t)(_torque * 10.23);
	_current_torque_enable_single = _torque_enable;
	/* Steering Angle */
	_current_steering_angle_target = (vint16_t)(_steering_angle_set * 10);
	_current_steering_angle_target_active_single = _steering_angle_target_active;

	/// Data Mapping
	m_CAN_Packet.data[0] = _current_target_acceleration_ACC;
	m_CAN_Packet.data[1] = (vuint8_t)((_current_target_deceleration_AEB >> 8) & 0xFF);
	m_CAN_Packet.data[2] = (vuint8_t)((_current_target_deceleration_AEB     ) & 0xFF);
	m_CAN_Packet.data[3] = (vuint8_t)( _rolling_counter_torque_AEB & 0x0F);
	m_CAN_Packet.data[3] = _current_target_acceleration_enable_single ?
						   (vuint8_t) ( m_CAN_Packet.data[3] | 0x80 ) :
						   (vuint8_t) ( m_CAN_Packet.data[3] & 0x7F ) ;
	m_CAN_Packet.data[3] = _current_target_deceleration_enable_single ?
						   (vuint8_t) ( m_CAN_Packet.data[3] | 0x40 ) :
						   (vuint8_t) ( m_CAN_Packet.data[3] & 0xBF ) ;
	m_CAN_Packet.data[4] = (vuint8_t)((_current_torque >> 2) & 0xFF);
	m_CAN_Packet.data[5] = (vuint8_t)((_current_torque << 6) & 0xC0);
	m_CAN_Packet.data[5] = _current_torque_enable_single              ?
						   (vuint8_t) ( m_CAN_Packet.data[5] | 0x20 ) :
						   (vuint8_t) ( m_CAN_Packet.data[5] & 0xDF ) ;
	m_CAN_Packet.data[5] = (vuint8_t) ((m_CAN_Packet.data[5] & 0xFC ) |
									  ( _current_steering_angle_target_active_single & 0x03));
	m_CAN_Packet.data[6] = (vuint8_t)((_current_steering_angle_target >> 8) & 0xFF);
	m_CAN_Packet.data[7] = (vuint8_t)((_current_steering_angle_target     ) & 0xFF);
	CAN0_TransmitMsg(m_CAN_Packet);
}

void Vehicle::VehicleContorlStep2()
{
	vuint8_t i;
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x6FF;
	m_CAN_Packet.length = 8;
	/// data buffer
	_current_gear_shift = _gear_shift;
	_current_gear_shift_enable_single = _gear_shift_enable;
	_current_gear_shift_valid_single = _gear_shift_valid;

	/// data mapping
	for(i=0;i<4;i++){m_CAN_Packet.data[i] = 0;}
	m_CAN_Packet.data[4] = (vuint8_t)(_rolling_counter_brake_ACC & 0x0f);
	m_CAN_Packet.data[5] = (vuint8_t)((_current_gear_shift << 4 ) & 0x70);
	m_CAN_Packet.data[5] = _current_gear_shift_enable_single          ?
						   (vuint8_t) ( m_CAN_Packet.data[5] | 0x80 ) :
						   (vuint8_t) ( m_CAN_Packet.data[5] & 0x7F ) ;
	m_CAN_Packet.data[5] = _current_gear_shift_valid_single          ?
						   (vuint8_t) ( m_CAN_Packet.data[5] | 0x08 ) :
						   (vuint8_t) ( m_CAN_Packet.data[5] & 0xF7 ) ;

	m_CAN_Packet.data[6] = (vuint8_t)(
								(
									( _rolling_counter_brake_ACC                 & 0x0f )
								+ 	((_current_target_acceleration_enable_single & 0x01 ) << 4)
								+ 	  _current_target_acceleration_ACC
								)^0xFF
							);
	m_CAN_Packet.data[7] = (vuint8_t)(
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
	vuint8_t i;
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x6E9;
	m_CAN_Packet.length = 8;

	/// data mapping
	m_CAN_Packet.data[0] = 0;
	m_CAN_Packet.data[1] = (vuint8_t)(_rolling_counter_steering_control & 0x0f);
	m_CAN_Packet.data[2] = (vuint8_t)	(
											(
												(_rolling_counter_steering_control & 0x0f)
											+	((_current_steering_angle_target >> 8) & 0xFF)
											+	((_current_steering_angle_target     ) & 0xFF)
											+	_current_steering_angle_target_active_single
											) ^ 0xFF
										);
	m_CAN_Packet.data[3] = (vuint8_t)	(
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

void Vehicle::SteeringAngleControlStateMachine()
{
	switch(_steering_angle_Control_state)
	{
		case 0:
			if(!_apa_epas_failed)
			{
				_steering_angle_Control_state = 1;
			}
			break;

		case 1:
			if(_apa_control_feedback)
			{
				_steering_angle_Control_state = 2;
			}
			if(_apa_epas_failed)
			{
				_steering_angle_Control_state = 0;
			}
			break;

		case 2:
			_steering_angle_target_active = 2;
			_steering_angle_Control_state = 3;
			break;

		case 3:
			if(!_apa_control_feedback)
			{
				_steering_angle_Control_state = 0;
			}
			if(_apa_epas_failed)
			{
				_steering_angle_Control_state = 0;
			}
			break;

		default:
			_steering_angle_Control_state = 0;
			break;
	}
}

void Vehicle::VehicleInformation(CAN_MB_tag mb_msg)
{
	switch(mb_msg.ID.B.ID_STD)
	{
		case 0x2A0://eps status
			_eps_failed = (vuint8_t)((mb_msg.DATA.B[1] >> 7) & 0x01);
			_apa_epas_failed = (vuint8_t)((mb_msg.DATA.B[1] >> 1) & 0x01);
			_torque_sensor_status = (vuint8_t)( mb_msg.DATA.B[1] & 0x01 );
			_apa_control_feedback = (vuint8_t)((mb_msg.DATA.B[3] >> 5) & 0x01);
			_steering_torque = (float)(mb_msg.DATA.B[2] * 0.1794 - 22.78);
			break;

		case 0x208:// wheel speed
			_wheel_speed_rear_left_direction = (vuint8_t)(mb_msg.DATA.B[0] >> 5) & 0x03;
			_wheel_speed_rear_left_valid = (vuint8_t)(  mb_msg.DATA.B[0] >> 7) & 0x01;
			_wheel_speed_rear_left_data = ((vuint16_t)(((mb_msg.DATA.B[0] & 0x1F) << 8) | mb_msg.DATA.B[1]))*0.05625;

			_wheel_speed_rear_right_direction = (vuint8_t)(mb_msg.DATA.B[2] >> 5) & 0x03;
			_wheel_speed_rear_right_valid = (vuint8_t)(  mb_msg.DATA.B[2] >> 7) & 0x01;
			_wheel_speed_rear_right_data = ((vuint16_t)(((mb_msg.DATA.B[2] & 0x1F) << 8) | mb_msg.DATA.B[3]))*0.05625;

			_wheel_speed_front_left_direction = (vuint8_t)(mb_msg.DATA.B[4] >> 5) & 0x03;
			_wheel_speed_front_left_valid = (vuint8_t)(  mb_msg.DATA.B[4] >> 7) & 0x01;
			_wheel_speed_front_left_data = ((vuint16_t)(((mb_msg.DATA.B[4] & 0x1F) << 8) | mb_msg.DATA.B[5]))*0.05625;

			_wheel_speed_front_right_direction = (vuint8_t)(mb_msg.DATA.B[6] >> 5) & 0x03;
			_wheel_speed_front_right_valid = (vuint8_t)(  mb_msg.DATA.B[6] >> 7) & 0x01;
			_wheel_speed_front_right_data = ((vuint16_t)(((mb_msg.DATA.B[6] & 0x1F) << 8) | mb_msg.DATA.B[7]))*0.05625;
			break;

		case 0x218://vehicle speed
			_vehicle_speed_valid = (vuint8_t)( mb_msg.DATA.B[4] >> 5 ) & 0x01;
			_vehicle_speed = (float)(((vuint16_t)(((mb_msg.DATA.B[4] & 0x1F) << 8) | mb_msg.DATA.B[5])) * 0.05625);
			break;

		case 0x258://Wheel speed pulse
			_wheel_speed_direction = (vuint8_t)(mb_msg.DATA.B[2] & 0x03);
			_wheel_speed_rear_right_pulse  = mb_msg.DATA.B[4];
			_wheel_speed_rear_left_pulse   = mb_msg.DATA.B[5];
			_wheel_speed_front_right_pulse = mb_msg.DATA.B[6];
			_wheel_speed_front_left_pulse  = mb_msg.DATA.B[7];
			break;

		case 0x277://ESP
			esp_qdc_acc = (vuint8_t)( (mb_msg.DATA.B[3] >> 1) & 0x03);
			break;

		case 0x26A://EMS
			ems_qec_acc = (vuint8_t)( (mb_msg.DATA.B[0] >> 1) & 0x03);
			break;

		case 0x180://actual steering angle
			_steering_angle_actual = (vint16_t)(((vint16_t)((mb_msg.DATA.B[0] << 8) | mb_msg.DATA.B[1])) * 0.1);
			_steering_angle_speed = (vuint16_t)(mb_msg.DATA.B[2] * 4);
			_steering_angle_valid = (vuint8_t)( mb_msg.DATA.B[3] >> 7 ) & 0x01;
			_sas_failure = (vuint8_t)( mb_msg.DATA.B[3] >> 6 ) & 0x01;
			break;

		default:

			break;
	}
}
void Vehicle::VehicleInformation(vuint32_t id,vuint8_t dat[])
{
	switch(id)
	{
		case 0x2A0://eps status
			_eps_failed = (vuint8_t)((dat[1] >> 7) & 0x01);
			_apa_epas_failed = (vuint8_t)((dat[1] >> 1) & 0x01);
			_torque_sensor_status = (vuint8_t)( dat[1] & 0x01 );
			_apa_control_feedback = (vuint8_t)((dat[3] >> 5) & 0x01);
			_steering_torque = (float)(dat[2] * 0.1794 - 22.78);
			break;

		case 0x208:// wheel speed
			_wheel_speed_rear_right_direction = (vuint8_t)(dat[0] >> 5) & 0x03;
			_wheel_speed_rear_right_valid = (vuint8_t)(  dat[0] >> 7) & 0x01;
			_wheel_speed_rear_right_data = ((vuint16_t)(((dat[0] & 0x1F) << 8) | dat[1]))*0.05625;

			_wheel_speed_rear_left_direction = (vuint8_t)(dat[2] >> 5) & 0x03;
			_wheel_speed_rear_left_valid = (vuint8_t)(  dat[2] >> 7) & 0x01;
			_wheel_speed_rear_left_data = ((vuint16_t)(((dat[2] & 0x1F) << 8) | dat[3]))*0.05625;

			_wheel_speed_front_right_direction = (vuint8_t)(dat[4] >> 5) & 0x03;
			_wheel_speed_front_right_valid = (vuint8_t)(  dat[4] >> 7) & 0x01;
			_wheel_speed_front_right_data = ((vuint16_t)(((dat[4] & 0x1F) << 8) | dat[5]))*0.05625;

			_wheel_speed_front_left_direction = (vuint8_t)(dat[6] >> 5) & 0x03;
			_wheel_speed_front_left_valid = (vuint8_t)(  dat[6] >> 7) & 0x01;
			_wheel_speed_front_left_data = ((vuint16_t)(((dat[6] & 0x1F) << 8) | dat[7]))*0.05625;
			break;

		case 0x218://vehicle speed
			_vehicle_speed_valid = (vuint8_t)( dat[4] >> 5 ) & 0x01;
			_vehicle_speed = (float)(((vuint16_t)(((dat[4] & 0x1F) << 8) | dat[5])) * 0.05625);
			break;

		case 0x258://Wheel speed pulse
			_wheel_speed_direction = (vuint8_t)(dat[2] & 0x03);
			_wheel_speed_rear_right_pulse  = dat[4];
			_wheel_speed_rear_left_pulse   = dat[5];
			_wheel_speed_front_right_pulse = dat[6];
			_wheel_speed_front_left_pulse  = dat[7];
			break;

		case 0x277://ESP
			esp_qdc_acc = (vuint8_t)( (dat[3] >> 1) & 0x03);
			break;

		case 0x26A://EMS
			ems_qec_acc = (vuint8_t)( (dat[0] >> 1) & 0x03);
			break;

		case 0x180://actual steering angle
			_steering_angle_actual = (float)(((vint16_t)((dat[0] << 8) | dat[1])) * 0.1);
			_steering_angle_speed = (vuint16_t)(dat[2] * 4);
			_steering_angle_valid = (vuint8_t)( dat[3] >> 7 ) & 0x01;
			_sas_failure = (vuint8_t)( dat[3] >> 6 ) & 0x01;
			break;

		default:

			break;
	}
}

// Terminal control
void Vehicle::TerminalControlCommandReceive(vuint8_t data)
{
	vuint8_t i;
	switch(_terminal_frame)
	{
		case FirstHead1:
			if(data == 0xAA)
			{
				_check_sum = data;
				_terminal_frame = FirstHead2;
			}
			break;
		case FirstHead2:
			if(data == 0x55)
			{
				_check_sum += data;
				_terminal_frame = ID;
			}
			else
			{
				_terminal_frame = FirstHead1;
			}
			break;

		case ID:
			_frame_id = data;
			_check_sum += data;
			_terminal_frame = Length;
			break;

		case Length:
			_frame_length = data;
			_check_sum += data;
			_frame_cnt = 0;
			_terminal_frame = Data;
		break;

		case Data:
			_data_buffer[_frame_cnt] = data;
			_check_sum += data;
			_frame_cnt++;
			if(_frame_cnt >= _frame_length)
			{
				_terminal_frame = CheckSum;
			}
			break;

		case CheckSum:
			if(_check_sum == data)
			{
				if(_frame_id == 0x3E)
				{
					_gear_shift_enable = _data_buffer[0] & 0x01;
					_gear_shift_valid = (_data_buffer[0] >> 1) & 0x01;

					if( (((_data_buffer[0] >> 2) & 0x01) == 0) || (_steering_angle_target_active == 0))
					{
						_steering_angle_target_active = (_data_buffer[0] >> 2) & 0x01;
					}

					_torque_enable = (_data_buffer[0] >> 3) & 0x01;
					_target_deceleration_enable = (_data_buffer[0] >> 4) & 0x01;
					_target_acceleration_enable = (_data_buffer[0] >> 5) & 0x01;

					_gear_shift = _data_buffer[1];

					_torque = _data_buffer[2];

					_steering_angle_target = (vint16_t)((_data_buffer[5] << 8) | _data_buffer[4]);

					_steering_angle_speed_target = (vuint16_t)((_data_buffer[7] << 8) | _data_buffer[6]);

					for(i = 0;i<4;i++)
					{
						_data_temp.b[3-i] = _data_buffer[i + 8];
					}
					_target_deceleration_aeb = _data_temp.f;
					for(i = 0;i<4;i++)
					{
						_data_temp.b[3-i] = _data_buffer[i + 12];
					}
					_target_acceleration_acc = _data_temp.f;
				}
				TerminalControlAckSend();
			}
			_terminal_frame = FirstHead1;
			break;
	}
}

void Vehicle::TerminalControlCommandSend(void)
{
	vuint8_t i,check_sum;
	_send_data_buffer[0] = 0x7F;
	_send_data_buffer[1] = 0x80;
	_send_data_buffer[2] = 0x6F;
	_send_data_buffer[3] = 3;
	_send_data_buffer[4] = ((ems_qec_acc << 2) & 0x04) | ((esp_qdc_acc << 1) & 0x02) | _apa_epas_failed;
	_send_data_buffer[5] = (vuint8_t)(_steering_angle_actual & 0xff);
	_send_data_buffer[6] = (vuint8_t)((_steering_angle_actual >> 8) & 0xff);

	check_sum = 0;
	for(i=0;i<7;i++)
	{
		check_sum += _send_data_buffer[i];
	}
	_send_data_buffer[7] = check_sum;
	for(i=0;i<8;i++)
	{
		TransmitData(_send_data_buffer[i]);
	}
}
void Vehicle::TerminalControlSpeedSend(void)
{
	vuint8_t i,check_sum;
	_send_data_buffer[0] = 0x7F;
	_send_data_buffer[1] = 0x80;
	_send_data_buffer[2] = 0x6E;
	_send_data_buffer[3] = 19;
	_speed_data_temp.f = _vehicle_speed;
	for(i = 0;i<4;i++)
	{
		_send_data_buffer[i + 4] = _speed_data_temp.b[3-i] ;
	}

	_send_data_buffer[8] = (vuint8_t)(( (vuint16_t)(_wheel_speed_front_left_data*10)    ) & 0xff);
	_send_data_buffer[9] = (vuint8_t)((((vuint16_t)(_wheel_speed_front_left_data*10))>>8) & 0xff);

	_send_data_buffer[10] = (vuint8_t)(( (vuint16_t)(_wheel_speed_front_right_data*10)    ) & 0xff);
	_send_data_buffer[11] = (vuint8_t)((((vuint16_t)(_wheel_speed_front_right_data*10))>>8) & 0xff);

	_send_data_buffer[12] = (vuint8_t)(( (vuint16_t)(_wheel_speed_rear_left_data*10)    ) & 0xff);
	_send_data_buffer[13] = (vuint8_t)((((vuint16_t)(_wheel_speed_rear_left_data*10))>>8) & 0xff);

	_send_data_buffer[14] = (vuint8_t)(( (vuint16_t)(_wheel_speed_rear_right_data*10)    ) & 0xff);
	_send_data_buffer[15] = (vuint8_t)((((vuint16_t)(_wheel_speed_rear_right_data*10))>>8) & 0xff);

	_send_data_buffer[16] = _wheel_speed_front_left_pulse;
	_send_data_buffer[17] = _wheel_speed_front_right_pulse;
	_send_data_buffer[18] = _wheel_speed_rear_left_pulse;
	_send_data_buffer[19] = _wheel_speed_rear_right_pulse;

	_send_data_buffer[20] = 0;
	_send_data_buffer[21] = 0;
	_send_data_buffer[22] = 0;

	check_sum = 0;
	for(i=0;i<23;i++)
	{
		check_sum += _send_data_buffer[i];
	}
	_send_data_buffer[23] = check_sum;
	for(i=0;i<24;i++)
	{
		TransmitData(_send_data_buffer[i]);
	}
}

void Vehicle::TerminalControlAckSend(void)
{
	vuint8_t i,check_sum;
	_send_data_buffer[0] = 0x7F;
	_send_data_buffer[1] = 0x80;
	_send_data_buffer[2] = 0x3E;
	_send_data_buffer[3] = 1;
	_send_data_buffer[4] = 0xA5;

	check_sum = 0;
	for(i=0;i<5;i++)
	{
		check_sum += _send_data_buffer[i];
	}
	_send_data_buffer[5] = check_sum;
	for(i=0;i<6;i++)
	{
		TransmitData(_send_data_buffer[i]);
	}
}

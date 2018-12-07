class Vehicle
{
public:
	/*** Function ***/
	Vehicle();
	virtual ~Vehicle();

	/*** Function ***/
	// Vehicle information receive
	void VehicleInformation(CAN_MB_tag mb_msg);

private:
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

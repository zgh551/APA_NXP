void Vehicle::VehicleInformation(CAN_MB_tag mb_msg)
{
	switch(mb_msg.ID.B.ID_STD)
	{
		case 0x2A0://eps status
			_eps_failed = (uint8_t)((mb_msg.DATA.B[1] >> 7) & 0x01);
			_apa_epas_failed = (uint8_t)((mb_msg.DATA.B[1] >> 1) & 0x01);
			_torque_sensor_status = (uint8_t)( mb_msg.DATA.B[1] & 0x01 );
			_apa_control_feedback = (uint8_t)((mb_msg.DATA.B[3] >> 5) & 0x01);
			_steering_torque = (float)(mb_msg.DATA.B[2] * 0.1794 - 22.78);
			break;

		case 0x208:// wheel speed
			_wheel_speed_rear_left_direction = (uint8_t)(mb_msg.DATA.B[0] >> 5) & 0x03;
			_wheel_speed_rear_left_valid = (uint8_t)(  mb_msg.DATA.B[0] >> 7) & 0x01;
			_wheel_speed_rear_left_data = ((uint16_t)(((mb_msg.DATA.B[0] & 0x1F) << 8) | mb_msg.DATA.B[1]))*0.05625;

			_wheel_speed_rear_right_direction = (uint8_t)(mb_msg.DATA.B[2] >> 5) & 0x03;
			_wheel_speed_rear_right_valid = (uint8_t)(  mb_msg.DATA.B[2] >> 7) & 0x01;
			_wheel_speed_rear_right_data = ((uint16_t)(((mb_msg.DATA.B[2] & 0x1F) << 8) | mb_msg.DATA.B[3]))*0.05625;

			_wheel_speed_front_left_direction = (uint8_t)(mb_msg.DATA.B[4] >> 5) & 0x03;
			_wheel_speed_front_left_valid = (uint8_t)(  mb_msg.DATA.B[4] >> 7) & 0x01;
			_wheel_speed_front_left_data = ((uint16_t)(((mb_msg.DATA.B[4] & 0x1F) << 8) | mb_msg.DATA.B[5]))*0.05625;

			_wheel_speed_front_right_direction = (uint8_t)(mb_msg.DATA.B[6] >> 5) & 0x03;
			_wheel_speed_front_right_valid = (uint8_t)(  mb_msg.DATA.B[6] >> 7) & 0x01;
			_wheel_speed_front_right_data = ((uint16_t)(((mb_msg.DATA.B[6] & 0x1F) << 8) | mb_msg.DATA.B[7]))*0.05625;
			break;

		case 0x218://vehicle speed
			_vehicle_speed_valid = (uint8_t)( mb_msg.DATA.B[4] >> 5 ) & 0x01;
			_vehicle_speed = (float)(((uint16_t)(((mb_msg.DATA.B[4] & 0x1F) << 8) | mb_msg.DATA.B[5])) * 0.05625);
			break;

		case 0x258://Wheel speed pulse
			_wheel_speed_direction = (uint8_t)(mb_msg.DATA.B[2] & 0x03);
			_wheel_speed_rear_right_pulse  = mb_msg.DATA.B[4];
			_wheel_speed_rear_left_pulse   = mb_msg.DATA.B[5];
			_wheel_speed_front_right_pulse = mb_msg.DATA.B[6];
			_wheel_speed_front_left_pulse  = mb_msg.DATA.B[7];
			break;

		case 0x277://ESP
			esp_qdc_acc = (uint8_t)( (mb_msg.DATA.B[3] >> 1) & 0x03);
			break;

		case 0x26A://EMS
			ems_qec_acc = (uint8_t)( (mb_msg.DATA.B[0] >> 1) & 0x03);
			break;

		case 0x180://actual steering angle
			_steering_angle_actual = (int16_t)(((int16_t)((mb_msg.DATA.B[0] << 8) | mb_msg.DATA.B[1])) * 0.1);
			_steering_angle_speed = (uint16_t)(mb_msg.DATA.B[2] * 4);
			_steering_angle_valid = (uint8_t)( mb_msg.DATA.B[3] >> 7 ) & 0x01;
			_sas_failure = (uint8_t)( mb_msg.DATA.B[3] >> 6 ) & 0x01;
			break;

		default:

			break;
	}
}
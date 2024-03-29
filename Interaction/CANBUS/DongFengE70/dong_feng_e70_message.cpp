/*
 * dong_feng_e70_message.cpp
 *
 *  Created on: 2019年6月20日
 *      Author: zhuguohua
 */

#include <DongFengE70/dong_feng_e70_message.h>

DongFengE70Message::DongFengE70Message() {
	// TODO Auto-generated constructor stub

}

DongFengE70Message::~DongFengE70Message() {
	// TODO Auto-generated destructor stub
}

void DongFengE70Message::Init()
{
	VCU_APA_ControlStatus.setContainer(this);
	VCU_APA_ControlStatus.getter(&DongFengE70Message::getVCU_APA_ControlStatus);
	VCU_APA_ControlStatus.setter(&DongFengE70Message::setVCU_APA_ControlStatus);

	ESP_AvailabStatus.setContainer(this);
	ESP_AvailabStatus.getter(&DongFengE70Message::getESP_AvailabStatus);
	ESP_AvailabStatus.setter(&DongFengE70Message::setESP_AvailabStatus);
}

void DongFengE70Message::Parse(const uint32_t id,const vuint8_t *data,const vuint32_t lenght)
{
	switch(id)
	{
		case 0x122:
			LatAcc = (uint16_t)(((data[0] & 0x0f) << 8) | data[1] ) * 0.1 - 204.8;
			YawRate = (((data[2] & 0x07) << 8 ) | data[3]) * 0.03 - 15.36;
			break;

		case 0x165:
			_esp_availab_status = (data[0] >> 6) & 0x03;
			break;

		case 0x355:
			switch((uint8_t)((data[0] >> 5) & 0x07))
			{
				case 1:
					Gear = Parking;
					break;

				case 2:
					Gear = Reverse;
					break;

				case 3:
					Gear = Neutral;
					break;

				case 4:
					Gear = Drive;
					break;

				default:
					Gear = None;
					break;
			}
			break;

		case 0x176://VCU 10
			_vcu_apa_control_st = (uint8_t)(( data[0] >> 5 ) & 0x03);
			break;

			case 0xA3://ESC
				if( 0 == ((data[0] >> 7) & 0x01))
				{
					WheelSpeedFrontLeft  = ((uint16_t)(((data[0] & 0x7F) << 8) | data[1])) * V_M_S;
				}
				if( 0 == ((data[2] >> 7) & 0x01))
				{
					WheelSpeedFrontRight  = ((uint16_t)(((data[2] & 0x7F) << 8) | data[3])) * V_M_S;
				}
				if( 0 == ((data[4] >> 7) & 0x01))
				{
					WheelSpeedRearLeft  = ((uint16_t)(((data[4] & 0x7F) << 8) | data[5])) * V_M_S;
				}
				if( 0 == ((data[6] >> 7) & 0x01))
				{
					WheelSpeedRearRight  = ((uint16_t)(((data[6] & 0x7F) << 8) | data[7])) * V_M_S;
				}
				break;

			case 0xA6://ESC
				if( 0 == ((data[5] >> 7) & 0x01))
				{
					WheelPulseFrontLeft  = (uint16_t)(((data[0] & 0xff) << 2) | ((data[1] >> 6) & 0x03));
				}
				if( 0 == ((data[5] >> 6) & 0x01))
				{
					WheelPulseRearLeft   = (uint16_t)(((data[1] & 0x3f) << 4) | ((data[2] >> 4) & 0x0f));
				}
				if( 0 == ((data[5] >> 5) & 0x01))
				{
					WheelPulseFrontRight = (uint16_t)(((data[2] & 0x0f) << 6) | ((data[3] >> 2) & 0x3f));
				}
				if( 0 == ((data[5] >> 4) & 0x01))
				{
					WheelPulseRearRight  = (uint16_t)(((data[3] & 0x03) << 8) |   data[4]              );
				}
				if( 0 == ((data[5] >> 3) & 0x01))
				{
					LonAcc = (uint16_t)(((data[5] & 0x07) << 8) | data[6]) * 0.03 - 15.36;
				}
				break;

			case 0xA5:
				SteeringAngle = (float)(((int16_t)((data[0] << 8) | data[1])) * 0.1);
				SteeringAngleRate = (uint16_t)(data[2] * 4);
//				SAS_Failure = (uint8_t)( data[3] >> 7 ) & 0x01;
				break;

			default:
				break;
	}
}


uint8_t DongFengE70Message::getVCU_APA_ControlStatus()			   {return _vcu_apa_control_st ;}
void    DongFengE70Message::setVCU_APA_ControlStatus(uint8_t value){_vcu_apa_control_st = value;}

uint8_t DongFengE70Message::getESP_AvailabStatus()			   {return _esp_availab_status ;}
void    DongFengE70Message::setESP_AvailabStatus(uint8_t value){_esp_availab_status = value;}

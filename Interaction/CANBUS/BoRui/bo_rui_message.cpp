/*
 * bo_rui_message.cpp
 *
 *  Created on: 2019年3月15日
 *      Author: zhuguohua
 */

#include "bo_rui_message.h"


BoRuiMessage::BoRuiMessage() {
	// TODO Auto-generated constructor stub
}

BoRuiMessage::~BoRuiMessage() {
	// TODO Auto-generated destructor stub
}

void BoRuiMessage::Init()
{
	_index = 29;
}

void BoRuiMessage::Parse(const uint32_t id,const vuint8_t *dat,const vuint32_t lenght)
{
//	uint8_t crc_temp,i;
//	uint8_t dat_temp[7];
	switch(id)
	{
		case 0x2A0://eps status

			break;
		case 0x2F1:
			if(0 == (dat[1] >> 1 & 0x01))
			{
				this->setAmbientTemperature(0.5f * dat[1] - 40);
				this->setAmbientTemperatureValid(0xA5);
			}
			break;

		case 0x122:// wheel speed
			WheelSpeedFrontRight = ((uint16_t)( (dat[0] << 5) | (dat[1] >> 3))) * V_M_S;
			WheelSpeedFrontLeft  = ((uint16_t)( (dat[2] << 5) | (dat[3] >> 3))) * V_M_S;
			break;

		case 0x123:// wheel speed
			switch((uint8_t)(dat[1] >> 1) & 0x03)
			{
				case 0:
					WheelSpeedDirection = StandStill;
					break;
				case 1:
					WheelSpeedDirection = Forward;
					break;
				case 2:
					WheelSpeedDirection = Backward;
					break;
				case 3:
					WheelSpeedDirection = Invalid;
					break;

				default:
					WheelSpeedDirection = Invalid;
					break;
			}
			WheelSpeedRearRight  = ((uint16_t)( (dat[0] << 5) | (dat[1] >> 3))) * V_M_S;
			WheelSpeedRearLeft   = ((uint16_t)( (dat[2] << 5) | (dat[3] >> 3))) * V_M_S;
			break;

		case 0x124://Wheel speed pulse
//			crc_temp = crc8.crcCompute((uint8_t*)dat, 7);
//			if(crc_temp == dat[7])
//			{
				WheelPulseFrontLeft  = (uint16_t)(( (dat[0] << 4) | (dat[1] >> 4)) & 0x0fff);
				WheelPulseFrontRight = (uint16_t)(( (dat[1] << 8) |  dat[2]      ) & 0x0fff);
				WheelPulseRearLeft   = (uint16_t)(( (dat[3] << 4) | (dat[4] >> 4)) & 0x0fff);
				WheelPulseRearRight  = (uint16_t)(( (dat[4] << 8) |  dat[5]      ) & 0x0fff);
//			}
			break;

		case 0x165:
			switch((uint8_t)(dat[1] & 0x1f))
			{
				case 0:
					this->setActualGear(None);
					break;

				case 1:
					this->setActualGear(Parking);
					break;

				case 2:
					this->setActualGear(Reverse);
					break;

				case 3:
					this->setActualGear(Neutral);
					break;

				case 4:
					this->setActualGear(Drive);
					break;

				default:
					this->setActualGear(None);
					break;
			}
			break;
		case 0x125:// ESC

			break;

		case 0x0E0://SAS
				for(int16_t i = _index;i>0;i--)
				{
					fifo_steering_angle_array[i] = fifo_steering_angle_array[i-1];
				}
				fifo_steering_angle_array[0] = ((int16_t)((dat[0] << 8) | dat[1])) * 0.1f;

				this->setSteeringAngle(fifo_steering_angle_array[_index]);
				this->setSteeringAngleRate(dat[2]*4.0f);

			break;

		case 0x131:
			LonAcc = (uint16_t)((dat[2] << 8) | dat[3]) * 0.001f - 2.0f;
			break;

		default:

			break;
	}
}

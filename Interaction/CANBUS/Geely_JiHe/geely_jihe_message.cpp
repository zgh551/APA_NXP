/*
 * bo_rui_message.cpp
 *
 *  Created on: 2019骞�3鏈�15鏃�
 *      Author: zhuguohua
 */

#include "geely_jihe_message.h"


GeelyJiHeMessage::GeelyJiHeMessage() {
	// TODO Auto-generated constructor stub
}

GeelyJiHeMessage::~GeelyJiHeMessage() {
	// TODO Auto-generated destructor stub
}

void GeelyJiHeMessage::Init()
{
	_index = 29;
}

void GeelyJiHeMessage::Parse(const uint32_t id,const vuint8_t *dat,const vuint32_t lenght)
{
//	uint8_t crc_temp,i;
//	uint8_t dat_temp[7];
	switch(id)
	{
		// the temperature
		case 0x2F1:
			if(0 == (dat[1] >> 1 & 0x01))
			{
				this->setAmbientTemperature(0.5f * dat[1] - 40);
				this->setAmbientTemperatureValid(0xA5);
			}
			break;

		case 0x121:// wheel speed
			this->setVlcActive   (( dat[2]       & 0x01) == 0 ? Inactive    : Active   );
			this->setVlcAvailable(((dat[2] >> 1) & 0x01) == 0 ? NoAvailable : Available);
			this->setVlcCddActive(((dat[1] >> 1) & 0x01) == 0 ? Inactive    : Active   );
			this->setVlcAvailable(( dat[1]       & 0x01) == 0 ? NoAvailable : Available);
			break;

		case 0x122:// wheel speed
			this->setWheelSpeedFrontLeft  (((uint16_t)( (dat[0] << 5) | (dat[1] >> 3))) * V_M_S);
			this->setWheelSpeedFrontRight (((uint16_t)( (dat[2] << 5) | (dat[3] >> 3))) * V_M_S);
			break;

		case 0x123:// wheel speed
			switch((uint8_t)(dat[1] >> 1) & 0x03)
			{
				case 0:
					this->setWheelSpeedDirection(StandStill);
					break;
				case 1:
					this->setWheelSpeedDirection(Forward);
					break;
				case 2:
					this->setWheelSpeedDirection(Backward);
					break;
				case 3:
					this->setWheelSpeedDirection(Invalid);
					break;

				default:
					this->setWheelSpeedDirection(Invalid);
					break;
			}
			this->setWheelSpeedRearLeft (((uint16_t)( (dat[0] << 5) | (dat[1] >> 3))) * V_M_S);
			this->setWheelSpeedRearRight(((uint16_t)( (dat[2] << 5) | (dat[3] >> 3))) * V_M_S);
			break;

		case 0x124://Wheel speed pulse
//			crc_temp = crc8.crcCompute((uint8_t*)dat, 7);
//			if(crc_temp == dat[7])
//			{
			this->setWheelPulseFrontLeft ((uint16_t)(( (dat[0] << 4) | (dat[1] >> 4)) & 0x0fff));
			this->setWheelPulseFrontRight((uint16_t)(( (dat[1] << 8) |  dat[2]      ) & 0x0fff));
			this->setWheelPulseRearLeft  ((uint16_t)(( (dat[3] << 4) | (dat[4] >> 4)) & 0x0fff));
			this->setWheelPulseRearRight ((uint16_t)(( (dat[4] << 8) |  dat[5]      ) & 0x0fff));
//			}
			break;

		// ESC
		case 0x125:
			this->setBrakePedalSts((dat[0] >> 1) & 0x01); // the brake single
			break;

		// ESC
		case 0x126:
			// the vehicle standstill single
			this->setVehicleStandstill((dat[1] >> 6) & 0x03);
			break;

		case 0x127:
			this->setApaActive   (((dat[0] >> 3) & 0x01) == 0 ? Inactive : Active);
			this->setApaCddActive(((dat[3] >> 7) & 0x01) == 0 ? Inactive : Active);
			this->setApaAvailable   (((dat[0] >> 2) & 0x01) == 0 ? NoAvailable : Available);
			this->setApaCddAvailable(((dat[2] >> 4) & 0x01) == 0 ? NoAvailable : Available);
			break;

		case 0x165:
//			switch((uint8_t)(dat[1] & 0x1f))
//			{
//				case 0:
//					this->setActualGear(None);
//					break;
//
//				case 1:
//					this->setActualGear(Parking);
//					break;
//
//				case 2:
//					this->setActualGear(Reverse);
//					break;
//
//				case 3:
//					this->setActualGear(Neutral);
//					break;
//
//				case 4:
//					this->setActualGear(Drive);
//					break;
//
//				default:
//					this->setActualGear(None);
//					break;
//			}
//			break;

			switch((uint8_t)((dat[2] >> 4) & 0x0f))
			{
				case 0:
					this->setActualGear(Neutral);
					break;

				case 0x0A:
					this->setActualGear(Parking);
					break;

				case 0x0B:
					this->setActualGear(Reverse);
					break;

				case 0x0C:
					this->setActualGear(Drive);
					break;

				default:
					this->setActualGear(None);
					break;
			}
			this->setVcuControlStatus((dat[3] >> 4) & 0x01);
			break;

		// VCU
		case 0xA6:
			this->setVcuDriverGearAbort((dat[3] >> 3) & 0x01);
			this->setVcuEptFault       ((dat[3] >> 2) & 0x01);
			break;

		case 0x0E0://SAS
			for(int16_t i = _index; i > 0; i--)
			{
				fifo_steering_angle_array[i] = fifo_steering_angle_array[i-1];
			}
			fifo_steering_angle_array[0] = ((int16_t)((dat[0] << 8) | dat[1])) * 0.1f;

			this->setSteeringAngle(fifo_steering_angle_array[_index]);
			this->setSteeringAngleRate(dat[2] * 4.0f);
			break;

		case 0x131:
			this->setLonAcc((uint16_t)((dat[2] << 8) | dat[3]) * 0.001f - 2.0f);
			break;

		// EPS Status
		case 0x150:
			this->setEPS_RequestFeedback((((dat[2] >> 6) & 0x01) == 0) ? Disable : Enable);
			this->setEPS_AbortFeedback  (  (dat[2] >> 4) & 0x01);
			break;

		default:

			break;
	}
}

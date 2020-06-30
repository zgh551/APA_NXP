/*
 * chery_s51ev_Message.cpp
 *
 *  Created on: 2020年6月29日
 *      Author: zhuguohua
 */

#include <CheryS51EV/chery_s51ev_Message.h>

CheryS51EV_Message::CheryS51EV_Message() {
	// TODO Auto-generated constructor stub

}

CheryS51EV_Message::~CheryS51EV_Message() {
	// TODO Auto-generated destructor stub
}

void CheryS51EV_Message::Init()
{

}

void CheryS51EV_Message::Parse(const uint32_t id,const vuint8_t *data,const vuint32_t lenght)
{
	uint8_t temp_check_sum,i;
	switch(id)
	{
		// EPS
		case 0x1D5:
			temp_check_sum = 0;
			for(i = 0; i < 7; i++)
			{
				temp_check_sum += data[i];
			}
			if(temp_check_sum == data[7])
			{
				this->setEPS_Status( ((data[0] >> 7) & 0x01) == 0 ? ActuatorNormal : ActuatorErr );
				this->setSteeringAngle( ((data[2] << 8) | data[3]) * 0.1f - 1080);
			}
			break;

		// VCU
		case 0x151:
			temp_check_sum = 0;
			for(i = 0; i < 7; i++)
			{
				temp_check_sum += data[i];
			}
			if(temp_check_sum == data[7])
			{
				this->setBrakePedalSts  (data[2] & 0x01);
				this->setAccPedalValid  (((data[2] >> 1) & 0x01) == 0 ? DataValid :DataInvalid);
				this->setBrakePedalValid(((data[2] >> 2) & 0x01) == 0 ? DataValid :DataInvalid);
				this->setTargetGearValid(((data[2] >> 3) & 0x01) == 0 ? DataValid :DataInvalid);
				this->setActualGearValid(((data[2] >> 4) & 0x01) == 0 ? DataValid :DataInvalid);

				// target gear
				if(DataValid == this->getTargetGearValid())
				{
					switch( data[0] & 0x0f )
					{
						case 0:
							this->setTargetGear(None);
							break;

						case 1:
						case 2:
						case 3:
							this->setTargetGear(Drive);
							break;

						case 9:
							this->setTargetGear(Reverse);
							break;

						case 10:
							this->setTargetGear(Neutral);
							break;

						case 11:
							this->setTargetGear(Parking);
							break;

						default:
							this->setTargetGear(None);
							break;
					}
				}

				// actual gear
				if(DataValid == this->getActualGearValid())
				{
					switch((data[0] >> 4) & 0x0f)
					{
						case 0:
							this->setGear(None);
							break;

						case 1:
						case 2:
						case 3:
							this->setGear(Drive);
							break;

						case 9:
							this->setGear(Reverse);
							break;

						case 10:
							this->setGear(Neutral);
							break;

						case 11:
							this->setGear(Parking);
							break;

						default:
							this->setGear(None);
							break;
					}
				}

				this->setVehicleGradient( data[5] * 0.1f - 10 );
			}
			break;

		// wheel speed
		case 0x300:
			this->setWheelSpeedFrontLeftValid (((data[0] >> 6) & 0x03) == 0 ? DataValid :DataInvalid);
			this->setWheelSpeedFrontRightValid(((data[2] >> 6) & 0x03) == 0 ? DataValid :DataInvalid);
			this->setWheelSpeedRearLeftValid  (((data[4] >> 6) & 0x03) == 0 ? DataValid :DataInvalid);
			this->setWheelSpeedRearRightValid (((data[6] >> 6) & 0x03) == 0 ? DataValid :DataInvalid);

			if(DataValid == this->getWheelSpeedFrontLeftValid())
			{
				this->setWheelSpeedFrontLeft ((((data[0] << 8) | data[1]) & 0x3fff) * V_M_S);//m/s
			}
			if(DataValid == this->getWheelSpeedFrontRightValid())
			{
				this->setWheelSpeedFrontRight((((data[2] << 8) | data[3]) & 0x3fff) * V_M_S);//m/s
			}
			if(DataValid == this->getWheelSpeedRearLeftValid())
			{
				this->setWheelSpeedRearLeft  ((((data[4] << 8) | data[5]) & 0x3fff) * V_M_S);//m/s
			}
			if(DataValid == this->getWheelSpeedRearRightValid())
			{
				this->setWheelSpeedRearRight ((((data[6] << 8) | data[7]) & 0x3fff) * V_M_S);//m/s
			}
			break;

		// ESC
		case 0x318:
			this->setESC_Status(((data[1] >> 6) & 0x01) == 0 ? ActuatorNormal : ActuatorErr);
			break;

		// wheel pulse
		case 0x319:
			this->setWheelPulseFrontLeftValid (((data[1] >> 7) & 0x01) == 0 ? DataValid :DataInvalid);
			this->setWheelPulseFrontRightValid(((data[3] >> 7) & 0x01) == 0 ? DataValid :DataInvalid);
			this->setWheelPulseRearLeftValid  (((data[5] >> 7) & 0x01) == 0 ? DataValid :DataInvalid);
			this->setWheelPulseRearRightValid (((data[6] >> 7) & 0x01) == 0 ? DataValid :DataInvalid);

			if(DataValid == this->getWheelPulseFrontLeftValid())
			{
				this->setWheelPulseFrontLeft(data[0]);
			}
			if(DataValid == this->getWheelPulseFrontRightValid())
			{
				this->setWheelPulseFrontRight(data[2]);
			}
			if(DataValid == this->getWheelPulseRearLeftValid())
			{
				this->setWheelPulseRearLeft(data[4]);
			}
			if(DataValid == this->getWheelPulseRearRightValid())
			{
				this->setWheelPulseRearRight(data[6]);
			}
			break;

		//
		case 0x221:
			this->setYawRateValid(( data[4]       & 0x01) == 0 ? DataValid :DataInvalid);
			this->setLonAccValid (((data[4] >> 1) & 0x01) == 0 ? DataValid :DataInvalid);
			this->setLatAccValid (((data[4] >> 2) & 0x01) == 0 ? DataValid :DataInvalid);
			if(DataValid == this->getYawRateValid())
			{
				this->setYawRate(((data[0] << 4) | (data[1] >> 4)) * 0.05f - 100.0f);
			}
			if(DataValid == this->getLonAccValid())
			{
				this->setLonAcc((((data[1] & 0x0f) << 8) | data[2]) * 0.01f - 20.48f);
			}
			if(DataValid == this->getLatAccValid())
			{
				this->setLatAcc(((data[3] << 4) | (data[4] >> 4)) * 0.01f - 20.48f);
			}
			break;
		// belt
		case 0x452:
			this->setDriverSeatBeltSwitchSts(((data[5] >> 1) & 0x01) == 0 ? Fasten : Unfasten);
		break;

		case 0x392:
			this->setTurnLightLeftSts (((data[0] >> 2) & 0x01 == 0) ? Off : On);
			this->setTurnLightRightSts(((data[0] >> 3) & 0x01 == 0) ? Off : On);
			this->setDriverDoorSts    (((data[1] >> 1) & 0x01 == 0) ? Close : Open);
			this->setPassangerDoorSts (((data[1] >> 2) & 0x01 == 0) ? Close : Open);
			this->setTrunkSts         (((data[1] >> 5) & 0x01 == 0) ? Close : Open);
		break;

		// EPB
		case 0x393:
			this->setEPB_Status( ((data[0] >> 2) & 0x03) == 0 ? ActuatorNormal : ActuatorErr);
			this->setEPB_SwitchPositionValid(((data[1] >> 7) & 0x01) == 0 ? DataValid :DataInvalid);
			if(DataValid == this->getEPB_SwitchPositionValid())
			{
				this->setEPB_SwitchPosition(EPB_Status((data[1] >> 5) & 0x03));
			}
		break;

		case 0x0E0://SAS


			break;

		default:

			break;
	}
}

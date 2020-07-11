/*
 * chery_s51ev_controller.cpp
 *
 *  Created on: 2020年6月29日
 *      Author: zhuguohua
 */

#include "chery_s51ev_controller.h"

CheryS51EV_Controller::CheryS51EV_Controller() {
	// TODO Auto-generated constructor stub
	push_time_cnt = 0;
}

CheryS51EV_Controller::~CheryS51EV_Controller() {
	// TODO Auto-generated destructor stub
}

void CheryS51EV_Controller::Init()
{

}
/**
 * @brief start the vehicle controller.
 * @return true if successfully started.
 */
void CheryS51EV_Controller::Start()
{

}

/**
 * @brief stop the vehicle controller.
 */
void CheryS51EV_Controller::Stop()
{

}
  /**
   * @brief update the vehicle controller.
   * @param command the control command
   * @return error_code
   */
void CheryS51EV_Controller::Update(ControlCommand cmd)
{

}

void CheryS51EV_Controller::Update(APAControlCommand cmd)
{
	APAEnable         = cmd.ControlEnable.B.APAEnable;

	Gear              = cmd.Gear;

	SteeringAngle 	  = cmd.SteeringAngle;
	SteeringAngleRate = cmd.SteeringAngleRate;

	Velocity          = cmd.Velocity;
	Distance          = cmd.Distance;
}

void CheryS51EV_Controller::VehicleContorl_10ms(void)
{
	uint8_t temp_check_sum = 0, i;
	uint16_t steering_angle_temp;
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.length = 8;
	/*** IDM2 ***/
	m_CAN_Packet.id = 0x1C5;
	/// Data Mapping
	// Gear Configure
	switch(this->getGear())
	{
		case Drive:
			m_CAN_Packet.data[0] = 0x01;
			break;

		case Reverse:
			m_CAN_Packet.data[0] = 0x09;
			break;

		case Neutral:
			m_CAN_Packet.data[0] = 0x0A;
			break;

		case Parking:
			m_CAN_Packet.data[0] = 0x0B;
			break;

		case None:
			m_CAN_Packet.data[0] = 0x00;
			break;

		default:
			m_CAN_Packet.data[0] = 0x0F;
			break;
	}

	// vehicle speed configure
	m_CAN_Packet.data[1] = this->getVelocity() * 9.0f; // (0 - 20)m/s
	m_CAN_Packet.data[2] = this->getBrakeDegree() * 2.5f; // (0 - 100)%
	m_CAN_Packet.data[3] = this->getAcceleration() * 50.0f;
	steering_angle_temp = (this->getSteeringAngle() +  1080) * 10;
	m_CAN_Packet.data[4] = (steering_angle_temp >> 8) & 0xff;
	m_CAN_Packet.data[5] =  steering_angle_temp & 0xff;
	m_CAN_Packet.data[6] = (this->getDecelerationReq() & 0x01) << 4;
	temp_check_sum = 0;
	for(i = 0; i < 7; i++)
	{
		temp_check_sum += m_CAN_Packet.data[i];
	}
	m_CAN_Packet.data[7] = temp_check_sum;
	CAN0_TransmitMsg(m_CAN_Packet);
}

void CheryS51EV_Controller::VehicleContorl_20ms()
{
	uint8_t temp_check_sum = 0, i;
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.length = 8;

	/*** IDM1 ***/
	m_CAN_Packet.id = 0x2C5;
	/// Data Mapping
	m_CAN_Packet.data[0] =   this->getAPAEnable()
			             | ((this->getEpbReq() & 0x01) << 2);

	m_CAN_Packet.data[1] =  (this->getTurnLightRightReq() & 0x01)
						 | ((this->getTurnLightLeftReq()  & 0x01) << 1);
	m_CAN_Packet.data[2] = 0;
	m_CAN_Packet.data[3] = 0;
	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	temp_check_sum = 0;
	for(i = 0; i < 7; i++)
	{
		temp_check_sum += m_CAN_Packet.data[i];
	}
	m_CAN_Packet.data[7] = temp_check_sum;
	CAN0_TransmitMsg(m_CAN_Packet);
}

// Steeing angle control base on the angle speed
void CheryS51EV_Controller::SteeringAngleControl(float dt)
{

}

// push the command to the vehicle
void CheryS51EV_Controller::DataPush(void)
{
	push_time_cnt = (push_time_cnt + 1) % 2;
	if(0 == push_time_cnt)
	{
		VehicleContorl_20ms();
	}
	VehicleContorl_10ms();
}

void CheryS51EV_Controller::WorkStateMachine(MessageManager& msg)
{
	switch(_control_state)
	{
		case CheryReady:
			if(Ready == msg.getSystemReadyStatus())
			{
				_control_state = CheryAutoDriverActive;
			}
			else
			{
				_control_state = CheryReady;
			}
			break;

		case CheryAutoDriverActive:
			if(AutoMode == msg.getAutoDriverModeStatus() &&
			   AutoMode == msg.getEpsAutoDriverModeStatus())
			{
				_control_state = CheryGearConfigure;
			}
			else
			{
				_control_state = CheryAutoDriverActive;
			}
			break;

		case CheryGearConfigure:
//			this->setGear(Neutral);
//			if(Neutral == msg.getTargetGear())
//			{
				_control_state = CheryExceptionHandling;
//			}
//			else
//			{
//				_control_state = CheryGearConfigure;
//			}
			break;

		case CheryExceptionHandling:
			if(AutoMode == msg.getAutoDriverModeStatus()
			&& AutoMode == msg.getEpsAutoDriverModeStatus()
			   )
			{
				//控制信号握手，超时200ms
//				if(this->getShakeHandsCnt() < 40)
//				{
					this->setShakeHandsCnt( this->getShakeHandsCnt() + 1 );
					// 人为干预 或 执行器故障
					if( ManualControlDetected == msg.getEPS_ManualControlDetectionStatus()
					|| ActuatorErr == msg.getEPS_Status()
					|| ActuatorErr == msg.getESC_Status()
					|| ActuatorErr == msg.getEPB_Status()) 
					{
//						_control_state = CheryAbnormalExit;
					}
					else
					{
						_control_state = CheryExceptionHandling;
					}	
//				}
//				else// shake hand fault
//				{
//					_control_state = CheryAbnormalExit;
//				}
			}
			else
			{
				_control_state = CheryReady;
			}
			break;

		case CheryAbnormalExit:
			if(msg.getVehicleMiddleSpeed() < 1.0e-6f)
			{
				this->setBrakeDegree(90);
				if(Parking == msg.getActualGear())
				{
					if(EPB_SystemApplied == msg.getEPB_SystemStatus())
					{
						this->setAPAEnable(0);
						if(	AutoMode == msg.getAutoDriverModeStatus() &&
			   				AutoMode == msg.getEpsAutoDriverModeStatus())
						{
							_control_state = CheryAbnormalExit;
						}
						else
						{
							_control_state = CheryReady;
						}
					}
					else// request epb
					{
						this->setEpbReq(AppliedRequest);
						_control_state = CheryAbnormalExit;
					}
				}
				else // set parking gear
				{
					this->setGear(Parking);
					_control_state = CheryAbnormalExit;
				}
			}
			else
			{
				this->setVelocity(0.0f);
				_control_state = CheryAbnormalExit;
			}
			break;

		default:
			break;
	}
}

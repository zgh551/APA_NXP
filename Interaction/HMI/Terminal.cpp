/*
 * terminal.cpp
 *
 *  Created on: January 8 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: terminal.cpp                        COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: Interaction terminal            					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 8 2019      Initial Version                  */
/*****************************************************************************/

#include "Terminal.h"

Terminal::Terminal() {
	Init();
}

Terminal::~Terminal() {

}

void Terminal::Init(void)
{
	_frame_err_cnt  = 0;
	_push_active    = 0;
	_work_mode      = 4;
	_function_state = 1;
	_ack_valid      = 0;
	_ack_echo       = 0;
	_check_sum      = 0;
	_command        = 0;
}


/**************************************************************************************/
void Terminal::Parse(vuint32_t id,vuint8_t dat[],VehicleController &ctl)
{
	uint8_t i,check_sum;
	switch(id)
	{
		case 0x516://eps status
			check_sum =0 ;
			for(i=0;i<7;i++){
				check_sum += dat[i];
			}
			check_sum = check_sum ^ 0xFF;
			if(check_sum == dat[7])
			{
//				ctl.gear_enable_	   =  dat[0]       & 0x01;
				ctl.AccelerationEnable = (dat[0] >> 2) & 0x01;
//				ctl.setDecelerationReq((dat[0] >> 4) & 0x01);
				ctl.TorqueEnable       = (dat[0] >> 5) & 0x01;
				ctl.VelocityEnable     = (dat[0] >> 3) & 0x01;
				if((0 == ctl.SteeringEnable) || (0 == ((dat[0] >> 1) & 0x01)))
				{
					ctl.SteeringEnable 	= (dat[0] >> 1) & 0x01;
				}
				ctl.Gear 				= (GearStatus)dat[1];
				ctl.SteeringAngle 		= (float)(((int16_t)((dat[3] << 8) | dat[2])) * 0.1);
				ctl.SteeringAngleRate 	= (float)(((uint16_t)((dat[5] << 8) | dat[4])) * 0.01);
				_ack_valid = 0xa5;
			}
			break;

		case 0x517://eps status
			check_sum =0 ;
			for(i=0;i<7;i++){
				check_sum += dat[i];
			}
			check_sum = check_sum ^ 0xFF;
			if(check_sum == dat[7])
			{
				ctl.Acceleration	= (float)(((int16_t )((dat[1] << 8) | dat[0])) * 0.001);
//				ctl.TargetAcceleration	= (float)(((int16_t )((dat[1] << 8) | dat[0])) * 0.001);
				ctl.Deceleration	= (float)(((int16_t )((dat[3] << 8) | dat[2])) * 0.001);
				ctl.Velocity		= (float)(((uint16_t)((dat[5] << 8) | dat[4])) * 0.001);
				ctl.Torque			= (float)(dat[6] * 2);
			}
			break;

		case 0x518:
			check_sum =0 ;
			for(i=0;i<7;i++){
				check_sum += dat[i];
			}
			check_sum = check_sum ^ 0xFF;
			if(check_sum == dat[7])
			{
				ctl.setSteeringAngle(((int16_t)((dat[1] << 8) | dat[0])) * 0.1f);
				ctl.setSteeringAngleRate(dat[2] * 4.0f);
				ctl.setVelocity(dat[3] * 0.01f);
				ctl.setDistance(((uint16_t)((dat[5] << 8) | dat[4])) * 0.001f);
				ctl.setGear((GearStatus)(dat[6] & 0x0f));
				ctl.setAPAEnable((uint8_t)((dat[6]>>4) & 0x03));
				setAckValid(0xa5);
				ctl.setShakeHandsCnt(0);
			}
			break;

		case 0x519://eps status
			check_sum =0 ;
			for(i=0;i<7;i++){
				check_sum += dat[i];
			}
			check_sum = check_sum ^ 0xFF;
			if(check_sum == dat[7])
			{
				ctl.SteeringAngle 		= (float)(((int16_t)((dat[1] << 8) | dat[0])) * 0.1);
				ctl.SteeringAngleRate 	= (float)(((uint16_t)((dat[3] << 8) | dat[2])) * 0.01);
				ctl.Torque			    = (float)(((uint16_t)((dat[5] << 8) | dat[4])));

//				ctl.GearEnable 		=  dat[6]       & 0x01;
				ctl.SteeringEnable 	= (dat[6] >> 1) & 0x01;
				ctl.AccelerationEnable = (dat[6] >> 2) & 0x01;
				ctl.VelocityEnable     = (dat[6] >> 3) & 0x01;
				ctl.setDecelerationReq((dat[6] >> 4) & 0x01);
				ctl.TorqueEnable       = (dat[6] >> 5) & 0x01;
				_ack_valid = 0xa5;
			}
			break;

		case 0x51A://eps status
			check_sum =0 ;
			for(i=0;i<7;i++){
				check_sum += dat[i];
			}
			check_sum = check_sum ^ 0xFF;
			if(check_sum == dat[7])
			{
				ctl.Acceleration	= (float)(((int16_t )((dat[1] << 8) | dat[0])) * 0.001);
//				ctl.TargetAcceleration	= (float)(((int16_t )((dat[1] << 8) | dat[0])) * 0.001);
				ctl.Deceleration	= (float)(((int16_t )((dat[3] << 8) | dat[2])) * 0.001);
				ctl.Velocity		= (float)(((uint16_t)((dat[5] << 8) | dat[4])) * 0.001);
				ctl.Gear 			= (GearStatus)dat[6];
			}
			break;

		case 0x51B://emerger status
			check_sum =0 ;
			for(i=0;i<7;i++){
				check_sum += dat[i];
			}
			check_sum = check_sum ^ 0xFF;
			if(check_sum == dat[7])
			{
				ctl.setBrakeDegree(dat[0] * 0.4f);
				ctl.setDecelerationReq(dat[1] & 0x01);
				ctl.setEpbReq((dat[1] & 0x01) == 0 ? ReleaseRequest : AppliedRequest);
			}
			break;
		default:

			break;
	}
}

void Terminal::Parse(vuint32_t id,vuint8_t dat[],Ultrasonic &u)
{
	Ultrasonic_Packet ultrasonic_packet;
	ObstacleLocationPacket obstacle_location_packet;
	switch(id)
	{
        case 0x508://传感器9
        case 0x509://传感器10
        case 0x50A://传感器11
        case 0x50B://传感器12
        	ultrasonic_packet.Distance1 = (float)(((uint16_t )((dat[1] << 8) | dat[0])) * 0.01);
        	ultrasonic_packet.Distance2 = (float)(((uint16_t )((dat[3] << 8) | dat[2])) * 0.01);
        	ultrasonic_packet.Level = dat[4] * 0.1;
        	ultrasonic_packet.Width = dat[5];
        	ultrasonic_packet.status = dat[6];
        	u.setUltrasonicPacket(id & 0x00f,ultrasonic_packet);
			break;

        case 0x50C:
        	obstacle_location_packet.Position.setX((float)(((int16_t )((dat[1] << 8) | dat[0])) * 0.01));
        	obstacle_location_packet.Position.setY((float)(((int16_t )((dat[3] << 8) | dat[2])) * 0.01));
        	obstacle_location_packet.Status = (UltrasonicStatus)dat[7];
        	u.setAbstacleGroundPositionTriangle(5, obstacle_location_packet);
        	break;

        case 0x50D:
        	obstacle_location_packet.Position.setX((float)(((int16_t )((dat[1] << 8) | dat[0])) * 0.01));
        	obstacle_location_packet.Position.setY((float)(((int16_t )((dat[3] << 8) | dat[2])) * 0.01));
        	obstacle_location_packet.Status = (UltrasonicStatus)dat[7];
        	u.setAbstacleGroundPositionTriangle(6, obstacle_location_packet);
        	break;

        case 0x50E:
        	obstacle_location_packet.Position.setX((float)(((int16_t )((dat[1] << 8) | dat[0])) * 0.01));
        	obstacle_location_packet.Position.setY((float)(((int16_t )((dat[3] << 8) | dat[2])) * 0.01));
        	obstacle_location_packet.Status = (UltrasonicStatus)dat[7];
        	u.setAbstacleGroundPositionTriangle(10, obstacle_location_packet);
        	break;

        case 0x50F:
        	obstacle_location_packet.Position.setX((float)(((int16_t )((dat[1] << 8) | dat[0])) * 0.01));
        	obstacle_location_packet.Position.setY((float)(((int16_t )((dat[3] << 8) | dat[2])) * 0.01));
        	obstacle_location_packet.Status = (UltrasonicStatus)dat[7];
        	u.setAbstacleGroundPositionTriangle(11, obstacle_location_packet);
        	_ack_valid = 0xa5;
        	break;

		default:

			break;
	}
}

void Terminal::Parse(vuint32_t id,vuint8_t dat[],MessageManager &msg)
{
//	Byte2Int temp_int;
	switch(id)
	{
        case 0x510:
//        	temp_int.b[1] = dat[2];
//        	temp_int.b[0] = dat[3];
//        	msg->setSteeringAngle(temp_int.i16 * 0.1);
        	break;

        case 0x520:
//        	temp_int.b[1] = dat[4];
//        	temp_int.b[0] = dat[5];
//        	msg->WheelSpeedRearLeft = temp_int.u16 * 0.001;
//        	temp_int.b[1] = dat[6];
//        	temp_int.b[0] = dat[7];
//        	msg->WheelSpeedRearRight = temp_int.u16 * 0.001;
        	break;
		default:

			break;
	}
}

void Terminal::Parse(vuint32_t id,vuint8_t dat[],PID &msg)
{
	Byte2Float temp_float;
	switch(id)
	{
		case 0x550:
			temp_float.b[0] = dat[0];
			temp_float.b[1] = dat[1];
			temp_float.b[2] = dat[2];
			temp_float.b[3] = dat[3];
			msg.KP = temp_float.f;
			temp_float.b[0] = dat[4];
			temp_float.b[1] = dat[5];
			temp_float.b[2] = dat[6];
			temp_float.b[3] = dat[7];
			msg.KI = temp_float.f;
			break;

		case 0x551:
			temp_float.b[0] = dat[0];
			temp_float.b[1] = dat[1];
			temp_float.b[2] = dat[2];
			temp_float.b[3] = dat[3];
			msg.KD = temp_float.f;
			temp_float.b[0] = dat[4];
			temp_float.b[1] = dat[5];
			temp_float.b[2] = dat[6];
			temp_float.b[3] = dat[7];
			msg.OutputLimit = temp_float.f;
			break;

		case 0x552:
			temp_float.b[0] = dat[0];
			temp_float.b[1] = dat[1];
			temp_float.b[2] = dat[2];
			temp_float.b[3] = dat[3];
			msg.Threshold = temp_float.f;
			temp_float.b[0] = dat[4];
			temp_float.b[1] = dat[5];
			temp_float.b[2] = dat[6];
			temp_float.b[3] = dat[7];
			msg.ILimit = temp_float.f;
			break;

		default:
			break;
	}
}

void Terminal::Parse(vuint32_t id,vuint8_t dat[])
{
	switch(id)
	{
        case 0x530:
        	_command = (uint8_t)dat[0];
        	break;

        case 0x531:
        	_work_mode      = (uint8_t)dat[0];
        	_function_state = (uint8_t)dat[1];
        	break;

        case 0x512:
        	_ack_echo = (uint8_t)dat[0];
        	break;

        default:
        	break;
	}
}

void Parse(vuint32_t id,vuint8_t dat[],LatControl *lat_ctl)
{
	switch(id)
	{
        case 0x5B0:
        	lat_ctl->setCommand((uint8_t)dat[0]);
        	break;

        default:
        	break;
	}
}
/**************************************************************************************/
/**
 * 推送车辆信息
 */
void Terminal::Push(MessageManager &msg)
{
	CAN_Packet m_CAN_Packet;
	int16_t  temp_int16;
	uint16_t temp_uint16;
	Byte2Int32 temp_int32;

	m_CAN_Packet.length = 8;
	m_CAN_Packet.id = 0x410;
	temp_uint16 = msg.getVehicleMiddleSpeed() * 1000;
	m_CAN_Packet.data[0] =  temp_uint16 & 0xff;
	m_CAN_Packet.data[1] = (temp_uint16 >> 8) & 0xff;
	temp_int16 = (int16_t)(msg.getSteeringAngle() * 10);
	m_CAN_Packet.data[2] =  temp_int16 & 0xff;
	m_CAN_Packet.data[3] = (temp_int16 >> 8) & 0xff;
	temp_uint16 = (uint16_t)(msg.getSteeringAngleRate() * 100);
	m_CAN_Packet.data[4] =  temp_uint16 & 0xff;
	m_CAN_Packet.data[5] = (temp_uint16 >> 8) & 0xff;
	m_CAN_Packet.data[6] = msg.getActualGear();
	m_CAN_Packet.data[7] = (msg.getWheelSpeedDirection() & 0x03)
			             | ((msg.getSystemReadyStatus()    & 0x01) << 2)
						 | ((msg.getAutoDriverModeStatus() & 0x01) << 3);
	CAN2_TransmitMsg(m_CAN_Packet);

	// 车速状态反馈
	m_CAN_Packet.id = 0x411;
	temp_uint16 = (uint16_t)(msg.getWheelSpeedFrontLeft() * 1000);
	m_CAN_Packet.data[0] =  temp_uint16 & 0xff;
	m_CAN_Packet.data[1] = (temp_uint16 >> 8) & 0xff;
	temp_uint16 = (uint16_t)(msg.getWheelSpeedFrontRight() * 1000);
	m_CAN_Packet.data[2] =  temp_uint16 & 0xff;
	m_CAN_Packet.data[3] = (temp_uint16 >> 8) & 0xff;
	temp_uint16 = (uint16_t)(msg.getWheelSpeedRearLeft() * 1000);
	m_CAN_Packet.data[4] =  temp_uint16 & 0xff;
	m_CAN_Packet.data[5] = (temp_uint16 >> 8) & 0xff;
	temp_uint16 = (uint16_t)(msg.getWheelSpeedRearRight() * 1000);
	m_CAN_Packet.data[6] =  temp_uint16 & 0xff;
	m_CAN_Packet.data[7] = (temp_uint16 >> 8) & 0xff;
	CAN2_TransmitMsg(m_CAN_Packet);

	// 车速状态反馈
	m_CAN_Packet.id = 0x412;
	temp_uint16 = msg.getWheelPulseFrontLeft();
	m_CAN_Packet.data[0] =  temp_uint16 & 0xff;
	m_CAN_Packet.data[1] = (temp_uint16 >> 8) & 0xff;
	temp_uint16 = msg.getWheelPulseFrontRight();
	m_CAN_Packet.data[2] =  temp_uint16 & 0xff;
	m_CAN_Packet.data[3] = (temp_uint16 >> 8) & 0xff;
	temp_uint16 = msg.getWheelPulseRearLeft();
	m_CAN_Packet.data[4] =  temp_uint16 & 0xff;
	m_CAN_Packet.data[5] = (temp_uint16 >> 8) & 0xff;
	temp_uint16 = msg.getWheelPulseRearRight();
	m_CAN_Packet.data[6] =  temp_uint16 & 0xff;
	m_CAN_Packet.data[7] = (temp_uint16 >> 8) & 0xff;
	CAN2_TransmitMsg(m_CAN_Packet);

	// the pulse sum
	m_CAN_Packet.id = 0x413;
	temp_int32.i32 = msg.getRearLeftSumPulse();
	m_CAN_Packet.data[0] = temp_int32.b[3];
	m_CAN_Packet.data[1] = temp_int32.b[2];
	m_CAN_Packet.data[2] = temp_int32.b[1];
	m_CAN_Packet.data[3] = temp_int32.b[0];
	temp_int32.i32 = msg.getRearRightSumPulse();
	m_CAN_Packet.data[4] = temp_int32.b[3];
	m_CAN_Packet.data[5] = temp_int32.b[2];
	m_CAN_Packet.data[6] = temp_int32.b[1];
	m_CAN_Packet.data[7] = temp_int32.b[0];
	CAN2_TransmitMsg(m_CAN_Packet);

	m_CAN_Packet.id = 0x417;
	temp_int16 = (int16_t)(msg.getLonAcc() * 1000);
	m_CAN_Packet.data[0] =  temp_int16 & 0xff ;
	m_CAN_Packet.data[1] = (temp_int16 >> 8) & 0xff ;
	temp_int16 = (int16_t)(msg.getLatAcc() * 1000);
	m_CAN_Packet.data[2] =  temp_int16 & 0xff;
	m_CAN_Packet.data[3] = (temp_int16 >> 8) & 0xff;
	temp_int16 = (int16_t)(msg.getYawRate() * 100);
	m_CAN_Packet.data[4] =  temp_int16 & 0xff;
	m_CAN_Packet.data[5] = (temp_int16 >> 8) & 0xff;
	temp_int16 = (int16_t)(msg.getAmbientTemperature() * 10);
	m_CAN_Packet.data[6] =  temp_int16 & 0xff;
	m_CAN_Packet.data[7] = (temp_int16 >> 8) & 0xff;
	CAN2_TransmitMsg(m_CAN_Packet);

	m_CAN_Packet.id = 0x418;
	m_CAN_Packet.data[0] = ( msg.getEPS_Status() & 0x01 )
						 | ( msg.getESC_Status() & 0x01 ) << 1
						 | ( msg.getEPB_Status() & 0x01 ) << 2
						 | ( msg.getVCU_Status() & 0x01 ) << 3
						 | ( msg.getSAS_Status() & 0x01 ) << 4
						 | ( msg.getTCU_Status() & 0x01 ) << 5
						 | ( msg.getEMS_Status() & 0x01 ) << 6;
						 
	m_CAN_Packet.data[1] = ( msg.getDriverDoorSts() & 0x01 ) 
						 | ( msg.getPassangerDoorSts() & 0x01) << 1
						 | ( msg.getTrunkSts() & 0x01 ) << 2;

	m_CAN_Packet.data[2] = ( msg.getTurnLightLeftSts() & 0x01 )
						 | ( msg.getTurnLightRightSts() & 0x01 ) << 1;

	m_CAN_Packet.data[3] = msg.getDriverSeatBeltSwitchSts() & 0x01; 
	m_CAN_Packet.data[4] = msg.getEPB_SwitchPosition();
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);
}

/*
 * 控制信号
 * */
void Terminal::Push(VehicleController &msg)
{
	CAN_Packet m_CAN_Packet;
	int16_t temp_int16;
	uint16_t temp_uint16;

	m_CAN_Packet.id = 0x414;
	m_CAN_Packet.length = 8;
	m_CAN_Packet.data[0] = 	 msg.getAccelerationEnable() 	   |
							(msg.getDecelerationReq() << 1) |
							(msg.getTorqueEnable()       << 2) |
							(msg.getVelocityEnable()     << 3) |
							(msg.getSteeringEnable()     << 4) ;
//							(msg.getGearEnable()         << 6) ;
	m_CAN_Packet.data[1] = msg.getGear();
//	temp_int16 = (int16_t)(msg.getAcceleration() * 100);
	temp_int16 = (int16_t)(msg.getTargetAcceleration() * 100);
	m_CAN_Packet.data[2] =  temp_int16       & 0xff ;
	m_CAN_Packet.data[3] = (temp_int16 >> 8) & 0xff ;
	temp_uint16 = (uint16_t)msg.getTorque();
	m_CAN_Packet.data[4] =  temp_uint16       & 0xff ;
	m_CAN_Packet.data[5] = (temp_uint16 >> 8) & 0xff ;
	temp_uint16 = (uint16_t)(msg.getVelocity() * 100);
	m_CAN_Packet.data[6] =  temp_uint16       & 0xff ;
	m_CAN_Packet.data[7] = (temp_uint16 >> 8) & 0xff ;
	CAN2_TransmitMsg(m_CAN_Packet);

	m_CAN_Packet.id = 0x415;
	temp_uint16 = (uint16_t)(msg.getDistance() * 1000);
	m_CAN_Packet.data[0] =  temp_uint16 & 0xff ;
	m_CAN_Packet.data[1] = (temp_uint16 >> 8) & 0xff ;
	temp_int16 = (int16_t)(msg.getTargetAcceleration() * 1000);
	m_CAN_Packet.data[2] =  temp_int16 & 0xff;
	m_CAN_Packet.data[3] = (temp_int16 >> 8) & 0xff;
	temp_int16 = (int16_t)(msg.getSteeringAngle() * 10);
	m_CAN_Packet.data[4] =  temp_int16 & 0xff;
	m_CAN_Packet.data[5] = (temp_int16 >> 8) & 0xff;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);
}

void Terminal::Push(VehicleState &msg)
{
	CAN_Packet m_CAN_Packet;
	Byte2Int temp_int;
	m_CAN_Packet.id = 0x442;
	m_CAN_Packet.length = 8;

	temp_int.i16 = (int16_t)(msg.getPosition().getX() * 100);
	m_CAN_Packet.data[0] = temp_int.b[1];
	m_CAN_Packet.data[1] = temp_int.b[0];
	temp_int.i16 = (int16_t)(msg.getPosition().getY() * 100);
	m_CAN_Packet.data[2] = temp_int.b[1];
	m_CAN_Packet.data[3] = temp_int.b[0];
	temp_int.i16 = (int16_t)(msg.Yaw * 100);
	m_CAN_Packet.data[4] = temp_int.b[1];
	m_CAN_Packet.data[5] = temp_int.b[0];

	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);

	m_CAN_Packet.id = 0x44E;
	m_CAN_Packet.length = 8;

	temp_int.u16 = (uint16_t)(msg.getPulseUpdateVelocity() * 10000);
	m_CAN_Packet.data[0] = temp_int.b[1];
	m_CAN_Packet.data[1] = temp_int.b[0];
	temp_int.u16 = (uint16_t)(msg.getAccUpdateVelocity() * 10000);
	m_CAN_Packet.data[2] = temp_int.b[1];
	m_CAN_Packet.data[3] = temp_int.b[0];

	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);
}

void Terminal::Push(Ultrasonic &u)
{
#if ULTRASONIC_PACKET == 1

#if ULTRASONIC_SCHEDULE_MODO == 2
	switch(u.ScheduleTimeCnt)
	{
		case 7:
			UltrasonicSend(1,u.UltrasonicPacket);
			UltrasonicSend(7,u.UltrasonicPacket);
			break;

		case 10:
		case 23:
			UltrasonicSend(8,u.UltrasonicPacket);
			UltrasonicSend(11,u.UltrasonicPacket);
			break;

		case 12:
		case 25:
			UltrasonicSend(9,u.UltrasonicPacket);
			UltrasonicSend(10,u.UltrasonicPacket);
			break;

		case 13:
			UltrasonicSend(3,u.UltrasonicPacket);
			UltrasonicSend(5,u.UltrasonicPacket);
			break;

		case 20:
			UltrasonicSend(0,u.UltrasonicPacket);
			UltrasonicSend(6,u.UltrasonicPacket);
			break;

		case 0:
			UltrasonicSend(2,u.UltrasonicPacket);
			UltrasonicSend(4,u.UltrasonicPacket);
			break;

		default:
			break;
	}
#endif

#if ULTRASONIC_SCHEDULE_MODO == 3
	switch(u.ScheduleTimeCnt)
	{
		case 9:
			// 直接测量的传感器值
			UltrasonicSend(1,u.UltrasonicPacket);
			UltrasonicSend(6,u.UltrasonicPacket);

			// 三角定位测量的传感器值
			UltrasonicLocationSend(0 ,u.UltrasonicLocationPacket);
			UltrasonicLocationSend(1 ,u.UltrasonicLocationPacket);
			UltrasonicLocationSend(2 ,u.UltrasonicLocationPacket);
			UltrasonicLocationSend(9 ,u.UltrasonicLocationPacket);
			UltrasonicLocationSend(10,u.UltrasonicLocationPacket);
			UltrasonicLocationSend(11,u.UltrasonicLocationPacket);

			// 三角定位车体坐标系
			UltrasonicBodyLocationSend(0,u.AbstacleBodyPositionTriangle[0]);
			UltrasonicBodyLocationSend(1,u.AbstacleBodyPositionTriangle[1]);
			UltrasonicBodyLocationSend(2,u.AbstacleBodyPositionTriangle[2]);
			UltrasonicBodyLocationSend(3,u.AbstacleBodyPositionTriangle[3]);
			// 三角定位地面坐标系
			UltrasonicGroundLocationSend(0,u.AbstacleGroundPositionTriangle[0]);
			UltrasonicGroundLocationSend(1,u.AbstacleGroundPositionTriangle[1]);
			UltrasonicGroundLocationSend(2,u.AbstacleGroundPositionTriangle[2]);
			UltrasonicGroundLocationSend(3,u.AbstacleGroundPositionTriangle[3]);
			break;

		case 23:
			UltrasonicSend(2,u.UltrasonicPacket);
			UltrasonicSend(5,u.UltrasonicPacket);
			// 三角定位测量的传感器值
			UltrasonicLocationSend(3,u.UltrasonicLocationPacket);
			UltrasonicLocationSend(4,u.UltrasonicLocationPacket);
			UltrasonicLocationSend(5,u.UltrasonicLocationPacket);
			UltrasonicLocationSend(6,u.UltrasonicLocationPacket);
			UltrasonicLocationSend(7,u.UltrasonicLocationPacket);
			UltrasonicLocationSend(8,u.UltrasonicLocationPacket);
			// 三角定位车体坐标系
			UltrasonicBodyLocationSend(4,u.AbstacleBodyPositionTriangle[4]);
			UltrasonicBodyLocationSend(5,u.AbstacleBodyPositionTriangle[5]);
			UltrasonicBodyLocationSend(6,u.AbstacleBodyPositionTriangle[6]);
			UltrasonicBodyLocationSend(7,u.AbstacleBodyPositionTriangle[7]);
			// 三角定位地面坐标系
			UltrasonicGroundLocationSend(4,u.AbstacleGroundPositionTriangle[4]);
			UltrasonicGroundLocationSend(5,u.AbstacleGroundPositionTriangle[5]);
			UltrasonicGroundLocationSend(6,u.AbstacleGroundPositionTriangle[6]);
			UltrasonicGroundLocationSend(7,u.AbstacleGroundPositionTriangle[7]);
			break;

		case 11:
		case 25:
			UltrasonicSend(8 ,u.UltrasonicPacket);
			UltrasonicSend(10,u.UltrasonicPacket);

			UltrasonicBodyLocationSend(8,u.AbstacleBodyPositionDirect[8]);
			UltrasonicBodyLocationSend(10,u.AbstacleBodyPositionDirect[10]);

			UltrasonicGroundLocationSend(8,u.AbstacleGroundPositionTriangle[8]);
			UltrasonicGroundLocationSend(10,u.AbstacleGroundPositionTriangle[10]);
		break;

		case 13:
		case 27:
			UltrasonicSend( 9,u.UltrasonicPacket);
			UltrasonicSend(11,u.UltrasonicPacket);

			UltrasonicBodyLocationSend(9,u.AbstacleBodyPositionDirect[9]);
			UltrasonicBodyLocationSend(11,u.AbstacleBodyPositionDirect[11]);

			UltrasonicGroundLocationSend(9,u.AbstacleGroundPositionTriangle[9]);
			UltrasonicGroundLocationSend(11,u.AbstacleGroundPositionTriangle[11]);
			break;

		case 14:
			UltrasonicSend(3,u.UltrasonicPacket);
			UltrasonicSend(4,u.UltrasonicPacket);
			break;

		case 0:
			UltrasonicSend(0,u.UltrasonicPacket);
			UltrasonicSend(7,u.UltrasonicPacket);
			break;

		default:
			break;
	}
#endif
#if ULTRASONIC_SCHEDULE_MODO == 4
	if(0 == _function_state)
	{
		switch(u.ScheduleTimeCnt)
		{
			case 6:
				UltrasonicSend(1,u.UltrasonicPacket);
				UltrasonicSend(6,u.UltrasonicPacket);
			break;

			case 14:
				UltrasonicSend(3,u.UltrasonicPacket);
				UltrasonicSend(4,u.UltrasonicPacket);
			break;

			case 22:
				UltrasonicSend(0,u.UltrasonicPacket);
				UltrasonicSend(7,u.UltrasonicPacket);
			break;

			case 30:
				UltrasonicSend(2,u.UltrasonicPacket);
				UltrasonicSend(5,u.UltrasonicPacket);
			break;

			case 7:
			case 15:
			case 23:
			case 31:
				UltrasonicSend(8 ,u.UltrasonicPacket);
				UltrasonicSend(10,u.UltrasonicPacket);
			break;

			case 8:
			case 16:
			case 24:
			case 0:
				UltrasonicSend(9,u.UltrasonicPacket);
				UltrasonicSend(11,u.UltrasonicPacket);
			break;

			default:
				break;
		}
	}
	else if(1 == _function_state)
	{
		switch(u.ScheduleTimeCnt)
		{
			case 0:
				UltrasonicSend(0,u.UltrasonicPacket);
				UltrasonicSend(7,u.UltrasonicPacket);
				break;

			case 8:
				// 直接测量的传感器值
				UltrasonicSend(1,u.UltrasonicPacket);
				UltrasonicSend(6,u.UltrasonicPacket);

				// 三角定位测量的传感器值
				UltrasonicLocationSend(0 ,u.UltrasonicLocationPacket);
				UltrasonicLocationSend(1 ,u.UltrasonicLocationPacket);
				UltrasonicLocationSend(2 ,u.UltrasonicLocationPacket);
				UltrasonicLocationSend(9 ,u.UltrasonicLocationPacket);
				UltrasonicLocationSend(10,u.UltrasonicLocationPacket);
				UltrasonicLocationSend(11,u.UltrasonicLocationPacket);

				// 三角定位车体坐标系
				UltrasonicBodyLocationSend(0,u.AbstacleBodyPositionTriangle[0]);
				UltrasonicBodyLocationSend(1,u.AbstacleBodyPositionTriangle[1]);
				UltrasonicBodyLocationSend(2,u.AbstacleBodyPositionTriangle[2]);
				UltrasonicBodyLocationSend(3,u.AbstacleBodyPositionTriangle[3]);
				// 三角定位地面坐标系
//				UltrasonicGroundLocationSend(0,u.AbstacleGroundPositionTriangle[0]);
//				UltrasonicGroundLocationSend(1,u.AbstacleGroundPositionTriangle[1]);
//				UltrasonicGroundLocationSend(2,u.AbstacleGroundPositionTriangle[2]);
//				UltrasonicGroundLocationSend(3,u.AbstacleGroundPositionTriangle[3]);
				break;

			case 19:
				UltrasonicSend(2,u.UltrasonicPacket);
				UltrasonicSend(5,u.UltrasonicPacket);
				// 三角定位测量的传感器值
				UltrasonicLocationSend(3,u.UltrasonicLocationPacket);
				UltrasonicLocationSend(4,u.UltrasonicLocationPacket);
				UltrasonicLocationSend(5,u.UltrasonicLocationPacket);
				UltrasonicLocationSend(6,u.UltrasonicLocationPacket);
				UltrasonicLocationSend(7,u.UltrasonicLocationPacket);
				UltrasonicLocationSend(8,u.UltrasonicLocationPacket);
				// 三角定位车体坐标系
				UltrasonicBodyLocationSend(4,u.AbstacleBodyPositionTriangle[4]);
				UltrasonicBodyLocationSend(5,u.AbstacleBodyPositionTriangle[5]);
				UltrasonicBodyLocationSend(6,u.AbstacleBodyPositionTriangle[6]);
				UltrasonicBodyLocationSend(7,u.AbstacleBodyPositionTriangle[7]);
//				// 三角定位地面坐标系
//				UltrasonicGroundLocationSend(4,u.AbstacleGroundPositionTriangle[4]);
//				UltrasonicGroundLocationSend(5,u.AbstacleGroundPositionTriangle[5]);
//				UltrasonicGroundLocationSend(6,u.AbstacleGroundPositionTriangle[6]);
//				UltrasonicGroundLocationSend(7,u.AbstacleGroundPositionTriangle[7]);
				break;

			case 9:
			case 20:
				UltrasonicSend(8 ,u.UltrasonicPacket);
				UltrasonicSend(10,u.UltrasonicPacket);

				UltrasonicBodyLocationSend(8,u.AbstacleBodyPositionDirect[8]);
				UltrasonicBodyLocationSend(10,u.AbstacleBodyPositionDirect[10]);

//				UltrasonicGroundLocationSend(8,u.AbstacleGroundPositionTriangle[8]);
//				UltrasonicGroundLocationSend(10,u.AbstacleGroundPositionTriangle[10]);
			break;

			case 10:
			case 21:
				UltrasonicSend(9,u.UltrasonicPacket);
				UltrasonicSend(11,u.UltrasonicPacket);

				UltrasonicBodyLocationSend(9,u.AbstacleBodyPositionDirect[9]);
				UltrasonicBodyLocationSend(11,u.AbstacleBodyPositionDirect[11]);

//				UltrasonicGroundLocationSend(9,u.AbstacleGroundPositionTriangle[9]);
//				UltrasonicGroundLocationSend(11,u.AbstacleGroundPositionTriangle[11]);
				break;

			case 11:
				UltrasonicSend(3,u.UltrasonicPacket);
				UltrasonicSend(4,u.UltrasonicPacket);
				break;



			default:
				break;
		}
	}
#endif
#else
#if ULTRASONIC_SCHEDULE_MODO == 2
	switch(u.ReadStage)
	{
		case 0:
			UltrasonicSend(1,u.UltrasonicDatas);
			UltrasonicSend(7,u.UltrasonicDatas);
			break;

		case 1:
		case 5:
			UltrasonicSend(8,u.UltrasonicDatas);
			UltrasonicSend(11,u.UltrasonicDatas);
			break;

		case 2:
		case 6:
			UltrasonicSend(9,u.UltrasonicDatas);
			UltrasonicSend(10,u.UltrasonicDatas);
			break;

		case 3:
			UltrasonicSend(3,u.UltrasonicDatas);
			UltrasonicSend(5,u.UltrasonicDatas);
			break;

		case 4:
			UltrasonicSend(0,u.UltrasonicDatas);
			UltrasonicSend(6,u.UltrasonicDatas);
			break;

		case 7:
			UltrasonicSend(2,u.UltrasonicDatas);
			UltrasonicSend(4,u.UltrasonicDatas);
			break;

		default:
			break;
	}
#endif

#if ULTRASONIC_SCHEDULE_MODO == 3
	switch(u.ReadStage)
	{
		case 0:
			UltrasonicSend(1,u.UltrasonicDatas);
			UltrasonicSend(6,u.UltrasonicDatas);
			UltrasonicLocationSend(0,u.UltrasonicLocationDatas);
			UltrasonicLocationSend(1,u.UltrasonicLocationDatas);
			UltrasonicLocationSend(2,u.UltrasonicLocationDatas);
			UltrasonicLocationSend(9,u.UltrasonicLocationDatas);
			UltrasonicLocationSend(10,u.UltrasonicLocationDatas);
			UltrasonicLocationSend(11,u.UltrasonicLocationDatas);
			break;

		case 1:
		case 5:
			UltrasonicSend(8,u.UltrasonicDatas);
			UltrasonicSend(10,u.UltrasonicDatas);
			break;

		case 2:
		case 6:
			UltrasonicSend(9,u.UltrasonicDatas);
			UltrasonicSend(11,u.UltrasonicDatas);
			break;

		case 3:
			UltrasonicSend(3,u.UltrasonicDatas);
			UltrasonicSend(4,u.UltrasonicDatas);
			break;

		case 4:
			UltrasonicSend(2,u.UltrasonicDatas);
			UltrasonicSend(5,u.UltrasonicDatas);
			UltrasonicLocationSend(3,u.UltrasonicLocationDatas);
			UltrasonicLocationSend(4,u.UltrasonicLocationDatas);
			UltrasonicLocationSend(5,u.UltrasonicLocationDatas);
			UltrasonicLocationSend(6,u.UltrasonicLocationDatas);
			UltrasonicLocationSend(7,u.UltrasonicLocationDatas);
			UltrasonicLocationSend(8,u.UltrasonicLocationDatas);
			break;

		case 7:
			UltrasonicSend(0,u.UltrasonicDatas);
			UltrasonicSend(7,u.UltrasonicDatas);
			break;

		default:
			break;
	}
#endif
#endif
}

void Terminal::Push(LonControl &lon_control)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x4A0;
	m_CAN_Packet.length = 8;

	m_CAN_Packet.data[0] = lon_control.getControlStateFlag();
	m_CAN_Packet.data[1] = 0;
	m_CAN_Packet.data[2] = 0;
	m_CAN_Packet.data[3] = 0;
	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);
}

void Terminal::Push(LatControl &lat_control)
{
	CAN_Packet m_CAN_Packet;
	Byte2Float temp_float;

	m_CAN_Packet.length = 8;

	m_CAN_Packet.id = 0x4B0;
	temp_float.f = lat_control.getTargetTrack().point.getX();
	m_CAN_Packet.data[0] = temp_float.b[3];
	m_CAN_Packet.data[1] = temp_float.b[2];
	m_CAN_Packet.data[2] = temp_float.b[1];
	m_CAN_Packet.data[3] = temp_float.b[0];
	temp_float.f = lat_control.getTargetTrack().point.getY();
	m_CAN_Packet.data[4] = temp_float.b[3];
	m_CAN_Packet.data[5] = temp_float.b[2];
	m_CAN_Packet.data[6] = temp_float.b[1];
	m_CAN_Packet.data[7] = temp_float.b[0];
	CAN2_TransmitMsg(m_CAN_Packet);

	m_CAN_Packet.id = 0x4B1;
	temp_float.f = lat_control.getX1();
	m_CAN_Packet.data[0] = temp_float.b[3];
	m_CAN_Packet.data[1] = temp_float.b[2];
	m_CAN_Packet.data[2] = temp_float.b[1];
	m_CAN_Packet.data[3] = temp_float.b[0];
	temp_float.f = lat_control.getX2();
	m_CAN_Packet.data[4] = temp_float.b[3];
	m_CAN_Packet.data[5] = temp_float.b[2];
	m_CAN_Packet.data[6] = temp_float.b[1];
	m_CAN_Packet.data[7] = temp_float.b[0];
	CAN2_TransmitMsg(m_CAN_Packet);

	m_CAN_Packet.id = 0x4B2;
	temp_float.f = lat_control.getSlidingVariable();
	m_CAN_Packet.data[0] = temp_float.b[3];
	m_CAN_Packet.data[1] = temp_float.b[2];
	m_CAN_Packet.data[2] = temp_float.b[1];
	m_CAN_Packet.data[3] = temp_float.b[0];

	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);
}
/**************************************************************************************/
/*
 * 直接测量超声波原始信号
 * */
void Terminal::UltrasonicSend(const uint8_t id,const LIN_RAM *msg)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x400 | id;
	m_CAN_Packet.length = 8;
	if(id < 8)
	{
		m_CAN_Packet.data[0] =  msg[id].STP318.TOF       & 0xff;
		m_CAN_Packet.data[1] = (msg[id].STP318.TOF >> 8) & 0xff;
		m_CAN_Packet.data[2] = 0;
		m_CAN_Packet.data[3] = 0;
		m_CAN_Packet.data[4] = 0;
		m_CAN_Packet.data[5] = 0;
		m_CAN_Packet.data[6] =  msg[id].STP318.Status;
		m_CAN_Packet.data[7] = 0;
	}
	else
	{
		m_CAN_Packet.data[0] =  msg[id].STP313.TOF1       & 0xff;
		m_CAN_Packet.data[1] = (msg[id].STP313.TOF1 >> 8) & 0xff;
		m_CAN_Packet.data[2] =  msg[id].STP313.TOF2       & 0xff;
		m_CAN_Packet.data[3] = (msg[id].STP313.TOF2 >> 8) & 0xff;
		m_CAN_Packet.data[4] =  msg[id].STP313.Level;
		m_CAN_Packet.data[5] =  msg[id].STP313.Width;
		m_CAN_Packet.data[6] =  msg[id].STP313.Status;
		m_CAN_Packet.data[7] =  0;
	}
	CAN2_TransmitMsg(m_CAN_Packet);
}
void Terminal::UltrasonicSend(const uint8_t id,const Ultrasonic_Packet *msg_pk)
{
	CAN_Packet m_CAN_Packet;
	uint16_t temp;

	m_CAN_Packet.id = 0x400 | id;
	m_CAN_Packet.length = 8;
	if(id < 8)
	{
		temp = msg_pk[id].Distance1 * 100;
		m_CAN_Packet.data[0] = (uint8_t) temp;
		m_CAN_Packet.data[1] = (uint8_t)(temp >> 8 );
		m_CAN_Packet.data[2] = 0;
		m_CAN_Packet.data[3] = 0;
		m_CAN_Packet.data[4] = 0;
		m_CAN_Packet.data[5] = 0;
		m_CAN_Packet.data[6] = msg_pk[id].status;
		m_CAN_Packet.data[7] = msg_pk->Time_Tx;
	}
	else
	{
		temp = msg_pk[id].Distance1 * 100;
		m_CAN_Packet.data[0] = (uint8_t) temp;
		m_CAN_Packet.data[1] = (uint8_t)(temp >> 8 );
		temp = msg_pk[id].Distance2 * 100;
		m_CAN_Packet.data[2] = (uint8_t) temp;
		m_CAN_Packet.data[3] = (uint8_t)(temp >> 8 );
		m_CAN_Packet.data[4] = (uint8_t)( msg_pk[id].Level * 10 ) ;
		m_CAN_Packet.data[5] = (uint8_t)( msg_pk[id].Width);
		m_CAN_Packet.data[6] =  msg_pk[id].status;
		m_CAN_Packet.data[7] =  msg_pk->Time_Tx;
	}
	CAN2_TransmitMsg(m_CAN_Packet);
}
/*
 * 三角定位的短距离超声波信号
 * */
void Terminal::UltrasonicLocationSend(const uint8_t id,const LIN_RAM *msg)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x470 | id;
	m_CAN_Packet.length = 8;
	m_CAN_Packet.data[0] =  msg[id].STP318.TOF       & 0xff;
	m_CAN_Packet.data[1] = (msg[id].STP318.TOF >> 8) & 0xff;
	m_CAN_Packet.data[2] = 0;
	m_CAN_Packet.data[3] = 0;
	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = msg[id].STP318.Status;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);
}
void Terminal::UltrasonicLocationSend(const uint8_t id,const Ultrasonic_Packet *msg_pk)
{
	CAN_Packet m_CAN_Packet;
	uint16_t temp;
	m_CAN_Packet.id = 0x470 | id;
	m_CAN_Packet.length = 8;

	temp = msg_pk[id].Distance1 * 100;
	m_CAN_Packet.data[0] = (uint8_t) temp ;
	m_CAN_Packet.data[1] = (uint8_t)(temp >> 8 );
	m_CAN_Packet.data[2] = 0;
	m_CAN_Packet.data[3] = 0;
	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] =  msg_pk[id].status;
	m_CAN_Packet.data[7] = 0;

	CAN2_TransmitMsg(m_CAN_Packet);
}

/*
 * 载体坐标系的数据发送
 * */
void Terminal::UltrasonicBodyLocationSend(const uint8_t id,ObstacleLocationPacket &packet)
{
	CAN_Packet m_CAN_Packet;
	int16_t temp;
	m_CAN_Packet.id = 0x480 | id;
	m_CAN_Packet.length = 8;

	temp = packet.Position.getX()*1000;
	m_CAN_Packet.data[0] = (uint8_t)((temp     ) & 0xff );
	m_CAN_Packet.data[1] = (uint8_t)((temp >> 8) & 0xff );
	temp = packet.Position.getY()*1000;
	m_CAN_Packet.data[2] = (uint8_t)((temp     ) & 0xff );
	m_CAN_Packet.data[3] = (uint8_t)((temp >> 8) & 0xff );

	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = packet.Status;

	CAN2_TransmitMsg(m_CAN_Packet);
}
/*
 * 地面坐标系的数据发送
 * */
void Terminal::UltrasonicGroundLocationSend(const uint8_t id,ObstacleLocationPacket &packet)
{
	CAN_Packet m_CAN_Packet;
	int16_t temp;
	m_CAN_Packet.id = 0x490 | id;
	m_CAN_Packet.length = 8;

	temp = packet.Position.getX()*100;
	m_CAN_Packet.data[0] = (uint8_t)((temp     ) & 0xff );
	m_CAN_Packet.data[1] = (uint8_t)((temp >> 8) & 0xff );
	temp = packet.Position.getY()*100;
	m_CAN_Packet.data[2] = (uint8_t)((temp     ) & 0xff );
	m_CAN_Packet.data[3] = (uint8_t)((temp >> 8) & 0xff );
	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = packet.Status;

	CAN2_TransmitMsg(m_CAN_Packet);
}

void Terminal::Ack(void)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.id = 0x416;
	m_CAN_Packet.length = 8;
	m_CAN_Packet.data[0] = 0x5A;
	m_CAN_Packet.data[1] = 0xA5;
	m_CAN_Packet.data[2] = 0;
	m_CAN_Packet.data[3] = 0;
	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);
}

void Terminal::ParkingCenterPointSend(Vector2d v)
{
	CAN_Packet m_CAN_Packet;
	Byte2Int temp_int;
	Vector2d temp_v;
	m_CAN_Packet.id = 0x447;
	m_CAN_Packet.length = 8;

	temp_v = v;
	temp_int.i16 = (int16_t)(temp_v.getX() * 100);
	m_CAN_Packet.data[0] = temp_int.b[1];
	m_CAN_Packet.data[1] = temp_int.b[0];
	temp_int.i16 = (int16_t)(temp_v.getY() * 100);
	m_CAN_Packet.data[2] = temp_int.b[1];
	m_CAN_Packet.data[3] = temp_int.b[0];

	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = 0;
	m_CAN_Packet.data[7] = 0;
	CAN2_TransmitMsg(m_CAN_Packet);
}

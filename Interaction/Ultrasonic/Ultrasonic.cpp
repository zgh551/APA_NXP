/*
 * Ultrasonic.cpp
 *
 *  Created on: 2018��12��15��
 *      Author: zhuguohua
 */

#include "Ultrasonic.h"

Ultrasonic::Ultrasonic() {
	// TODO Auto-generated constructor stub
	_schedule_time_cnt = 0;

		ScheduleTimeCnt.setContainer(this);
		ScheduleTimeCnt.getter(&Ultrasonic::getScheduleTimeCnt);
		ScheduleTimeCnt.setter(&Ultrasonic::setScheduleTimeCnt);

		UltrasonicDatas.setContainer(this);
		UltrasonicDatas.getter(&Ultrasonic::getUltrasonicDatas);
}

Ultrasonic::~Ultrasonic() {
	// TODO Auto-generated destructor stub
}

uint8_t Ultrasonic::getScheduleTimeCnt()
{
		return _schedule_time_cnt;
}
void  Ultrasonic::setScheduleTimeCnt(uint8_t value)
{
		_schedule_time_cnt = value;
}

LIN_RAM* Ultrasonic::getUltrasonicDatas()
{
	return _ultrasonic_datas;
}

void Ultrasonic::InitSensing_STP318(uint8_t tx,uint8_t rx,void (*TransmitFrame)(LIN_RAM))
{
	LIN_RAM m_LIN_RAM;
	m_LIN_RAM.BIDR.B.ID  = 0;
	m_LIN_RAM.BIDR.B.DFL = 1;
	m_LIN_RAM.BDRL.B.DATA0 = tx;
	m_LIN_RAM.BDRL.B.DATA1 = rx;
	TransmitFrame(m_LIN_RAM);
}

void Ultrasonic::InitSensing_STP313(uint8_t tx_rx,void (*TransmitFrame)(LIN_RAM))
{
	LIN_RAM m_LIN_RAM;
	m_LIN_RAM.BIDR.B.ID  = 1;
	m_LIN_RAM.BIDR.B.DFL = 0;
	m_LIN_RAM.BDRL.B.DATA0 = tx_rx;
	TransmitFrame(m_LIN_RAM);
}

void Ultrasonic::ReadSensing_STP318(uint8_t id,void (*ReceiveFrame)(LIN_RAM *),LIN_RAM *m_LIN_RAM)
{
	m_LIN_RAM->BIDR.B.ID = id;
	m_LIN_RAM->BIDR.B.DFL = 2;
	ReceiveFrame(m_LIN_RAM);
}

void Ultrasonic::ReadSensing_STP313(uint8_t id,void (*ReceiveFrame)(LIN_RAM *),LIN_RAM *m_LIN_RAM)
{
	m_LIN_RAM->BIDR.B.ID = id;
	m_LIN_RAM->BIDR.B.DFL = 6;
	ReceiveFrame(m_LIN_RAM);
}

void Ultrasonic::InitUltrasonicSensor(uint8_t n)
{
	switch(n)
	{
		case 0:
		case 1:
		case 2:
		case 3:
			#ifdef FRONT_ULTRASONIC_ENABLE
			InitSensing_STP318( (1 << n),(1 << n),LIN0_TransmitFrame_DMA);
			#endif
			break;

		case 4:
		case 5:
		case 6:
		case 7:
			#ifdef REAR_ULTRASONIC_ENABLE
			InitSensing_STP318( (1 << (n & 0x3)),(1 << (n & 0x3)),LIN1_TransmitFrame_DMA);
			#endif
			break;

		case 8:
		case 9:
			#ifdef FRONT_ULTRASONIC_ENABLE
			InitSensing_STP313((1 << (n & 0x1)),LIN0_TransmitFrame_DMA);
			#endif
			break;

		case 10:
		case 11:
			#ifdef REAR_ULTRASONIC_ENABLE
			InitSensing_STP313((1 << (n & 0x1)),LIN1_TransmitFrame_DMA);
			#endif
			break;
	}
}

void Ultrasonic::ReadUltrasonicSensor(uint8_t n)
{
	switch(n)
	{
		case 0:
		case 1:
		case 2:
		case 3:
			#ifdef FRONT_ULTRASONIC_ENABLE
			ReadSensing_STP318(0xf - n,LIN0_ReceiveFrame_DMA,&_ultrasonic_datas[n]);
			#endif
		break;

		case 4:
		case 5:
		case 6:
		case 7:
			#ifdef REAR_ULTRASONIC_ENABLE
			ReadSensing_STP318(0xf - (n & 0x3),LIN1_ReceiveFrame_DMA,&_ultrasonic_datas[n]);
			#endif
		break;

		case 8:
		case 9:
			#ifdef FRONT_ULTRASONIC_ENABLE
			ReadSensing_STP313(0x1f - (n & 0x1),LIN0_ReceiveFrame_DMA,&_ultrasonic_datas[n]);
			#endif
		break;

		case 10:
		case 11:
			#ifdef REAR_ULTRASONIC_ENABLE
			ReadSensing_STP313(0x1f - (n & 0x1),LIN1_ReceiveFrame_DMA,&_ultrasonic_datas[n]);
			#endif
		break;

		default:

			break;
	}

}

void Ultrasonic::UltrasonicScheduleStatusMachine(void)
{
	  switch(_schedule_time_cnt)
    {
		case 0:
			InitUltrasonicSensor(8);
			InitUltrasonicSensor(11);
			break;

		case 1:
			InitUltrasonicSensor(1);
			InitUltrasonicSensor(7);
			break;

		case 2:
			InitUltrasonicSensor(9);
			InitUltrasonicSensor(10);
			break;

		case 5:
			ReadUltrasonicSensor(1);
			ReadUltrasonicSensor(7);
			break;

		case 6:
			InitUltrasonicSensor(3);
			InitUltrasonicSensor(5);
			break;

		case 8:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(11);
			break;

		case 10:
			ReadUltrasonicSensor(3);
			ReadUltrasonicSensor(5);
			break;

		case 11:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(10);
			break;

		case 13:
			InitUltrasonicSensor(8);
			InitUltrasonicSensor(11);
			break;

		case 14:
			InitUltrasonicSensor(0);
			InitUltrasonicSensor(6);
			break;

		case 15:
			InitUltrasonicSensor(9);
			InitUltrasonicSensor(10);
			break;

		case 18:
			ReadUltrasonicSensor(0);
			ReadUltrasonicSensor(6);
			break;

		case 19:
			InitUltrasonicSensor(2);
			InitUltrasonicSensor(4);
			break;

		case 21:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(11);
			break;

		case 23:
			ReadUltrasonicSensor(2);
			ReadUltrasonicSensor(4);
			break;

		case 24:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(10);
			break;

		default:
			break;
    }
}

void Ultrasonic::UltrasonicScheduleStatusMachine_V2(void)
{
	  switch(_schedule_time_cnt)
    {
		case 0:
			InitUltrasonicSensor(8);
			InitUltrasonicSensor(11);
			break;

		case 1:
			InitUltrasonicSensor(1);
			InitUltrasonicSensor(7);
			break;

		case 2:
			InitUltrasonicSensor(9);
			InitUltrasonicSensor(10);
			break;

		case 6:
			ReadUltrasonicSensor(1);
			ReadUltrasonicSensor(7);
			break;

		case 7:
			InitUltrasonicSensor(3);
			InitUltrasonicSensor(5);
			break;

		case 8:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(11);
			break;

		case 10:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(10);
			break;

		case 12:
			ReadUltrasonicSensor(3);
			ReadUltrasonicSensor(5);
			break;

		case 13:
			InitUltrasonicSensor(8);
			InitUltrasonicSensor(11);
			break;

		case 14:
			InitUltrasonicSensor(0);
			InitUltrasonicSensor(6);
			break;

		case 15:
			InitUltrasonicSensor(9);
			InitUltrasonicSensor(10);
			break;

		case 19:
			ReadUltrasonicSensor(0);
			ReadUltrasonicSensor(6);
			break;

		case 20:
			InitUltrasonicSensor(2);
			InitUltrasonicSensor(4);
			break;

		case 21:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(11);
			break;

		case 23:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(10);
			break;

		case 25:
			ReadUltrasonicSensor(2);
			ReadUltrasonicSensor(4);
			break;

		default:
			break;
    }
}

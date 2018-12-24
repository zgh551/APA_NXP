/*
 * Ultrasonic.cpp
 *
 *  Created on: 2018��12��15��
 *      Author: zhuguohua
 */

#include "Ultrasonic.h"

Ultrasonic::Ultrasonic() {
	// TODO Auto-generated constructor stub
	UltrasonicDatas.setContainer(this);
	UltrasonicDatas.getter(&Ultrasonic::getUltrasonicDatas);
}

Ultrasonic::~Ultrasonic() {
	// TODO Auto-generated destructor stub
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
			InitSensing_STP318( (1 << n),(1 << n),LIN0_TransmitFrame_DMA);
			break;

		case 4:
		case 5:
		case 6:
		case 7:
			InitSensing_STP318( (1 << (n & 0x3)),(1 << (n & 0x3)),LIN0_TransmitFrame_DMA);
			break;

		case 8:
		case 9:
			InitSensing_STP313((1 << (n & 0x1)),LIN0_TransmitFrame_DMA);
			break;

		case 10:
		case 11:
			InitSensing_STP313((1 << (n & 0x1)),LIN0_TransmitFrame_DMA);
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
			ReadSensing_STP318(0xf - n,LIN0_ReceiveFrame_DMA,&_ultrasonic_datas[n]);
		break;

		case 4:
		case 5:
		case 6:
		case 7:
			ReadSensing_STP318(0xf - (n & 0x3),LIN0_ReceiveFrame_DMA,&_ultrasonic_datas[n]);
		break;

		case 8:
		case 9:
			ReadSensing_STP313(0x1f - (n & 0x1),LIN0_ReceiveFrame_DMA,&_ultrasonic_datas[n]);
		break;

		case 10:
		case 11:
			ReadSensing_STP313(0x1f - (n & 0x1),LIN0_ReceiveFrame_DMA,&_ultrasonic_datas[n]);
		break;

		default:

			break;
	}

}

void Ultrasonic::TimeScheduleStatus1_SR(void)
{
	ReadSensing_STP318(0xCf,LIN0_ReceiveFrame_DMA,&_ultrasonic_datas[1]);
}

//void Ultrasonic::InitSensing_STP313(void (*TransmitFrame)(LIN_Packet),uint8_t tx_rx)
//{
//	LIN_Packet m_LIN_Packet;
//	m_LIN_Packet.id = 0xC1;
//	m_LIN_Packet.length = 1;
//	m_LIN_Packet.BufferData.B.Data0 = tx_rx;
//	TransmitFrame(m_LIN_Packet);
//}
//
//LIN_STP313_Packet Ultrasonic::ReadSensing_STP313(void (*ReceiveFrame)(LIN_Packet *),uint8_t id)
//{
//	LIN_Packet m_LIN_Packet;
//	LIN_STP313_Packet m_LIN_STP313_Packet;
//
//	m_LIN_Packet.id = id;
//	m_LIN_Packet.length = 7;
//	ReceiveFrame(&m_LIN_Packet);
//
//	m_LIN_STP313_Packet.TOF1 = (uint16_t)((m_LIN_Packet.BufferData.B.Data1 << 8) | m_LIN_Packet.BufferData.B.Data0);
//	m_LIN_STP313_Packet.Level = m_LIN_Packet.BufferData.B.Data2;
//	m_LIN_STP313_Packet.Width = m_LIN_Packet.BufferData.B.Data3;
//	m_LIN_STP313_Packet.TOF2 = (uint16_t)((m_LIN_Packet.BufferData.B.Data5 << 8) | m_LIN_Packet.BufferData.B.Data4);
//	m_LIN_STP313_Packet.status = m_LIN_Packet.BufferData.B.Data6;
//
//	return m_LIN_STP313_Packet;
//}

/*
 * Ultrasonic.cpp
 *
 *  Created on: 2018��12��15��
 *      Author: zhuguohua
 */

#include "Ultrasonic.h"

Ultrasonic::Ultrasonic() {
	// TODO Auto-generated constructor stub

}

Ultrasonic::~Ultrasonic() {
	// TODO Auto-generated destructor stub
}

void Ultrasonic::InitSensing_STP318(void (*TransmitFrame)(LIN_Packet),uint8_t tx,uint8_t rx)
{
	LIN_Packet m_LIN_Packet;
	m_LIN_Packet.id = 0x80;
	m_LIN_Packet.length = 2;
	m_LIN_Packet.BufferData.B.Data0 = tx;
	m_LIN_Packet.BufferData.B.Data1 = rx;
	TransmitFrame(m_LIN_Packet);
}

LIN_STP318_Packet Ultrasonic::ReadSensing_STP318(void (*ReceiveFrame)(LIN_Packet *),uint8_t id)
{
	LIN_Packet m_LIN_Packet;
	LIN_STP318_Packet m_LIN_STP318_Packet;
	m_LIN_Packet.id = id;
	m_LIN_Packet.length = 3;
	ReceiveFrame(&m_LIN_Packet);

	m_LIN_STP318_Packet.TOF = (uint16_t)((m_LIN_Packet.BufferData.B.Data1 << 8) | m_LIN_Packet.BufferData.B.Data0);
	m_LIN_STP318_Packet.status = m_LIN_Packet.BufferData.B.Data2;
	return m_LIN_STP318_Packet;
}

void Ultrasonic::InitSensing_STP313(void (*TransmitFrame)(LIN_Packet),uint8_t tx_rx)
{
	LIN_Packet m_LIN_Packet;
	m_LIN_Packet.id = 0xC1;
	m_LIN_Packet.length = 1;
	m_LIN_Packet.BufferData.B.Data0 = tx_rx;
	TransmitFrame(m_LIN_Packet);
}

LIN_STP313_Packet Ultrasonic::ReadSensing_STP313(void (*ReceiveFrame)(LIN_Packet *),uint8_t id)
{
	LIN_Packet m_LIN_Packet;
	LIN_STP313_Packet m_LIN_STP313_Packet;

	m_LIN_Packet.id = id;
	m_LIN_Packet.length = 7;
	ReceiveFrame(&m_LIN_Packet);

	m_LIN_STP313_Packet.TOF1 = (uint16_t)((m_LIN_Packet.BufferData.B.Data1 << 8) | m_LIN_Packet.BufferData.B.Data0);
	m_LIN_STP313_Packet.Level = m_LIN_Packet.BufferData.B.Data2;
	m_LIN_STP313_Packet.Width = m_LIN_Packet.BufferData.B.Data3;
	m_LIN_STP313_Packet.TOF2 = (uint16_t)((m_LIN_Packet.BufferData.B.Data5 << 8) | m_LIN_Packet.BufferData.B.Data4);
	m_LIN_STP313_Packet.status = m_LIN_Packet.BufferData.B.Data6;

	return m_LIN_STP313_Packet;
}

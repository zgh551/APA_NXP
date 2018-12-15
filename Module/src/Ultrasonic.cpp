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

void Ultrasonic::InitSensing_STP318(uint8_t tx,uint8_t rx)
{
  LIN_Packet m_LIN_Packet;
  m_LIN_Packet.id = 0x80;
  m_LIN_Packet.length = 2;
  m_LIN_Packet.BufferData.B.Data0 = tx;
  m_LIN_Packet.BufferData.B.Data1 = rx;
  LIN0_TransmitFrame(m_LIN_Packet);
}

LIN_STP318_Packet Ultrasonic::ReadData_STP318(uint8_t id)
{
	LIN_Packet m_LIN_Packet;
	LIN_STP318_Packet m_LIN_STP318_Packet;
	m_LIN_Packet.id = id;
	m_LIN_Packet.length = 3;
	LIN0_ReceiveFrame(&m_LIN_Packet);

	m_LIN_STP318_Packet.TOF = (uint16_t)((m_LIN_Packet.BufferData.B.Data1 << 8) | m_LIN_Packet.BufferData.B.Data0);
	m_LIN_STP318_Packet.status = m_LIN_Packet.BufferData.B.Data2;
	return m_LIN_STP318_Packet;
}

void Ultrasonic::InitSensing_STP313(uint8_t tx_rx)
{

}

LIN_STP313_Packet Ultrasonic::ReadData_STP313(uint8_t id)
{

}

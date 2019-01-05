/*
 * Ultrasonic.cpp
 *
 *  Created on: 2018��12��15��
 *      Author: zhuguohua
 */

#include "Ultrasonic.h"

Ultrasonic::Ultrasonic() {
	// TODO Auto-generated constructor stub
	Init();

	ScheduleTimeCnt.setContainer(this);
	ScheduleTimeCnt.getter(&Ultrasonic::getScheduleTimeCnt);
	ScheduleTimeCnt.setter(&Ultrasonic::setScheduleTimeCnt);

	ReadStage.setContainer(this);
	ReadStage.getter(&Ultrasonic::getReadStage);
	ReadStage.setter(&Ultrasonic::setReadStage);

	SystemTime.setContainer(this);
	SystemTime.getter(&Ultrasonic::getSystemTime);
	SystemTime.setter(&Ultrasonic::setSystemTime);

	UltrasonicPacket.setContainer(this);
	UltrasonicPacket.getter(&Ultrasonic::getUltrasonicPacket);

	UltrasonicDatas.setContainer(this);
	UltrasonicDatas.getter(&Ultrasonic::getUltrasonicDatas);
}

Ultrasonic::~Ultrasonic() {
	// TODO Auto-generated destructor stub
}

void Ultrasonic::Init(void)
{
	_system_time = 0;
	_schedule_time_cnt = 0;
	_read_stage = 0;
}

uint8_t Ultrasonic::getScheduleTimeCnt()             {return _schedule_time_cnt ;}
void    Ultrasonic::setScheduleTimeCnt(uint8_t value){_schedule_time_cnt = value;}

uint8_t Ultrasonic::getReadStage()             {return _read_stage ;}
void    Ultrasonic::setReadStage(uint8_t value){_read_stage = value;}

uint32_t Ultrasonic::getSystemTime()              {return _system_time ;}
void     Ultrasonic::setSystemTime(uint32_t value){_system_time = value;}

Ultrasonic_Packet* Ultrasonic::getUltrasonicPacket(){return _ultrasonic_packet;}

LIN_RAM* Ultrasonic::getUltrasonicDatas(){return _ultrasonic_datas;}

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
			_ultrasonic_packet[8].Time_Tx  = _system_time;
			_ultrasonic_packet[11].Time_Tx = _system_time;
			break;

		case 1:
			InitUltrasonicSensor(1);
			InitUltrasonicSensor(7);
			_ultrasonic_packet[1].Time_Tx = _system_time;
			_ultrasonic_packet[7].Time_Tx = _system_time;
			break;

		case 2:
			InitUltrasonicSensor(9);
			InitUltrasonicSensor(10);
			_ultrasonic_packet[9].Time_Tx  = _system_time;
			_ultrasonic_packet[10].Time_Tx = _system_time;
			break;

		case 6:
			ReadUltrasonicSensor(1);
			ReadUltrasonicSensor(7);
			_read_stage = 0;
			break;

		case 7:
			InitUltrasonicSensor(3);
			InitUltrasonicSensor(5);
			_ultrasonic_packet[3].Time_Tx = _system_time;
			_ultrasonic_packet[5].Time_Tx = _system_time;
			break;

		case 8:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(11);
			_read_stage = 1;
			break;

		case 10:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(10);
			_read_stage = 2;
			break;

		case 12:
			ReadUltrasonicSensor(3);
			ReadUltrasonicSensor(5);
			_read_stage = 3;
			break;

		case 13:
			InitUltrasonicSensor(8);
			InitUltrasonicSensor(11);
			_ultrasonic_packet[8].Time_Tx  = _system_time;
			_ultrasonic_packet[11].Time_Tx = _system_time;
			break;

		case 14:
			InitUltrasonicSensor(0);
			InitUltrasonicSensor(6);
			_ultrasonic_packet[0].Time_Tx = _system_time;
			_ultrasonic_packet[6].Time_Tx = _system_time;
			break;

		case 15:
			InitUltrasonicSensor(9);
			InitUltrasonicSensor(10);
			_ultrasonic_packet[9].Time_Tx = _system_time;
			_ultrasonic_packet[10].Time_Tx = _system_time;
			break;

		case 19:
			ReadUltrasonicSensor(0);
			ReadUltrasonicSensor(6);
			_read_stage = 4;
			break;

		case 20:
			InitUltrasonicSensor(2);
			InitUltrasonicSensor(4);
			_ultrasonic_packet[2].Time_Tx = _system_time;
			_ultrasonic_packet[4].Time_Tx = _system_time;
			break;

		case 21:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(11);
			_read_stage = 5;
			break;

		case 23:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(10);
			_read_stage = 6;
			break;

		case 25:
			ReadUltrasonicSensor(2);
			ReadUltrasonicSensor(4);
			_read_stage = 7;
			break;

		default:
			break;
    }
}

float Ultrasonic::Compensation(float temp)
{
	return (331.5 + 0.60714 * temp) * 0.0000005; // (m/us)/2
}

void Ultrasonic::Update(uint8_t id,float t)
{
	switch(_read_stage)
	{
		case 0:
			if(id == 0)
			{
				 _ultrasonic_packet[1].Distance1 = _ultrasonic_datas[1].STP318.TOF * Compensation(t);
				 _ultrasonic_packet[1].status = _ultrasonic_datas[1].STP318.Status;
				 _ultrasonic_packet[1].Time_Ms = _ultrasonic_packet[1].Time_Tx * 5 + _ultrasonic_datas[1].STP318.TOF * 0.0005;
			}
			else
			{
				 _ultrasonic_packet[7].Distance1 = _ultrasonic_datas[7].STP318.TOF * Compensation(t);
				 _ultrasonic_packet[7].status = _ultrasonic_datas[7].STP318.Status;
				 _ultrasonic_packet[7].Time_Ms = _ultrasonic_packet[7].Time_Tx * 5 + _ultrasonic_datas[7].STP318.TOF * 0.0005;
			}
			break;

		case 1:
		case 5:
			if(id == 0)
			{
				 _ultrasonic_packet[8].Distance1 = _ultrasonic_datas[8].STP313.TOF1 * Compensation(t);
				 _ultrasonic_packet[8].Distance2 = _ultrasonic_datas[8].STP313.TOF2 * Compensation(t);
				 _ultrasonic_packet[8].Level = _ultrasonic_datas[8].STP313.Level * LEVEL_RATIO;
				 _ultrasonic_packet[8].Width = _ultrasonic_datas[8].STP313.Width * WIDTH_RATIO;
				 _ultrasonic_packet[8].status = _ultrasonic_datas[8].STP313.Status;
				 _ultrasonic_packet[8].Time_Ms = _ultrasonic_packet[8].Time_Tx * 5 + _ultrasonic_datas[8].STP313.TOF1 * 0.0005;
			}
			else
			{
				 _ultrasonic_packet[11].Distance1 = _ultrasonic_datas[11].STP313.TOF1 * Compensation(t);
				 _ultrasonic_packet[11].Distance2 = _ultrasonic_datas[11].STP313.TOF2 * Compensation(t);
				 _ultrasonic_packet[11].Level = _ultrasonic_datas[11].STP313.Level * LEVEL_RATIO;
				 _ultrasonic_packet[11].Width = _ultrasonic_datas[11].STP313.Width * WIDTH_RATIO;
				 _ultrasonic_packet[11].status = _ultrasonic_datas[11].STP313.Status;
				 _ultrasonic_packet[11].Time_Ms = _ultrasonic_packet[11].Time_Tx * 5 + _ultrasonic_datas[11].STP313.TOF1 * 0.0005;
			}
			break;

		case 2:
		case 6:
			if(id == 0)
			{
				 _ultrasonic_packet[9].Distance1 = _ultrasonic_datas[9].STP313.TOF1 * Compensation(t);
				 _ultrasonic_packet[9].Distance2 = _ultrasonic_datas[9].STP313.TOF2 * Compensation(t);
				 _ultrasonic_packet[9].Level = _ultrasonic_datas[9].STP313.Level * LEVEL_RATIO;
				 _ultrasonic_packet[9].Width = _ultrasonic_datas[9].STP313.Width * WIDTH_RATIO;
				 _ultrasonic_packet[9].status = _ultrasonic_datas[9].STP313.Status;
				 _ultrasonic_packet[9].Time_Ms = _ultrasonic_packet[9].Time_Tx * 5 + _ultrasonic_datas[9].STP313.TOF1 * 0.0005;
			}
			else
			{
				 _ultrasonic_packet[10].Distance1 = _ultrasonic_datas[10].STP313.TOF1 * Compensation(t);
				 _ultrasonic_packet[10].Distance2 = _ultrasonic_datas[10].STP313.TOF2 * Compensation(t);
				 _ultrasonic_packet[10].Level     = _ultrasonic_datas[10].STP313.Level * LEVEL_RATIO;
				 _ultrasonic_packet[10].Width     = _ultrasonic_datas[10].STP313.Width * WIDTH_RATIO;
				 _ultrasonic_packet[10].status    = _ultrasonic_datas[10].STP313.Status;
				 _ultrasonic_packet[10].Time_Ms   = _ultrasonic_packet[10].Time_Tx * 5 + _ultrasonic_datas[10].STP313.TOF1 * 0.0005;
			}
			break;

		case 3:
			if(id == 0)
			{
				 _ultrasonic_packet[3].Distance1 = _ultrasonic_datas[3].STP318.TOF * Compensation(t);
				 _ultrasonic_packet[3].status    = _ultrasonic_datas[3].STP318.Status;
				 _ultrasonic_packet[3].Time_Ms   = _ultrasonic_packet[3].Time_Tx * 5 + _ultrasonic_datas[3].STP318.TOF * 0.0005;
			}
			else
			{
				 _ultrasonic_packet[5].Distance1 = _ultrasonic_datas[5].STP318.TOF * Compensation(t);
				 _ultrasonic_packet[5].status    = _ultrasonic_datas[5].STP318.Status;
				 _ultrasonic_packet[5].Time_Ms   = _ultrasonic_packet[5].Time_Tx * 5 + _ultrasonic_datas[5].STP318.TOF * 0.0005;
			}
			break;

		case 4:
			if(id == 0)
			{
				 _ultrasonic_packet[0].Distance1 = _ultrasonic_datas[0].STP318.TOF * Compensation(t);
				 _ultrasonic_packet[0].status    = _ultrasonic_datas[0].STP318.Status;
				 _ultrasonic_packet[0].Time_Ms   = _ultrasonic_packet[0].Time_Tx * 5 + _ultrasonic_datas[0].STP318.TOF * 0.0005;
			}
			else
			{
				 _ultrasonic_packet[6].Distance1 = _ultrasonic_datas[6].STP318.TOF * Compensation(t);
				 _ultrasonic_packet[6].status    = _ultrasonic_datas[6].STP318.Status;
				 _ultrasonic_packet[6].Time_Ms   = _ultrasonic_packet[6].Time_Tx * 5 + _ultrasonic_datas[6].STP318.TOF * 0.0005;
			}
			break;

		case 7:
			if(id == 0)
			{
				 _ultrasonic_packet[2].Distance1 = _ultrasonic_datas[2].STP318.TOF * Compensation(t);
				 _ultrasonic_packet[2].status    = _ultrasonic_datas[2].STP318.Status;
				 _ultrasonic_packet[2].Time_Ms   = _ultrasonic_packet[2].Time_Tx * 5 + _ultrasonic_datas[2].STP318.TOF * 0.0005;
			}
			else
			{
				 _ultrasonic_packet[4].Distance1 = _ultrasonic_datas[4].STP318.TOF * Compensation(t);
				 _ultrasonic_packet[4].status    = _ultrasonic_datas[4].STP318.Status;
				 _ultrasonic_packet[4].Time_Ms   = _ultrasonic_packet[4].Time_Tx * 5 + _ultrasonic_datas[4].STP318.TOF * 0.0005;
			}
			break;

		default:
			break;
	}
}


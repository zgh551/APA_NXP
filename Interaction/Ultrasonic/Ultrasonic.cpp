/*
 * ultrasonic.cpp
 *
 *  Created on: January 8 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: ultrasonic.cpp                      COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this module process the ultrasonic data				         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 8 2019      Initial Version                  */
/*****************************************************************************/

#include "Ultrasonic.h"

Ultrasonic::Ultrasonic() {
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

	UltrasonicDatas.setContainer(this);
	UltrasonicDatas.getter(&Ultrasonic::getUltrasonicDatas);

	UltrasonicLocationDatas.setContainer(this);
	UltrasonicLocationDatas.getter(&Ultrasonic::getUltrasonicLocationDatas);

	UltrasonicPacket.setContainer(this);
	UltrasonicPacket.getter(&Ultrasonic::getUltrasonicPacket);

	UltrasonicLocationPacket.setContainer(this);
	UltrasonicLocationPacket.getter(&Ultrasonic::getUltrasonicLocationPacket);

	AbstacleBodyPositionDirect.setContainer(this);
	AbstacleBodyPositionDirect.getter(&Ultrasonic::getAbstacleBodyPositionDirect);

	AbstacleBodyPositionTriangle.setContainer(this);
	AbstacleBodyPositionTriangle.getter(&Ultrasonic::getAbstacleBodyPositionTriangle);

	AbstacleGroundPositionTriangle.setContainer(this);
	AbstacleGroundPositionTriangle.getter(&Ultrasonic::getAbstacleGroundPositionTriangle);
}

Ultrasonic::~Ultrasonic() {

}

void Ultrasonic::Init(void)
{
	uint8_t i;
	_system_time = 0;
	_schedule_time_cnt = 0;
	_read_stage = 0;

	for(i=0;i<4;i++)
	{
		_ultrasonic_data_buffer[i].Position.setX(0.0f);
		_ultrasonic_data_buffer[i].Position.setY(0.0f);
		_ultrasonic_data_buffer[i].UltrasonicData.Distance1 = 0.0f;
		_ultrasonic_data_buffer[i].UltrasonicData.Distance2 = 0.0f;
		_ultrasonic_data_buffer[i].UltrasonicData.Level = 0.0f;
		_ultrasonic_data_buffer[i].UltrasonicData.Width = 0.0f;
		_ultrasonic_data_buffer[i].UltrasonicData.status = 0;
		_ultrasonic_data_buffer[i].UltrasonicData.Time_Tx = 0;
		_ultrasonic_data_buffer[i].UltrasonicData.Time_Ms = 0.0f;
	}
}

void Ultrasonic::GainConfigure(float t)
{
	int8_t temp_gain_adj;

	temp_gain_adj = GainAdjustmentTemperatureCompensation_STP318(t);
	GainAdj_STP318(temp_gain_adj, temp_gain_adj, temp_gain_adj,LIN0_TransmitFrame_DMA);
	GainAdj_STP318(temp_gain_adj, temp_gain_adj, temp_gain_adj,LIN1_TransmitFrame_DMA);

	temp_gain_adj = GainAdjustmentTemperatureCompensation_STP313(t);
	GainAdj_STP313(temp_gain_adj, temp_gain_adj, temp_gain_adj,LIN0_TransmitFrame_DMA);
	GainAdj_STP313(temp_gain_adj, temp_gain_adj, temp_gain_adj,LIN1_TransmitFrame_DMA);
}

uint8_t Ultrasonic::getScheduleTimeCnt()             {return _schedule_time_cnt ;}
void    Ultrasonic::setScheduleTimeCnt(uint8_t value){_schedule_time_cnt = value;}

uint8_t Ultrasonic::getReadStage()             {return _read_stage ;}
void    Ultrasonic::setReadStage(uint8_t value){_read_stage = value;}

uint32_t Ultrasonic::getSystemTime()              {return _system_time ;}
void     Ultrasonic::setSystemTime(uint32_t value){_system_time = value;}

LIN_RAM* Ultrasonic::getUltrasonicDatas(){return _ultrasonic_datas;}

LIN_RAM* Ultrasonic::getUltrasonicLocationDatas(){return _ultrasonic_location_datas;}

Ultrasonic_Packet* Ultrasonic::getUltrasonicPacket(){return _ultrasonic_packet;}

Ultrasonic_Packet* Ultrasonic::getUltrasonicLocationPacket(){return _ultrasonic_location_packet;}

ObstacleLocationPacket* Ultrasonic::getAbstacleBodyPositionDirect(){return _abstacle_body_position_direct;}

ObstacleLocationPacket* Ultrasonic::getAbstacleBodyPositionTriangle(){return _abstacle_body_position_triangle;}

ObstacleLocationPacket* Ultrasonic::getAbstacleGroundPositionTriangle(){return _abstacle_ground_position_triangle;}

void Ultrasonic::setUltrasonicPacket(uint8_t n,Ultrasonic_Packet p)
{
	_ultrasonic_packet[n] = p;
}

void Ultrasonic::setUltrasonicLocationPacket(uint8_t n,Ultrasonic_Packet p)
{
	_ultrasonic_location_packet[n] = p;
}

void Ultrasonic::setAbstacleGroundPositionTriangle(uint8_t n,ObstacleLocationPacket p)
{
	_abstacle_ground_position_triangle[n] = p;
}

int8_t Ultrasonic::GainAdjustmentTemperatureCompensation_STP318(float t)
{
	int8_t gain_adj;
	if(t >= -40 && t < -20)
	{
		gain_adj = -(t + 20) / 20 - 14;
	}
	else if(t >= -20 && t < 0)
	{
		gain_adj = (t * 3 / 20) - 11;
	}
	else if(t >= 0 && t <= 50)
	{
		gain_adj = (t - 25) * 22 / 50;
	}
	else
	{
		gain_adj = 0;
	}
	return gain_adj;
}

int8_t Ultrasonic::GainAdjustmentTemperatureCompensation_STP313(float t)
{
	int8_t gain_adj;
	if(t >= -40 && t < -10)
	{
		gain_adj = -4 * (t + 10) / 30 - 8.7;
	}
	else if(t >= -10 && t < 10)
	{
		gain_adj = (t - 10) * 3 / 20 - 5.7;
	}
	else if(t >= 10 && t < 20)
	{
		gain_adj = (t - 20) * 4 / 10 - 1.7;
	}
	else if(t >= 20 && t < 80)
	{
		gain_adj = (t - 25) * 20 / 60;
	}
	else
	{
		gain_adj = 0;
	}
	return gain_adj;
}

void Ultrasonic::GainAdj_STP318(int8_t s, int8_t m, int8_t e, void (*TransmitFrame)(LIN_RAM))
{
	LIN_RAM m_LIN_RAM;
	m_LIN_RAM.BIDR.B.ID  = 0x3C;
	m_LIN_RAM.BIDR.B.DFL = 7;
	m_LIN_RAM.BDRL.B.DATA0 = 19; //NAD
	m_LIN_RAM.BDRL.B.DATA1 = 0x06; //PCI
	m_LIN_RAM.BDRL.B.DATA2 = 0x2E; //SID
	m_LIN_RAM.BDRL.B.DATA3 = 200;
	m_LIN_RAM.BDRM.B.DATA4 = s + 128;
	m_LIN_RAM.BDRM.B.DATA5 = 128;
	m_LIN_RAM.BDRM.B.DATA6 = e + 128;
	m_LIN_RAM.BDRM.B.DATA7 = 128;
	TransmitFrame(m_LIN_RAM);
}

void Ultrasonic::GainAdj_STP313(int8_t s, int8_t m, int8_t e, void (*TransmitFrame)(LIN_RAM))
{
	LIN_RAM m_LIN_RAM;
	m_LIN_RAM.BIDR.B.ID  = 0x3C;
	m_LIN_RAM.BIDR.B.DFL = 7;
	m_LIN_RAM.BDRL.B.DATA0 = 0x09; //NAD
	m_LIN_RAM.BDRL.B.DATA1 = 0x06; //PCI
	m_LIN_RAM.BDRL.B.DATA2 = 0x2E; //SID
	m_LIN_RAM.BDRL.B.DATA3 = 200;
	m_LIN_RAM.BDRM.B.DATA4 = s + 128;
	m_LIN_RAM.BDRM.B.DATA5 = m + 128;
	m_LIN_RAM.BDRM.B.DATA6 = e + 128;
	m_LIN_RAM.BDRM.B.DATA7 = 128;
	TransmitFrame(m_LIN_RAM);
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

/*
 * 激活单发单收
 * */
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

/*
 * 激活一发多收
 * */
void Ultrasonic::InitUltrasonicSensorRx(uint8_t n)
{
	switch(n)
	{
		case 0:
		case 3:
			break;

		case 4:
		case 7:
			break;

		case 1:
		case 2:
			#ifdef FRONT_ULTRASONIC_ENABLE
			InitSensing_STP318( (1 << n),(7 << (n - 1)),LIN0_TransmitFrame_DMA);
			#endif
			break;



		case 5:
		case 6:

			#ifdef REAR_ULTRASONIC_ENABLE
			InitSensing_STP318( (1 << (n & 0x3)),(7 << ((n & 0x3) - 1)),LIN1_TransmitFrame_DMA);
			#endif
			break;

		case 8:
		case 9:
			#ifdef FRONT_ULTRASONIC_ENABLE
			InitSensing_STP313(3,LIN0_TransmitFrame_DMA);
			#endif
			break;

		case 10:
		case 11:
			#ifdef REAR_ULTRASONIC_ENABLE
			InitSensing_STP313(3,LIN1_TransmitFrame_DMA);
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

void Ultrasonic::ReadUltrasonicSensorRxs(uint8_t step,uint8_t n)
{
	switch(n)
	{
		case 0:
		case 1:
		case 2:
		case 3:
			#ifdef FRONT_ULTRASONIC_ENABLE
			ReadSensing_STP318(0xf - n,LIN0_ReceiveFrame_DMA,&_ultrasonic_location_datas[ (n - (step & 0x1)) + 3*step]);
			#endif
		break;

		case 4:
		case 5:
		case 6:
		case 7:
			#ifdef REAR_ULTRASONIC_ENABLE
			ReadSensing_STP318(0xf - (n & 0x3),LIN1_ReceiveFrame_DMA,&_ultrasonic_location_datas[ ((n - (step & 0x1)) & 0x3) + 3*step]);
			#endif
		break;

		case 8:
		case 9:
			#ifdef FRONT_ULTRASONIC_ENABLE
//			ReadSensing_STP313(0x1f - (n & 0x1),LIN0_ReceiveFrame_DMA,&UltrasonicLocationDatas[n]);
			#endif
		break;

		case 10:
		case 11:
			#ifdef REAR_ULTRASONIC_ENABLE
//			ReadSensing_STP313(0x1f - (n & 0x1),LIN1_ReceiveFrame_DMA,&UltrasonicLocationDatas[n]);
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

/*
 * 单发单收调度
 * */
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
/*
 * 多发多收调度
 * */
void Ultrasonic::UltrasonicScheduleStatusMachine_V3(void)
{
	switch(_schedule_time_cnt)
	{
		case 0:
			_read_stage = 7;
			InitUltrasonicSensorRx(1);
			InitUltrasonicSensorRx(6);
			_ultrasonic_packet[1].Time_Tx  = _system_time;
			_ultrasonic_packet[6].Time_Tx = _system_time;
			break;

		case 1:
			InitUltrasonicSensorRx(8);
			InitUltrasonicSensorRx(10);
			_ultrasonic_packet[8].Time_Tx = _system_time;
			_ultrasonic_packet[9].Time_Tx = _system_time;
			_ultrasonic_packet[10].Time_Tx = _system_time;
			_ultrasonic_packet[11].Time_Tx = _system_time;
			break;

		case 5:
			ReadUltrasonicSensorRxs(0,0);
			ReadUltrasonicSensorRxs(3,5);
			break;

		case 6:
			ReadUltrasonicSensorRxs(0,1);
			ReadUltrasonicSensorRxs(3,6);
			break;

		case 7:
			ReadUltrasonicSensorRxs(0,2);
			ReadUltrasonicSensorRxs(3,7);

			_ultrasonic_datas[1] = _ultrasonic_location_datas[1];
			_ultrasonic_datas[6] = _ultrasonic_location_datas[10];
			break;

		case 8:
			_read_stage = 0;
			InitUltrasonicSensor(3);
			InitUltrasonicSensor(4);
			_ultrasonic_packet[3].Time_Tx = _system_time;
			_ultrasonic_packet[4].Time_Tx = _system_time;
			break;

		case 9:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(10);
			break;

		case 11:
			_read_stage = 1;
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(11);

			break;

		case 13:
			_read_stage = 2;
			ReadUltrasonicSensor(3);
			ReadUltrasonicSensor(4);

			break;

		case 14:
			_read_stage = 3;
			InitUltrasonicSensorRx(2);
			InitUltrasonicSensorRx(5);
			_ultrasonic_packet[2].Time_Tx = _system_time;
			_ultrasonic_packet[5].Time_Tx = _system_time;
			break;

		case 15:
			InitUltrasonicSensorRx(8);
			InitUltrasonicSensorRx(10);
			_ultrasonic_packet[8].Time_Tx = _system_time;
			_ultrasonic_packet[9].Time_Tx = _system_time;
			_ultrasonic_packet[10].Time_Tx = _system_time;
			_ultrasonic_packet[11].Time_Tx = _system_time;
			break;

		case 19:
			ReadUltrasonicSensorRxs(1,1);
			ReadUltrasonicSensorRxs(2,4);
			break;

		case 20:
			ReadUltrasonicSensorRxs(1,2);
			ReadUltrasonicSensorRxs(2,5);
			break;

		case 21:
			ReadUltrasonicSensorRxs(1,3);
			ReadUltrasonicSensorRxs(2,6);

			_ultrasonic_datas[2] = _ultrasonic_location_datas[4];
			_ultrasonic_datas[5] = _ultrasonic_location_datas[7];
			break;

		case 22:
			_read_stage = 4;
			InitUltrasonicSensor(0);
			InitUltrasonicSensor(7);
			_ultrasonic_packet[0].Time_Tx = _system_time;
			_ultrasonic_packet[7].Time_Tx = _system_time;
			break;

		case 23:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(10);
			break;

		case 25:
			_read_stage = 5;
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(11);

			break;

		case 27:
			_read_stage = 6;
			ReadUltrasonicSensor(0);
			ReadUltrasonicSensor(7);
			break;

		default:
			break;
	}
}


/*
 * 调度状态类型1: 多发多收调度,针对定时器为6ms时的优化
 * */
void Ultrasonic::UltrasonicScheduleStatusMachineType1_V3(void)
{
	switch(_schedule_time_cnt)
	{
		case 0:
			InitUltrasonicSensorRx(1);
			InitUltrasonicSensorRx(6);
			_ultrasonic_packet[1].Time_Tx  = _system_time;
			_ultrasonic_packet[6].Time_Tx = _system_time;
			break;

		case 1:
			InitUltrasonicSensorRx(8);
			InitUltrasonicSensorRx(10);
			_ultrasonic_packet[8].Time_Tx = _system_time;
			_ultrasonic_packet[9].Time_Tx = _system_time;
			_ultrasonic_packet[10].Time_Tx = _system_time;
			_ultrasonic_packet[11].Time_Tx = _system_time;
			break;

		case 4:
			ReadUltrasonicSensorRxs(0,1);
			ReadUltrasonicSensorRxs(3,6);
			break;

		case 5:
			InitUltrasonicSensor(3);
			InitUltrasonicSensor(4);
			_ultrasonic_packet[3].Time_Tx = _system_time;
			_ultrasonic_packet[4].Time_Tx = _system_time;
			_ultrasonic_datas[1] = _ultrasonic_location_datas[1];
			_ultrasonic_datas[6] = _ultrasonic_location_datas[10];
			break;

		case 6:
			ReadUltrasonicSensorRxs(0,0);
			ReadUltrasonicSensorRxs(3,5);
			break;

		case 7:
			ReadUltrasonicSensorRxs(0,2);
			ReadUltrasonicSensorRxs(3,7);
			break;

		case 8:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(10);
			break;

		case 9:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(11);
			break;

		case 10:
			ReadUltrasonicSensor(3);
			ReadUltrasonicSensor(4);
			break;

		case 11:
			InitUltrasonicSensorRx(2);
			InitUltrasonicSensorRx(5);
			_ultrasonic_packet[2].Time_Tx = _system_time;
			_ultrasonic_packet[5].Time_Tx = _system_time;
			break;

		case 12:
			InitUltrasonicSensorRx(8);
			InitUltrasonicSensorRx(10);
			_ultrasonic_packet[8].Time_Tx = _system_time;
			_ultrasonic_packet[9].Time_Tx = _system_time;
			_ultrasonic_packet[10].Time_Tx = _system_time;
			_ultrasonic_packet[11].Time_Tx = _system_time;
			break;

		case 15:
			ReadUltrasonicSensorRxs(1,2);
			ReadUltrasonicSensorRxs(2,5);
			break;

		case 16:
			InitUltrasonicSensor(0);
			InitUltrasonicSensor(7);
			_ultrasonic_packet[0].Time_Tx = _system_time;
			_ultrasonic_packet[7].Time_Tx = _system_time;
			_ultrasonic_datas[2] = _ultrasonic_location_datas[4];
			_ultrasonic_datas[5] = _ultrasonic_location_datas[7];
			break;

		case 17:
			ReadUltrasonicSensorRxs(1,1);
			ReadUltrasonicSensorRxs(2,4);
			break;

		case 18:
			ReadUltrasonicSensorRxs(1,3);
			ReadUltrasonicSensorRxs(2,6);
			break;

		case 19:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(10);
			break;

		case 20:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(11);
			break;

		case 21:
			ReadUltrasonicSensor(0);
			ReadUltrasonicSensor(7);
			break;

		default:
			break;
	}
}

/*
 * 调度状态类型2: 优先长距调度，定时6ms，最小化调度周期
 */
void Ultrasonic::UltrasonicScheduleStatusMachineType2_V3(void)
{
	switch(_schedule_time_cnt)
	{
		case 0:
			InitUltrasonicSensorRx(8);
			InitUltrasonicSensorRx(10);
			_ultrasonic_packet[8].Time_Tx  = _system_time;
			_ultrasonic_packet[9].Time_Tx  = _system_time;
			_ultrasonic_packet[10].Time_Tx = _system_time;
			_ultrasonic_packet[11].Time_Tx = _system_time;
			break;

		case 1:
			InitUltrasonicSensorRx(1);
			InitUltrasonicSensorRx(6);
			_ultrasonic_packet[1].Time_Tx = _system_time;
			_ultrasonic_packet[6].Time_Tx = _system_time;
			break;

		case 5:
			ReadUltrasonicSensor(1);
			ReadUltrasonicSensor(6);
			break;

		case 6:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(10);
			break;

		case 7:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(11);
			break;

		case 8:
			InitUltrasonicSensorRx(8);
			InitUltrasonicSensorRx(10);
			_ultrasonic_packet[8].Time_Tx  = _system_time;
			_ultrasonic_packet[9].Time_Tx  = _system_time;
			_ultrasonic_packet[10].Time_Tx = _system_time;
			_ultrasonic_packet[11].Time_Tx = _system_time;
			break;

		case 9:
			InitUltrasonicSensorRx(3);
			InitUltrasonicSensorRx(4);
			_ultrasonic_packet[3].Time_Tx = _system_time;
			_ultrasonic_packet[4].Time_Tx = _system_time;
			break;

		case 13:
			ReadUltrasonicSensor(3);
			ReadUltrasonicSensor(4);
			break;

		case 14:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(10);
			break;

		case 15:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(11);
			break;

		case 16:
			InitUltrasonicSensorRx(8);
			InitUltrasonicSensorRx(10);
			_ultrasonic_packet[8].Time_Tx  = _system_time;
			_ultrasonic_packet[9].Time_Tx  = _system_time;
			_ultrasonic_packet[10].Time_Tx = _system_time;
			_ultrasonic_packet[11].Time_Tx = _system_time;
			break;

		case 17:
			InitUltrasonicSensorRx(0);
			InitUltrasonicSensorRx(7);
			_ultrasonic_packet[0].Time_Tx = _system_time;
			_ultrasonic_packet[7].Time_Tx = _system_time;
			break;

		case 21:
			ReadUltrasonicSensor(0);
			ReadUltrasonicSensor(7);
			break;

		case 22:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(10);
			break;

		case 23:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(11);
			break;

		case 24:
			InitUltrasonicSensorRx(8);
			InitUltrasonicSensorRx(10);
			_ultrasonic_packet[8].Time_Tx  = _system_time;
			_ultrasonic_packet[9].Time_Tx  = _system_time;
			_ultrasonic_packet[10].Time_Tx = _system_time;
			_ultrasonic_packet[11].Time_Tx = _system_time;
			break;

		case 25:
			InitUltrasonicSensorRx(2);
			InitUltrasonicSensorRx(5);
			_ultrasonic_packet[2].Time_Tx = _system_time;
			_ultrasonic_packet[5].Time_Tx = _system_time;
			break;

		case 29:
			ReadUltrasonicSensor(2);
			ReadUltrasonicSensor(5);
			break;

		case 30:
			ReadUltrasonicSensor(8);
			ReadUltrasonicSensor(10);
			break;

		case 31:
			ReadUltrasonicSensor(9);
			ReadUltrasonicSensor(11);
			break;
		default:
			break;
	}
}
/*
 * type: 0 -> 短距解码
 * 		 1 -> 长距解码
 * */
void Ultrasonic::UltrasonicConvert(uint8_t type,LIN_RAM d,Ultrasonic_Packet *p,float t)
{
	float temp;
	temp = Compensation(t);
	if(0 == type)
	{
		 p->Distance1 = d.STP318.TOF * temp;
		 p->status    = d.STP318.Status;
		 p->Time_Ms   = p->Time_Tx * 5 + d.STP318.TOF * 0.0005;
	}
	else if(1 == type)
	{
		p->Distance1 = d.STP313.TOF1 * temp;
		p->Distance2 = d.STP313.TOF2 * temp;
		p->Level     = d.STP313.Level * LEVEL_RATIO;
		p->Width     = d.STP313.Width * WIDTH_RATIO;
		p->status    = d.STP313.Status;
		p->Time_Ms   = p->Time_Tx * 5 + d.STP313.TOF1 * 0.0005;
	}
	else
	{

	}
}

float Ultrasonic::Compensation(float temp)
{
	return (331.5 + 0.60714 * temp) * 0.0000005; // (m/us)/2
}

/*
 * 老版本的更新状态机
 * 基于DMA中断
 * */
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

/*
 * 基于定时器的调度更新
 * */
void Ultrasonic::Update(float t)
{

#if ULTRASONIC_SCHEDULE_MODO == 2
	switch(_schedule_time_cnt)
	{
		case 0:
			UltrasonicConvert(0,_ultrasonic_datas[2],&_ultrasonic_packet[2],t);
			UltrasonicConvert(0,_ultrasonic_datas[4],&_ultrasonic_packet[4],t);
		break;

		case 7:
			UltrasonicConvert(0,_ultrasonic_datas[1],&_ultrasonic_packet[1],t);
			UltrasonicConvert(0,_ultrasonic_datas[7],&_ultrasonic_packet[7],t);
		break;

		case 10:
			UltrasonicConvert(1,_ultrasonic_datas[8],&_ultrasonic_packet[8],t);
			UltrasonicConvert(1,_ultrasonic_datas[11],&_ultrasonic_packet[11],t);
		break;

		case 12:
			UltrasonicConvert(1,_ultrasonic_datas[9],&_ultrasonic_packet[9],t);
			UltrasonicConvert(1,_ultrasonic_datas[10],&_ultrasonic_packet[10],t);
		break;

		case 13:
			UltrasonicConvert(0,_ultrasonic_datas[3],&_ultrasonic_packet[3],t);
			UltrasonicConvert(0,_ultrasonic_datas[5],&_ultrasonic_packet[5],t);
		break;

		case 20:
			UltrasonicConvert(0,_ultrasonic_datas[0],&_ultrasonic_packet[0],t);
			UltrasonicConvert(0,_ultrasonic_datas[6],&_ultrasonic_packet[6],t);
		break;

		case 23:
			UltrasonicConvert(1,_ultrasonic_datas[8],&_ultrasonic_packet[8],t);
			UltrasonicConvert(1,_ultrasonic_datas[11],&_ultrasonic_packet[11],t);
		break;

		case 25:
			UltrasonicConvert(1,_ultrasonic_datas[9],&_ultrasonic_packet[9],t);
			UltrasonicConvert(1,_ultrasonic_datas[10],&_ultrasonic_packet[10],t);
		break;

		default:

		break;
	}
#endif

#if ULTRASONIC_SCHEDULE_MODO == 3
	switch(_schedule_time_cnt)
	{
		case 0:
			UltrasonicConvert(0,_ultrasonic_datas[0],&_ultrasonic_packet[0],t);
			UltrasonicConvert(0,_ultrasonic_datas[7],&_ultrasonic_packet[7],t);
		break;

		case 8:
			UltrasonicConvert(0,_ultrasonic_datas[1],&_ultrasonic_packet[1],t);
			UltrasonicConvert(0,_ultrasonic_datas[6],&_ultrasonic_packet[6],t);

			UltrasonicConvert(0,_ultrasonic_location_datas[0],&_ultrasonic_location_packet[0],t);
			UltrasonicConvert(0,_ultrasonic_location_datas[1],&_ultrasonic_location_packet[1],t);
			UltrasonicConvert(0,_ultrasonic_location_datas[2],&_ultrasonic_location_packet[2],t);
			_ultrasonic_location_packet[0].Distance1 = 2 * _ultrasonic_location_packet[0].Distance1 - _ultrasonic_location_packet[1].Distance1;
			_ultrasonic_location_packet[2].Distance1 = 2 * _ultrasonic_location_packet[2].Distance1 - _ultrasonic_location_packet[1].Distance1;

			UltrasonicConvert(0,_ultrasonic_location_datas[9],&_ultrasonic_location_packet[9],t);
			UltrasonicConvert(0,_ultrasonic_location_datas[10],&_ultrasonic_location_packet[10],t);
			UltrasonicConvert(0,_ultrasonic_location_datas[11],&_ultrasonic_location_packet[11],t);
			_ultrasonic_location_packet[9].Distance1  = 2 * _ultrasonic_location_packet[9].Distance1  - _ultrasonic_location_packet[10].Distance1;
			_ultrasonic_location_packet[11].Distance1 = 2 * _ultrasonic_location_packet[11].Distance1 - _ultrasonic_location_packet[10].Distance1;
		break;

		case 11:
			UltrasonicConvert(1,_ultrasonic_datas[8],&_ultrasonic_packet[8],t);
			UltrasonicConvert(1,_ultrasonic_datas[10],&_ultrasonic_packet[10],t);
		break;

		case 13:
			UltrasonicConvert(1,_ultrasonic_datas[9],&_ultrasonic_packet[9],t);
			UltrasonicConvert(1,_ultrasonic_datas[11],&_ultrasonic_packet[11],t);
		break;

		case 14:
			UltrasonicConvert(0,_ultrasonic_datas[3],&_ultrasonic_packet[3],t);
			UltrasonicConvert(0,_ultrasonic_datas[4],&_ultrasonic_packet[4],t);
		break;

		case 22:
			UltrasonicConvert(0,_ultrasonic_datas[2],&_ultrasonic_packet[2],t);
			UltrasonicConvert(0,_ultrasonic_datas[5],&_ultrasonic_packet[5],t);

			UltrasonicConvert(0,_ultrasonic_location_datas[3],&_ultrasonic_location_packet[3],t);
			UltrasonicConvert(0,_ultrasonic_location_datas[4],&_ultrasonic_location_packet[4],t);
			UltrasonicConvert(0,_ultrasonic_location_datas[5],&_ultrasonic_location_packet[5],t);
			_ultrasonic_location_packet[3].Distance1 = 2 * _ultrasonic_location_packet[3].Distance1 - _ultrasonic_location_packet[4].Distance1;
			_ultrasonic_location_packet[5].Distance1 = 2 * _ultrasonic_location_packet[5].Distance1 - _ultrasonic_location_packet[4].Distance1;

			UltrasonicConvert(0,_ultrasonic_location_datas[6],&_ultrasonic_location_packet[6],t);
			UltrasonicConvert(0,_ultrasonic_location_datas[7],&_ultrasonic_location_packet[7],t);
			UltrasonicConvert(0,_ultrasonic_location_datas[8],&_ultrasonic_location_packet[8],t);
			_ultrasonic_location_packet[6].Distance1 = 2 * _ultrasonic_location_packet[6].Distance1 - _ultrasonic_location_packet[7].Distance1;
			_ultrasonic_location_packet[8].Distance1 = 2 * _ultrasonic_location_packet[8].Distance1 - _ultrasonic_location_packet[7].Distance1;
		break;

		case 25:
			UltrasonicConvert(1,_ultrasonic_datas[8],&_ultrasonic_packet[8],t);
			UltrasonicConvert(1,_ultrasonic_datas[10],&_ultrasonic_packet[10],t);
		break;

		case 27:
			UltrasonicConvert(1,_ultrasonic_datas[9],&_ultrasonic_packet[9],t);
			UltrasonicConvert(1,_ultrasonic_datas[11],&_ultrasonic_packet[11],t);
		break;

		default:

		break;
	}
#endif
}

/*
 * 数据更新类型1 ：三角定位全部更新，使整个周期时间最短
 */
void Ultrasonic::DateUpdateType1_V3(float t)
{
	switch(_schedule_time_cnt)
	{
		case 0:
			UltrasonicConvert(0, _ultrasonic_datas[0], &_ultrasonic_packet[0],t);
			UltrasonicConvert(0, _ultrasonic_datas[7], &_ultrasonic_packet[7],t);
		break;

		case 8:
			UltrasonicConvert(0, _ultrasonic_datas[1], &_ultrasonic_packet[1],t);
			UltrasonicConvert(0, _ultrasonic_datas[6], &_ultrasonic_packet[6],t);

			UltrasonicConvert(0, _ultrasonic_location_datas[0], &_ultrasonic_location_packet[0],t);
			UltrasonicConvert(0, _ultrasonic_location_datas[1], &_ultrasonic_location_packet[1],t);
			UltrasonicConvert(0, _ultrasonic_location_datas[2], &_ultrasonic_location_packet[2],t);
			_ultrasonic_location_packet[0].Distance1 = 2 * _ultrasonic_location_packet[0].Distance1 - _ultrasonic_location_packet[1].Distance1;
			_ultrasonic_location_packet[2].Distance1 = 2 * _ultrasonic_location_packet[2].Distance1 - _ultrasonic_location_packet[1].Distance1;

			UltrasonicConvert(0, _ultrasonic_location_datas[9] , &_ultrasonic_location_packet[9] , t);
			UltrasonicConvert(0, _ultrasonic_location_datas[10], &_ultrasonic_location_packet[10], t);
			UltrasonicConvert(0, _ultrasonic_location_datas[11], &_ultrasonic_location_packet[11], t);
			_ultrasonic_location_packet[9].Distance1  = 2 * _ultrasonic_location_packet[9].Distance1  - _ultrasonic_location_packet[10].Distance1;
			_ultrasonic_location_packet[11].Distance1 = 2 * _ultrasonic_location_packet[11].Distance1 - _ultrasonic_location_packet[10].Distance1;
		break;

		case 9:
			UltrasonicConvert(1, _ultrasonic_datas[8] , &_ultrasonic_packet[8] , t);
			UltrasonicConvert(1, _ultrasonic_datas[10], &_ultrasonic_packet[10], t);
		break;

		case 10:
			UltrasonicConvert(1, _ultrasonic_datas[9] , &_ultrasonic_packet[9],t);
			UltrasonicConvert(1, _ultrasonic_datas[11], &_ultrasonic_packet[11],t);
		break;

		case 11:
			UltrasonicConvert(0, _ultrasonic_datas[3],& _ultrasonic_packet[3],t);
			UltrasonicConvert(0, _ultrasonic_datas[4],& _ultrasonic_packet[4],t);
		break;

		case 19:
			UltrasonicConvert(0, _ultrasonic_datas[2], &_ultrasonic_packet[2],t);
			UltrasonicConvert(0, _ultrasonic_datas[5], &_ultrasonic_packet[5],t);

			UltrasonicConvert(0, _ultrasonic_location_datas[3], &_ultrasonic_location_packet[3], t);
			UltrasonicConvert(0, _ultrasonic_location_datas[4], &_ultrasonic_location_packet[4], t);
			UltrasonicConvert(0, _ultrasonic_location_datas[5], &_ultrasonic_location_packet[5], t);
			_ultrasonic_location_packet[3].Distance1 = 2 * _ultrasonic_location_packet[3].Distance1 - _ultrasonic_location_packet[4].Distance1;
			_ultrasonic_location_packet[5].Distance1 = 2 * _ultrasonic_location_packet[5].Distance1 - _ultrasonic_location_packet[4].Distance1;

			UltrasonicConvert(0, _ultrasonic_location_datas[6], &_ultrasonic_location_packet[6], t);
			UltrasonicConvert(0, _ultrasonic_location_datas[7], &_ultrasonic_location_packet[7], t);
			UltrasonicConvert(0, _ultrasonic_location_datas[8], &_ultrasonic_location_packet[8], t);
			_ultrasonic_location_packet[6].Distance1 = 2 * _ultrasonic_location_packet[6].Distance1 - _ultrasonic_location_packet[7].Distance1;
			_ultrasonic_location_packet[8].Distance1 = 2 * _ultrasonic_location_packet[8].Distance1 - _ultrasonic_location_packet[7].Distance1;
		break;

		case 20:
			UltrasonicConvert(1, _ultrasonic_datas[8] , &_ultrasonic_packet[8] , t);
			UltrasonicConvert(1, _ultrasonic_datas[10], &_ultrasonic_packet[10], t);
		break;

		case 21:
			UltrasonicConvert(1, _ultrasonic_datas[9] , &_ultrasonic_packet[9] , t);
			UltrasonicConvert(1, _ultrasonic_datas[11], &_ultrasonic_packet[11], t);
		break;

		default:
		break;
	}
}
/*
 * 数据更新类型2 ：对于长距传感器采集优先策略，保证长距传感器单个周期最小
 */
void Ultrasonic::DateUpdateType2_V3(float t)
{
	switch(_schedule_time_cnt)
	{
		case 6:
			UltrasonicConvert(0,_ultrasonic_datas[1],&_ultrasonic_packet[1],t);
			UltrasonicConvert(0,_ultrasonic_datas[6],&_ultrasonic_packet[6],t);
		break;

		case 14:
			UltrasonicConvert(0,_ultrasonic_datas[3],&_ultrasonic_packet[3],t);
			UltrasonicConvert(0,_ultrasonic_datas[4],&_ultrasonic_packet[4],t);
		break;

		case 22:
			UltrasonicConvert(0,_ultrasonic_datas[0],&_ultrasonic_packet[0],t);
			UltrasonicConvert(0,_ultrasonic_datas[7],&_ultrasonic_packet[7],t);
		break;

		case 30:
			UltrasonicConvert(0,_ultrasonic_datas[2],&_ultrasonic_packet[2],t);
			UltrasonicConvert(0,_ultrasonic_datas[5],&_ultrasonic_packet[5],t);
		break;

		case 7:
		case 15:
		case 23:
		case 31:
			UltrasonicConvert(1,_ultrasonic_datas[8],&_ultrasonic_packet[8],t);
			UltrasonicConvert(1,_ultrasonic_datas[10],&_ultrasonic_packet[10],t);
		break;

		case 8:
		case 16:
		case 24:
		case 0:
			UltrasonicConvert(1,_ultrasonic_datas[9],&_ultrasonic_packet[9],t);
			UltrasonicConvert(1,_ultrasonic_datas[11],&_ultrasonic_packet[11],t);
		break;

		default:
		break;
	}
}
/*
 * 直接测量数据的传感器坐标系与载体坐标系的转换
 * position: 传感器的安装坐标
 * data    : 超声波数据
 * location: 障碍物相对车体的位置
 * */
void Ultrasonic::BodyDirectCalculate(Location position,Ultrasonic_Packet data,ObstacleLocationPacket *location)
{
	Vector2d temp_angle;
	if(0 == data.status)
	{
		if(0 == data.Distance1)
		{
			location->Position = position.Point;
			location->Status = OverDetection;
		}
		else
		{
			temp_angle = Vector2d(data.Distance1,0);
			location->Position = position.Point + temp_angle.rotate(position.Angle);
			location->Status = Normal;
		}
	}
	else
	{
		location->Position = position.Point;
		if(16 == data.status)
		{
			location->Status = BlindZone;
		}
		else if(2 == data.status)
		{
			location->Status = Noise;
		}
	}
}

/*
 * 三角定位测量值 由传感器坐标系转到载体坐标系
 * position_a:传感器安装位置a
 * position_b:传感器安装位置b
 * data_ul:三角定位测量的左边长值
 * data_ur:三角定位测量的右边长值
 * location:障碍物定位坐标
 * */
void Ultrasonic::BodyTriangleCalculate( Location position_a,Location position_b,
										Ultrasonic_Packet data_ul,Ultrasonic_Packet data_ur,
										ObstacleLocationPacket *location)
{
	float bottom_edge;
	float alpha,beta;
	Vector2d temp_angle;

	if( (0 == data_ul.status) && (0 == data_ur.status))
	{
		if( (data_ul.Distance1 < 1.0e-6f) || (data_ur.Distance1 < 1.0e-6f))
		{
			location->Position = position_a.Point;
			location->Status   = OverDetection;
		}
		else
		{
			bottom_edge = ( position_b.Point - position_a.Point ).Length();
			if(   (     (data_ul.Distance1 + data_ur.Distance1) >  bottom_edge      )
				&&( fabs(data_ul.Distance1 - data_ur.Distance1) <  bottom_edge      )
			)
			{
				alpha = ( position_b.Point - position_a.Point ).Angle();
				if(position_a.Angle > (PI + 1.0e-6f))
				{
					beta =  acosf( (bottom_edge * bottom_edge + data_ul.Distance1 * data_ul.Distance1 - data_ur.Distance1 * data_ur.Distance1) / (2 * data_ul.Distance1 * bottom_edge));
				}
				else
				{
					beta = -acosf( (bottom_edge * bottom_edge + data_ul.Distance1 * data_ul.Distance1 - data_ur.Distance1 * data_ur.Distance1) / (2 * data_ul.Distance1 * bottom_edge));
				}
				temp_angle = Vector2d(data_ul.Distance1,0);
				location->Position = position_a.Point + temp_angle.rotate(alpha + beta);
				location->Status   = Normal;
			}
			else
			{
				location->Position = position_a.Point;
				location->Status   = InvalidPoint;
			}
		}
	}
	else
	{
		location->Position = position_a.Point;
		if( (16 == data_ul.status) || (16 == data_ur.status))
		{
			location->Status = BlindZone;
		}
		else if((2 == data_ul.status) || (2 == data_ur.status))
		{
			location->Status = Noise;
		}
		else
		{
			location->Status = InvalidPoint;
		}
	}
}

/*
 * 三角定位地面坐标系的转换
 * vehicle:车辆状态信息
 * body   :障碍物相对于载体坐标系的坐标
 * ground :障碍物相对于地面坐标系的坐标
 * */
void Ultrasonic::GroundTriangleCalculate(VehicleState *vehicle,ObstacleLocationPacket body,ObstacleLocationPacket *ground)
{
	ground->Position = vehicle->getPosition() + body.Position.rotate(vehicle->getYaw());
	ground->Status   = body.Status;
}

/*
 * 基于三角定位算法的边沿检测
 * vehicle:车辆跟踪位置信息
 * position：超声波的车体坐标系下的位置
 * u_data：超声数据包
 * buf_dat:缓存的上次有效超声数据
 * body_location：车辆坐标系下的车辆边沿坐标位置
 * */
void Ultrasonic::ParkingEdgeCalculate(	VehicleState *vehicle,Location position,Ultrasonic_Packet u_data,
										ParkingEdgeBufferLocationPacket *buf_dat,ObstacleLocationPacket *body_location)
{
	Vector2d body_sensor_position;
	Location start_point,end_point;

	body_sensor_position = vehicle->getPosition() + position.Point.rotate(vehicle->getYaw());

	start_point.Point = Vector2d(0.0f,0.0f);
	start_point.Angle = position.Angle;

	end_point.Point = buf_dat->Position - body_sensor_position;
	end_point.Angle = position.Angle;

	BodyTriangleCalculate(start_point,end_point,u_data,buf_dat->UltrasonicData,body_location);

	buf_dat->Position = body_sensor_position;
	buf_dat->UltrasonicData = u_data;
}
/*
 * 基于直接测量值的载体坐标系转换
 * */
void Ultrasonic::BodyDirectLocation()
{
	switch(ScheduleTimeCnt)
	{
		case 0:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[0],UltrasonicPacket[0],&AbstacleBodyPositionDirect[0]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[7],UltrasonicPacket[7],&AbstacleBodyPositionDirect[7]);
			break;


		case 8:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[1],UltrasonicPacket[1],&AbstacleBodyPositionDirect[1]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[6],UltrasonicPacket[6],&AbstacleBodyPositionDirect[6]);
			break;

		case 11:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[8],UltrasonicPacket[8],&AbstacleBodyPositionDirect[8]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[10],UltrasonicPacket[10],&AbstacleBodyPositionDirect[10]);
			break;

		case 13:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[9],UltrasonicPacket[9],&AbstacleBodyPositionDirect[9]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[11],UltrasonicPacket[11],&AbstacleBodyPositionDirect[11]);
			break;

		case 14:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[3],UltrasonicPacket[3],&AbstacleBodyPositionDirect[3]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[4],UltrasonicPacket[4],&AbstacleBodyPositionDirect[4]);
			break;

		case 22:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[2],UltrasonicPacket[2],&AbstacleBodyPositionDirect[2]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[5],UltrasonicPacket[5],&AbstacleBodyPositionDirect[5]);
			break;


		case 25:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[8],UltrasonicPacket[8],&AbstacleBodyPositionDirect[8]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[10],UltrasonicPacket[10],&AbstacleBodyPositionDirect[10]);

			break;

		case 27:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[9],UltrasonicPacket[9],&AbstacleBodyPositionDirect[9]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[11],UltrasonicPacket[11],&AbstacleBodyPositionDirect[11]);
			break;

		default:
			break;
	}
}

void Ultrasonic::BodyTriangleLocation()
{
	switch(ScheduleTimeCnt)
	{
		case 8:
			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[0],_abstacle_config.UltrasonicLocationArray[1],
								  UltrasonicLocationPacket[0],UltrasonicLocationPacket[1],
								  &AbstacleBodyPositionTriangle[0]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[1],_abstacle_config.UltrasonicLocationArray[2],
								  UltrasonicLocationPacket[1],UltrasonicLocationPacket[2],
								  &AbstacleBodyPositionTriangle[1]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[1],_abstacle_config.UltrasonicLocationArray[2],
								  UltrasonicLocationPacket[3],UltrasonicLocationPacket[4],
								  &AbstacleBodyPositionTriangle[2]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[2],_abstacle_config.UltrasonicLocationArray[3],
								  UltrasonicLocationPacket[4],UltrasonicLocationPacket[5],
								  &AbstacleBodyPositionTriangle[3]);
			break;

		case 22:
			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[4],_abstacle_config.UltrasonicLocationArray[5],
								  UltrasonicLocationPacket[6],UltrasonicLocationPacket[7],
								  &AbstacleBodyPositionTriangle[4]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[5],_abstacle_config.UltrasonicLocationArray[6],
								  UltrasonicLocationPacket[7],UltrasonicLocationPacket[8],
								  &AbstacleBodyPositionTriangle[5]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[5],_abstacle_config.UltrasonicLocationArray[6],
								  UltrasonicLocationPacket[9],UltrasonicLocationPacket[10],
								  &AbstacleBodyPositionTriangle[6]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[6],_abstacle_config.UltrasonicLocationArray[7],
								  UltrasonicLocationPacket[10],UltrasonicLocationPacket[11],
								  &AbstacleBodyPositionTriangle[7]);
			break;

		default:
			break;
	}
}

/*
 * 类型1： 基于直接测量值的载体坐标系转换
 * */
void Ultrasonic::BodyDirectLocationType1()
{
	switch(ScheduleTimeCnt)
	{
		case 0:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[0],UltrasonicPacket[0],&AbstacleBodyPositionDirect[0]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[7],UltrasonicPacket[7],&AbstacleBodyPositionDirect[7]);
			break;


		case 5:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[1],UltrasonicPacket[1],&AbstacleBodyPositionDirect[1]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[6],UltrasonicPacket[6],&AbstacleBodyPositionDirect[6]);
			break;

		case 9:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[8],UltrasonicPacket[8],&AbstacleBodyPositionDirect[8]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[10],UltrasonicPacket[10],&AbstacleBodyPositionDirect[10]);
			break;

		case 10:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[9],UltrasonicPacket[9],&AbstacleBodyPositionDirect[9]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[11],UltrasonicPacket[11],&AbstacleBodyPositionDirect[11]);
			break;

		case 11:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[3],UltrasonicPacket[3],&AbstacleBodyPositionDirect[3]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[4],UltrasonicPacket[4],&AbstacleBodyPositionDirect[4]);
			break;

		case 16:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[2],UltrasonicPacket[2],&AbstacleBodyPositionDirect[2]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[5],UltrasonicPacket[5],&AbstacleBodyPositionDirect[5]);
			break;


		case 20:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[8],UltrasonicPacket[8],&AbstacleBodyPositionDirect[8]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[10],UltrasonicPacket[10],&AbstacleBodyPositionDirect[10]);

			break;

		case 21:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[9],UltrasonicPacket[9],&AbstacleBodyPositionDirect[9]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[11],UltrasonicPacket[11],&AbstacleBodyPositionDirect[11]);
			break;

		default:
			break;
	}
}
/*
 * 类型1： 基于三角定位载体坐标系转换
 * */
void Ultrasonic::BodyTriangleLocationType1()
{
	switch(ScheduleTimeCnt)
	{
		case 8:
			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[0],_abstacle_config.UltrasonicLocationArray[1],
								  UltrasonicLocationPacket[0],UltrasonicLocationPacket[1],
								  &AbstacleBodyPositionTriangle[0]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[1],_abstacle_config.UltrasonicLocationArray[2],
								  UltrasonicLocationPacket[1],UltrasonicLocationPacket[2],
								  &AbstacleBodyPositionTriangle[1]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[1],_abstacle_config.UltrasonicLocationArray[2],
								  UltrasonicLocationPacket[3],UltrasonicLocationPacket[4],
								  &AbstacleBodyPositionTriangle[2]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[2],_abstacle_config.UltrasonicLocationArray[3],
								  UltrasonicLocationPacket[4],UltrasonicLocationPacket[5],
								  &AbstacleBodyPositionTriangle[3]);
			break;

		case 19:
			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[4],_abstacle_config.UltrasonicLocationArray[5],
								  UltrasonicLocationPacket[6],UltrasonicLocationPacket[7],
								  &AbstacleBodyPositionTriangle[4]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[5],_abstacle_config.UltrasonicLocationArray[6],
								  UltrasonicLocationPacket[7],UltrasonicLocationPacket[8],
								  &AbstacleBodyPositionTriangle[5]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[5],_abstacle_config.UltrasonicLocationArray[6],
								  UltrasonicLocationPacket[9],UltrasonicLocationPacket[10],
								  &AbstacleBodyPositionTriangle[6]);

			BodyTriangleCalculate(_abstacle_config.UltrasonicLocationArray[6],_abstacle_config.UltrasonicLocationArray[7],
								  UltrasonicLocationPacket[10],UltrasonicLocationPacket[11],
								  &AbstacleBodyPositionTriangle[7]);
			break;

		default:
			break;
	}
}

void Ultrasonic::GroundTriangleLocation(VehicleState *vehicle_state)
{
	switch(ScheduleTimeCnt)
	{
		case 8:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[0],&AbstacleGroundPositionTriangle[0]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[1],&AbstacleGroundPositionTriangle[1]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[2],&AbstacleGroundPositionTriangle[2]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[3],&AbstacleGroundPositionTriangle[3]);
			break;

		case 11:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[8],&AbstacleGroundPositionTriangle[8]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[10],&AbstacleGroundPositionTriangle[10]);
			break;

		case 13:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[9],&AbstacleGroundPositionTriangle[9]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[11],&AbstacleGroundPositionTriangle[11]);
			break;

		case 22:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[4],&AbstacleGroundPositionTriangle[4]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[5],&AbstacleGroundPositionTriangle[5]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[6],&AbstacleGroundPositionTriangle[6]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[7],&AbstacleGroundPositionTriangle[7]);
			break;

		case 25:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[8],&AbstacleGroundPositionTriangle[8]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[10],&AbstacleGroundPositionTriangle[10]);
			break;

		case 27:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[9],&AbstacleGroundPositionTriangle[9]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[11],&AbstacleGroundPositionTriangle[11]);
			break;

		default:
			break;
	}
}

/*
 * Parking Edge Detect
 * */
void Ultrasonic::ParkingEdgeTriangleLocation(VehicleState *vehicle_state)
{
	switch(ScheduleTimeCnt)
		{
			case 11:
				ParkingEdgeCalculate(vehicle_state,_abstacle_config.UltrasonicLocationArray[8],UltrasonicLocationPacket[8],
						             &_ultrasonic_data_buffer[0],&_abstacle_body_position_triangle[8]);
				ParkingEdgeCalculate(vehicle_state,_abstacle_config.UltrasonicLocationArray[10],UltrasonicLocationPacket[10],
						             &_ultrasonic_data_buffer[2],&_abstacle_body_position_triangle[10]);
				break;

			case 13:
				ParkingEdgeCalculate(vehicle_state,_abstacle_config.UltrasonicLocationArray[9],UltrasonicLocationPacket[9],
						             &_ultrasonic_data_buffer[1],&_abstacle_body_position_triangle[9]);
				ParkingEdgeCalculate(vehicle_state,_abstacle_config.UltrasonicLocationArray[11],UltrasonicLocationPacket[11],
						             &_ultrasonic_data_buffer[3],&_abstacle_body_position_triangle[11]);
				break;

			case 25:
				ParkingEdgeCalculate(vehicle_state,_abstacle_config.UltrasonicLocationArray[8],UltrasonicLocationPacket[8],
						             &_ultrasonic_data_buffer[0],&_abstacle_body_position_triangle[8]);
				ParkingEdgeCalculate(vehicle_state,_abstacle_config.UltrasonicLocationArray[10],UltrasonicLocationPacket[10],
						             &_ultrasonic_data_buffer[2],&_abstacle_body_position_triangle[10]);
				break;

			case 27:
				ParkingEdgeCalculate(vehicle_state,_abstacle_config.UltrasonicLocationArray[9],UltrasonicLocationPacket[9],
						             &_ultrasonic_data_buffer[1],&_abstacle_body_position_triangle[9]);
				ParkingEdgeCalculate(vehicle_state,_abstacle_config.UltrasonicLocationArray[11],UltrasonicLocationPacket[11],
						             &_ultrasonic_data_buffer[3],&_abstacle_body_position_triangle[11]);
				break;

			default:
				break;
		}
}

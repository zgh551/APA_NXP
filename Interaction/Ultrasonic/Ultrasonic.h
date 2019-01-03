/*
 * Ultrasonic.h
 *
 *  Created on: 2018��12��15��
 *      Author: zhuguohua
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "derivative.h"
#include "property.h"

#define FRONT_ULTRASONIC_ENABLE
#define REAR_ULTRASONIC_ENABLE

#define LEVEL_RATIO    (0.01294117647058823529411764705882)
#define WIDTH_RATIO    (16)
/*** LIN Device Data Struct ***/
typedef struct _LIN_STP318_Packet
{
	uint16_t TOF;
	uint8_t status;
}LIN_STP318_Packet;

typedef struct _LIN_STP313_Packet
{
	uint16_t TOF1;
	uint16_t TOF2;
	uint8_t Level;
	uint8_t Width;
	uint8_t status;
}LIN_STP313_Packet;

typedef struct _Ultrasonic_Packet
{
	float Distance1;
	float Distance2;
	float Level;
	float Width;
	float Time_Ms;
	uint8_t status;
	uint32_t Time_Tx;

}Ultrasonic_Packet;

class Ultrasonic {
public:
	Ultrasonic();
	virtual ~Ultrasonic();

	void Init(void);

	void InitSensing_STP318(uint8_t tx,uint8_t rx,void (*TransmitFrame)(LIN_RAM));
	void InitSensing_STP313(uint8_t tx_rx,void (*TransmitFrame)(LIN_RAM));

	void ReadSensing_STP318(uint8_t id,void (*ReceiveFrame)(LIN_RAM *),LIN_RAM *m_LIN_RAM);
	void ReadSensing_STP313(uint8_t id,void (*ReceiveFrame)(LIN_RAM *),LIN_RAM *m_LIN_RAM);

	void InitUltrasonicSensor(uint8_t n);
	void ReadUltrasonicSensor(uint8_t n);

	void UltrasonicScheduleStatusMachine(void);
	void UltrasonicScheduleStatusMachine_V2(void);

	float Compensation(float temp);

	void Update(uint8_t id,float t);

	/// Property
	uint8_t getScheduleTimeCnt();
	void    setScheduleTimeCnt(uint8_t value);
	Property<Ultrasonic,uint8_t,READ_WRITE> ScheduleTimeCnt;

	uint32_t getSystemTime();
	void     setSystemTime(uint32_t value);
	Property<Ultrasonic,uint32_t,READ_WRITE> SystemTime;

	Ultrasonic_Packet* getUltrasonicPacket();
	Property<Ultrasonic,Ultrasonic_Packet*,READ_ONLY> UltrasonicPacket;

private:
	uint32_t _system_time;
	uint8_t  _schedule_time_cnt;
	uint8_t  _read_stage;

	LIN_RAM _ultrasonic_datas[12];

	Ultrasonic_Packet _ultrasonic_packet[12];
};

#endif /* ULTRASONIC_H_ */

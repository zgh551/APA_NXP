/*
 * Ultrasonic.h
 *
 *  Created on: 2018��12��15��
 *      Author: zhuguohua
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "Property.h"
#include "linflexd.h"
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

class Ultrasonic {
public:
	Ultrasonic();
	virtual ~Ultrasonic();

	void InitSensing_STP318(uint8_t tx,uint8_t rx,void (*TransmitFrame)(LIN_RAM));
	void InitSensing_STP313(uint8_t tx_rx,void (*TransmitFrame)(LIN_RAM));

	void ReadSensing_STP318(uint8_t id,void (*ReceiveFrame)(LIN_RAM *),LIN_RAM *m_LIN_RAM);
	void ReadSensing_STP313(uint8_t id,void (*ReceiveFrame)(LIN_RAM *),LIN_RAM *m_LIN_RAM);

	void InitUltrasonicSensor(uint8_t n);
	void ReadUltrasonicSensor(uint8_t n);

	void TimeScheduleStatus1_SR(void);

	/// Property
	LIN_RAM* getUltrasonicDatas();
	Property<Ultrasonic,LIN_RAM*,READ_ONLY> UltrasonicDatas;

private:
	LIN_RAM _ultrasonic_datas[12];
};

#endif /* ULTRASONIC_H_ */

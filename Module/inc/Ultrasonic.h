/*
 * Ultrasonic.h
 *
 *  Created on: 2018��12��15��
 *      Author: zhuguohua
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

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

	void InitSensing_STP318(uint8_t tx,uint8_t rx);
	LIN_STP318_Packet ReadData_STP318(uint8_t id);

	void InitSensing_STP313(uint8_t tx_rx);
	LIN_STP313_Packet ReadData_STP313(uint8_t id);
};

#endif /* ULTRASONIC_H_ */

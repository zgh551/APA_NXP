/*
 * linflexd_lin.h
 *
 *  Created on: Mar 2, 2016
 *      Author: B55457
 */

#ifndef LINFLEXD_H_
#define LINFLEXD_H_

#include "derivative.h"
#include "project.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _LIN_Packet
{
	uint8_t id;
	uint8_t length;
	union
	{
		struct
		{
			union
			{
				struct
				{
					uint32_t Data3:8;
					uint32_t Data2:8;
					uint32_t Data1:8;
					uint32_t Data0:8;
				};
			};
			union
			{
				struct
				{
					uint32_t Data7:8;
					uint32_t Data6:8;
					uint32_t Data5:8;
					uint32_t Data4:8;
				};
			};
		}B;
		struct
		{
			uint32_t L;
			uint32_t M;
		}R;
	}BufferData;
}LIN_Packet;

void InitLINFlexD0( uint16_t MegaHertz, uint16_t BaudRate );

void LIN0_TransmitFrame (LIN_Packet m_LIN_Packet);
void LIN0_ReceiveFrame(LIN_Packet *m_LIN_Packet);

void transmitLINframe_0(void);
void receiveLINframe_0(void);

void initLINFlexD_1 (void);
void transmitLINframe_1 (void);
void receiveLINframe_1(void);

#ifdef __cplusplus
}
#endif

#endif /* LINFLEXD_LIN_H_ */

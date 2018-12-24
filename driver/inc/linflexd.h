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

typedef struct _LIN_RAM
{
	LINFlexD_LINCR2_tag LINCR2;          /* LIN Control Register 2 */
	LINFlexD_BIDR_tag 	BIDR;              /* Buffer Identifier Register */
	union
	{
		struct
		{
			LINFlexD_BDRL_tag 	BDRL;              /* Buffer Data Register Least Significant */
			LINFlexD_BDRM_tag 	BDRM;              /* Buffer Data Register Most Significant */
		};
		struct
		{
			vuint32_t null:8;
			vuint32_t Status:8;
			vuint32_t TOF:16;
			vuint32_t Null;
		}STP318;
		struct
		{
			vuint32_t Width:8;
			vuint32_t Level:8;
			vuint32_t TOF1:16;
			vuint32_t null:8;
			vuint32_t Status:8;
			vuint32_t TOF2:16;
		}STP313;
	};
}LIN_RAM;

void InitLINFlexD0( uint16_t MegaHertz, uint16_t BaudRate );
void InitLINFlexD0_DMA ( uint16_t MegaHertz, uint16_t BaudRate );

void FlexLin0_DMA_TX_M2S_Init(void);
void FlexLin0_DMA_TX_S2M_Init(void);
void FlexLin0_DMA_RX_S2M_Init(void);

void LIN0_TransmitFrame (LIN_RAM m_LIN_RAM);
void LIN0_TransmitFrame_DMA (LIN_RAM m_LIN_RAM);
void LIN0_ReceiveFrame(LIN_RAM *m_LIN_RAM);
void LIN0_ReceiveFrame_DMA(LIN_RAM *m_LIN_RAM);

//void LIN0_TransmitFrame (LIN_Packet m_LIN_Packet);
//void LIN0_TransmitFrame_DMA (LIN_Packet m_LIN_Packet);
//
//void LIN0_ReceiveFrame(LIN_Packet *m_LIN_Packet);
//void LIN0_ReceiveFrame_DMA(LIN_Packet *m_LIN_Packet);

//void transmitLINframe_0(void);
//void receiveLINframe_0(void);
//
//void initLINFlexD_1 (void);
//void transmitLINframe_1 (void);
//void receiveLINframe_1(void);

#ifdef __cplusplus
}
#endif

#endif /* LINFLEXD_LIN_H_ */

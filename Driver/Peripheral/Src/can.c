/*
 * can.c
 *
 *  Created on: December 29, 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: can.c                          COPYRIGHT (c) Motovis 2018      */

/*                                                      All Rights Reserved  */
/* DESCRIPTION: Transmit & receive LIN messages using LINflexD modules.      */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 29 2018    Initial Version                  */
/*****************************************************************************/
#include "can.h"

void FlexCAN0_Init(void) {              /* General init. No MB IDs iniialized */
  uint8_t	i;

  CAN_0.MCR.B.MDIS = 1;       /* Disable module before selecting clock source*/
  CAN_0.CTRL1.B.CLKSRC= 0;     /* Clock Source = oscillator Clock (8 MHz) */
  CAN_0.MCR.B.MDIS = 0;       /* Enable module for config. (Sets FRZ, HALT)*/
  while (!CAN_0.MCR.B.FRZACK) {} /* Wait for freeze acknowledge to set */
	/* CAN bus: 10 MHz clksrc, 500K bps with 16 tq */
	/* PRESDIV+1 = Fclksrc/Ftq = 8 MHz/8MHz = 1 */
	/*    so PRESDIV = 0 */
	/* PSEG2 = Phase_Seg2 - 1 = 4 - 1 = 3 */
	/* PSEG1 = PSEG2 = 3 */
	/* PROPSEG= Prop_Seg - 1 = 7 - 1 = 6 */
	/* RJW = Resync Jump Width - 1 = 4 = 1 */
	/* SMP = 1: use 3 bits per CAN sample */
	/* CLKSRC=0 (unchanged): Fcanclk= Fxtal= 10 MHz*/
  CAN_0.CTRL1.B.PRESDIV = 0;

  CAN_0.CTRL1.B.PSEG1 = 3;
  CAN_0.CTRL1.B.PSEG2 = 3;
  CAN_0.CTRL1.B.PROPSEG = 6;

  CAN_0.CTRL1.B.RJW = 3;
  CAN_0.CTRL1.B.SMP = 1;


  for(i=0; i<64; i++){ 			//MPC574xP has 64 buffers
    CAN_0.MB[i].CS.B.CODE = 0;   /* Inactivate all message buffers */
  }
  CAN_0.MB[0].CS.B.CODE = 8;     /* Message Buffer 0 set to TX INACTIVE */

// interrupt mask
  CAN_0.IMASK1.R = 0x00000100;
// mail buffer 8 set to receive
  CAN_0.MB[8].CS.B.IDE = 0;      /* MB 8 will look for a standard ID */
  CAN_0.MB[8].ID.B.ID_STD = 0x200; /* MB 8 will look for ID = 0x555 */
  CAN_0.MB[8].CS.B.CODE = 4;     /* MB 8 set to RX EMPTY */
  CAN_0.RXMGMASK.R = 0x1003ffff; /* Global acceptance mask */
  /* set mask registers - all ID bits must match */
  for(i=0;i<64;i++)
  {
      CAN_0.RXIMR[i].R = 0x00;
  }

  /* Configure the CAN0_TX pin to transmit. */
  SIUL2.MSCR[PB0].B.SSS = 1; //PTB0 is for CAN0_TX. Select signal source select to CAN0_TX
  SIUL2.MSCR[PB0].B.OBE = 1; //Set pin to output. Enable output buffer
  SIUL2.MSCR[PB0].B.SRC = 3; //Maximum slew rate

  /* Configure the CAN0_RX pin. */
  SIUL2.MSCR[PB1].B.IBE = 1; //PB1 is CAN0_RX pin. Enable input buffer
  SIUL2.IMCR[32].B.SSS = 0b0010; //Set PB1 as CAN0_RX.

//  CAN_0.MCR.B.AEN = 1;
  CAN_0.MCR.B.MAXMB = 0x3f;
//  CAN_0.MCR.B.IRMQ = 1;
  CAN_0.MCR.B.HALT = 0;
  while (CAN_0.MCR.B.FRZACK & CAN_0.MCR.B.NOTRDY) {} /* Wait to clear */
                 /* Good practice: wait for FRZACK on freeze mode entry/exit */
  INTC_0.PSR[524].R = 0x8009;
}

void FlexCAN1_Init(void) {              /* General init. No MB IDs iniialized */
	uint8_t	i;

	CAN_1.MCR.B.MDIS = 1;       /* Disable module before selecting clock source*/
	CAN_1.CTRL1.B.CLKSRC = 0;     /* Clock Source = oscillator clock (8 MHz) */
	CAN_1.MCR.B.MDIS = 0;       /* Enable module for config. (Sets FRZ, HALT)*/
	while (!CAN_1.MCR.B.FRZACK) {} /* Wait for freeze acknowledge to set */
	/* CAN bus: 40 MHz clksrc, 500K bps with 16 tq */
	/* PRESDIV+1 = Fclksrc/Ftq = 40 MHz/8MHz = 5 */
	/*    so PRESDIV = 4 */
	/* PSEG2 = Phase_Seg2 - 1 = 4 - 1 = 3 */
	/* PSEG1 = PSEG2 = 3 */
	/* PROPSEG= Prop_Seg - 1 = 7 - 1 = 6 */
	/* RJW = Resync Jump Width - 1 = 4 = 1 */
	/* SMP = 1: use 3 bits per CAN sample */
	/* CLKSRC=0 (unchanged): Fcanclk= Fxtal= 40 MHz*/
	CAN_1.CTRL1.B.PRESDIV = 0;
	CAN_1.CTRL1.B.PSEG1 = 3;
	CAN_1.CTRL1.B.PSEG2 = 3;
	CAN_1.CTRL1.B.RJW = 3;
	CAN_1.CTRL1.B.SMP = 1;
	CAN_1.CTRL1.B.PROPSEG = 6;

	for(i=0; i<64; i++)//MPC574xP has 64 buffers
	{
		CAN_1.MB[i].CS.B.CODE = 0;   /* Inactivate all message buffers */
	}
	CAN_1.MB[0].CS.B.CODE = 8;     /* Message Buffer 0 set to TX INACTIVE */

	// interrupt mask
	CAN_1.IMASK1.R = 0x00000100;
	// mail buffer 8 set to receive
	CAN_1.MB[8].CS.B.IDE = 0;      /* MB 8 will look for a standard ID */
	CAN_1.MB[8].ID.B.ID_STD = 0x200; /* MB 8 will look for ID = 0x555 */
	CAN_1.MB[8].CS.B.CODE = 4;     /* MB 8 set to RX EMPTY */
//	CAN_1.RXMGMASK.R = 0x1C03ffff; /* Global acceptance mask */
	CAN_1.RXMGMASK.R = 0x0003ffff; /* Global acceptance mask */
	/* set mask registers - all ID bits must match */
	for(i=0;i<64;i++)
	{
		CAN_1.RXIMR[i].R = 0x00;
	}

	/* Configure the CAN0_TX pin to transmit. */
	SIUL2.MSCR[PA14].B.SSS = 1; //PTA140 is for CAN1_TX. Select signal source select to CAN0_TX
	SIUL2.MSCR[PA14].B.OBE = 1; //Set pin to output. Enable output buffer
	SIUL2.MSCR[PA14].B.SRC = 3; //Maximum slew rate

	/* Configure the CAN0_RX pin. */
	SIUL2.MSCR[PA15].B.IBE = 1; //PA15 is CAN1_RX pin. Enable input buffer
	SIUL2.IMCR[33].B.SSS = 0b0001; //Set PA15 as CAN1_RX.

	//  CAN_1.MCR.B.AEN = 1;
	CAN_1.MCR.B.MAXMB = 0x3f;
	//  CAN_1.MCR.B.IRMQ = 1;
	CAN_1.MCR.B.HALT = 0;
	while (CAN_1.MCR.B.FRZACK & CAN_1.MCR.B.NOTRDY) {} /* Wait to clear */
	/* Good practice: wait for FRZACK on freeze mode entry/exit */
	INTC_0.PSR[537].R = 0x8009;
}

void FlexCAN2_Init(void) {              /* General init. No MB IDs iniialized */
	uint8_t	i;

	CAN_2.MCR.B.MDIS = 1;       /* Disable module before selecting clock source*/
	CAN_2.CTRL1.B.CLKSRC=0;     /* Clock Source = oscillator clock (40 MHz) */
	CAN_2.MCR.B.MDIS = 0;       /* Enable module for config. (Sets FRZ, HALT)*/
	while (!CAN_2.MCR.B.FRZACK) {} /* Wait for freeze acknowledge to set */
	/* CAN bus: 40 MHz clksrc, 500K bps with 16 tq */
	/* PRESDIV+1 = Fclksrc/Ftq = 40 MHz/8MHz = 5 */
	/*    so PRESDIV = 4 */
	/* PSEG2 = Phase_Seg2 - 1 = 4 - 1 = 3 */
	/* PSEG1 = PSEG2 = 3 */
	/* PROPSEG= Prop_Seg - 1 = 7 - 1 = 6 */
	/* RJW = Resync Jump Width - 1 = 4 = 1 */
	/* SMP = 1: use 3 bits per CAN sample */
	/* CLKSRC=0 (unchanged): Fcanclk= Fxtal= 40 MHz*/
	CAN_2.CTRL1.B.PRESDIV = 0;
	CAN_2.CTRL1.B.PSEG1 = 3;
	CAN_2.CTRL1.B.PSEG2 = 3;
	CAN_2.CTRL1.B.RJW = 3;
	CAN_2.CTRL1.B.SMP = 1;
	CAN_2.CTRL1.B.PROPSEG = 6;

	for(i=0; i<64; i++)//MPC574xP has 64 buffers
	{
		CAN_2.MB[i].CS.B.CODE = 0;   /* Inactivate all message buffers */
	}
	CAN_2.MB[0].CS.B.CODE = 8;     /* Message Buffer 0 set to TX INACTIVE */

	// interrupt mask
	CAN_2.IMASK1.R = 0x00000100;
	// mail buffer 8 set to receive
	CAN_2.MB[8].CS.B.IDE = 0;      /* MB 8 will look for a standard ID */
	CAN_2.MB[8].ID.B.ID_STD = 0x500; /* MB 8 will look for ID = 0x555 */
	CAN_2.MB[8].CS.B.CODE = 4;     /* MB 8 set to RX EMPTY */
	CAN_2.RXMGMASK.R = 0x1C03ffff; /* Global acceptance mask */
	/* set mask registers - all ID bits must match */
	for(i=0;i<64;i++)
	{
		CAN_2.RXIMR[i].R = 0x00;
	}

	/* Configure the CAN2_TX pin to transmit. */
	SIUL2.MSCR[PF14].B.SSS = 2; //PTF14 is for CAN2_TX. Select signal source select to CAN2_TX
	SIUL2.MSCR[PF14].B.OBE = 1; //Set pin to output. Enable output buffer
	SIUL2.MSCR[PF14].B.SRC = 3; //Maximum slew rate

	/* Configure the CAN2_RX pin. */
	SIUL2.MSCR[PF15].B.IBE = 1; //PF15 is CAN2_RX pin. Enable input buffer
	SIUL2.IMCR[34].B.SSS = 0b0001; //Set PF15 as CAN2_RX.

	//  CAN_2.MCR.B.AEN = 1;
	CAN_2.MCR.B.MAXMB = 0x3f;
	//  CAN_2.MCR.B.IRMQ = 1;
	CAN_2.MCR.B.HALT = 0;
	while (CAN_2.MCR.B.FRZACK & CAN_2.MCR.B.NOTRDY) {} /* Wait to clear */
	/* Good practice: wait for FRZACK on freeze mode entry/exit */
	INTC_0.PSR[550].R = 0x8009;
}

void CAN_Configure()
{
    FlexCAN0_Init();
    FlexCAN1_Init();
    FlexCAN2_Init();
}

void CAN0_TransmitMsg(CAN_Packet m_CAN_Packet)
{
	vuint8_t i;
	CAN_0.MB[0].CS.B.IDE = 0;       /* Use standard ID length */
	CAN_0.MB[0].ID.B.ID_STD = m_CAN_Packet.id;/* Transmit ID is 0x555 */
	CAN_0.MB[0].CS.B.RTR = 0;       /* Data frame, not remote Tx request frame */
	for (i=0; i<m_CAN_Packet.length; i++)
	{
		CAN_0.MB[0].DATA.B[i] = m_CAN_Packet.data[i];      /* Data to be transmitted */
	}
	CAN_0.MB[0].CS.B.DLC = m_CAN_Packet.length; /*#bytes to transmit w/o null*/
	CAN_0.MB[0].CS.B.SRR = 1;     /* Tx frame (not req'd for standard frame)*/
	CAN_0.MB[0].CS.B.CODE =0xC;   /* Activate msg. buf. to transmit data frame */
	while(CAN_0.MB[0].CS.B.CODE != 0x8 ){}
}

void CAN1_TransmitMsg(CAN_Packet m_CAN_Packet)
{
	vuint8_t i;
	CAN_1.MB[0].CS.B.IDE = 0;       /* Use standard ID length */
	CAN_1.MB[0].ID.B.ID_STD = m_CAN_Packet.id;/* Transmit ID is 0x555 */
	CAN_1.MB[0].CS.B.RTR = 0;       /* Data frame, not remote Tx request frame */
	for (i=0; i<m_CAN_Packet.length; i++)
	{
		CAN_1.MB[0].DATA.B[i] = m_CAN_Packet.data[i];      /* Data to be transmitted */
	}
	CAN_1.MB[0].CS.B.DLC = m_CAN_Packet.length; /*#bytes to transmit w/o null*/
	CAN_1.MB[0].CS.B.SRR = 1;     /* Tx frame (not req'd for standard frame)*/
	CAN_1.MB[0].CS.B.CODE =0xC;   /* Activate msg. buf. to transmit data frame */
	while(CAN_1.MB[0].CS.B.CODE != 0x8 ){}
}

void CAN2_TransmitMsg(CAN_Packet m_CAN_Packet)
{
	vuint8_t i;
	CAN_2.MB[0].CS.B.IDE = 0;       /* Use standard ID length */
	CAN_2.MB[0].ID.B.ID_STD = m_CAN_Packet.id;/* Transmit ID is 0x555 */
	CAN_2.MB[0].CS.B.RTR = 0;       /* Data frame, not remote Tx request frame */
	for (i=0; i<m_CAN_Packet.length; i++)
	{
		CAN_2.MB[0].DATA.B[i] = m_CAN_Packet.data[i];      /* Data to be transmitted */
	}
	CAN_2.MB[0].CS.B.DLC = m_CAN_Packet.length; /*#bytes to transmit w/o null*/
	CAN_2.MB[0].CS.B.SRR = 1;     /* Tx frame (not req'd for standard frame)*/
	CAN_2.MB[0].CS.B.CODE =0xC;   /* Activate msg. buf. to transmit data frame */
	while(CAN_2.MB[0].CS.B.CODE != 0x8 ){}
}

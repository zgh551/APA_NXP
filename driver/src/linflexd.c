/*
 * linflexd.c
 *
 *  Created on: December 14, 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: linflexd.c                   COPYRIGHT (c) Motovis 2018      */

/*                                                      All Rights Reserved  */
/* DESCRIPTION: Transmit & receive LIN messages using LINflexD modules.      */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 14 2018    Initial Version                  */
/*****************************************************************************/
#include "linflexd.h"

uint8_t RxBuffer[8] = {0};

/*******************************************************************************
*
* @brief    initLINFlexD_0 - Init the LinFlexD Module
* @param    MegaHertz: the frequency of the HALFSYS_CLK
* @param    BaudRate : the baudrate of the Lin
* @return   none
*
*******************************************************************************/
void InitLINFlexD0 ( uint16_t MegaHertz, uint16_t BaudRate )
{
  uint16_t Fraction,Integer;

  LINFlexD_0.LINCR1.B.SLEEP = 0; /* Exit Sleep Mode */
  LINFlexD_0.LINCR1.B.INIT = 1;  /* Put LINFlex hardware in init mode */
  /* wait for the INIT mode */
  while (0x1000 != (LINFlexD_0.LINSR.R & 0xF000)) {}

//  LINFlexD_0.LINCR1.R= 0x00000311; /* Configure module as LIN master & header */

  BaudRate  = (MegaHertz * 1000000) / BaudRate;
  Integer   = BaudRate / 16;
  Fraction  = BaudRate - (Integer * 16);

  LINFlexD_0.LINIBRR.B.IBR= Integer;  /* Mantissa baud rate divider component */
         /* Baud rate divider = 100 MHz LIN_CLK input / (16*19200 bps) ~= 326 */
  LINFlexD_0.LINFBRR.B.FBR = Fraction; /* Fraction baud rate divider comonent */

  LINFlexD_0.LINCR2.B.IOBE = 1;

   LINFlexD_0.LINTCSR.B.MODE = 1;
   LINFlexD_0.LINOCR.R = 0x0000ffff;
   LINFlexD_0.LINTCSR.B.MODE = 0;
   LINFlexD_0.LINTCSR.B.IOT = 1;

  LINFlexD_0.LINCR1.R= 0x00000310;  /* Change module mode from init to normal */

  /* Configure LINFlexD_0 TxD Pin. */
  SIUL2.MSCR[PB2].B.SSS = 0b0001; //Pad PB2: Set to LINFlex_0 TxD.
  SIUL2.MSCR[PB2].B.OBE = 1; //Enable output buffer
  SIUL2.MSCR[PB2].B.ODE = 1;    /* Pad PB2: Output Drain Enable */
  SIUL2.MSCR[PB2].B.PUS = 1;    /* Pad PB2: Pull up selected */
  SIUL2.MSCR[PB2].B.PUE = 1;    /* Pad PB2: Pull Enable */
  SIUL2.MSCR[PB2].B.SRC = 3; //Full drive-strength without slew rate control

  /* Configure LINFlexD_0 RxD Pin. */
  SIUL2.MSCR[PB3].B.IBE = 1; //Pad PB3: Enable input buffer
  SIUL2.IMCR[165].B.SSS = 0b0001; //Connect LINFlexD_0 signal to PB3
}

void LIN0_TransmitFrame (LIN_Packet m_LIN_Packet)
{
  LINFlexD_0.BDRL.R = m_LIN_Packet.BufferData.R.L;
  LINFlexD_0.BDRM.R = m_LIN_Packet.BufferData.R.M;

  LINFlexD_0.BIDR.B.ID = m_LIN_Packet.id;
  LINFlexD_0.BIDR.B.DFL = m_LIN_Packet.length - 1;
  LINFlexD_0.BIDR.B.CCS = 0;
  LINFlexD_0.BIDR.B.DIR = 1;
  LINFlexD_0.LINCR2.B.HTRQ = 1;   /* Request header transmission */

  while (!LINFlexD_0.LINSR.B.DTF); /* Wait for data transfer complete flag */
  LINFlexD_0.LINSR.R = 0x00000002;   /* Clear DTF flag */
}

void LIN0_ReceiveFrame(LIN_Packet *m_LIN_Packet)
{
  LINFlexD_0.BIDR.B.ID  = m_LIN_Packet->id;
  LINFlexD_0.BIDR.B.DFL = m_LIN_Packet->length - 1;
  LINFlexD_0.BIDR.B.CCS = 0;
  LINFlexD_0.BIDR.B.DIR = 0;// receive the data
  LINFlexD_0.LINCR2.B.HTRQ = 1;     /* Request header transmission */

  while (!LINFlexD_0.LINSR.B.DRF); /* Wait for data receive complete flag */
                                   /* Code waits here if no slave response */
  m_LIN_Packet->BufferData.B.Data0 = LINFlexD_0.BDRL.B.DATA0;
  m_LIN_Packet->BufferData.B.Data1 = LINFlexD_0.BDRL.B.DATA1;
  m_LIN_Packet->BufferData.B.Data2 = LINFlexD_0.BDRL.B.DATA2;
  m_LIN_Packet->BufferData.B.Data3 = LINFlexD_0.BDRL.B.DATA3;
  m_LIN_Packet->BufferData.B.Data4 = LINFlexD_0.BDRM.B.DATA4;
  m_LIN_Packet->BufferData.B.Data5 = LINFlexD_0.BDRM.B.DATA5;
  m_LIN_Packet->BufferData.B.Data6 = LINFlexD_0.BDRM.B.DATA6;
  m_LIN_Packet->BufferData.B.Data7 = LINFlexD_0.BDRM.B.DATA7;

  LINFlexD_0.LINSR.R = 0x00000204;   /* Clear DRF flag */
}

void transmitLINframe_0 (void) {   /* Transmit one frame 'hello    ' to ID 0x35*/
  LINFlexD_0.BDRM.R = 0x2020206F; /* Load most significant bytes '   o' */
  LINFlexD_0.BDRL.R = 0x6C6C6548; /* Load least significant bytes 'lleh' */
  LINFlexD_0.BIDR.R = 0x00001E35; /* Init header: ID=0x35, 8 B, Tx, enh cksum*/
  LINFlexD_0.LINCR2.B.HTRQ = 1;   /* Request header transmission */

  while (!LINFlexD_0.LINSR.B.DTF); /* Wait for data transfer complete flag */
  LINFlexD_0.LINSR.R = 0x00000002;   /* Clear DTF flag */
}

void receiveLINframe_0 (void) {      /* Request data from ID 0x15 */
  uint8_t i;

  LINFlexD_0.BIDR.R = 0x00001C15; /* Init header: ID=0x15, 8 B, Rx, enh cksum */
  LINFlexD_0.LINCR2.B.HTRQ = 1;   /* Request header transmission */

  while (!LINFlexD_0.LINSR.B.DRF); /* Wait for data receive complete flag */
                              /* Code waits here if no slave response */
  for (i=0; i<4;i++){         /* If received less than or equal 4 data bytes */
	RxBuffer[i]= (LINFlexD_0.BDRL.R>>(i*8)); /* Fill buffer in reverse order */
  }
  for (i=4; i<8;i++){         /* If received more than 4 data bytes: */
	RxBuffer[i]= (LINFlexD_0.BDRM.R>>((i-4)*8)); /* Fill rest in reverse order */
	if(RxBuffer[i]){}
  }
  LINFlexD_0.LINSR.R = 0x00000204;   /* Clear DRF flag */
}

void initLINFlexD_1 (void) {     /* Master at 10.417K baud with 80MHz LIN_CLK */

  LINFlexD_1.LINCR1.B.INIT = 1;    /* Put LINFlex hardware in init mode       */
  LINFlexD_1.LINCR1.R= 0x00000311; /* Configure module as LIN master & header */
                                   /* Master Break Leght -> 13 Bit            */
                                   /* Master Mode Enable                      */
                                   /* Initialization Mode Request             */
  LINFlexD_1.LINIBRR.B.IBR= 480; /* Mantissa baud rate divider component */
  /* Baud rate divider = 80 MHz LIN_CLK input / (16*10417K bps) ~=480 */
  LINFlexD_1.LINFBRR.B.FBR = 0; /* Fraction baud rate divider comonent */
  LINFlexD_1.LINCR1.R= 0x00000310; /* Change module mode from init to normal */

  /* Configure LINFlexD_1 TxD Pin. */
  SIUL2.MSCR[PD9].B.SSS = 0b0010; //Pad PD9: Set to LINFlex_1 TxD.
  SIUL2.MSCR[PD9].B.OBE = 1; //Enable output buffer
  SIUL2.MSCR[PD9].B.SRC = 3; //Full drive-strength without slew rate control

  /* Configure LINFlexD_1 RxD Pin. */
  SIUL2.MSCR[PD12].B.IBE = 1; //Pad PD12: Enable input buffer
  SIUL2.IMCR[166].B.SSS = 0b0010; //Connect LINFlexD_1 signal to PD12
}
void transmitLINframe_1 (void) {   /* Transmit one frame 'hello    ' to ID 0x35*/
  LINFlexD_1.BDRM.R = 0x2020206F; /* Load most significant bytes '   o' */
  LINFlexD_1.BDRL.R = 0x6C6C6548; /* Load least significant bytes 'lleh' */
  LINFlexD_1.BIDR.R = 0x00001E35; /* Init header: ID=0x35, 8 B, Tx, enh cksum*/
  LINFlexD_1.LINCR2.B.HTRQ = 1;   /* Request header transmission */

  while (!LINFlexD_1.LINSR.B.DTF); /* Wait for data transfer complete flag */
  LINFlexD_1.LINSR.R = 0x00000002;   /* Clear DTF flag */
}
void receiveLINframe_1 (void) {      /* Request data from ID 0x15 */
	uint8_t RxBuffer[8] = {0};
	uint8_t i;

  LINFlexD_1.BIDR.R = 0x00001C15; /* Init header: ID=0x15, 8 B, Rx, enh cksum */
  LINFlexD_1.LINCR2.B.HTRQ = 1;   /* Request header transmission */
  while (!LINFlexD_1.LINSR.B.DRF); /* Wait for data receive complete flag */
                              /* Code waits here if no slave response */
  for (i=0; i<4;i++){         /* If received less than or equal 4 data bytes */
	RxBuffer[i]= (LINFlexD_1.BDRL.R>>(i*8)); /* Fill buffer in reverse order */
  }
  for (i=4; i<8;i++){         /* If received more than 4 data bytes: */
	RxBuffer[i]= (LINFlexD_1.BDRM.R>>((i-4)*8)); /* Fill rest in reverse order */
	if(RxBuffer[i]){}
  }
  LINFlexD_1.LINSR.R = 0x00000004;   /* Clear DRF flag */
}

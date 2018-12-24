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

LIN_RAM m_LIN_TX_RAM;
LIN_RAM m_LIN_RX_RAM;
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

/*******************************************************************************
*
* @brief    initLINFlexD_0_DMA - Init the LinFlexD Module with DMA
* @param    MegaHertz: the frequency of the HALFSYS_CLK
* @param    BaudRate : the baudrate of the Lin
* @return   none
*
*******************************************************************************/
void InitLINFlexD0_DMA ( uint16_t MegaHertz, uint16_t BaudRate )
{
	uint16_t Fraction,Integer;

	m_LIN_TX_RAM.BIDR.B.ID  = 0x0;
	m_LIN_TX_RAM.BIDR.B.DFL = 0;
	m_LIN_TX_RAM.BIDR.B.CCS = 0;
	m_LIN_TX_RAM.BIDR.B.DIR = 1;
	m_LIN_TX_RAM.LINCR2.B.DDRQ = 0;
	m_LIN_TX_RAM.LINCR2.B.DTRQ = 0;
	m_LIN_TX_RAM.LINCR2.B.HTRQ = 0;   /* Request header transmission */
	m_LIN_TX_RAM.BDRL.R = 0;
	m_LIN_TX_RAM.BDRM.R = 0;

  LINFlexD_0.LINCR1.B.SLEEP = 0; /* Exit Sleep Mode */
  LINFlexD_0.LINCR1.B.INIT = 1;  /* Put LINFlex hardware in init mode */
  /* wait for the INIT mode */
  while (0x1000 != (LINFlexD_0.LINSR.R & 0xF000)) {}

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

  LINFlexD_0.DMATXE.B.DTE = 0xFFFF;  //enable DMA
  LINFlexD_0.DMARXE.B.DRE = 0xFFFF;  //enable DMA

  LINFlexD_0.LINIER.B.DRIE = 1; // enable the data reception complete interrupt

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

  INTC_0.PSR[55].R  = 0x800C; //set priority and core for DMA Channel1 interrupt
  INTC_0.PSR[376].R = 0x800C; //set priority and core for LIN0 RX Channel1 interrupt
}

/*******************************************************************************
Function Name : FlexLin0_DMA_TX_M2S_Init
Engineer      : Guohua Zhu
Date          : Dec-19-2018
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 initialization function initialize TCD for simple transfer
                from variable TransData to LINFlexD_0.BDRL.DATA0. One major loop is used.
                Number of minor loop is determined by length of sanded data. Every minor loop transfers 1 byte.
                DMA is triggered by UART.
Issues        : NONE
*******************************************************************************/
void FlexLin0_DMA_TX_M2S_Init(void)
{
	// Clear the ENBL and TRIG bits of the DMA channel
    DMAMUX_0.CHCFG[0].R = 0;

    // DMA_0 config
    DMA_0.ERQ.B.ERQ0 = 0;    //The DMA request signal for channel 0 is disabled.

    //TCD config for channel[0]
    // TCD[0] Word 0 config
    DMA_0.TCD[0].SADDR.R = (uint32_t)&m_LIN_TX_RAM;	//Source Address
    DMA_0.TCD[0].ATTR.B.SSIZE = 2 ; // 32 bit
    DMA_0.TCD[0].ATTR.B.SMOD  = 0 ; // Source Address Module
    // TCD[0] SOFF minor transfer iteration offset = 0x1
    DMA_0.TCD[0].SOFF.R = 4;
    // TCD[0] Word 3 config SLAST - TCD Last Source Address Adjustment
    DMA_0.TCD[0].SLAST.R = -16;

    // TCD[0] Word 4 config DADDR - TCD Destination Address
    DMA_0.TCD[0].DADDR.R = (vuint32_t)&LINFlexD_0.LINCR2.R;
    DMA_0.TCD[0].ATTR.B.DSIZE = 2 ; // 32 bit
    DMA_0.TCD[0].ATTR.B.DMOD  = 0 ; // Destinaion Address Module
    // TCD[0] DOFF minor transfer iteration offset = 0x0
    DMA_0.TCD[0].DOFF.R = 0x4;
    // TCD[0] Word 6 config DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address
    DMA_0.TCD[0].DLASTSGA.R = -16;	 // Destination last address adjustment

    // TCD[0] Word 2 config NBYTES - Minor Byte Transfer Count
    // Number of bytes to be transferred in each service request of the channel
    DMA_0.TCD[0].NBYTES.MLNO.R = 16;
    // TCD[0] Word 5 config CITER - TCD Current Minor Loop Link, Major Loop Count
    // ELINK | CITER
    DMA_0.TCD[0].CITER.ELINKNO.R = 1;  //Destination Address
    // TCD[0] Word 7 config BITER - TCD Beginning Minor Loop Link, Major Loop Count
    // ELINK | BITER
    DMA_0.TCD[0].BITER.ELINKNO.R = 1;	 // Destination last address adjustment

    DMA_0.TCD[0].CSR.R = 0x0008;    //Stop after major loop finished

    DMA_0.ERQ.B.ERQ0 = 1;            //The DMA request signal for channel 0 is enabled.

    DMAMUX_0.CHCFG[0].B.TRIG = 0;
    DMAMUX_0.CHCFG[0].B.SOURCE = 0x14;
    DMAMUX_0.CHCFG[0].B.ENBL = 1;
}

/*******************************************************************************
Function Name : FlexLin0_DMA_TX_S2M_Init
Engineer      : Guohua Zhu
Date          : Dec-19-2018
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 initialization function initialize TCD for simple transfer
                from variable TransData to LINFlexD_0.BDRL.DATA0. One major loop is used.
                Number of minor loop is determined by length of sanded data. Every minor loop transfers 1 byte.
                DMA is triggered by UART.
Issues        : NONE
*******************************************************************************/
void FlexLin0_DMA_TX_S2M_Init(void)
{
	// Clear the ENBL and TRIG bits of the DMA channel
    DMAMUX_0.CHCFG[1].R = 0;

    // DMA_0 config
    DMA_0.ERQ.B.ERQ1 = 0;    //The DMA request signal for channel 0 is disabled.

    //TCD config for channel[0]
    // TCD[0] Word 0 config
    DMA_0.TCD[1].SADDR.R = (uint32_t)&m_LIN_RX_RAM.LINCR2.R;	//Source Address
    DMA_0.TCD[1].ATTR.B.SSIZE = 2 ; // 32 bit
    DMA_0.TCD[1].ATTR.B.SMOD  = 0 ; // Source Address Module
    // TCD[0] SOFF minor transfer iteration offset = 0x1
    DMA_0.TCD[1].SOFF.R = 4;
    // TCD[0] Word 3 config SLAST - TCD Last Source Address Adjustment
    DMA_0.TCD[1].SLAST.R = -8;

    // TCD[0] Word 4 config DADDR - TCD Destination Address
    DMA_0.TCD[1].DADDR.R = (vuint32_t)&LINFlexD_0.LINCR2.R;
    DMA_0.TCD[1].ATTR.B.DSIZE = 2 ; // 32 bit
    DMA_0.TCD[1].ATTR.B.DMOD  = 0 ; // Destinaion Address Module
    // TCD[0] DOFF minor transfer iteration offset = 0x0
    DMA_0.TCD[1].DOFF.R = 4;
    // TCD[0] Word 6 config DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address
    DMA_0.TCD[1].DLASTSGA.R = -8;	 // Destination last address adjustment

    // TCD[0] Word 2 config NBYTES - Minor Byte Transfer Count
    // Number of bytes to be transferred in each service request of the channel
    DMA_0.TCD[1].NBYTES.MLNO.R = 8;
    // TCD[0] Word 5 config CITER - TCD Current Minor Loop Link, Major Loop Count
    // ELINK | CITER
    DMA_0.TCD[1].CITER.ELINKNO.R = 1;  //Destination Address
    // TCD[0] Word 7 config BITER - TCD Beginning Minor Loop Link, Major Loop Count
    // ELINK | BITER
    DMA_0.TCD[1].BITER.ELINKNO.R = 1;	 // Destination last address adjustment

    DMA_0.TCD[1].CSR.R = 0x0008;    //Stop after major loop finished

    DMA_0.ERQ.B.ERQ1 = 1;            //The DMA request signal for channel 0 is enabled.

    DMAMUX_0.CHCFG[1].B.TRIG = 0;
    DMAMUX_0.CHCFG[1].B.SOURCE = 0x14;
    DMAMUX_0.CHCFG[1].B.ENBL = 1;
}

/*******************************************************************************
Function Name : FlexLin0_DMA_RX_S2M_Init
Engineer      : Guohua Zhu
Date          : Dec-22-2018
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 initialization function initialize TCD for simple transfer
                from variable TransData to LINFlexD_0.BIDR. One major loop is used.
                Number of minor loop is determined by length of sanded data.
                Every minor loop transfers 1 byte.DMA is triggered by Software.
Issues        : NONE
*******************************************************************************/
void FlexLin0_DMA_RX_S2M_Init(void)
{
	// Clear the ENBL and TRIG bits of the DMA channel
    DMAMUX_0.CHCFG[2].R = 0;

    // DMA_0 config
    DMA_0.ERQ.B.ERQ2 = 0;    //The DMA request signal for channel 0 is disabled.

    //TCD config for channel[0]
    // TCD[0] Word 0 config
    DMA_0.TCD[2].SADDR.R = (vuint32_t)&LINFlexD_0.BIDR.R;//	//Source Address
    DMA_0.TCD[2].ATTR.B.SSIZE = 2 ; // 32 bit
    DMA_0.TCD[2].ATTR.B.SMOD  = 0 ; // Source Address Module
    // TCD[0] SOFF minor transfer iteration offset = 0x1
    DMA_0.TCD[2].SOFF.R = 4;
    // TCD[0] Word 3 config SLAST - TCD Last Source Address Adjustment
    DMA_0.TCD[2].SLAST.R = -12;

    // TCD[0] Word 4 config DADDR - TCD Destination Address
    DMA_0.TCD[2].DADDR.R = (uint32_t)&m_LIN_RX_RAM.BIDR.R;
    DMA_0.TCD[2].ATTR.B.DSIZE = 2 ; // 32 bit
    DMA_0.TCD[2].ATTR.B.DMOD  = 0 ; // Destinaion Address Module
    // TCD[0] DOFF minor transfer iteration offset = 0x0
    DMA_0.TCD[2].DOFF.R = 0x4;
    // TCD[0] Word 6 config DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address
    DMA_0.TCD[2].DLASTSGA.R = -12;	 // Destination last address adjustment

    // TCD[0] Word 2 config NBYTES - Minor Byte Transfer Count
    // Number of bytes to be transferred in each service request of the channel
    DMA_0.TCD[2].NBYTES.MLNO.R = 12;
    // TCD[0] Word 5 config CITER - TCD Current Minor Loop Link, Major Loop Count
    // ELINK | CITER
    DMA_0.TCD[2].CITER.ELINKNO.R = 1;  //Destination Address
    // TCD[0] Word 7 config BITER - TCD Beginning Minor Loop Link, Major Loop Count
    // ELINK | BITER
    DMA_0.TCD[2].BITER.ELINKNO.R = 1;	 // Destination last address adjustment

    DMA_0.TCD[2].CSR.R = 0x0008;    //Stop after major loop finished
    DMA_0.TCD[2].CSR.B.INTMAJOR = 1;

    DMA_0.ERQ.B.ERQ2 = 1;            //The DMA request signal for channel 0 is enabled.

    DMAMUX_0.CHCFG[2].B.TRIG = 0;
    DMAMUX_0.CHCFG[2].B.SOURCE = 0x15;
    DMAMUX_0.CHCFG[2].B.ENBL = 1;
}

void LIN0_TransmitFrame (LIN_RAM m_LIN_RAM)
{
//  LINFlexD_0 = m_LIN_RAM;
//  LINFlexD_0.BIDR.B.CCS = 0;
//  LINFlexD_0.BIDR.B.DIR = 1;
//  LINFlexD_0.LINCR2.B.HTRQ = 1;   /* Request header transmission */
//  while (!LINFlexD_0.LINSR.B.DTF); /* Wait for data transfer complete flag */
//  LINFlexD_0.LINSR.R = 0x00000002;   /* Clear DTF flag */
}

void LIN0_TransmitFrame_DMA (LIN_RAM m_LIN_RAM)
{
	m_LIN_TX_RAM = m_LIN_RAM;
	m_LIN_TX_RAM.BIDR.B.CCS = 0;
	m_LIN_TX_RAM.BIDR.B.DIR = 1;
	m_LIN_TX_RAM.LINCR2.B.HTRQ = 1;   /* Request header transmission */
	DMA_0.TCD[0].CSR.B.START = 1;
}

void LIN0_ReceiveFrame(LIN_RAM *m_LIN_RAM)
{
//  LINFlexD_0 = *m_LIN_RAM;
//  LINFlexD_0.BIDR.B.CCS = 0;
//  LINFlexD_0.BIDR.B.DIR = 0;// receive the data
//  LINFlexD_0.LINCR2.B.HTRQ = 1;     /* Request header transmission */
//
//  while (!LINFlexD_0.LINSR.B.DRF); /* Wait for data receive complete flag */
//                                   /* Code waits here if no slave response */
//  *m_LIN_RAM = LINFlexD_0;

  LINFlexD_0.LINSR.R = 0x00000204;   /* Clear DRF flag */
}

void LIN0_ReceiveFrame_DMA(LIN_RAM *m_LIN_RAM)
{
	m_LIN_RX_RAM = *m_LIN_RAM;
	m_LIN_RX_RAM.BIDR.B.CCS = 0;
	m_LIN_RX_RAM.BIDR.B.DIR = 0;
	m_LIN_RX_RAM.LINCR2.B.DDRQ = 0;
	m_LIN_RX_RAM.LINCR2.B.DTRQ = 0;
	m_LIN_RX_RAM.LINCR2.B.HTRQ = 1;   /* Request header transmission */
	DMA_0.TCD[1].CSR.B.START = 1;

    // DMA_0 config
    DMA_0.ERQ.B.ERQ2 = 0;    //The DMA request signal for channel 2 is disabled.
    DMA_0.TCD[2].DADDR.R = (uint32_t)&(m_LIN_RAM->BIDR.R);
    DMA_0.ERQ.B.ERQ2 = 1;            //The DMA request signal for channel 2 is enabled.
}

//void LIN0_ReceiveFrame(LIN_Packet *m_LIN_Packet)
//{
//  LINFlexD_0.BIDR.B.ID  = m_LIN_Packet->id;
//  LINFlexD_0.BIDR.B.DFL = m_LIN_Packet->length - 1;
//  LINFlexD_0.BIDR.B.CCS = 0;
//  LINFlexD_0.BIDR.B.DIR = 0;// receive the data
//  LINFlexD_0.LINCR2.B.HTRQ = 1;     /* Request header transmission */
//
//  while (!LINFlexD_0.LINSR.B.DRF); /* Wait for data receive complete flag */
//                                   /* Code waits here if no slave response */
//  m_LIN_Packet->BufferData.B.Data0 = LINFlexD_0.BDRL.B.DATA0;
//  m_LIN_Packet->BufferData.B.Data1 = LINFlexD_0.BDRL.B.DATA1;
//  m_LIN_Packet->BufferData.B.Data2 = LINFlexD_0.BDRL.B.DATA2;
//  m_LIN_Packet->BufferData.B.Data3 = LINFlexD_0.BDRL.B.DATA3;
//  m_LIN_Packet->BufferData.B.Data4 = LINFlexD_0.BDRM.B.DATA4;
//  m_LIN_Packet->BufferData.B.Data5 = LINFlexD_0.BDRM.B.DATA5;
//  m_LIN_Packet->BufferData.B.Data6 = LINFlexD_0.BDRM.B.DATA6;
//  m_LIN_Packet->BufferData.B.Data7 = LINFlexD_0.BDRM.B.DATA7;
//
//  LINFlexD_0.LINSR.R = 0x00000204;   /* Clear DRF flag */
//}
//
//void LIN0_ReceiveFrame_DMA(LIN_Packet *m_LIN_Packet)
//{
//	m_LIN_RX_RAM.BIDR.B.ID  = m_LIN_Packet->id;
//	m_LIN_RX_RAM.BIDR.B.DFL = m_LIN_Packet->length - 1;
//	m_LIN_RX_RAM.BIDR.B.CCS = 0;
//	m_LIN_RX_RAM.BIDR.B.DIR = 0;
//	m_LIN_RX_RAM.LINCR2.B.DDRQ = 0;
//	m_LIN_RX_RAM.LINCR2.B.DTRQ = 0;
//	m_LIN_RX_RAM.LINCR2.B.HTRQ = 1;   /* Request header transmission */
//	DMA_0.TCD[1].CSR.B.START = 1;
//}

void FlexLin0_Uart_Isr(void)
{
	DMA_0.TCD[2].CSR.B.START = 1;
	LINFlexD_0.LINSR.B.RMB = 1;/* Clear RMB flag */
}

void eDMA_Channel2_Isr(void)
{
	DMA_0.INT.B.INT2 = 1;
}





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
/* 1.1	 Guohua Zhu     December 24 2018    DMA Version                      */
/*****************************************************************************/
#include "linflexd.h"

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

	INTC_0.PSR[55].R  = 0x8005; //set priority and core for DMA Channel1 interrupt
	INTC_0.PSR[376].R = 0x8006; //set priority and core for LIN0 RX Channel1 interrupt
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

/*******************************************************************************
Function Name : LIN0_TransmitFrame_DMA
Engineer      : Guohua Zhu
Date          : Dec-24-2018
Parameters    : m_LIN_RAM: the mapping data of send ram
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 move the m_LIN_TX_RAM data to the peripheral LINFlex0.
Issues        : NONE
*******************************************************************************/
void LIN0_TransmitFrame_DMA (LIN_RAM m_LIN_RAM)
{
	m_LIN_TX_RAM = m_LIN_RAM;
	m_LIN_TX_RAM.BIDR.B.CCS = 0;
	m_LIN_TX_RAM.BIDR.B.DIR = 1;
	m_LIN_TX_RAM.LINCR2.B.HTRQ = 1;   /* Request header transmission */
	DMA_0.TCD[0].CSR.B.START = 1;
}

/*******************************************************************************
Function Name : LIN0_ReceiveFrame_DMA
Engineer      : Guohua Zhu
Date          : Dec-24-2018
Parameters    : m_LIN_RAM: the mapping data point of receive ram
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 move the peripheral LINFlex0 to the m_LIN_RAM point Sapce.
Issues        : NONE
*******************************************************************************/
void LIN0_ReceiveFrame_DMA(LIN_RAM *m_LIN_RAM)
{
    m_LIN_RX_RAM = *m_LIN_RAM;
    m_LIN_RX_RAM.BIDR.B.CCS = 0;
    m_LIN_RX_RAM.BIDR.B.DIR = 0;
    m_LIN_RX_RAM.LINCR2.B.DDRQ = 0;
    m_LIN_RX_RAM.LINCR2.B.DTRQ = 0;
    m_LIN_RX_RAM.LINCR2.B.HTRQ = 1;   /* Request header transmission */

    // DMA_0 config
    DMA_0.ERQ.B.ERQ2 = 0;    //The DMA request signal for channel 2 is disabled.
    DMA_0.TCD[2].DADDR.R = (uint32_t)&(m_LIN_RAM->BIDR.R);
    DMA_0.ERQ.B.ERQ2 = 1;            //The DMA request signal for channel 2 is enabled.

    DMA_0.TCD[1].CSR.B.START = 1;
}

/**************** LIN1 relative init and receive send function ****************/
/*******************************************************************************
*
* @brief    initLINFlexD_1_DMA - Init the LinFlexD1 Module with DMA
* @param    MegaHertz: the frequency of the HALFSYS_CLK
* @param    BaudRate : the baudrate of the Lin
* @return   none
*
*******************************************************************************/
void InitLINFlexD1_DMA ( uint16_t MegaHertz, uint16_t BaudRate )
{
	uint16_t Fraction,Integer;

	LINFlexD_1.LINCR1.B.SLEEP = 0; /* Exit Sleep Mode */
	LINFlexD_1.LINCR1.B.INIT = 1;  /* Put LINFlex hardware in init mode */
	/* wait for the INIT mode */
	while (0x1000 != (LINFlexD_1.LINSR.R & 0xF000)) {}

	BaudRate  = (MegaHertz * 1000000) / BaudRate;
	Integer   = BaudRate / 16;
	Fraction  = BaudRate - (Integer * 16);

	LINFlexD_1.LINIBRR.B.IBR= Integer;  /* Mantissa baud rate divider component */
		 /* Baud rate divider = 100 MHz LIN_CLK input / (16*19200 bps) ~= 326 */
	LINFlexD_1.LINFBRR.B.FBR = Fraction; /* Fraction baud rate divider comonent */

	LINFlexD_1.LINCR2.B.IOBE = 1;

	LINFlexD_1.LINTCSR.B.MODE = 1;
	LINFlexD_1.LINOCR.R = 0x0000ffff;
	LINFlexD_1.LINTCSR.B.MODE = 0;
	LINFlexD_1.LINTCSR.B.IOT = 1;

	LINFlexD_1.DMATXE.B.DTE = 0xFFFF;  //enable DMA
	LINFlexD_1.DMARXE.B.DRE = 0xFFFF;  //enable DMA

	LINFlexD_1.LINIER.B.DRIE = 1; // enable the data reception complete interrupt

	LINFlexD_1.LINCR1.R= 0x00000310;  /* Change module mode from init to normal */

	/* Configure LINFlexD_1 TxD Pin. */
	SIUL2.MSCR[PD9].B.SSS = 0b0010; // Pad PD9: Set to LINFlex_1 TxD.
	SIUL2.MSCR[PD9].B.OBE = 1;      // Enable output buffer
	SIUL2.MSCR[PD9].B.ODE = 1;      // Pad PD9: Output Drain Enable */
	SIUL2.MSCR[PD9].B.PUS = 1;      // Pad PD9: Pull up selected */
	SIUL2.MSCR[PD9].B.PUE = 1;      // Pad PD9: Pull Enable */
	SIUL2.MSCR[PD9].B.SRC = 3;      // Full drive-strength without slew rate control

	/* Configure LINFlexD_1 RxD Pin. */
	SIUL2.MSCR[PD12].B.IBE = 1;     //Pad PD12: Enable input buffer
	SIUL2.IMCR[166].B.SSS = 0b0010; //Connect LINFlexD_1 signal to PD12

	INTC_0.PSR[71].R  = 0x8005; //set priority and core for DMA Channel18 interrupt
	INTC_0.PSR[380].R = 0x8006; //set priority and core for LIN1 RX interrupt
}

/*******************************************************************************
Function Name : FlexLin1_DMA_TX_M2S_Init
Engineer      : Guohua Zhu
Date          : Dec-24-2018
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 initialization function initialize TCD for simple transfer
                from variable TransData to LINFlexD_1.BDRL.DATA0. One major loop
                is used.Number of minor loop is determined by length of sanded
                data.Every minor loop transfers 1 byte.DMA is triggered by UART.
Issues        : NONE
*******************************************************************************/
void FlexLin1_DMA_TX_M2S_Init(void)
{
	// Clear the ENBL and TRIG bits of the DMA channel
    DMAMUX_1.CHCFG[0].R = 0;

    // DMA_0 config
    DMA_0.ERQ.B.ERQ16 = 0;    //The DMA request signal for channel 0 is disabled.

    //TCD config for channel[0]
    // TCD[0] Word 0 config
    DMA_0.TCD[16].SADDR.R = (uint32_t)&m_LIN_TX_RAM;	//Source Address
    DMA_0.TCD[16].ATTR.B.SSIZE = 2 ; // 32 bit
    DMA_0.TCD[16].ATTR.B.SMOD  = 0 ; // Source Address Module
    // TCD[0] SOFF minor transfer iteration offset = 0x1
    DMA_0.TCD[16].SOFF.R = 4;
    // TCD[0] Word 3 config SLAST - TCD Last Source Address Adjustment
    DMA_0.TCD[16].SLAST.R = -16;

    // TCD[0] Word 4 config DADDR - TCD Destination Address
    DMA_0.TCD[16].DADDR.R = (vuint32_t)&LINFlexD_1.LINCR2.R;
    DMA_0.TCD[16].ATTR.B.DSIZE = 2 ; // 32 bit
    DMA_0.TCD[16].ATTR.B.DMOD  = 0 ; // Destinaion Address Module
    // TCD[0] DOFF minor transfer iteration offset = 0x0
    DMA_0.TCD[16].DOFF.R = 0x4;
    // TCD[0] Word 6 config DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address
    DMA_0.TCD[16].DLASTSGA.R = -16;	 // Destination last address adjustment

    // TCD[0] Word 2 config NBYTES - Minor Byte Transfer Count
    // Number of bytes to be transferred in each service request of the channel
    DMA_0.TCD[16].NBYTES.MLNO.R = 16;
    // TCD[0] Word 5 config CITER - TCD Current Minor Loop Link, Major Loop Count
    // ELINK | CITER
    DMA_0.TCD[16].CITER.ELINKNO.R = 1;  //Destination Address
    // TCD[0] Word 7 config BITER - TCD Beginning Minor Loop Link, Major Loop Count
    // ELINK | BITER
    DMA_0.TCD[16].BITER.ELINKNO.R = 1;	 // Destination last address adjustment

    DMA_0.TCD[16].CSR.R = 0x0008;    //Stop after major loop finished

    DMA_0.ERQ.B.ERQ16 = 1;            //The DMA request signal for channel 0 is enabled.

    DMAMUX_1.CHCFG[0].B.TRIG = 0;
    DMAMUX_1.CHCFG[0].B.SOURCE = 0xE;
    DMAMUX_1.CHCFG[0].B.ENBL = 1;
}

/*******************************************************************************
Function Name : FlexLin1_DMA_TX_S2M_Init
Engineer      : Guohua Zhu
Date          : Dec-24-2018
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 initialization function initialize TCD for simple transfer
                from variable TransData to LINFlexD_1.BDRL.DATA0. One major loop
                is used. Number of minor loop is determined by length of sanded
                data.Every minor loop transfers 1 byte.DMA is triggered by UART.
Issues        : NONE
*******************************************************************************/
void FlexLin1_DMA_TX_S2M_Init(void)
{
	// Clear the ENBL and TRIG bits of the DMA channel
    DMAMUX_1.CHCFG[1].R = 0;

    // DMA_0 config
    DMA_0.ERQ.B.ERQ17 = 0;    //The DMA request signal for channel 0 is disabled.

    //TCD config for channel[0]
    // TCD[0] Word 0 config
    DMA_0.TCD[17].SADDR.R = (uint32_t)&m_LIN_RX_RAM.LINCR2.R;	//Source Address
    DMA_0.TCD[17].ATTR.B.SSIZE = 2 ; // 32 bit
    DMA_0.TCD[17].ATTR.B.SMOD  = 0 ; // Source Address Module
    // TCD[0] SOFF minor transfer iteration offset = 0x1
    DMA_0.TCD[17].SOFF.R = 4;
    // TCD[0] Word 3 config SLAST - TCD Last Source Address Adjustment
    DMA_0.TCD[17].SLAST.R = -8;

    // TCD[0] Word 4 config DADDR - TCD Destination Address
    DMA_0.TCD[17].DADDR.R = (vuint32_t)&LINFlexD_1.LINCR2.R;
    DMA_0.TCD[17].ATTR.B.DSIZE = 2 ; // 32 bit
    DMA_0.TCD[17].ATTR.B.DMOD  = 0 ; // Destinaion Address Module
    // TCD[0] DOFF minor transfer iteration offset = 0x0
    DMA_0.TCD[17].DOFF.R = 4;
    // TCD[0] Word 6 config DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address
    DMA_0.TCD[17].DLASTSGA.R = -8;	 // Destination last address adjustment

    // TCD[0] Word 2 config NBYTES - Minor Byte Transfer Count
    // Number of bytes to be transferred in each service request of the channel
    DMA_0.TCD[17].NBYTES.MLNO.R = 8;
    // TCD[0] Word 5 config CITER - TCD Current Minor Loop Link, Major Loop Count
    // ELINK | CITER
    DMA_0.TCD[17].CITER.ELINKNO.R = 1;  //Destination Address
    // TCD[0] Word 7 config BITER - TCD Beginning Minor Loop Link, Major Loop Count
    // ELINK | BITER
    DMA_0.TCD[17].BITER.ELINKNO.R = 1;	 // Destination last address adjustment

    DMA_0.TCD[17].CSR.R = 0x0008;    //Stop after major loop finished

    DMA_0.ERQ.B.ERQ17 = 1;            //The DMA request signal for channel 0 is enabled.

    DMAMUX_1.CHCFG[1].B.TRIG = 0;
    DMAMUX_1.CHCFG[1].B.SOURCE = 0xE;
    DMAMUX_1.CHCFG[1].B.ENBL = 1;
}

/*******************************************************************************
Function Name : FlexLin1_DMA_RX_S2M_Init
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
void FlexLin1_DMA_RX_S2M_Init(void)
{
	// Clear the ENBL and TRIG bits of the DMA channel
    DMAMUX_1.CHCFG[2].R = 0;

    // DMA_0 config
    DMA_0.ERQ.B.ERQ18 = 0;    //The DMA request signal for channel 0 is disabled.

    //TCD config for channel[0]
    // TCD[0] Word 0 config
    DMA_0.TCD[18].SADDR.R = (vuint32_t)&LINFlexD_1.BIDR.R;//	//Source Address
    DMA_0.TCD[18].ATTR.B.SSIZE = 2 ; // 32 bit
    DMA_0.TCD[18].ATTR.B.SMOD  = 0 ; // Source Address Module
    // TCD[0] SOFF minor transfer iteration offset = 0x1
    DMA_0.TCD[18].SOFF.R = 4;
    // TCD[0] Word 3 config SLAST - TCD Last Source Address Adjustment
    DMA_0.TCD[18].SLAST.R = -12;

    // TCD[0] Word 4 config DADDR - TCD Destination Address
    DMA_0.TCD[18].DADDR.R = (uint32_t)&m_LIN_RX_RAM.BIDR.R;
    DMA_0.TCD[18].ATTR.B.DSIZE = 2 ; // 32 bit
    DMA_0.TCD[18].ATTR.B.DMOD  = 0 ; // Destinaion Address Module
    // TCD[0] DOFF minor transfer iteration offset = 0x0
    DMA_0.TCD[18].DOFF.R = 4;
    // TCD[0] Word 6 config DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address
    DMA_0.TCD[18].DLASTSGA.R = -12;	 // Destination last address adjustment

    // TCD[0] Word 2 config NBYTES - Minor Byte Transfer Count
    // Number of bytes to be transferred in each service request of the channel
    DMA_0.TCD[18].NBYTES.MLNO.R = 12;
    // TCD[0] Word 5 config CITER - TCD Current Minor Loop Link, Major Loop Count
    // ELINK | CITER
    DMA_0.TCD[18].CITER.ELINKNO.R = 1;  //Destination Address
    // TCD[0] Word 7 config BITER - TCD Beginning Minor Loop Link, Major Loop Count
    // ELINK | BITER
    DMA_0.TCD[18].BITER.ELINKNO.R = 1;	 // Destination last address adjustment

    DMA_0.TCD[18].CSR.R = 0x0008;    //Stop after major loop finished
    DMA_0.TCD[18].CSR.B.INTMAJOR = 1;

    DMA_0.ERQ.B.ERQ18 = 1;            //The DMA request signal for channel 0 is enabled.

    DMAMUX_1.CHCFG[2].B.TRIG = 0;
    DMAMUX_1.CHCFG[2].B.SOURCE = 0xF;
    DMAMUX_1.CHCFG[2].B.ENBL = 1;
}

/*******************************************************************************
Function Name : LIN1_TransmitFrame_DMA
Engineer      : Guohua Zhu
Date          : Dec-24-2018
Parameters    : m_LIN_RAM: the mapping data of send ram
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 move the m_LIN_TX_RAM data to the peripheral LINFlex0.
Issues        : NONE
*******************************************************************************/
void LIN1_TransmitFrame_DMA (LIN_RAM m_LIN_RAM)
{
	m_LIN_TX_RAM = m_LIN_RAM;
	m_LIN_TX_RAM.BIDR.B.CCS = 0;
	m_LIN_TX_RAM.BIDR.B.DIR = 1;
	m_LIN_TX_RAM.LINCR2.B.HTRQ = 1;   /* Request header transmission */
	DMA_0.TCD[16].CSR.B.START = 1;
}

/*******************************************************************************
Function Name : LIN1_ReceiveFrame_DMA
Engineer      : Guohua Zhu
Date          : Dec-24-2018
Parameters    : m_LIN_RAM: the mapping data point of receive ram
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 move the peripheral LINFlex0 to the m_LIN_RAM point Sapce.
Issues        : NONE
*******************************************************************************/
void LIN1_ReceiveFrame_DMA(LIN_RAM *m_LIN_RAM)
{
    m_LIN_RX_RAM = *m_LIN_RAM;
    m_LIN_RX_RAM.BIDR.B.CCS = 0;
    m_LIN_RX_RAM.BIDR.B.DIR = 0;
    m_LIN_RX_RAM.LINCR2.B.DDRQ = 0;
    m_LIN_RX_RAM.LINCR2.B.DTRQ = 0;
    m_LIN_RX_RAM.LINCR2.B.HTRQ = 1;   /* Request header transmission */

    // DMA_0 config
    DMA_0.ERQ.B.ERQ18 = 0; //The DMA request signal for channel 18 is disabled.
    DMA_0.TCD[18].DADDR.R = (uint32_t)&(m_LIN_RAM->BIDR.R);
    DMA_0.ERQ.B.ERQ18 = 1; //The DMA request signal for channel 18 is enabled.

    DMA_0.TCD[17].CSR.B.START = 1;
}
/**************************   LIN Module Configure   **************************/
void LINFlexD_Configure (void)
{
	m_LIN_TX_RAM.BIDR.B.ID  = 0x0;
	m_LIN_TX_RAM.BIDR.B.DFL = 0;
	m_LIN_TX_RAM.BIDR.B.CCS = 0;
	m_LIN_TX_RAM.BIDR.B.DIR = 1;
	m_LIN_TX_RAM.LINCR2.B.DDRQ = 0;
	m_LIN_TX_RAM.LINCR2.B.DTRQ = 0;
	m_LIN_TX_RAM.LINCR2.B.HTRQ = 0;   /* Request header transmission */
	m_LIN_TX_RAM.BDRL.R = 0;
	m_LIN_TX_RAM.BDRM.R = 0;

	m_LIN_RX_RAM.BIDR.B.ID  = 0x0;
	m_LIN_RX_RAM.BIDR.B.DFL = 0;
	m_LIN_RX_RAM.BIDR.B.CCS = 0;
	m_LIN_RX_RAM.BIDR.B.DIR = 1;
	m_LIN_RX_RAM.LINCR2.B.DDRQ = 0;
	m_LIN_RX_RAM.LINCR2.B.DTRQ = 0;
	m_LIN_RX_RAM.LINCR2.B.HTRQ = 0;   /* Request header transmission */
	m_LIN_RX_RAM.BDRL.R = 0;
	m_LIN_RX_RAM.BDRM.R = 0;

    InitLINFlexD0_DMA(100,19200);
    InitLINFlexD1_DMA(100,19200);


    FlexLin0_DMA_TX_M2S_Init();
    FlexLin0_DMA_TX_S2M_Init();
    FlexLin0_DMA_RX_S2M_Init();


    FlexLin1_DMA_TX_M2S_Init();
    FlexLin1_DMA_TX_S2M_Init();
    FlexLin1_DMA_RX_S2M_Init();
}

/**************************      LIN and DMA isr     **************************/

/*******************************************************************************
Function Name : FlexLin0_Uart_Isr
Engineer      : Guohua Zhu
Date          : Dec-24-2018
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : When receive finish,generate the isr
Issues        : NONE
*******************************************************************************/
void FlexLin0_Uart_Isr(void)
{
	DMA_0.TCD[2].CSR.B.START = 1;
	LINFlexD_0.LINSR.B.RMB = 1;/* Clear RMB flag */
}

/*******************************************************************************
Function Name : FlexLin1_Uart_Isr
Engineer      : Guohua Zhu
Date          : Dec-24-2018
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : When receive finish,generate the isr
Issues        : NONE
*******************************************************************************/
void FlexLin1_Uart_Isr(void)
{
	DMA_0.TCD[18].CSR.B.START = 1;
	LINFlexD_1.LINSR.B.RMB = 1;/* Clear RMB flag */
}



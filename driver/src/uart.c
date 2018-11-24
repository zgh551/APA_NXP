/*
 * uart.c
 *
 *  Created on: 2018Äê11ÔÂ20ÈÕ
 *      Author: zhuguohua
 */
#include "uart.h"
static vuint8_t  TransData[] = "Hello World"; /* Transmit string & CR*/
vuint8_t  Receive_buffer[100];
void FlexLin1_Uart_Init( unsigned int MegaHertz, unsigned int BaudRate )
{
	unsigned int Fraction;
	unsigned int Integer;
	/* enter INIT mode */
	LINFlexD_1.LINCR1.B.INIT = 1;     /* Enter Initialization Mode */
	LINFlexD_1.LINCR1.B.SLEEP = 0;    /* Exit Sleep Mode */
//	LINFlexD_1.LINCR1.R = 0x0081; /* SLEEP=0, INIT=1 */
	/* wait for the INIT mode */
	while (0x1000 != (LINFlexD_1.LINSR.R & 0xF000)) {}

	LINFlexD_1.UARTCR.B.UART = 1;     /* UART Enable- Req'd before UART config.*/
	 /*Tx/Rx enabled, TXFIFO/RXFIFO enabled, 8-bit data without parity*/
	LINFlexD_1.UARTCR.B.WL0 = 1;
	LINFlexD_1.UARTCR.B.RxEn = 1;
	LINFlexD_1.UARTCR.B.TxEn = 1;
//	LINFlexD_1.UARTCR.B.RFBM = 1;
//	LINFlexD_1.UARTCR.B.TFBM = 1;
//	LINFlexD_1.UARTCR.R = 0x0333;

	/*Tx/Rx interrupts enable*/
//	LINFlexD_1.LINIER.B.DTIE = 1;
	LINFlexD_1.LINIER.B.DRIE = 1;

	LINFlexD_1.UARTSR.B.SZF = 1;      /* CHANGE THIS LINE   Clear the Zero status bit */
	LINFlexD_1.UARTSR.B.DRFRFE = 1;   /* CHANGE THIS LINE  Clear DRFRFE flag - W1C */

//	LINFlexD_1.DMATXE.R = 0x0000ffff;  //enable DMA
//	LINFlexD_1.DMARXE.R = 0x0000ffff;  //enable DMA

	BaudRate  = (MegaHertz * 1000000) / BaudRate;
	Integer   = BaudRate / 16;
	Fraction  = BaudRate - (Integer * 16);

	LINFlexD_1.LINIBRR.R = Integer;
	LINFlexD_1.LINFBRR.R = Fraction;

	LINFlexD_1.LINCR1.B.INIT = 0;     /* Exit Initialization Mode */

	/* Configure LINFlexD_1 TxD Pin. */
	SIUL2.MSCR[PF14].B.SSS = 0b0001; //Pad PF14: Set to LINFlex_1 TxD. Must choose this option because F14 leads to LIN PHY of motherboard
	SIUL2.MSCR[PF14].B.OBE = 1; //Enable output buffer
	SIUL2.MSCR[PF14].B.SRC = 3; //Full drive-strength without slew rate control

	/* Configure LINFlexD_1 RxD Pin. */
	SIUL2.MSCR[PF15].B.IBE = 1; //Pad PF15: Enable input buffer
	SIUL2.IMCR[166].B.SSS = 0b0011; //Connect LINFlexD_1 signal to PF15

	INTC_0.PSR[380].R = 0x8003; //set priority and core for RX UART interrupt
}

/*******************************************************************************
Function Name : DMA_TX_Init
Engineer      : Martin Kovar
Date          : Dec-16-2015
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 initialization function initialize TCD for simple transfer
                from variable TransData to LINFlexD_0.BDRL.DATA0. One major loop is used.
                Number of minor loop is determined by length of sanded data. Every minor loop transfers 1 byte.
                DMA is triggered by UART.
Issues        : NONE
*******************************************************************************/
void FlexLin1_DMA_TX_Init(void)
{
	vuint32_t TransDataLength = sizeof(TransData)-1;     //Length of the transmitted data

	// Clear the ENBL and TRIG bits of the DMA channel
    DMAMUX_1.CHCFG[0].B.ENBL = 0;
    DMAMUX_1.CHCFG[0].B.TRIG = 0;
    DMAMUX_1.CHCFG[0].B.SOURCE = 0x0E;
    DMAMUX_1.CHCFG[0].B.ENBL = 1;

    // DMA_0 config
    DMA_0.ERQ.B.ERQ16 = 0;    //The DMA request signal for channel 0 is disabled.

    //TCD config for channel[0]

    // TCD[0] Word 0 config
    DMA_0.TCD[16].SADDR.R = (uint32_t)TransData;  	//Source Address

    // TCD[0] Word 1 config SMOD(0) | SSIZE(8-bit) | DMOD(0) | DSIZE(8-bit)
    DMA_0.TCD[16].ATTR.R = 0x0|0x000|0x0|0x0;	//Source transfer size 8-bit, no Address Modulo used

    // TCD[0] Word 2 config NBYTES - Minor Byte Transfer Count
    // Number of bytes to be transferred in each service request of the channel
    DMA_0.TCD[16].NBYTES.MLNO.R = 0x00000001;

    // TCD[0] SOFF minor transfer iteration offset = 0x1
    DMA_0.TCD[16].SOFF.R = 0x1;
    // TCD[0] DOFF minor transfer iteration offset = 0x0
    DMA_0.TCD[16].DOFF.R = 0x0;

    // TCD[0] Word 3 config SLAST - TCD Last Source Address Adjustment
    DMA_0.TCD[16].SLAST.R = 0;

    // TCD[0] Word 4 config DADDR - TCD Destination Address
    DMA_0.TCD[16].DADDR.R = ((vuint32_t)&LINFlexD_1.BDRL.R) + 3;	 //Destination Address (LINFlexD_0.BDRL.DATA0)

    // TCD[0] Word 5 config CITER - TCD Current Minor Loop Link, Major Loop Count
    // ELINK | CITER
    DMA_0.TCD[16].CITER.ELINKNO.R = 0x0 | TransDataLength;  //Destination Address

    // TCD[0] Word 6 config DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address
    DMA_0.TCD[16].DLASTSGA.R = 0x0;	 // Destination last address adjustment

    // TCD[0] Word 7 config BITER - TCD Beginning Minor Loop Link, Major Loop Count
    // ELINK | BITER
    DMA_0.TCD[16].BITER.ELINKNO.R = 0x0 | TransDataLength;	 // Destination last address adjustment

    DMA_0.TCD[16].CSR.R = 0x0008;    //Stop after major loop finished

    DMA_0.ERQ.B.ERQ16 = 1;            //The DMA request signal for channel 0 is enabled.
}

/*******************************************************************************
Function Name : DMA_RX_Init
Engineer      : Martin Kovar
Date          : Dec-22-2015
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : DMA_0 initialization function initialize TCD for simple receive
                from LINFlexD_0.BDRM.DATA4 to SRAM address 0x40004000.
                DMA channel uses 1 minor loop in every major loop.Every minor
                loop transfers 1 byte. After the RXFIFO receives data, DMA
                channel is triggered and data are written to the RAM.
Issues        : NONE
*******************************************************************************/
void FlexLin1_DMA_RX_Init(void)
{

	// Clear the ENBL and TRIG bits of the DMA channel
    DMAMUX_1.CHCFG[2].B.ENBL = 1;
    DMAMUX_1.CHCFG[2].B.TRIG = 0;
    DMAMUX_1.CHCFG[2].B.SOURCE = 0x0F;

    // DMA_0 config
    DMA_0.ERQ.B.ERQ18 = 0;    //The DMA request signal for channel 0 is disabled.

    //TCD config for channel[1]

    // TCD[1] Word 0 config
    DMA_0.TCD[18].SADDR.R = ((vuint32_t)&LINFlexD_1.BDRM.R) + 3;  	//Source Address

    // TCD[1] Word 1 config SMOD(0) | SSIZE(8-bit) | DMOD(0) | DSIZE(8-bit)
    DMA_0.TCD[18].ATTR.R = 0x0|0x000|0x0|0x0;	//Source transfer size 8-bit, no Address Modulo used



    // Number of bytes to be transferred in each service request of the channel
    DMA_0.TCD[18].NBYTES.MLNO.R = 0x00000001;

    // TCD[1] SOFF minor transfer iteration offset = 0x0
    DMA_0.TCD[18].SOFF.R = 0x0;

    // TCD[1] DOFF minor transfer iteration offset = 0x1
    DMA_0.TCD[18].DOFF.R = 0x1;

    // TCD[1] Word 3 config SLAST - TCD Last Source Address Adjustment
    DMA_0.TCD[18].SLAST.R = 0;

    // TCD[0] Word 4 config DADDR - TCD Destination Address
    DMA_0.TCD[18].DADDR.R = (uint32_t)&Receive_buffer;	 //Destination Address (LINFlexD_0.BDRL.DATA0)

    // TCD[1] Word 5 config CITER - TCD Current Minor Loop Link, Major Loop Count
    // ELINK | CITER
    DMA_0.TCD[18].CITER.ELINKNO.R = 0x0 | 0x1;  //Destination Address

    // TCD[1] Word 6 config DLAST_SGA - TCD Last Destination Address Adjustment/Scatter Gather Address
    DMA_0.TCD[18].DLASTSGA.R = 0x0;	 // Destination last address adjustment

    // TCD[1] Word 7 config BITER - TCD Beginning Minor Loop Link, Major Loop Count
    // ELINK | BITER
    DMA_0.TCD[18].BITER.ELINKNO.R = 0x0 | 0x1;	 // Destination last address adjustment

    DMA_0.ERQ.B.ERQ18 = 1; 	//The DMA request signal for channel 1 is enabled.
}

void TransmitData(uint8_t dat)
{
	LINFlexD_1.BDRL.B.DATA0 = dat;//write character to transmit buffer
	while (1 != LINFlexD_1.UARTSR.B.DTFTFF) {}// Wait for data transmission completed flag
	LINFlexD_1.UARTSR.R = 0x0002;// clear the DTF flag and not the other flags
}


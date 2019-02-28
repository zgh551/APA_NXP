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
/*  - LINFlexD_1 module divides its 80 MHz LIN_CLK input to get 10.417K baud.*/
/*  - Transmit function sends 'hello   ' as 8 bytes with ID 0x35.            */
/*  - Receive function requests data from slave with ID 0x35. An external    */
/*    node or LIN tool is needed complete reception.  Without the node or    */
/*    tool, code will wait forever for the receive flag.                     */
/*                                                                           */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 14 2018    Initial Version                  */
/*****************************************************************************/

#include "pit.h"


void PIT0_init(uint32_t LDVAL) {
	PIT_0.TIMER[0].LDVAL.R = LDVAL; /* Load # PIT clocks to count */
	PIT_0.TIMER[0].TCTRL.B.TIE = 1; /* Enable interrupt */
	INTC_0.PSR[226].R = 0x801f;
	PIT_0.TIMER[0].TCTRL.B.TEN = 1; /* enable channel */
}

void PIT1_init(uint32_t LDVAL) {
	PIT_0.TIMER[1].LDVAL.R = LDVAL; /* Load PIT counter */
	PIT_0.TIMER[1].TCTRL.B.TIE = 1; /* Enable interrupt */
	//INTC_0.PSR[227].B.PRC_SELN0 = 0b1000; /* Send interrupt to processor 0 */
	//INTC_0.PSR[227].B.PRIN = 9; /* Set interrupt to priority 9 */
	INTC_0.PSR[227].R = 0x8009;
	PIT_0.TIMER[1].TCTRL.B.TEN = 1; /* Enable channel */
}

void PIT2_init(uint32_t LDVAL) {
	PIT_0.TIMER[2].LDVAL.R = LDVAL; /* Load PIT counter */
	PIT_0.TIMER[2].TCTRL.B.TIE = 1; /* Enable interrupt */
	//INTC_0.PSR[228].B.PRC_SELN0 = 0b1000; /* Send IRQ to processor 0 */
	//INTC_0.PSR[228].B.PRIN = 11; /* Set priority to 11 */
	INTC_0.PSR[228].R = 0x800B;
	PIT_0.TIMER[2].TCTRL.B.TEN = 1; /* Enable channel */
}


void PIT_Configure()
{
		/// Init the PIT0
		PIT_0.MCR.B.MDIS = 0; /* Enable PIT module. NOTE: PIT module must be       */
	                        /* enabled BEFORE writing to it's registers.         */
	                        /* Other cores will write to PIT registers so the    */
	                        /* PIT is enabled here before starting other cores.  */
		PIT_0.MCR.B.FRZ = 1;  /* Freeze PIT timers in debug mode */
//		PIT0_init(1000000);//20ms
		PIT0_init(250000);//5ms
		/* timeout= 0.8M  PITclks x 4 sysclks/1 PITclk x 1 sec/160Msysck */
		/*        = 0.8M x 4 / 160M = 3.2/160 = 0.02 sec.  */
		PIT_0.MCR.B.FRZ = 0; //Unfreeze timers
}

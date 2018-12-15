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
	INTC_0.PSR[226].R = 0x800A;
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

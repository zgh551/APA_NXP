/*
 * etimer.c
 *
 *  Created on: 2019年6月27日
 *      Author: zhuguohua
 */
/*****************************************************************************/
/* FILE NAME: etimer.c                          COPYRIGHT (c) Motovis 2019   */
/*                                                      All Rights Reserved  */
/* DESCRIPTION: Input single capture and measure pulse width or the period   */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     June 27 2019        Initial Version                  */
/*****************************************************************************/
#include "etimer.h"

uint16_t cmp_value_array[5][2]={
		{24999,4999},
		{4999,9999},
		{4999,9999},
		{9999,4999},
		{9999,4999}
};

uint8_t cmp1_cnt = 0,cmp2_cnt;
uint8_t pulse_sum_cnt = 0;
/*******************************************************************************
Function Name : eTimer0_Init
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer0 channel 0, 1 init for capture features
Issues        : NONE
*******************************************************************************/
void eTimer0_Init(void)
{
    ETIMER_0.ENBL.R = 0x0;			// disable Timer0 channels

    ETIMER_0.CH[0].CTRL1.R = 0x3801;		// Counts only rising edge of the MC_CLK (100MHz in RUN0), divide by 1, count up, count repeatedly, rollover
    ETIMER_0.CH[0].COMP1.R = 0xFFFF;
    ETIMER_0.CH[0].CCCTRL.R = 0x0264;		// compare on COMP1 when counting up, COMP2 when counting down
    						// CAPT2 on falling edge, CAPT1 on rising edge, 2 entries
    						// free-running mode
    ETIMER_0.CH[0].CTRL3.R	= 1;

    ETIMER_0.CH[1].CTRL1.R = 0xF001;		// cascaded mode, count up, rollover, count repeatedly
    ETIMER_0.CH[1].COMP1.R = 0xFFFF;
    ETIMER_0.CH[1].CCCTRL.R = 0x0264;		// compare on COMP1 when counting up, COMP2 when counting down
    						// CAPT2 on falling edge, CAPT1 on rising edge, 2 entries
    						// free-running mode
    ETIMER_0.CH[1].CTRL3.R	= 1;

    ETIMER_0.ENBL.R = 0x0003;			// Enable Timer0 channel 1
}

/*******************************************************************************
Function Name : eTimer1_Init
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer1 channel 0, 1 init for capture features
Issues        : NONE
*******************************************************************************/
void eTimer1_Init(void)
{
    ETIMER_1.ENBL.R = 0x0;					// disable Timer1 channels

    ETIMER_1.CH[0].CTRL1.R = 0x3805;		// Counts only rising edge of the MC_CLK (100MHz in RUN0), divide by 1, count up, count repeatedly, rollover
    ETIMER_1.CH[0].COMP1.R = 0xFFFF;
    ETIMER_1.CH[0].CCCTRL.R = 0x0264;		// compare on COMP1 when counting up, COMP2 when counting down
    										// CAPT2 on falling edge, CAPT1 on rising edge, 2 entries
    										// free-running mode
    ETIMER_1.CH[0].CTRL3.R	= 1;

    ETIMER_1.CH[5].CTRL1.R  = 0xF005;		// cascaded mode, count up, rollover, count repeatedly
    ETIMER_1.CH[5].COMP1.R  = 0xFFFF;
    ETIMER_1.CH[5].CCCTRL.R = 0x0264;		// compare on COMP1 when counting up, COMP2 when counting down
    										// CAPT2 on falling edge, CAPT1 on rising edge, 2 entries
    										// free-running mode
    ETIMER_1.CH[5].CTRL3.R	= 1;

    SIUL2.MSCR[PA5].B.IBE = 1;    /* PA5: Enable pad for input - eTimer1 ch5 */
    SIUL2.IMCR[70].B.SSS = 1;   /* eTimer0 ch1: connected to pad PA5 */

    ETIMER_1.ENBL.R = 0x0021;			// Enable Timer1 channel 0 5
}

/*******************************************************************************
Function Name : eTimer2_Init
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer2 channel 0, 1 init for capture features
Issues        : NONE
*******************************************************************************/
void eTimer2_Init(void)
{
    ETIMER_2.ENBL.R = 0x0;			// disable Timer0 channels

    ETIMER_2.CH[0].CTRL1.R  = 0x3804;		// Counts only rising edge of the MC_CLK (100MHz in RUN0), divide by 1, count up, count repeatedly, rollover
    ETIMER_2.CH[0].COMP1.R  = 0xFFFF;
    ETIMER_2.CH[0].CCCTRL.R = 0x0264;		// compare on COMP1 when counting up, COMP2 when counting down
    						// CAPT2 on falling edge, CAPT1 on rising edge, 2 entries
    						// free-running mode
    ETIMER_2.CH[0].CTRL3.R	= 1;

    ETIMER_2.CH[4].CTRL1.R  = 0xF004;		// cascaded mode, count up, rollover, count repeatedly
    ETIMER_2.CH[4].COMP1.R  = 0xFFFF;
    ETIMER_2.CH[4].CCCTRL.R = 0x0264;		// compare on COMP1 when counting up, COMP2 when counting down
    						// CAPT2 on falling edge, CAPT1 on rising edge, 2 entries
    						// free-running mode
    ETIMER_2.CH[4].CTRL3.R	= 1;

    SIUL2.MSCR[PA8].B.IBE = 1;    /* PA8: Enable pad for input - eTimer2 ch4 */
    SIUL2.IMCR[75].B.SSS  = 1;   /* eTimer2 ch4: connected to pad PA8 */

    ETIMER_2.ENBL.R = 0x0011;			// Enable Timer2 channel 0 4
}

/*******************************************************************************
Function Name : eTimer0_StartInputCapture
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer0 start the input single capture with channel 0 1
Issues        : NONE
*******************************************************************************/
void eTimer0_StartInputCapture()
{
    ETIMER_0.CH[0].CCCTRL.B.ARM = 1;		// starts the input capture process
    ETIMER_0.CH[1].CCCTRL.B.ARM = 1;
}

/*******************************************************************************
Function Name : eTimer1_StartInputCapture
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer1 start the input single capture with channel 0 5
Issues        : NONE
*******************************************************************************/
void eTimer1_StartInputCapture()
{
    ETIMER_1.CH[0].CCCTRL.B.ARM = 1;		// starts the input capture process
    ETIMER_1.CH[5].CCCTRL.B.ARM = 1;
}

/*******************************************************************************
Function Name : eTimer2_StartInputCapture
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer2 start the input single capture with channel 0 2 3 4
Issues        : NONE
*******************************************************************************/
void eTimer2_StartInputCapture()
{
    ETIMER_2.CH[0].CCCTRL.B.ARM = 1;		// starts the input capture process
    ETIMER_2.CH[4].CCCTRL.B.ARM = 1;
}

/*******************************************************************************
Function Name : eTimer1_CalculatePulse
Engineer      : Zhuguohua
Date          : June-28-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer1 calculate the pulse single frequence,period,duty
Issues        : NONE
*******************************************************************************/
void eTimer1_CalculatePulse()
{
	uint32_t counts, edge1,edge2,edge3,edge4 ;
	uint32_t capture_ch0[8],capture_ch1[8];

	float freq, period, duty,pulseH, pulseL;

	while(!(0x0080 & ETIMER_1.CH[5].STS.R)){}  // wait for channel 1's capture2 flag
	while(!(0x0080 & ETIMER_1.CH[0].STS.R)){}

	capture_ch1[0] = ETIMER_1.CH[5].CAPT1.R;
	capture_ch1[1] = ETIMER_1.CH[5].CAPT2.R;
	capture_ch1[2] = ETIMER_1.CH[5].CAPT1.R;
	capture_ch1[3] = ETIMER_1.CH[5].CAPT2.R;

	capture_ch0[0] = ETIMER_1.CH[0].CAPT1.R;
	capture_ch0[1] = ETIMER_1.CH[0].CAPT2.R;
	capture_ch0[2] = ETIMER_1.CH[0].CAPT1.R;
	capture_ch0[3] = ETIMER_1.CH[0].CAPT2.R;

	capture_ch1[4] = ETIMER_1.CH[5].CAPT1.R;
	capture_ch1[5] = ETIMER_1.CH[5].CAPT2.R;
	capture_ch1[6] = ETIMER_1.CH[5].CAPT1.R;
	capture_ch1[7] = ETIMER_1.CH[5].CAPT2.R;

	capture_ch0[4] = ETIMER_1.CH[0].CAPT1.R;
	capture_ch0[5] = ETIMER_1.CH[0].CAPT2.R;
	capture_ch0[6] = ETIMER_1.CH[0].CAPT1.R;
	capture_ch0[7] = ETIMER_1.CH[0].CAPT2.R;

	edge1 = capture_ch1[0]*65536 + capture_ch0[0];	// save 1st rising edge
	edge2 = capture_ch1[1]*65536 + capture_ch0[1];	// save 1st falling edge
	edge3 = capture_ch1[2]*65536 + capture_ch0[2];	// save 2nd rising edge
	edge4 = capture_ch1[3]*65536 + capture_ch0[3];	// save 2nd falling edge

	// calculate period, pulseH, pulseL, freq and duty
	if(edge3>edge1)
	{
		counts = edge3 - edge1;
	}
	else
	{
		counts = (0xFFFFFFFF - edge1 +1) + edge3;
	}

	freq = (float)100000000.0/counts;
	period = counts / (float)100000.0;

	if(edge2 > edge1)
	{
		counts = edge2 - edge1;
	}
	else
	{
		counts = (0xFFFFFFFF - edge1 +1) + edge2;
	}

	pulseH = counts / (float)100000.0;
	pulseL = period-pulseH;

	duty = pulseH/period*100;

    ETIMER_1.CH[5].STS.R = 0x00C0;		// clear eTimer0 channel 1's capture1/2 flags
    ETIMER_1.CH[0].STS.R = 0x00C0;		// clear eTimer0 channel 0's capture1/2 flags
}

/*******************************************************************************
Function Name : eTimer2_CalculatePulse
Engineer      : Zhuguohua
Date          : June-28-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer2 calculate the pulse single frequence,period,duty
Issues        : NONE
*******************************************************************************/
void eTimer2_CalculatePulse()
{
	uint32_t counts, edge1,edge2,edge3,edge4 ;
	uint32_t capture_ch0[8],capture_ch1[8];

	float freq, period, duty,pulseH, pulseL;

	while(!(0x0080 & ETIMER_2.CH[4].STS.R)){}  // wait for channel 1's capture2 flag
	while(!(0x0080 & ETIMER_2.CH[0].STS.R)){}

	capture_ch1[0] = ETIMER_2.CH[4].CAPT1.R;
	capture_ch1[1] = ETIMER_2.CH[4].CAPT2.R;
	capture_ch1[2] = ETIMER_2.CH[4].CAPT1.R;
	capture_ch1[3] = ETIMER_2.CH[4].CAPT2.R;

	capture_ch0[0] = ETIMER_2.CH[0].CAPT1.R;
	capture_ch0[1] = ETIMER_2.CH[0].CAPT2.R;
	capture_ch0[2] = ETIMER_2.CH[0].CAPT1.R;
	capture_ch0[3] = ETIMER_2.CH[0].CAPT2.R;

	capture_ch1[4] = ETIMER_2.CH[4].CAPT1.R;
	capture_ch1[5] = ETIMER_2.CH[4].CAPT2.R;
	capture_ch1[6] = ETIMER_2.CH[4].CAPT1.R;
	capture_ch1[7] = ETIMER_2.CH[4].CAPT2.R;

	capture_ch0[4] = ETIMER_2.CH[0].CAPT1.R;
	capture_ch0[5] = ETIMER_2.CH[0].CAPT2.R;
	capture_ch0[6] = ETIMER_2.CH[0].CAPT1.R;
	capture_ch0[7] = ETIMER_2.CH[0].CAPT2.R;

	edge1 = capture_ch1[0]*65536 + capture_ch0[0];	// save 1st rising edge
	edge2 = capture_ch1[1]*65536 + capture_ch0[1];	// save 1st falling edge
	edge3 = capture_ch1[2]*65536 + capture_ch0[2];	// save 2nd rising edge
	edge4 = capture_ch1[3]*65536 + capture_ch0[3];	// save 2nd falling edge

	// calculate period, pulseH, pulseL, freq and duty
	if(edge3>edge1)
	{
		counts = edge3 - edge1;
	}
	else
	{
		counts = (0xFFFFFFFF - edge1 +1) + edge3;
	}

	freq = (float)100000000.0/counts;
	period = counts / (float)100000.0;

	if(edge2 > edge1)
	{
		counts = edge2 - edge1;
	}
	else
	{
		counts = (0xFFFFFFFF - edge1 +1) + edge2;
	}

	pulseH = counts / (float)100000.0;
	pulseL = period - pulseH;

	duty = pulseH/period*100;

    ETIMER_2.CH[4].STS.R = 0x00C0;		// clear eTimer0 channel 1's capture1/2 flags
    ETIMER_2.CH[0].STS.R = 0x00C0;		// clear eTimer0 channel 0's capture1/2 flags
}
/*************************************************************************************/
/*******************************************************************************
Function Name : eTimer1_OutputInit
Engineer      : Zhuguohua
Date          : July-2-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer1 variable frequency PWM mode
Issues        : NONE
*******************************************************************************/
void eTimer1_OutputInit(void)
{
    ETIMER_1.ENBL.R = 0x0;					// disable Timer1 channels

    ETIMER_1.CH[5].CTRL1.B.CNTMODE = 1;
    ETIMER_1.CH[5].CTRL1.B.PRISRC = 0x18;
    ETIMER_1.CH[5].CTRL1.B.ONCE = 0;
    ETIMER_1.CH[5].CTRL1.B.DIR = 0;
    ETIMER_1.CH[5].CTRL1.B.LENGTH = 1;

    ETIMER_1.CH[5].CTRL2.B.OEN = 1;
    ETIMER_1.CH[5].CTRL2.B.OUTMODE = 4;

    ETIMER_1.CH[5].CTRL3.B.DBGEN   = 1;

    ETIMER_1.CH[5].COMP1.R  = 24999;
    ETIMER_1.CH[5].COMP2.R  = 4999;

    ETIMER_1.CH[5].CMPLD1.R = 4999;
    ETIMER_1.CH[5].CMPLD2.R = 9999;

    ETIMER_1.CH[5].CCCTRL.B.CLC1 = 2;
    ETIMER_1.CH[5].CCCTRL.B.CLC2 = 5;

    /*
     * Interrupt an DMA Enable
     * */
    ETIMER_1.CH[5].INTDMA.B.TCF1IE = 1;
    ETIMER_1.CH[5].INTDMA.B.TCF2IE = 1;

    /*
     * Pin Port assign
     * */
    SIUL2.MSCR[PA5].B.OBE = 1;    /* PA5: Enable pad for output - eTimer1 ch5 */
    SIUL2.MSCR[PA5].B.SRC = 3;    //Maximum slew rate
    SIUL2.MSCR[PA5].B.SSS = 2;    /* eTimer0 ch5: connected to pad PA5 */

    ETIMER_1.ENBL.R = 0x0020;			// Enable Timer1 channel 0 5

    INTC_0.PSR[627].R = 0x8009; //set priority and core for eTimer1 Channel5 interrupt
}

/*******************************************************************************
Function Name : eTimer2_OutputInit
Engineer      : Zhuguohua
Date          : July-2-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer2 variable frequency PWM mode
Issues        : NONE
*******************************************************************************/
void eTimer2_OutputInit(void)
{
    ETIMER_2.ENBL.R = 0x0;					// disable Timer2 channels

    ETIMER_2.CH[4].CTRL1.B.CNTMODE = 1;
    ETIMER_2.CH[4].CTRL1.B.PRISRC = 0x18;
    ETIMER_2.CH[4].CTRL1.B.ONCE = 0;
    ETIMER_2.CH[4].CTRL1.B.DIR = 0;
    ETIMER_2.CH[4].CTRL1.B.LENGTH = 1;

    ETIMER_2.CH[4].CTRL2.B.OUTMODE = 4;

    ETIMER_2.CH[4].CTRL3.B.DBGEN   = 1;

    ETIMER_2.CH[4].COMP1.R  = 24999;
    ETIMER_2.CH[4].COMP2.R  = 4999;

    ETIMER_2.CH[4].CMPLD1.R = 4999;
    ETIMER_2.CH[4].CMPLD2.R = 9999;

    ETIMER_2.CH[4].CCCTRL.R = 0x0000;

    /*
     * Pin Port assign
     * */
    SIUL2.MSCR[PA8].B.OBE = 1;    /* PA5: Enable pad for output - eTimer1 ch5 */
    SIUL2.MSCR[PA8].B.SRC = 3;    //Maximum slew rate
    SIUL2.MSCR[PA8].B.SSS = 2;    /* eTimer0 ch5: connected to pad PA5 */

    ETIMER_2.ENBL.R = 0x0010;			// Enable Timer2 channel 0 4
}

void eTimer1Channel5OutputStart(void)
{
//	ETIMER_1.CH[5].CNTR.R = 0;
//    ETIMER_1.CH[5].COMP1.R  = 9999;
//    ETIMER_1.CH[5].COMP2.R  = 4999;

    ETIMER_1.CH[5].CTRL2.B.OEN = 1;
    ETIMER_1.CH[5].CTRL2.B.OUTMODE = 4;
	ETIMER_1.CH[5].CTRL1.B.CNTMODE = 1;

    SIUL2.MSCR[PA5].B.OBE = 1;    /* PA5: Enable pad for output - eTimer1 ch5 */
    SIUL2.MSCR[PA5].B.SRC = 1;    //Maximum slew rate
    SIUL2.MSCR[PA5].B.SSS = 2;    /* eTimer0 ch5: connected to pad PA5 */
    SIUL2.MSCR[PA5].B.IBE = 0;    /* PA5: Enable pad for input - eTimer1 ch5 */
	SIUL2.MSCR[PA5].B.PUS = 0;
	SIUL2.MSCR[PA5].B.PUE = 0;
//
//    ETIMER_1.CH[5].CMPLD1.R = 4999;
//    ETIMER_1.CH[5].CMPLD2.R = 9999;
}

void eTimer1Channel5OutputStop(void)
{
	ETIMER_1.CH[5].CTRL1.B.CNTMODE = 0;
    ETIMER_1.CH[5].CTRL2.B.OEN = 0;
    ETIMER_1.CH[5].CTRL2.B.OUTMODE = 0;

	SIUL2.MSCR[PA5].B.OBE = 0;    /* PA5: Enable pad for output - eTimer1 ch5 */
	SIUL2.MSCR[PA5].B.IBE = 1;    /* PA5: Enable pad for input - eTimer1 ch5 */
//	SIUL2.MSCR[PA5].B.PUS = 1;
//	SIUL2.MSCR[PA5].B.PUE = 1;
	SIUL2.MSCR[PA5].B.SSS = 0;    /* eTimer0 ch5: connected to pad PA5 */
    SIUL2.IMCR[70].B.SSS = 1;   /* eTimer0 ch1: connected to pad PA5 */
//	ETIMER_1.CH[5].CTRL2.B.OEN = 0;
}

void eTimer1_Channel5_Isr(void)
{
	if(ETIMER_1.CH[5].STS.B.TCF1 == 1)
	{
//		cmp1_cnt = (cmp1_cnt + 1) % 2;
//	    ETIMER_1.CH[5].CMPLD1.R = cmp_value_array[cmp1_cnt][0];
//		pulse_sum_cnt++;

		ETIMER_1.CH[5].STS.B.TCF1 = 1;
	}
	else if(ETIMER_1.CH[5].STS.B.TCF2 == 1)
	{
		if(cmp2_cnt == 0)
		{
			eTimer1Channel5OutputStop();
		}
		cmp2_cnt = (cmp2_cnt + 1) % 5;

		ETIMER_1.CH[5].CMPLD2.R = cmp_value_array[cmp2_cnt][1];
		ETIMER_1.CH[5].CMPLD1.R = cmp_value_array[cmp2_cnt][0];

		ETIMER_1.CH[5].STS.B.TCF2 = 1;
	}

}

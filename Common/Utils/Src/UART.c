/*******************************************************************************
* Freescale Semiconductor Inc.
* (c) Copyright 2010 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
********************************************************************************
Services performed by FREESCALE in this matter are performed AS IS and without
any warranty. CUSTOMER retains the final decision relative to the total design
and functionality of the end product. FREESCALE neither guarantees nor will be
held liable by CUSTOMER for the success of this project.
FREESCALE DISCLAIMS ALL WARRANTIES, EXPRESSED, IMPLIED OR STATUTORY INCLUDING,
BUT NOT LIMITED TO, IMPLIED WARRANTY OF MERCHANTABILITY OR FITNESS FOR
A PARTICULAR PURPOSE ON ANY HARDWARE, SOFTWARE ORE ADVISE SUPPLIED 
TO THE PROJECT BY FREESCALE, AND OR NAY PRODUCT RESULTING FROM FREESCALE 
SERVICES. IN NO EVENT SHALL FREESCALE BE LIABLE FOR INCIDENTAL OR CONSEQUENTIAL 
DAMAGES ARISING OUT OF THIS AGREEMENT.
CUSTOMER agrees to hold FREESCALE harmless against any and all claims demands 
or actions by anyone on account of any damage, or injury, whether commercial,
contractual, or tortuous, rising directly or indirectly as a result 
of the advise or assistance supplied CUSTOMER in connection with product, 
services or goods supplied under this Agreement.
********************************************************************************
* File:             uart.c
* Owner:            Martin Kovar
* Version:          1.0
* Date:             Dec-09-2015
* Classification:   General Business Information
* Brief:            terminal IO. implements CodeWarrior MSL library calls
*                   MSL_C library printf() output to MPC5607B's LINFlex0
********************************************************************************
* Detailed Description: 
* 
* implements CW MSL library function calls to implement printf() on LINFlex0
* header file UART.h is taken from the CW as is
* global functions defined in this module replace the ones from the library
* so finally we send printf() to LINFlex0
*
* ------------------------------------------------------------------------------
* Test HW:  XPC5607B 176LQFP, XPC56XX EVB MOTHEBOARD Rev.C
* Target :  PPC5607BMLUAM03Y
* Terminal: 19200-8-no parity-1 stop bit-no flow control on LINFlexD_0
* Fsys:     64/48 MHz
*
********************************************************************************
Revision History:
1.0     Dec-21-2015     Martin Kovar  Initial Version
*******************************************************************************/

#include "sys/UART.h"
#include "MPC5744P.h"



/*******************************************************************************
* Global variables
*******************************************************************************/

/*******************************************************************************
* Constants and macros
*******************************************************************************/

/*******************************************************************************
* Local types
*******************************************************************************/

/*******************************************************************************
* Local function prototypes
*******************************************************************************/
//static void Init_LINFlexD_0(void);
static void TransmitData(const char * pBuf, const uint32_t cnt);
static int32_t ReceiveData(char * const pBuf);

/*******************************************************************************
* Local variables
*******************************************************************************/

/*******************************************************************************
* Local functions
*******************************************************************************/ 

/*******************************************************************************
Function Name : Init_LINFlexD_0
Engineer      : Martin Kovar
Date          : Dec-09-2015
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : initializes MPC5644P's Init_LINFlexD_0 module for UART mode
Issues        : expecting system clock is 100MHz
*******************************************************************************/
void LINFlexD_0_Init (void)
{
	/* enter INIT mode */
	LINFlexD_0.LINCR1.R = 0x0081; /* SLEEP=0, INIT=1 */
	

	/* wait for the INIT mode */
	while (0x1000 != (LINFlexD_0.LINSR.R & 0xF000)) {}
		
	/* configure pads */
	 SIUL2.MSCR[18].B.SSS = 1;    /* Pad PB2: Source signal is LIN0_TXD  */
	    SIUL2.MSCR[18].B.OBE = 1;    /* Pad PB2: Output Buffer Enable */
	    SIUL2.MSCR[18].B.SRC = 3;    /* Pad PB2: Maximum slew rate */
	    SIUL2.IMCR[165].B.SSS = 1;   /* LIN0_RXD: connected to pad PB3 */
	    /* Configure pad PB3 for LIN0RX */
	    SIUL2.MSCR[19].B.IBE = 1;    /* Pad PB3: Enable pad for input - LIN0_RXD */


	
	/* configure for UART mode */
	LINFlexD_0.UARTCR.R = 0x0001; /* set the UART bit first to be able to write the other bits */
	
	LINFlexD_0.UARTCR.R = 0x0033; /* 8bit data, no parity, Tx and Rx enabled, UART mode */
								 /* Transmit buffer size = 1 (TDFL = 0 */
								 /* Receive buffer size = 1 (RDFL = 0) */
	
	/* configure baudrate 19200 */
	/* assuming 64 MHz peripheral set 1 clock (fsys below)*/
	/* LFDIV = fsys / (16 * desired baudrate)
	   LINIBRR = integer part of LFDIV
	   LINFBRR = 16 * fractional part of LFDIV (after decimal point)
	   
	   for instance:
	   LFDIV = 100e6/(16*19200) = 325.52
	   LINIBRR = 325
	   LINFBRR = 16*0.52 = 8.32
	*/

		LINFlexD_0.LINFBRR.R = 8;
		LINFlexD_0.LINIBRR.R = 325;

	/* enter NORMAL mode */
	LINFlexD_0.LINCR1.R = 0x0080; /* INIT=0 */
}


/*******************************************************************************
Function Name : TransmitData
Engineer      : b05111
Date          : Apr-14-2011
Parameters    : pBuf - input string. won't be modified by the function
              : cnt  - input number of characters to be transmitted
Modifies      : NONE
Returns       : NONE
Notes         : Tx data on LINFlexD_0. polled mode.
Issues        : NONE 
*******************************************************************************/
void TransmitData(const char * const pBuf, const uint32_t cnt)
{
    uint8_t	j = 0; // Dummy variable
    
    for (j=0; j< cnt; j++) 
    {  // Loop for character string  
   	    LINFlexD_0.BDRL.B.DATA0 = *(pBuf+j);
   	      //write character to transmit buffer
	    while (1 != LINFlexD_0.UARTSR.B.DTFTFF) {}
	      // Wait for data transmission completed flag
	    LINFlexD_0.UARTSR.R = 0x0002;
	      // clear the DTF flag and not the other flags
    }

    
}

/*******************************************************************************
Function Name : ReceiveData
Engineer      : b05111
Date          : Apr-14-2011
Parameters    : pBuf - address of a char where the received char is written to
                       the address (pBuf) doesn't change in the function
Modifies      : NONE
Returns       : NONE
Notes         : Rx data on LINFlexD_0. polled mode.
Issues        : NONE 
*******************************************************************************/
int32_t ReceiveData(char * const pBuf)
{
    
    int32_t rx_data;
    
    	/* wait for DRF */
	while (1 != LINFlexD_0.UARTSR.B.DRFRFE) {}  /* Wait for data reception completed flag */
		
	/* wait for RMB */
	while (1 != LINFlexD_0.UARTSR.B.RMB) {}  /* Wait for Release Message Buffer */
	
	/* get the data */
	
	rx_data = LINFlexD_0.BDRM.R; // read whole register due to erratum e4897PS
    
	*pBuf = (uint8_t)rx_data; // take 
	
	/* clear the DRF and RMB flags by writing 1 to them */
	LINFlexD_0.UARTSR.R = 0x0204;
	return 0;
}


/*******************************************************************************
* Global functions
*******************************************************************************/ 

/*
Methods called by MW MSL libraries to perform console IO:
*/


UARTError InitializeUART(UARTBaudRate baudRate)
{
#pragma unused(baudRate)
	LINFlexD_0_Init ();
	return kUARTNoError;
}


UARTError ReadUARTN(void* bytes, unsigned long limit)
{
	int count;
	UARTError err;

	for (count = 0, err = kUARTNoError;
		count < limit && err == kUARTNoError;
		count++)
        {
		err = ReceiveData( (char *)bytes + count );
        }

	return err;
}


UARTError ReadUARTPoll(char* c)
{
    int32_t rx_data;

    rx_data = LINFlexD_0.BDRM.R; // read whole register due to erratum e4897PS

    if (LINFlexD_0.UARTSR.B.RMB == 0)
        return  kUARTNoData;  // return no data
    else
    {
        LINFlexD_0.UARTSR.R = 0x0204;
        *c =(unsigned char) rx_data; // read byte of Data
        return kUARTNoError;  // return no error
    }
}



UARTError WriteUARTN(const void * bytes, unsigned long length)
{ 
	TransmitData ((const char * const)bytes,length);
  	return kUARTNoError;
}


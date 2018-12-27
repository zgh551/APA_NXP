/*
 * mode_entry.c
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


#include "project.h"

void PeripheralClockGating(void)
{
  MC_ME.RUN_PC[0].R = 0x00000000;  /* gate off clock for all RUN modes */
  MC_ME.RUN_PC[1].R = 0x000000FE;  /* config. peri clock for all RUN modes */

  /// CAN Module
  MC_ME.PCTL77.B.RUN_CFG  = 0b001; //FlexCAN 2: select peripheral config RUN_PC[1]
  MC_ME.PCTL78.B.RUN_CFG  = 0b001; //FlexCAN 1: select peripheral config RUN_PC[1]
  MC_ME.PCTL79.B.RUN_CFG  = 0b001; //FlexCAN 0: select peripheral config RUN_PC[1]
  /// LIN Module
  MC_ME.PCTL91.B.RUN_CFG  = 0b001; //LINFlexD_1: Select peripheral config RUN_PC[1].
  MC_ME.PCTL204.B.RUN_CFG = 0b001; //LINFlexD_0: Select peripheral config RUN_PC[1].
  /// PIT time module
  MC_ME.PCTL30.B.RUN_CFG  = 0b001; //PCTL30 is PIT0 Peripheral Control Registers for Panther
  /// DMA module
  MC_ME.PCTL36.B.RUN_CFG  = 0b001; //DMAMUX_0:
  MC_ME.PCTL146.B.RUN_CFG = 0b001; //DMAMUX_1:
}

void PLL_160MHz(void)
{
  /* PBRIDGEx_CLK Divide */
  MC_CGM.SC_DC0.B.DIV = 3;  /* Freq = sysclk / (0+1) = sysclk */
  MC_CGM.SC_DC0.B.DE  = 1;  /* Enable divided clock */

  /* Connect XOSC to PLL. We ultimately use the output of PLL1. PLL1 must be fed the output of PLL0 */
  MC_CGM.AC3_SC.B.SELCTL = 1; //40 MHz XOSC selected as input of PLL0

  MC_CGM.AC4_SC.B.SELCTL=0b11; //PLL0_PHI1 selected as input of PHI1

  PeripheralClockGating();
  /* Configure PLL0 Dividers - 160MHz from 40Mhx XOSC */
  /* PLL input = FXOSC = 40MHz
     VCO range = 600-1200MHz
     MPC5744P uses PLL1 for fractional divide options.
     Configure PLL1 first, because it depends on PLL0. So configure while
     PLL0 still off
  */

  /* Program PLL1 to same frequency as PLL0.
   * MFD multiplies input by at least 10. So multiply by 10 and divide by 10.
   * 10/10 = 1, so same frequency as PLL0
   */
  PLLDIG.PLL1DV.B.RFDPHI = 16;
  PLLDIG.PLL1DV.B.MFD = 16;

  /* Configure PLL0 to 160 MHz. */
  PLLDIG.PLL0DV.B.RFDPHI1 = 4;
  PLLDIG.PLL0DV.B.RFDPHI = 4;
  PLLDIG.PLL0DV.B.PREDIV  = 1;
  PLLDIG.PLL0DV.B.MFD     = 16;

  /* switch to PLL */
  MC_ME.DRUN_MC.R = 0x00130072;
  MC_ME.MCTL.R = 0x30005AF0;
  MC_ME.MCTL.R = 0x3000A50F;
  while(MC_ME.GS.B.S_MTRANS == 1);      /* Wait for mode transition complete */
}

void PLL_200MHz(void)
{
	/* PBRIDGEx_CLK Divide */
	MC_CGM.SC_DC0.B.DIV = 3;  /* Freq = sysclk / (0+1) = sysclk */
	MC_CGM.SC_DC0.B.DE  = 1;  /* Enable divided clock */

	/* Connect XOSC to PLL. We ultimately use the output of PLL1. PLL1 must be fed the output of PLL0 */
	MC_CGM.AC3_SC.B.SELCTL = 1; //8 MHz XOSC selected as input of PLL0

	MC_CGM.AC4_SC.B.SELCTL=0b11; //PLL0_PHI1 selected as input of PHI1

	MC_CGM.AC2_DC0.B.DIV = 4;// set the divider division value
	MC_CGM.AC2_DC0.B.DE = 1; // Enable the auxiliary clock2 divider 0

	PeripheralClockGating();
  /* Configure PLL0 Dividers - 200MHz from 8Mhx XOSC */
  /* PLL input = FXOSC = 8MHz
     VCO range = 18 - 2032MHz
     MPC5744P uses PLL1 for fractional divide options.
     Configure PLL1 first, because it depends on PLL0. So configure while
     PLL0 still off
  */

  /* Program PLL1 to same frequency as PLL0.
   * MFD multiplies input by at least 16. So multiply by 16 and divide by 32.
   * 10/10 = 1, so same frequency as PLL0
   */
  PLLDIG.PLL1DV.B.RFDPHI = 16;
  PLLDIG.PLL1DV.B.MFD = 16;

  /* Configure PLL0 to 200 MHz. */
  PLLDIG.PLL0DV.B.RFDPHI1 = 4 ;
  PLLDIG.PLL0DV.B.RFDPHI  = 4 ;
  PLLDIG.PLL0DV.B.PREDIV  = 1 ;
  PLLDIG.PLL0DV.B.MFD     = 100;

  /* switch to PLL */
  MC_ME.DRUN_MC.R = 0x00130072;
  MC_ME.MCTL.R = 0x30005AF0;
  MC_ME.MCTL.R = 0x3000A50F;
  while(!MC_ME.GS.B.S_PLL0);      //ME_GS Wait for PLL stabilization.
  while(MC_ME.GS.B.S_MTRANS == 1);      /* Wait for mode transition complete */
}

void SystemClockConfigure(void)
{
  PLL_200MHz();

  AIPS_0.MPRA.R |= 0x77777770;       /* All masters have RW & user level access */
  AIPS_1.MPRA.R |= 0x77777770;       /* All masters have RW & user level access */
}

void enter_STOP_mode (void) {
  MC_ME.MCTL.R = 0xA0005AF0;      /* Enter STOP mode and key */
  MC_ME.MCTL.R = 0xA000A50F;      /* Enter STOP mode and inverted key */
  while (MC_ME.GS.B.S_MTRANS) {}  /* Wait for STOP mode transition to complete */
}

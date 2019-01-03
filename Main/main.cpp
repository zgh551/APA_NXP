/*
 * mian.cpp
 *
 *  Created on: December 28 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: main.cpp                            COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: main function                   					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 28 2018    Initial Version                  */
/*****************************************************************************/

#include <math.h>
//#include <vector>
#include "derivative.h" /* include peripheral declarations */

//#include "SystemWork.h"
#include "Ultrasonic.h"
//#include "PathPlanning.h"
#include "Terminal.h"
#include "ChangAn/chang_an_controller.h"
#include "ChangAn/chang_an_message.h"
#include "../Common/VehicleState/GeometricTrack/geometric_track.h"

//using namespace std;

#ifdef __cplusplus
extern "C" {
#endif
extern void xcptn_xmpl(void);
#ifdef __cplusplus
}
#endif

Terminal m_Terminal_CA = Terminal(0.02,3.5,0.1,0.1,0.3,1,0.1);
Ultrasonic m_Ultrasonic;
ChangAnController m_ChangAnController;
ChangAnMessage m_ChangAnMessage;
GeometricTrack m_GeometricTrack;

vuint8_t cnt;
bool TerminalSendFlag = false;
float temp;

__attribute__ ((section(".text")))
int main()
{
		/* Configure and Enable Interrupts */
		xcptn_xmpl();

		/* Sysclk = 200MHz, dividers configured, mode trans*/
		SystemClockConfigure();

		/* Init the GPIO */
		initGPIO();

		/* Init CANFlex Module */
		CAN_Configure();

		/* Init LinFlexD Module */
		LINFlexD_Configure();

		/* Init PIT Module */
		PIT_Configure();
		/* Loop forever */
		for(;;)
		{
			m_ChangAnMessage.SteeringAngle = 420;
			m_ChangAnMessage.WheelSpeedRearLeft = 0.3;
			m_ChangAnMessage.WheelSpeedRearRight = 0.4;
		}
}

#ifdef __cplusplus
extern "C" {
#endif
/*******************************************************************************
Function Name : PIT0_isr
Engineer      : Guohua Zhu
Date          : Wed-2-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : 5ms time generate this isr
Issues        : NONE
*******************************************************************************/
void PIT0_isr(void)
{
	m_ChangAnController.Push(0.02);
	m_GeometricTrack.Update(&m_ChangAnMessage,0.02);
	m_Ultrasonic.UltrasonicScheduleStatusMachine_V2();
	m_Ultrasonic.ScheduleTimeCnt = (m_Ultrasonic.ScheduleTimeCnt + 1) % 26;

	if(m_Ultrasonic.ScheduleTimeCnt == 0)
	{
		SYSTEM_LED = ~SYSTEM_LED;
	}
	m_Ultrasonic.SystemTime = m_Ultrasonic.SystemTime + 1;
	PIT_0.TIMER[0].TFLG.R |= 1;  /* Clear interrupt flag. w1c */
}
/*******************************************************************************
Function Name : FlexCAN0_Isr
Engineer      : Guohua Zhu
Date          : Wed-2-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : When CAN0 receive,generate this isr
Issues        : NONE
*******************************************************************************/
void FlexCAN0_Isr(void)
{
	if(CAN_0.IFLAG1.B.BUF31TO8I & 0x000001)
	{
//		m_Terminal_CA.VehicleInformation(CAN_0.MB[8].ID.B.ID_STD,CAN_0.MB[8].DATA.B);
		/* release the internal lock for all Rx MBs
		 * by reading the TIMER */
//		uint32_t temp = CAN_0.TIMER.R;
		CAN_0.IFLAG1.R = 0x00000100;
	}
}

/*******************************************************************************
Function Name : FlexCAN1_Isr
Engineer      : Guohua Zhu
Date          : Wed-2-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : When CAN1 receive,generate this isr
Issues        : NONE
*******************************************************************************/
void FlexCAN1_Isr(void)
{
	if(CAN_1.IFLAG1.B.BUF31TO8I & 0x000001)
	{
		m_ChangAnMessage.Parse(CAN_1.MB[8].ID.B.ID_STD, CAN_1.MB[8].DATA.B, CAN_1.MB[8].CS.B.DLC);
		/* release the internal lock for all Rx MBs
		 * by reading the TIMER */
//		uint32_t temp = CAN_1.TIMER.R;
		CAN_1.IFLAG1.R = 0x00000100;
	}
}

/*******************************************************************************
Function Name : FlexCAN2_Isr
Engineer      : Guohua Zhu
Date          : Wed-2-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : When CAN2 receive,generate this isr
Issues        : NONE
*******************************************************************************/
void FlexCAN2_Isr(void)
{
	if(CAN_2.IFLAG1.B.BUF31TO8I & 0x000001)
	{
		m_Terminal_CA.VehicleInformation(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B);
		/* release the internal lock for all Rx MBs
		 * by reading the TIMER */
//		uint32_t temp = CAN_2.TIMER.R;
		CAN_2.IFLAG1.R = 0x00000100;
	}
}

/*******************************************************************************
Function Name : eDMA_Channel2_Isr
Engineer      : Guohua Zhu
Date          : Dec-24-2018
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : When DMA major finish,generate this isr
Issues        : NONE
*******************************************************************************/
void eDMA_Channel2_Isr(void)
{
	m_Ultrasonic.Update(0, 25);
	DMA_0.INT.B.INT2 = 1;
}

/*******************************************************************************
Function Name : eDMA_Channel18_Isr
Engineer      : Guohua Zhu
Date          : Dec-24-2018
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : When DMA major finish,generate this isr
Issues        : NONE
*******************************************************************************/
void eDMA_Channel18_Isr(void)
{
	m_Ultrasonic.Update(1, 25);
	DMA_0.INT.B.INT18 = 1;
}
#ifdef __cplusplus
}
#endif

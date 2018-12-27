/*
 * main implementation: use this 'C++' sample to create your own application
 *
 */
#include <math.h>
#include "derivative.h" /* include peripheral declarations */
#include "mode_entry.h"
//#include "SystemWork.h"
#include "Ultrasonic.h"
//#include "PathPlanning.h"
#include "Terminal.h"

#include "can.h"
#include "pit.h"
#include "uart.h"
#include "gpio.h"
#include "linflexd.h"

#ifdef __cplusplus
extern "C" {
#endif
extern void xcptn_xmpl(void);
#ifdef __cplusplus
}
#endif

Terminal m_Terminal_CA = Terminal(0.02,3.5,0.1,0.1,0.3,1,0.1);
Ultrasonic m_Ultrasonic;

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

		}
}

#ifdef __cplusplus
extern "C" {
#endif
void PIT0_isr(void)
{
	m_Ultrasonic.UltrasonicScheduleStatusMachine_V2();
	m_Ultrasonic.ScheduleTimeCnt = (m_Ultrasonic.ScheduleTimeCnt + 1) % 26;
	if(m_Ultrasonic.ScheduleTimeCnt == 0)
	{
		SYSTEM_LED = ~SYSTEM_LED;
	}

//cnt = (cnt + 1) % 20;
//if(cnt == 1)
//{
//	m_Ultrasonic.InitUltrasonicSensor(7);
//}
//else if(cnt == 5)
//{
//	m_Ultrasonic.ReadUltrasonicSensor(7);
//}

	PIT_0.TIMER[0].TFLG.R |= 1;  /* Clear interrupt flag. w1c */
}

void FlexCAN0_Isr(void)
{
	if(CAN_0.IFLAG1.B.BUF31TO8I & 0x000001)
	{
		m_Terminal_CA.VehicleInformation(CAN_0.MB[8].ID.B.ID_STD,CAN_0.MB[8].DATA.B);
		/* release the internal lock for all Rx MBs
		 * by reading the TIMER */
		uint32_t temp = CAN_0.TIMER.R;
		CAN_0.IFLAG1.R = 0x00000100;
	}
}
void FlexCAN1_Isr(void)
{
	if(CAN_1.IFLAG1.B.BUF31TO8I & 0x000001)
	{
		m_Terminal_CA.VehicleInformation(CAN_1.MB[8].ID.B.ID_STD,CAN_1.MB[8].DATA.B);
		/* release the internal lock for all Rx MBs
		 * by reading the TIMER */
		uint32_t temp = CAN_1.TIMER.R;
		CAN_1.IFLAG1.R = 0x00000100;
	}
}
void FlexCAN2_Isr(void)
{
	if(CAN_2.IFLAG1.B.BUF31TO8I & 0x000001)
	{
		m_Terminal_CA.VehicleInformation(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B);
		/* release the internal lock for all Rx MBs
		 * by reading the TIMER */
		uint32_t temp = CAN_2.TIMER.R;
		CAN_2.IFLAG1.R = 0x00000100;
	}
}
#ifdef __cplusplus
}
#endif

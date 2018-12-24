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
LIN_STP318_Packet m_LIN_STP318_Packet;
LIN_Packet m_LIN_Packet;

vuint8_t cnt;
bool TerminalSendFlag = false;
float temp;

uint8_t d0,d1,d2;
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
	FlexCAN0_Init();
	FlexCAN1_Init();
	FlexCAN2_Init();

/// Init Flex Lin1 Uart used to communication with terminal pc
//  FlexLin1_Uart_Buffer_Init(100,38400);

//	InitLINFlexD0(100,19200);

	InitLINFlexD0_DMA(100,19200);
	FlexLin0_DMA_TX_M2S_Init();
	FlexLin0_DMA_TX_S2M_Init();
	FlexLin0_DMA_RX_S2M_Init();
/// Init the PIT0
	PIT_0.MCR.B.MDIS = 0; /* Enable PIT module. NOTE: PIT module must be       */
                        /* enabled BEFORE writing to it's registers.         */
                        /* Other cores will write to PIT registers so the    */
                        /* PIT is enabled here before starting other cores.  */
	PIT_0.MCR.B.FRZ = 1;  /* Freeze PIT timers in debug mode */
	PIT0_init(1000000);
	/* timeout= 0.8M  PITclks x 4 sysclks/1 PITclk x 1 sec/160Msysck */
	/*        = 0.8M x 4 / 160M = 3.2/160 = 0.02 sec.  */
	PIT_0.MCR.B.FRZ = 0; //Unfreeze timers

   /* Loop forever */
   for(;;)
   {
//		if(TerminalSendFlag)
//		{
//			m_Terminal_CA.TerminalControlCommandSend();
//			m_Terminal_CA.TerminalControlSpeedSend();
//			m_Terminal_CA.TerminalSystemStateSend();
//			TerminalSendFlag = false;
//		}
//		if(m_Terminal_CA.AckValid == 0x5A)
//		{
//			m_Terminal_CA.TerminalControlAckSend();
//			m_Terminal_CA.AckValid = 0;
//		}
  }
}

#ifdef __cplusplus
extern "C" {
#endif
void PIT0_isr(void)
{
//	m_Terminal_CA.SystemWorkState();
	SYSTEM_LED = ~SYSTEM_LED;
	cnt = (cnt + 1) % 5;
	if(cnt == 1)
	{
		m_Ultrasonic.InitUltrasonicSensor(0);
	}
	if(cnt == 2)
	{
//		m_Ultrasonic.ReadSensing_STP318(LIN0_ReceiveFrame_DMA,0xCf);
		m_Ultrasonic.ReadUltrasonicSensor(0);
//		d0 = m_Ultrasonic.UltrasonicDatas[1].BDRL.B.DATA0;
//		d1 = m_Ultrasonic.UltrasonicDatas[1].BDRL.B.DATA1;
//		d2 = m_Ultrasonic.UltrasonicDatas[1].BDRL.B.DATA2;
	}
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



void FlexLin1_Uart_Isr(void)
{
	m_Terminal_CA.TerminalControlCommandReceive(LINFlexD_1.BDRM.B.DATA4);
	LINFlexD_1.UARTSR.B.DRFRFE = 1;
}
#ifdef __cplusplus
}
#endif

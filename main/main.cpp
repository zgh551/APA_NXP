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

__attribute__ ((section(".text")))
int main()
{
  /* Configure and Enable Interrupts */
  xcptn_xmpl();

  /* Gate the peripheral clock */
//  PeripheralClockGating();

  /* sysclk=200MHz, dividers configured, mode trans*/
  System200Mhz();

  // Init the GPIO
//  initGPIO();

  // Init  CAN Flex Module
//  FlexCAN0_Init();
//  FlexCAN1_Init();
//  FlexCAN2_Init();

  // Init Flex Lin1 Uart used to communication with terminal pc

//  FlexLin1_Uart_Buffer_Init(100,38400);
//    FlexLin1_Uart_FIFO_Init(80,19200);

//  InitLINFlexD0(100,19200);
  // Init the PIT0
//  PIT_0.MCR.B.MDIS = 0; /* Enable PIT module. NOTE: PIT module must be       */
//                        /* enabled BEFORE writing to it's registers.         */
//                        /* Other cores will write to PIT registers so the    */
//                        /* PIT is enabled here before starting other cores.  */
//   PIT_0.MCR.B.FRZ = 1;  /* Freeze PIT timers in debug mode */
//   PIT0_init(800000);
//   /* timeout= 0.8M  PITclks x 4 sysclks/1 PITclk x 1 sec/160Msysck */
//   /*        = 0.8M x 4 / 160M = 3.2/160 = 0.02 sec.  */
//   PIT_0.MCR.B.FRZ = 0; //Unfreeze timers

   /* Loop forever */
//   m_LIN_Packet.id = 0x35;
//   m_LIN_Packet.length = 8;
//
//   m_LIN_Packet.BufferData.B.Data0 = 0x1;
//   m_LIN_Packet.BufferData.B.Data1 = 0x2;
//   m_LIN_Packet.BufferData.B.Data2 = 0x3;
//   m_LIN_Packet.BufferData.B.Data3 = 0x4;
//   m_LIN_Packet.BufferData.B.Data4 = 0x5;
//   m_LIN_Packet.BufferData.B.Data5 = 0x6;
//   m_LIN_Packet.BufferData.B.Data6 = 0x7;
//   m_LIN_Packet.BufferData.B.Data7 = 0x8;

//   m_LIN_Packet.BufferData.L.B.Data0 = 0x1;
//   m_LIN_Packet.BufferData.L.B.Data1 = 0x1;
//   m_LIN_Packet.BufferData.L.B.Data2 = 0x1;
//   m_LIN_Packet.BufferData.L.B.Data3 = 0x1;
//   m_LIN_Packet.BufferData.M.B.Data4 = 0x1;
//   m_LIN_Packet.BufferData.M.B.Data5 = 0x1;
//   m_LIN_Packet.BufferData.M.B.Data6 = 0x1;

//   m_LIN_Packet.BufferData.Data[0]=0x1;
//   m_LIN_Packet.BufferData.Data[1]=0x2;
//   m_LIN_Packet.BufferData.Data[2]=0x3;
//   m_LIN_Packet.BufferData.Data[3]=0x4;
//   m_LIN_Packet.BufferData.Data[4]=0x5;
//   m_LIN_Packet.BufferData.Data[5]=0x6;
//   m_LIN_Packet.BufferData.Data[6]=0x7;
//   m_LIN_Packet.BufferData.Data[7]=0x8;

//   LIN0_TransmitFrame(m_LIN_Packet);
   for(;;)
   {
//     SYSTEM_LED = LED_ON;
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
	cnt = (cnt + 1) % 5;
//	if(cnt == 0)
//	{
//		TerminalSendFlag = true;
//		m_Ultrasonic.InitSensing_STP318(1, 1);
//	}
//	if(cnt == 3)
//	{
//		m_LIN_STP318_Packet = m_Ultrasonic.ReadData_STP318(0xCf);
//	}
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

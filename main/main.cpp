/*
 * main implementation: use this 'C++' sample to create your own application
 *
 */

#include "PathPlanning.h"
#include "Vehicle.h"
#include "mode_entry.h"
#include "can.h"
#include "pit.h"
#include "uart.h"
#include "derivative.h" /* include peripheral declarations */


#ifdef __cplusplus
extern "C" {
#endif

extern void xcptn_xmpl(void);

#ifdef __cplusplus
}
#endif

void peri_clock_gating(void);

CAN_Packet m_TestPacket;

__attribute__ ((section(".text")))
int main()
{
	uint8_t i;
	Vehicle m_PathPlanning;
//	m_PathPlanning.SteeringWheelTargetAngle = 30;

    xcptn_xmpl();	/* Configure and Enable Interrupts */
    peri_clock_gating();
    system160mhz();	/* sysclk=160MHz, dividers configured, mode trans*/
    // Flex CAN0
    FlexCAN0_Init();

    // Flex Lin1 Uart
    FlexLin1_Uart_Init(80,19200);
//    FlexLin1_DMA_TX_Init();
//    FlexLin1_DMA_RX_Init();

    // PIT0
    PIT_0.MCR.B.MDIS = 0; /* Enable PIT module. NOTE: PIT module must be       */
                          /* enabled BEFORE writing to it's registers.         */
                          /* Other cores will write to PIT registers so the    */
                          /* PIT is enabled here before starting other cores.  */
	PIT_0.MCR.B.FRZ = 1;  /* Freeze PIT timers in debug mode */
	PIT0_init(800000);
    /* timeout= 0.8M  PITclks x 4 sysclks/1 PITclk x 1 sec/160Msysck */
    /*        = 0.8M x 4 / 160M = 3.2/160 = 0.02 sec.  */
    PIT_0.MCR.B.FRZ = 0; //Unfreeze timers

    m_TestPacket.id = 0x123;
    m_TestPacket.length = 8;
    for(i=0;i<m_TestPacket.length;i++)
    {
    	m_TestPacket.data[i] = i;
    }
    DMA_0.TCD[16].CSR.B.START = 1;
    /* Loop forever */
	for(;;)
	{

	}
}

void peri_clock_gating(void)
{
  MC_ME.RUN_PC[0].R = 0x00000000;  /* gate off clock for all RUN modes */
  MC_ME.RUN_PC[1].R = 0x000000FE;  /* config. peri clock for all RUN modes */

  MC_ME.PCTL79.B.RUN_CFG  = 0b001; //FlexCAN 0: select peripheral config RUN_PC[1]
  MC_ME.PCTL30.B.RUN_CFG  = 0b001; //PCTL30 is PIT0 Peripheral Control Registers for Panther
  MC_ME.PCTL91.B.RUN_CFG  = 0b001; //LINFlexD_1: Select peripheral config RUN_PC[1]. No LINFlex_D_2 on Panther
  MC_ME.PCTL146.B.RUN_CFG = 0b001;
}

#ifdef __cplusplus
extern "C" {
#endif
void PIT0_isr(void)
{
	CAN0_TransmitMsg(m_TestPacket);

	TransmitData(0xaa);
	PIT_0.TIMER[0].TFLG.R |= 1;  /* Clear interrupt flag. w1c */
}
#ifdef __cplusplus
}
#endif


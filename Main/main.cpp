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

// 系统外设配置
#include "derivative.h" /* include peripheral declarations */
// 传感器驱动
#include "Ultrasonic.h"
// 终端交互
#include "Terminal.h"
// 车辆轨迹跟踪
#include "../Common/VehicleState/GeometricTrack/geometric_track.h"
// 车辆信息解码和控制
#ifdef CHANGAN
#include "ChangAn/chang_an_controller.h"
#include "ChangAn/chang_an_message.h"
#endif

#ifdef BORUI
#include "BoRui/bo_rui_controller.h"
#include "BoRui/bo_rui_message.h"
#endif
// 车辆控制
#include "pid.h"
#include "lon_control.h"
// 规划
#include "parallel_planning.h"
#include "vertical_planning.h"
// 感知
#include "percaption.h"
#include "ultrasonic_obstacle_percption.h"
//
#include "link_list.h"

#ifdef __cplusplus
extern "C" {
#endif
extern void xcptn_xmpl(void);
#ifdef __cplusplus
}
#endif

/****************System Variable******************/
Terminal m_Terminal_CA;
Ultrasonic m_Ultrasonic;
GeometricTrack m_GeometricTrack;
LonControl m_LonControl;
PID m_VehicleVelocityControlPID = PID(0.02,3.5,0.1,0.1,0.3,1,0.1);

#ifdef CHANGAN
ChangAnController m_ChangAnController;
ChangAnMessage m_ChangAnMessage;
#endif
#ifdef BORUI
BoRuiController m_BoRuiController;
BoRuiMessage    m_BoRuiMessage;
#endif

ParallelPlanning m_ParallelPlanning;
VerticalPlanning m_VerticalPlanning;

Percaption m_PercaptionInformation;
UltrasonicObstaclePercption m_UltrasonicObstaclePercption;

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
			//Task 一次性的计算任务 泊车规划任务
			if(0x10 == m_Terminal_CA.Command)//平行泊车
			{
				if(0x50 == m_ParallelPlanning.Command)
				{
					m_GeometricTrack.Init(&m_PercaptionInformation);
					m_ParallelPlanning.Init();
					m_ParallelPlanning.Command = 0x60;
				}
				else if(0x80 == m_ParallelPlanning.Command)//泊车结束
				{
#ifdef CHANGAN
					m_ChangAnController.Stop();
#endif
					m_ParallelPlanning.Init();
					m_ParallelPlanning.Command = 0x00;
				}
				else
				{

				}
				m_ParallelPlanning.Work(&m_PercaptionInformation);
			}
			else if(0x20 == m_Terminal_CA.Command)//垂直泊车
			{
				if(0x50 == m_VerticalPlanning.Command)
				{
					m_GeometricTrack.Init(&m_PercaptionInformation);
					m_VerticalPlanning.Init();
					m_VerticalPlanning.Command = 0x60;
				}
				else if(0x80 == m_VerticalPlanning.Command)//泊车结束
				{
#ifdef CHANGAN
					m_ChangAnController.Stop();
#endif
					m_VerticalPlanning.Init();
					m_VerticalPlanning.Command = 0x00;
				}
				else
				{

				}
				if(SUCCESS == m_UltrasonicObstaclePercption.ParkingCalculateStateMachine())
				{
					m_Terminal_CA.Push(&m_UltrasonicObstaclePercption);
					m_VerticalPlanning.Command = 0x61;
				}
				m_VerticalPlanning.Work(&m_PercaptionInformation,&m_GeometricTrack);
			}
			else if(0x30 == m_Terminal_CA.Command)//斜向泊车
			{

			}
			else if(0xA0 == m_Terminal_CA.Command)//检车位
			{

			}
			else if(0xB0 == m_Terminal_CA.Command)//停止检车位
			{

			}
			else if(0xC0 == m_Terminal_CA.Command)//障碍物定位
			{
				if(SUCCESS == m_UltrasonicObstaclePercption.ParkingCalculateStateMachine())
				{
					m_Terminal_CA.Push(&m_UltrasonicObstaclePercption);
				}
//				if(0xA5 == m_Terminal_CA.AckValid)
//				{
//					m_UltrasonicObstaclePercption.DataPushStateMachine(&m_Ultrasonic);
//				}
			}
			else//车辆
			{

			}
			if(0xA5 == m_Terminal_CA.PushActive)//数据推送内容,5ms进行一次推送
			{
				m_Terminal_CA.PushActive = 0;
				m_Terminal_CA.Push(&m_Ultrasonic);//5ms

				if(m_Ultrasonic.SystemTime % 4 == 0)//20ms
				{
#ifdef CHANGAN
					m_Terminal_CA.Push(&m_ChangAnController);
#endif
#ifdef BORUI
					m_Terminal_CA.Push(&m_BoRuiController);
#endif
				}
				if(m_Ultrasonic.SystemTime % 4 == 1)//20ms
				{
					m_Terminal_CA.Push(&m_GeometricTrack);
					m_Terminal_CA.Push(m_GeometricTrack);
				}
				if(m_Ultrasonic.SystemTime % 4 == 2)//20ms
				{
#ifdef CHANGAN
					m_Terminal_CA.Push(&m_ChangAnMessage);
#endif
#ifdef BORUI
					m_Terminal_CA.Push(&m_BoRuiMessage);
#endif
				}
				if(m_Ultrasonic.SystemTime % 4 == 3)//20ms
				{
					if(0x10 == m_Terminal_CA.Command)
					{
						m_Terminal_CA.Push(&m_ParallelPlanning);
					}
					else if(0x20 == m_Terminal_CA.Command)
					{
						m_Terminal_CA.Push(&m_VerticalPlanning);
					}
					//推送障碍物检测信息
					m_Terminal_CA.Push(m_UltrasonicObstaclePercption);
				}
			}
			// 终端应答信号
			if(0xA5 == m_Terminal_CA.AckValid)
			{
				m_Terminal_CA.Ack();
				if(0x5A == m_Terminal_CA.AckEcho)
				{
					m_Terminal_CA.AckEcho = 0;
					m_Terminal_CA.AckValid = 0;
				}
			}
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
	if(m_Ultrasonic.SystemTime % 4 == 0)//20ms
	{
#ifdef CHANGAN
		if(0x10 == m_Terminal_CA.Command)
		{
			m_ParallelPlanning.Control(&m_ChangAnController, &m_ChangAnMessage, &m_GeometricTrack, &m_Ultrasonic);
		}
		else if(0x20 == m_Terminal_CA.Command)
		{
			m_VerticalPlanning.Control(&m_ChangAnController, &m_ChangAnMessage, &m_GeometricTrack, &m_Ultrasonic);
		}
#endif
#ifdef BORUI
		if(0x10 == m_Terminal_CA.Command)
		{
			m_ParallelPlanning.Control(&m_BoRuiController, &m_BoRuiMessage, &m_GeometricTrack, &m_Ultrasonic);
		}
		else if(0x20 == m_Terminal_CA.Command)
		{
			m_VerticalPlanning.Control(&m_BoRuiController, &m_BoRuiMessage, &m_GeometricTrack, &m_UltrasonicObstaclePercption);
		}
#endif
	}
	if(m_Ultrasonic.SystemTime % 4 == 1)//20ms
	{
		// TODO 检车位测试时可以屏蔽
		#ifdef CHANGAN
//		m_LonControl.Proc(&m_ChangAnMessage, &m_ChangAnController, &m_VehicleVelocityControlPID);//20ms
//		m_ChangAnController.SteeringAngleControlStateMachine(m_ChangAnMessage.APA_ControlFeedback);
//		m_ChangAnController.Push(0.02);
		#endif

		#ifdef BORUI
		m_BoRuiController.Push(0.02);
		#endif
	}
	if(m_Ultrasonic.SystemTime % 4 == 2)//20ms
	{
#ifdef CHANGAN
		#if 1 == SIMULATION
		m_GeometricTrack.VelocityUpdate(&m_ChangAnMessage,0.02);
		#else
		m_GeometricTrack.PulseUpdate(&m_ChangAnMessage);
		#endif
#endif
#ifdef BORUI
		#if 1 == SIMULATION
		m_GeometricTrack.VelocityUpdate(&m_BoRuiMessage,0.02);
		#else
		m_GeometricTrack.PulseUpdate(&m_BoRuiMessage);
		// 超声波避障功能
		m_UltrasonicObstaclePercption.UltrasonicCollisionDiatance(&m_Ultrasonic,&m_BoRuiMessage);
#endif
#endif
	}
	if(m_Ultrasonic.SystemTime % 4 == 3)//20ms
	{

	}

#if ULTRASONIC_SCHEDULE_MODO == 2
	m_Ultrasonic.UltrasonicScheduleStatusMachine_V2();//5ms
	m_Ultrasonic.Update(25);
	m_Ultrasonic.ScheduleTimeCnt = (m_Ultrasonic.ScheduleTimeCnt + 1) % 26;
#endif
#if ULTRASONIC_SCHEDULE_MODO == 3
	m_Ultrasonic.UltrasonicScheduleStatusMachine_V3();//5ms
	m_Ultrasonic.Update(25);
	m_Ultrasonic.BodyDirectLocation();
	m_Ultrasonic.BodyTriangleLocation();
	m_Ultrasonic.GroundTriangleLocation(&m_GeometricTrack);
	/*
	 * 障碍物检测的库位定位状态机
	 * */
	m_UltrasonicObstaclePercption.DataPushStateMachine(&m_Ultrasonic);
	m_Ultrasonic.ScheduleTimeCnt = (m_Ultrasonic.ScheduleTimeCnt + 1) % 28;
#endif

	m_Ultrasonic.SystemTime = m_Ultrasonic.SystemTime + 1;
	m_Terminal_CA.PushActive = 0xA5;
	if(m_Ultrasonic.ScheduleTimeCnt == 0)
	{
		SYSTEM_LED = ~SYSTEM_LED;
	}
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
#ifdef CHANGAN
		m_ChangAnMessage.Parse(CAN_0.MB[8].ID.B.ID_STD, CAN_0.MB[8].DATA.B, CAN_0.MB[8].CS.B.DLC);
#endif

#ifdef BORUI
		m_BoRuiMessage.Parse(CAN_0.MB[8].ID.B.ID_STD, CAN_0.MB[8].DATA.B, CAN_0.MB[8].CS.B.DLC);
#endif
		/* release the internal lock for all Rx MBs
		 * by reading the TIMER */
		uint32_t temp = CAN_0.TIMER.R;
		if(!temp){}
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
		/* release the internal lock for all Rx MBs
		 * by reading the TIMER */
		uint32_t temp = CAN_1.TIMER.R;
		if(!temp){}
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
		// terminal command decode
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B);
		// PID Control
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B,&m_VehicleVelocityControlPID);
#ifdef CHANGAN
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, &m_ChangAnController);
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, &m_ChangAnMessage);
#endif

#ifdef BORUI
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, &m_BoRuiController);
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, &m_BoRuiMessage);
#endif
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, &m_UltrasonicObstaclePercption);
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, &m_Ultrasonic);

		if(0x10 == m_Terminal_CA.Command)
		{
			m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, &m_ParallelPlanning);
			m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, &m_PercaptionInformation, &m_ParallelPlanning);
		}
		else if(0x20 == m_Terminal_CA.Command)
		{
			m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, &m_VerticalPlanning);
			m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, &m_PercaptionInformation, &m_VerticalPlanning);
		}
		else
		{

		}
		/* release the internal lock for all Rx MBs
		 * by reading the TIMER */
		uint32_t temp = CAN_2.TIMER.R;
		if(!temp){}
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
	DMA_0.INT.B.INT18 = 1;
}
#ifdef __cplusplus
}
#endif

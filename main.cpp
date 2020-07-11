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
#include "Driver/System/derivative.h" /* include peripheral declarations */
// 传感器驱动
#include "Interaction/Ultrasonic/Ultrasonic.h"
// 终端交互
#include "Interaction/HMI/Terminal.h"
// 车辆轨迹跟踪
#include "Common/VehicleState/GeometricTrack/geometric_track.h"
// 车辆信息解码和控制
#ifdef CHANGAN
#include "ChangAn/chang_an_controller.h"
#include "ChangAn/chang_an_message.h"
#endif

#ifdef BORUI
#include "Interaction/CANBUS/BoRui/bo_rui_controller.h"
#include "Interaction/CANBUS/BoRui/bo_rui_message.h"
#endif

#ifdef DONG_FENG_E70
#include "Interaction/CANBUS/DongFengE70/dong_feng_e70_controller.h"
#include "Interaction/CANBUS/DongFengE70/dong_feng_e70_message.h"
#endif

#ifdef CHERY_S51EV
#include "Interaction/CANBUS/CheryS51EV/chery_s51ev_controller.h"
#include "Interaction/CANBUS/CheryS51EV/chery_s51ev_Message.h"
#endif
// 车辆控制
#include "Control/Common/pid.h"
#include "Control/LonControl/lon_control.h"
#include "Control/LatControl/lat_control.h"
// math
#include "Common/Utils/Inc/link_list.h"
#include "Common/Math/interpolation.h"

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
LatControl m_LatControl;
TrackLinkList *_target_curvature_data_sets;
TargetTrack temp_node;
TargetTrack end_node;
/**********************************************************************/
#ifdef BORUI
//速度PID参数
PID m_VelocityUpdatePID  = PID(0.02,0.01f,0.0f,0.0f,0.0f,1,0.2);
#endif
#ifdef DONG_FENG_E70
//正向PID取消积分项
PID m_VelocityControlPID = PID(0.02f,2.0f,0.0f,0.0f,0.6,0.6f,0.1f);
PID m_VelocityStratControlPID  = PID(0.02f,1.8f,0.1f,0.3f,0.2f,0.6f,0.2f);
#endif
/**********************************************************************/
#ifdef BORUI
BoRuiController m_Vehicle_Controller;
BoRuiMessage    m_Vehicle_Message;
#endif
#ifdef DONG_FENG_E70
DongFengE70Controller m_Vehicle_Controller;
DongFengE70Message    m_Vehicle_Message;
#endif
#ifdef CHERY_S51EV
CheryS51EV_Controller m_Vehicle_Controller;
CheryS51EV_Message    m_Vehicle_Message;
#endif
/**********************************************************************/
uint8_t ultrasonic_gain_adjust_flag = 0;

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
		/************************************** 周期控制任务 ******************************************/
		if(0xA5 == m_Terminal_CA.PushActive)//数据推送内容,5ms进行一次推送
		{
			m_Terminal_CA.PushActive = 0;
			//测试控制代码
			if(m_Ultrasonic.SystemTime % 4 == 0)//20ms
			{
				m_Vehicle_Controller.WorkStateMachine(m_Vehicle_Message);
				m_Vehicle_Controller.DataPush();
			}
			else if(m_Ultrasonic.SystemTime % 2 == 1)//20ms
			{
				m_GeometricTrack.VelocityPulseUpdate(m_Vehicle_Message);
			}
			else if(m_Ultrasonic.SystemTime % 4 == 2)//20ms
			{
				
			}
			else if(m_Ultrasonic.SystemTime % 4 == 3)//20ms
			{

			}
			else
			{
				
			}

			/***********************************控制台消息推送***********************************************/
			m_Terminal_CA.Push(m_Ultrasonic);//5ms
			if(m_Ultrasonic.SystemTime % 4 == 0)//20ms
			{
				m_Terminal_CA.Push(m_Vehicle_Controller);
			}
			else if(m_Ultrasonic.SystemTime % 4 == 1)//20ms
			{
				m_Terminal_CA.Push(m_GeometricTrack);
			}
			else if(m_Ultrasonic.SystemTime % 4 == 2)//20ms
			{
				m_Terminal_CA.Push(m_Vehicle_Message);
			}
			else if(m_Ultrasonic.SystemTime % 4 == 3)//20ms
			{
				m_Terminal_CA.Push(m_LonControl);
				m_Terminal_CA.Push(m_LatControl);
			}
			else
			{
				
			}
			/********************************温度补偿处理**********************************************************/
			#ifdef BORUI
			if(0xAA != ultrasonic_gain_adjust_flag)
			{
				if(0xA5 == m_Vehicle_Message.getAmbientTemperatureValid())
				{
					m_Ultrasonic.GainConfigure(m_Vehicle_Message.getAmbientTemperature());
					ultrasonic_gain_adjust_flag = 0xAA;
				}
			}
			#endif
		}
		// 终端应答信号
		if(0xA5 == m_Terminal_CA.AckValid)
		{
			m_Terminal_CA.Ack();
			m_Terminal_CA.AckValid = 0;
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
	#ifdef BORUI
		if(m_Vehicle_Controller.getAPAEnable())
		{
			if(m_Vehicle_Controller.getShakeHandsCnt() < 50)
			{
				m_Vehicle_Controller.setShakeHandsCnt(m_Vehicle_Controller.getShakeHandsCnt() + 1);
			}
			else
			{
				m_Vehicle_Controller.setVelocity(0);
				if(m_Vehicle_Message.getVehicleMiddleSpeed() < 1.0e-6)
				{
					m_Vehicle_Controller.setDistance(0);
					if(m_Vehicle_Controller.getShakeHandsCnt() < 150)
					{
						m_Vehicle_Controller.setShakeHandsCnt(m_Vehicle_Controller.getShakeHandsCnt() + 1);
					}
					else
					{
						m_Vehicle_Controller.setGear(Parking);
					}
					if(Parking == m_Vehicle_Message.getGear())
					{
						m_Vehicle_Controller.setAPAEnable(0);
					}
				}
			}
		}
		else
		{
			m_Vehicle_Controller.setShakeHandsCnt(0);
		}
	#endif


#if ULTRASONIC_SCHEDULE_MODO == 2
	m_Ultrasonic.UltrasonicScheduleStatusMachine_V2();//5ms
	m_Ultrasonic.Update(25);
	m_Ultrasonic.ScheduleTimeCnt = (m_Ultrasonic.ScheduleTimeCnt + 1) % 26;
#endif
#if ULTRASONIC_SCHEDULE_MODO == 3
	m_Ultrasonic.UltrasonicScheduleStatusMachine_V3();//5ms
	m_Ultrasonic.Update(m_Vehicle_Message.getAmbientTemperature());
	m_Ultrasonic.BodyDirectLocation();
	m_Ultrasonic.BodyTriangleLocation();
	m_Ultrasonic.GroundTriangleLocation(&m_GeometricTrack);
	m_Ultrasonic.ScheduleTimeCnt = (m_Ultrasonic.ScheduleTimeCnt + 1) % 28;
#endif

#if ULTRASONIC_SCHEDULE_MODO == 4
	if(4 == m_Terminal_CA.getWorkMode())
	{
		if(0 == m_Terminal_CA.getFunctionState())
		{
			m_Ultrasonic.UltrasonicScheduleStatusMachineType2_V3();//6ms
			m_Ultrasonic.DateUpdateType2_V3(25);
			m_Ultrasonic.setScheduleTimeCnt((m_Ultrasonic.getScheduleTimeCnt() + 1) % 32);
		}
		else if(1 == m_Terminal_CA.getFunctionState())
		{
			m_Ultrasonic.UltrasonicScheduleStatusMachineType1_V3();//6ms
			m_Ultrasonic.DateUpdateType1_V3(25);
			m_Ultrasonic.BodyDirectLocationType1();
			m_Ultrasonic.BodyTriangleLocationType1();
			m_Ultrasonic.setScheduleTimeCnt((m_Ultrasonic.getScheduleTimeCnt() + 1) % 22);
		}
	}
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
		m_Vehicle_Message.Parse(CAN_0.MB[8].ID.B.ID_STD, CAN_0.MB[8].DATA.B, CAN_0.MB[8].CS.B.DLC);
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
		//m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B,&m_VelocityControlPID);
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, m_Vehicle_Controller);
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, m_Vehicle_Message);
		m_Terminal_CA.Parse(CAN_2.MB[8].ID.B.ID_STD,CAN_2.MB[8].DATA.B, m_Ultrasonic);

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

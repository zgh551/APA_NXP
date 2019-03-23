/*
 * ultrasonic.h
 *
 *  Created on: January 8 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: ultrasonic.h                        COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this module process the ultrasonic data				         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 8 2019      Initial Version                  */
/*****************************************************************************/

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

//#include "../HMI/Terminal.h"
#include "derivative.h"
#include "property.h"
// math
#include "math.h"
#include "vector_2d.h"
#include "vehilce_config.h"
// Track
#include "../../VehicleState/Interface/vehicle_state.h"

#define FRONT_ULTRASONIC_ENABLE
#define REAR_ULTRASONIC_ENABLE

#define LEVEL_RATIO    (0.01294117647058823529411764705882)
#define WIDTH_RATIO    (16)

/************************超声发送格式按钮*************************/
#define ULTRASONIC_PACKET         ( 1 ) // 超声包格式
#define ULTRASONIC_SCHEDULE_MODO  ( 3 ) // 超声调度模式

/*** LIN Device Data Struct ***/
typedef struct _LIN_STP318_Packet
{
	uint16_t TOF;
	uint8_t status;
}LIN_STP318_Packet;

typedef struct _LIN_STP313_Packet
{
	uint16_t TOF1;
	uint16_t TOF2;
	uint8_t Level;
	uint8_t Width;
	uint8_t status;
}LIN_STP313_Packet;

typedef struct _Ultrasonic_Packet
{
	float Distance1;
	float Distance2;
	float Level;
	float Width;
	float Time_Ms;
	uint8_t status;
	uint32_t Time_Tx;
}Ultrasonic_Packet;

class Ultrasonic {
public:
	Ultrasonic();
	virtual ~Ultrasonic();

	void Init(void);

	void InitSensing_STP318(uint8_t tx,uint8_t rx,void (*TransmitFrame)(LIN_RAM));
	void InitSensing_STP313(uint8_t tx_rx,void (*TransmitFrame)(LIN_RAM));

	void ReadSensing_STP318(uint8_t id,void (*ReceiveFrame)(LIN_RAM *),LIN_RAM *m_LIN_RAM);
	void ReadSensing_STP313(uint8_t id,void (*ReceiveFrame)(LIN_RAM *),LIN_RAM *m_LIN_RAM);

	void InitUltrasonicSensor(uint8_t n);
	void InitUltrasonicSensorRx(uint8_t n);
	void ReadUltrasonicSensor(uint8_t n);
	void ReadUltrasonicSensorRxs(uint8_t step,uint8_t n);

	void UltrasonicScheduleStatusMachine(void);
	void UltrasonicScheduleStatusMachine_V2(void);
	void UltrasonicScheduleStatusMachine_V3(void);

	void UltrasonicConvert(uint8_t type,LIN_RAM d,Ultrasonic_Packet *p,float t);
	float Compensation(float temp);


	void Update(uint8_t id,float t);
	void Update(float t);

	// 正常测距的坐标转换
	void VehicleAxisAbstacleDirectCalculate(Location l,float d,Vector2d *p);
	/*
	 * 三角定位计算障碍物位置
	 * */
	void VehicleAxisAbstacleTriangleCalculate(int8_t d,Location a,Location b,float ul,float ur,Vector2d *p);

	void TriangleLocation();

	void DirectLocation();

	void GroundAxisAbstacleTriangleCalculate(VehicleState *s,Vector2d v,Vector2d *g);

	void TriangleLocationGround(VehicleState *s);

	/// Property
	uint8_t getScheduleTimeCnt();
	void    setScheduleTimeCnt(uint8_t value);
	Property<Ultrasonic,uint8_t,READ_WRITE> ScheduleTimeCnt;

	uint8_t getReadStage();
	void    setReadStage(uint8_t value);
	Property<Ultrasonic,uint8_t,READ_WRITE> ReadStage;

	uint32_t getSystemTime();
	void     setSystemTime(uint32_t value);
	Property<Ultrasonic,uint32_t,READ_WRITE> SystemTime;



	LIN_RAM* getUltrasonicDatas();
	Property<Ultrasonic,LIN_RAM*,READ_ONLY> UltrasonicDatas;

	LIN_RAM* getUltrasonicLocationDatas();
	Property<Ultrasonic,LIN_RAM*,READ_ONLY> UltrasonicLocationDatas;

	Ultrasonic_Packet* getUltrasonicPacket();
	void setUltrasonicPacket(uint8_t n,Ultrasonic_Packet p);
	Property<Ultrasonic,Ultrasonic_Packet*,READ_ONLY> UltrasonicPacket;

	Ultrasonic_Packet* getUltrasonicLocationPacket();
	void setUltrasonicLocationPacket(uint8_t n,Ultrasonic_Packet p);
	Property<Ultrasonic,Ultrasonic_Packet*,READ_ONLY> UltrasonicLocationPacket;

	Vector2d* getAbstaclePositionDirect();
	Property<Ultrasonic,Vector2d*,READ_ONLY> AbstaclePositionDirect;

	Vector2d* getAbstaclePositionTriangle();
	Property<Ultrasonic,Vector2d*,READ_ONLY> AbstaclePositionTriangle;

	Vector2d* getAbstacleGroundPositionTriangle();
	Property<Ultrasonic,Vector2d*,READ_ONLY> AbstacleGroundPositionTriangle;
private:
	uint32_t _system_time;
	uint8_t  _schedule_time_cnt;
	uint8_t  _read_stage;

	LIN_RAM _ultrasonic_datas[12];

	LIN_RAM _ultrasonic_location_datas[12];

	Ultrasonic_Packet _ultrasonic_packet[12];

	Ultrasonic_Packet _ultrasonic_location_packet[12];


	VehilceConfig _abstacle_config;

	Vector2d _abstacle_position_direct[12];
	Vector2d _abstacle_position_triangle[8];

	Vector2d _abstacle_ground_position_triangle[8];
};

#endif /* ULTRASONIC_H_ */

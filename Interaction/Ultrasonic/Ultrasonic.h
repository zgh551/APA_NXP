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
typedef enum _UltrasonicStatus
{
    Normal = 0,
    BlindZone,
    OverDetection,
    Noise,
	InvalidPoint
}UltrasonicStatus;

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

typedef struct _ObstacleLocationPacket
{
	Vector2d Position;
	UltrasonicStatus  Status;
}ObstacleLocationPacket;

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
	/*
	 * 数据更新 定时器触发
	 * */
	void Update(float t);

	/*
	 * 直接测量数据的传感器坐标系与载体坐标系的转换
	 * position: 传感器的安装坐标
	 * data    : 超声波数据
	 * location: 障碍物相对车体的位置
	 * */
	void BodyDirectCalculate(Location position,Ultrasonic_Packet data,ObstacleLocationPacket *location);
	/*
	 * 三角定位测量值 由传感器坐标系转到载体坐标系
	 * type:三角顶点朝向；1 -> 朝上   0 -> 朝下
	 * position_a:传感器安装位置a
	 * position_b:传感器安装位置b
	 * data_ul:三角定位测量的左边长值
	 * data_ur:三角定位测量的右边长值
	 * location:障碍物定位坐标
	 * */
	void BodyTriangleCalculate(int8_t type,Location position_a,Location position_b,Ultrasonic_Packet data_ul,Ultrasonic_Packet data_ur,ObstacleLocationPacket *location);

	/*
	 * 三角定位地面坐标系的转换
	 * vehicle:车辆状态信息
	 * body   :障碍物相对于载体坐标系的坐标
	 * ground :障碍物相对于地面坐标系的坐标
	 * */
	void GroundTriangleCalculate(VehicleState *vehicle,ObstacleLocationPacket body,ObstacleLocationPacket *ground);

	/*
	 * 直接测量数据的车辆坐标定位
	 * */
	void BodyDirectLocation();

	/*
	 * 三角定位的车辆坐标定位
	 * */
	void BodyTriangleLocation();

	/*
	 * 地面坐标超声波数据定位
	 * */
	void GroundTriangleLocation(VehicleState *vehicle_state);

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

	ObstacleLocationPacket* getAbstacleBodyPositionDirect();
	Property<Ultrasonic,ObstacleLocationPacket*,READ_ONLY> AbstacleBodyPositionDirect;

	ObstacleLocationPacket* getAbstacleBodyPositionTriangle();
	Property<Ultrasonic,ObstacleLocationPacket*,READ_ONLY> AbstacleBodyPositionTriangle;

	ObstacleLocationPacket* getAbstacleGroundPositionTriangle();
	void setAbstacleGroundPositionTriangle(uint8_t n,ObstacleLocationPacket p);
	Property<Ultrasonic,ObstacleLocationPacket*,READ_ONLY> AbstacleGroundPositionTriangle;
private:
	uint32_t _system_time;
	uint8_t  _schedule_time_cnt;
	uint8_t  _read_stage;

	LIN_RAM _ultrasonic_datas[12];

	LIN_RAM _ultrasonic_location_datas[12];

	Ultrasonic_Packet _ultrasonic_packet[12];

	Ultrasonic_Packet _ultrasonic_location_packet[12];

	ObstacleLocationPacket _abstacle_body_position_direct[12];

	ObstacleLocationPacket _abstacle_body_position_triangle[12];
	ObstacleLocationPacket _abstacle_ground_position_triangle[12];

	VehilceConfig _abstacle_config;
};

#endif /* ULTRASONIC_H_ */

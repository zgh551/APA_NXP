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
#include "../../Driver/System/derivative.h"
#include "../../Common/Utils/Inc/property.h"
// math
#include "math.h"
#include "../../Common/Math/vector_2d.h"
#include "../../Common/Configure/Configs/vehicle_config.h"
// Track
#include "../../Common/VehicleState/Interface/vehicle_state.h"

#define FRONT_ULTRASONIC_ENABLE
#define REAR_ULTRASONIC_ENABLE

#define LEVEL_RATIO    (0.01294117647058823529411764705882)
#define WIDTH_RATIO    (16)

/************************瓒呭０鍙戦�佹牸寮忔寜閽�*************************/
#define ULTRASONIC_PACKET         ( 1 ) // 瓒呭０鍖呮牸寮�
#define ULTRASONIC_SCHEDULE_MODO  ( 3 ) // 瓒呭０璋冨害妯″紡

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

typedef struct _ParkingEdgeBufferLocationPacket
{
	Vector2d Position;
	Ultrasonic_Packet  UltrasonicData;
}ParkingEdgeBufferLocationPacket;

class Ultrasonic {
public:
	Ultrasonic();
	virtual ~Ultrasonic();

	void Init(void);

	void GainConfigure(float t);

	int8_t GainAdjustmentTemperatureCompensation_STP318(float t);
	int8_t GainAdjustmentTemperatureCompensation_STP313(float t);

	void GainAdj_STP318(int8_t s, int8_t m, int8_t e, void (*TransmitFrame)(LIN_RAM));
	void GainAdj_STP313(int8_t s, int8_t m, int8_t e, void (*TransmitFrame)(LIN_RAM));

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

	/*
	 * 璋冨害鐘舵�佺被鍨�1: 澶氬彂澶氭敹璋冨害,閽堝瀹氭椂鍣ㄤ负6ms鏃剁殑浼樺寲
	 * */
	void UltrasonicScheduleStatusMachineType1_V3(void);

	/*
	 * 璋冨害鐘舵�佺被鍨�2: 浼樺厛闀胯窛璋冨害锛屽畾鏃�6ms锛屾渶灏忓寲璋冨害鍛ㄦ湡
	 */
	void UltrasonicScheduleStatusMachineType2_V3(void);

	void UltrasonicConvert(uint8_t type,LIN_RAM d,Ultrasonic_Packet *p,float t);
	float Compensation(float temp);


	void Update(uint8_t id,float t);
	/*
	 * 鏁版嵁鏇存柊 瀹氭椂鍣ㄨЕ鍙�
	 * */
	void Update(float t);

	/*
	 * 鏁版嵁鏇存柊绫诲瀷1 锛氫笁瑙掑畾浣嶅叏閮ㄦ洿鏂帮紝浣挎暣涓懆鏈熸椂闂存渶鐭�
	 */
	void DateUpdateType1_V3(float t);

	/*
	 * 鏁版嵁鏇存柊绫诲瀷2 锛氬浜庨暱璺濅紶鎰熷櫒閲囬泦浼樺厛绛栫暐锛屼繚璇侀暱璺濅紶鎰熷櫒鍗曚釜鍛ㄦ湡鏈�灏�
	 */
	void DateUpdateType2_V3(float t);

	/*
	 * 鐩存帴娴嬮噺鏁版嵁鐨勪紶鎰熷櫒鍧愭爣绯讳笌杞戒綋鍧愭爣绯荤殑杞崲
	 * position: 浼犳劅鍣ㄧ殑瀹夎鍧愭爣
	 * data    : 瓒呭０娉㈡暟鎹�
	 * location: 闅滅鐗╃浉瀵硅溅浣撶殑浣嶇疆
	 * */
	void BodyDirectCalculate(Location position,Ultrasonic_Packet data,ObstacleLocationPacket *location);
	/*
	 * 涓夎瀹氫綅娴嬮噺鍊� 鐢变紶鎰熷櫒鍧愭爣绯昏浆鍒拌浇浣撳潗鏍囩郴
	 * type:涓夎椤剁偣鏈濆悜锛�1 -> 鏈濅笂   0 -> 鏈濅笅
	 * position_a:浼犳劅鍣ㄥ畨瑁呬綅缃產
	 * position_b:浼犳劅鍣ㄥ畨瑁呬綅缃産
	 * data_ul:涓夎瀹氫綅娴嬮噺鐨勫乏杈归暱鍊�
	 * data_ur:涓夎瀹氫綅娴嬮噺鐨勫彸杈归暱鍊�
	 * location:闅滅鐗╁畾浣嶅潗鏍�
	 * */
	void BodyTriangleCalculate(Location position_a,Location position_b,Ultrasonic_Packet data_ul,Ultrasonic_Packet data_ur,ObstacleLocationPacket *location);

	/*
	 * 涓夎瀹氫綅鍦伴潰鍧愭爣绯荤殑杞崲
	 * vehicle:杞﹁締鐘舵�佷俊鎭�
	 * body   :闅滅鐗╃浉瀵逛簬杞戒綋鍧愭爣绯荤殑鍧愭爣
	 * ground :闅滅鐗╃浉瀵逛簬鍦伴潰鍧愭爣绯荤殑鍧愭爣
	 * */
	void GroundTriangleCalculate(VehicleState *vehicle,ObstacleLocationPacket body,ObstacleLocationPacket *ground);

	/*
	 * 鐩存帴娴嬮噺鏁版嵁鐨勮溅杈嗗潗鏍囧畾浣�
	 * */
	void BodyDirectLocation();

	/*
	 * 涓夎瀹氫綅鐨勮溅杈嗗潗鏍囧畾浣�
	 * */
	void BodyTriangleLocation();

	/*
	 * 绫诲瀷1锛� 鍩轰簬鐩存帴娴嬮噺鍊肩殑杞戒綋鍧愭爣绯昏浆鎹�
	 * */
	void BodyDirectLocationType1();

	/*
	 * 绫诲瀷1锛� 鍩轰簬涓夎瀹氫綅杞戒綋鍧愭爣绯昏浆鎹�
	 * */
	void BodyTriangleLocationType1();

	/*
	 * 鍦伴潰鍧愭爣瓒呭０娉㈡暟鎹畾浣�
	 * */
	void GroundTriangleLocation(VehicleState *vehicle_state);

	/*
	 * base on triangle parking location
	 *
	 * */
	void ParkingEdgeTriangleLocation(VehicleState *vehicle_state);

	void ParkingEdgeCalculate( VehicleState *vehicle,Location position,Ultrasonic_Packet u_data,
			                   ParkingEdgeBufferLocationPacket *buf_dat,ObstacleLocationPacket *body_location);

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

	ParkingEdgeBufferLocationPacket _ultrasonic_data_buffer[4];
};

#endif /* ULTRASONIC_H_ */

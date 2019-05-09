/*
 * ultrasonic_abstacle_percption.h
 *
 *  Created on: 2019年1月29日
 *      Author: zhuguohua
 */

#ifndef ULTRASONICPERCAPTION_ULTRASONIC_OBSTACLE_PERCPTION_H_
#define ULTRASONICPERCAPTION_ULTRASONIC_OBSTACLE_PERCPTION_H_

#include "percaption.h"
#include "link_list.h"

#define LEVEL_THRESHOLD       (2.2f)
#define DISTANCE_THRESHOLD    (0.3f)
#define STEP_DISTANCE         (0.1f)

typedef struct _ObstacleInformationPacket
{
	Vector2d First_Position;
	Vector2d Second_Position;
	float    Length;
}ObstacleInformationPacket;

// 平行泊车控制总体状态
typedef enum _EdgeFindingState
{
	ObstacleWaitEdge= 0,
	VehicleEdgeWaitstate,
	WaitEnterDenseArea,
	JudgeParkingValid,
	waitParking
}EdgeFindingState;

typedef enum _LocationStatus
{
	LocationReady= 0,
	LocationDataPush,
	LocationCalculate,
	LocationFinish
}LocationStatus;

typedef enum _UltrasonicLocationPushState
{
	StateInit= 0,
	UltrasonicDataPush_LRU,
	UltrasonicDataPushSRU
}UltrasonicLocationPushState;

typedef enum _UltrasonicLocationCalculateState
{
	WaitForFrontEdgeCalculate = 0,
	FrontEdgeCalculate,
	WaitForRearEdgeCalculate,
	RearEdgeCalculate
}UltrasonicLocationCalculateState;

//typedef enum _StateControlCommand
//{
//	LocationStart= 1,
//	VehicleStop,
//	FrontEdgeCalculateStart,
//	RearEdgeCalculateStart
//}StateControlCommand;

class UltrasonicObstaclePercption : public Percaption
{
public:
	UltrasonicObstaclePercption();
	virtual ~UltrasonicObstaclePercption();

	void Init();
	void Push(Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat);
	void Push(ObstacleLocationPacket p_dat);
	void SpaceDelete();

	void EdgeFinding();

	float HighestDistribution(uint8_t group_number,uint16_t* group_value_array,float min_value);
	void  ValueDistributed();

	void ObstacleLocationPushStateMachine(Ultrasonic* u_dat);
	int8_t ObstacleLocationCalculateStateMachine();

	uint16_t getPositionListLength();
	uint16_t getLocationListLength();
	/////////////////////////////////////////////////////////////////////////////////////////////////
	LocationStatus getUltrasonicLocationStatus();
	void    setUltrasonicLocationStatus(LocationStatus value);
	Property<UltrasonicObstaclePercption,LocationStatus,READ_WRITE> UltrasonicLocationStatus;

	ObstacleInformationPacket getValidParkingPosition();
	void    setValidParkingPosition(ObstacleInformationPacket value);
	Property<UltrasonicObstaclePercption,ObstacleInformationPacket,READ_WRITE> ValidParkingPosition;
private:
	LocationStatus _ultrasonic_location_sts;

	EdgeFindingState _edge_finding_state;

	UltrasonicLocationPushState      _location_push_state;
	UltrasonicLocationCalculateState _location_calculate_state;

	LinkList *_ultrasonic_position_list;
	LinkList *_ultrasonic_triangle_location_list;

	ObstacleInformationPacket _parking_position;
	ObstacleInformationPacket _vehicle_position;
	// 最终输出的库位信息
	ObstacleInformationPacket _valid_parking_position;
	/******************************************/
	Node* _current_node;//当前节点
	Node* _last_node;//上一节点
	Node* _current_node_triangle;//当前节点零时变量
	float _err_distance;
};

#endif /* ULTRASONICPERCAPTION_ULTRASONIC_ABSTACLE_PERCPTION_H_ */

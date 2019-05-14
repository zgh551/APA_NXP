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
	/*********************************************超声避障相关函数***************************************/
	int8_t UltrasonicCollisionStatus(Ultrasonic *u,MessageManager *msg);

	void UltrasonicCollisionDiatance(Ultrasonic *u,MessageManager *msg);
	/*********************************************基础函数***************************************/
	uint16_t getPositionListLength();
	uint16_t getLocationListLength();
	/////////////////////////////////////////////////////////////////////////////////////////////////
	LocationStatus getUltrasonicLocationStatus();
	void    setUltrasonicLocationStatus(LocationStatus value);
	Property<UltrasonicObstaclePercption,LocationStatus,READ_WRITE> UltrasonicLocationStatus;


private:
	LocationStatus _ultrasonic_location_sts;

	EdgeFindingState _edge_finding_state;

	UltrasonicLocationPushState      _location_push_state;
	UltrasonicLocationCalculateState _location_calculate_state;

	LinkList *_ultrasonic_position_list;
	LinkList *_ultrasonic_triangle_location_list;

	ObstacleInformationPacket _parking_position;
	ObstacleInformationPacket _vehicle_position;

	/******************************************/
	Node* _current_node;//当前节点
	Node* _last_node;//上一节点
	Node* _current_node_triangle;//当前节点零时变量
	float _err_distance;
	/******************************************/
	VehilceConfig _ultrasonic_obstacle_config;
};

#endif /* ULTRASONICPERCAPTION_ULTRASONIC_ABSTACLE_PERCPTION_H_ */

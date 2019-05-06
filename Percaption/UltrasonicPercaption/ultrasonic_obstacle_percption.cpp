/*
 * ultrasonic_abstacle_percption.cpp
 *
 *  Created on: 2019年1月29日
 *      Author: zhuguohua
 */

#include "ultrasonic_obstacle_percption.h"

static Vector2d _last_point;
static float _last_x_value;
static float _last_triangle_x_value;
static uint8_t time_ms_cnt;

UltrasonicObstaclePercption::UltrasonicObstaclePercption() {

	UltrasonicLocationStatus.setContainer(this);
	UltrasonicLocationStatus.getter(&UltrasonicObstaclePercption::getUltrasonicLocationStatus);
	UltrasonicLocationStatus.setter(&UltrasonicObstaclePercption::setUltrasonicLocationStatus);

	ValidParkingPosition.setContainer(this);
	ValidParkingPosition.getter(&UltrasonicObstaclePercption::getValidParkingPosition);
	ValidParkingPosition.setter(&UltrasonicObstaclePercption::setValidParkingPosition);

	Init();
}
////////////////////////////////////////////////////////////////////////
LocationStatus UltrasonicObstaclePercption::getUltrasonicLocationStatus()           { return  _ultrasonic_location_sts;}
void  UltrasonicObstaclePercption::setUltrasonicLocationStatus(LocationStatus value){ _ultrasonic_location_sts = value;}

ObstacleInformationPacket UltrasonicObstaclePercption::getValidParkingPosition()           { return  _valid_parking_position;}
void  UltrasonicObstaclePercption::setValidParkingPosition(ObstacleInformationPacket value){ _valid_parking_position = value;}
////////////////////////////////////////////////////////////////////////
UltrasonicObstaclePercption::~UltrasonicObstaclePercption() {
	delete _ultrasonic_position_list;
	delete _ultrasonic_triangle_location_list;
	_ultrasonic_position_list = NULL;
	_ultrasonic_triangle_location_list = NULL;
}

void UltrasonicObstaclePercption::Init()
{
	_ultrasonic_location_sts = LocationReady;

	_edge_finding_state = ObstacleWaitEdge;
	_location_push_state = StateInit;
	_location_calculate_state = WaitForFrontEdgeCalculate;

	_parking_position.First_Position  = Vector2d(0,0);
	_parking_position.Second_Position = Vector2d(0,0);
	_parking_position.Length = 0.0f;

	_vehicle_position.First_Position  = Vector2d(0,0);
	_vehicle_position.Second_Position = Vector2d(0,0);
	_vehicle_position.Length = 0.0f;
	// 最终输出的库位信息
	_valid_parking_position.First_Position  = Vector2d(0,0);
	_valid_parking_position.Second_Position = Vector2d(0,0);
	_valid_parking_position.Length = 0.0f;
	/******************************************/
	_current_node = NULL;//当前节点
	_last_node    = NULL;//上一节点
	_err_distance = 0.0f;

	delete _ultrasonic_position_list;
	delete _ultrasonic_triangle_location_list;
	_ultrasonic_position_list = NULL;
	_ultrasonic_triangle_location_list = NULL;

	_ultrasonic_position_list = new LinkList;
	_ultrasonic_triangle_location_list = new LinkList;
}

void UltrasonicObstaclePercption::Push(Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat)
{
	if((u_dat.Level > LEVEL_THRESHOLD) && (fabs(p_dat.Position.getX() - _last_x_value) > 0.005))
	{
		_ultrasonic_position_list->Add(p_dat);
	}
	_last_x_value = p_dat.Position.getX();
}
void UltrasonicObstaclePercption::Push(ObstacleLocationPacket p_dat)
{
	if( (p_dat.Status == 0) && (fabs(p_dat.Position.getX() - _last_triangle_x_value) > 0.001))
//	if( p_dat.Status == 0 )
	{
		_ultrasonic_triangle_location_list->Add(p_dat);
	}
	_last_triangle_x_value = p_dat.Position.getX();
}

void UltrasonicObstaclePercption::SpaceDelete()
{
	_ultrasonic_position_list->Delete();
	_ultrasonic_triangle_location_list->Delete();
}

void  UltrasonicObstaclePercption::EdgeFinding()
{
	ObstacleInformationPacket parking_position_temp;
	ObstacleInformationPacket vehicle_position_temp;

	_current_node = _ultrasonic_position_list->HeadNode;//当前节点
	_last_node    = _ultrasonic_position_list->HeadNode;//上一节点

	while(_current_node->next != NULL)
	{
		_err_distance = (_current_node->data.Position - _last_node->data.Position).Length();
		switch(_edge_finding_state)
		{
			case ObstacleWaitEdge:
				if(_err_distance < DISTANCE_THRESHOLD)
				{
					vehicle_position_temp.First_Position = _last_node->data.Position;
					_edge_finding_state = VehicleEdgeWaitstate;
				}
				break;

			case VehicleEdgeWaitstate:
				if(_err_distance > DISTANCE_THRESHOLD)
				{
					vehicle_position_temp.Second_Position = _last_node->data.Position;
					parking_position_temp.First_Position  = _last_node->data.Position;

					_vehicle_position.Length          = fabs(vehicle_position_temp.First_Position.getX() - vehicle_position_temp.Second_Position.getX());
					_vehicle_position.First_Position  = vehicle_position_temp.First_Position;
					_vehicle_position.Second_Position = vehicle_position_temp.Second_Position;
					_edge_finding_state = WaitEnterDenseArea;
				}
				break;

			case WaitEnterDenseArea:
				if( (_err_distance < DISTANCE_THRESHOLD) && (_err_distance != 0 ))
				{
					parking_position_temp.Second_Position = _last_node->data.Position;
					vehicle_position_temp.First_Position  = _last_node->data.Position;
					_edge_finding_state = JudgeParkingValid;
				}
				break;

			case JudgeParkingValid:
				if(_err_distance < DISTANCE_THRESHOLD)
				{
					vehicle_position_temp.Second_Position = _current_node->data.Position;
					if((vehicle_position_temp.First_Position - vehicle_position_temp.Second_Position).Length() > DISTANCE_THRESHOLD)
					{
						_parking_position.Length          = fabs(parking_position_temp.First_Position.getX() - parking_position_temp.Second_Position.getX());
						_parking_position.First_Position  = parking_position_temp.First_Position;
						_parking_position.Second_Position = parking_position_temp.Second_Position;
						_edge_finding_state = VehicleEdgeWaitstate;
					}
				}
				else//突然出现稀疏点
				{
					_edge_finding_state = WaitEnterDenseArea;
				}
				break;

			default:

				break;
		}
		_last_node = _current_node;
		_current_node = _current_node->next;
	}
	if(VehicleEdgeWaitstate == _edge_finding_state)
	{
		_vehicle_position.First_Position  = vehicle_position_temp.First_Position;
		_vehicle_position.Second_Position = _current_node->data.Position;
		_vehicle_position.Length          = fabs(_vehicle_position.First_Position.getX() - _vehicle_position.Second_Position.getX());
	}
	_valid_parking_position.First_Position = _vehicle_position.Second_Position;
}

float UltrasonicObstaclePercption::HighestDistribution(uint8_t group_number,uint16_t* group_value_array,float min_value)
{
	uint8_t i;
	uint8_t max_distribute_number_id;
	uint16_t max_distribute_number_value;

	uint16_t sum_value;
	float master_ratio,slave_ratio;

	for(i = 0; i < group_number; i++)
	{
		if(i == 0)
		{
			max_distribute_number_id = i;
			max_distribute_number_value = group_value_array[i];
		}
		else
		{
			if(group_value_array[i] > max_distribute_number_value)
			{
				max_distribute_number_value = group_value_array[i];
				max_distribute_number_id    = i;
			}
		}
	}
	//
	if((0 == max_distribute_number_id) && (group_number > 1))
	{
		if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id + 1])
		{
			sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id + 1];
			master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
			slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id + 1 ] / sum_value;
			return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id + 1.5f) * slave_ratio);
		}
		else
		{
			return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
		}
	}
	else if( (group_number - 1) == max_distribute_number_id )
	{
		if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id - 1])
		{
			sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id - 1];
			master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
			slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id - 1 ] / sum_value;
			return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id - 0.5f) * slave_ratio);
		}
		else
		{
			return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
		}
	}
	else
	{
		if(group_value_array[max_distribute_number_id - 1] > group_value_array[max_distribute_number_id + 1])
		{
			if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id - 1])
			{
				sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id - 1];
				master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
				slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id - 1 ] / sum_value;
				return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id - 0.5f) * slave_ratio);
			}
			else
			{
				return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
			}
		}
		else
		{
			if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id + 1])
			{
				sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id + 1];
				master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
				slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id + 1 ] / sum_value;
				return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id + 1.5f) * slave_ratio);
			}
			else
			{
				return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
			}
		}
	}
}

void  UltrasonicObstaclePercption::ValueDistributed()
{
	uint8_t i;

	float min_x,max_x;
	float min_y,max_y;
	uint8_t  x_group_number,y_group_number;

	_current_node_triangle = _ultrasonic_triangle_location_list->HeadNode;
	min_x = _current_node_triangle->data.Position.getX();
	max_x = _current_node_triangle->data.Position.getX();
	min_y = _current_node_triangle->data.Position.getY();
	max_y = _current_node_triangle->data.Position.getY();

	while(_current_node_triangle->next != NULL)
	{
		min_x = _current_node_triangle->data.Position.getX() < min_x ? _current_node_triangle->data.Position.getX() : min_x;
		max_x = _current_node_triangle->data.Position.getX() > max_x ? _current_node_triangle->data.Position.getX() : max_x;
		min_y = _current_node_triangle->data.Position.getY() < min_y ? _current_node_triangle->data.Position.getY() : min_y;
		max_y = _current_node_triangle->data.Position.getY() > max_y ? _current_node_triangle->data.Position.getY() : max_y;

		_current_node_triangle = _current_node_triangle->next;
	}

	x_group_number = (uint8_t)((max_x - min_x)/STEP_DISTANCE);
	y_group_number = (uint8_t)((max_y - min_y)/STEP_DISTANCE);

	_current_node_triangle = _ultrasonic_triangle_location_list->HeadNode;

	uint16_t *distribute_number_x = new uint16_t[x_group_number];
	uint16_t *distribute_number_y = new uint16_t[y_group_number];

	for(i = 0;i<x_group_number;i++)
	{
		distribute_number_x[i] = 0;
	}
	for(i = 0;i<y_group_number;i++)
	{
		distribute_number_y[i] = 0;
	}

	while(_current_node_triangle->next != NULL)
	{
		for(i = 0;i<x_group_number;i++)
		{
			if( (_current_node_triangle->data.Position.getX() >= (min_x + STEP_DISTANCE*i)) && (_current_node_triangle->data.Position.getX() < (min_x + STEP_DISTANCE*(i + 1))))
			{
				distribute_number_x[i]++;
				break;
			}
		}

		for(i = 0;i<y_group_number;i++)
		{
			if( (_current_node_triangle->data.Position.getY() >= (min_y + STEP_DISTANCE*i)) && (_current_node_triangle->data.Position.getY() < (min_y + STEP_DISTANCE*(i + 1))))
			{
				distribute_number_y[i]++;
				break;
			}
		}
		_current_node_triangle = _current_node_triangle->next;
	}
	_valid_parking_position.Second_Position.setX(HighestDistribution(x_group_number,distribute_number_x,min_x));
	_valid_parking_position.Second_Position.setY(HighestDistribution(y_group_number,distribute_number_y,min_y));

	delete []distribute_number_x;
	delete []distribute_number_y;
}


//void  UltrasonicObstaclePercption::ObstacleLocationPushStateMachine(Ultrasonic_Packet* u_dat,ObstacleLocationPacket* p_dat)
//{
//	switch(_location_push_state)
//	{
//		case StateInit:
//			if(LocationStart == Command)
//			{
//				Init();
//				_location_push_state = UltrasonicDataPush_LRU;
//			}
//			break;
//
//		case UltrasonicDataPush_LRU:
//			Push(u_dat[11],p_dat[11]);
//			if(VehicleStop == Command)
//			{
//				_location_push_state = UltrasonicDataPushSRU;
//			}
//			break;
//
//		case UltrasonicDataPushSRU:
//			Push(p_dat[5]);
//			Push(p_dat[6]);
//			if(_ultrasonic_triangle_location_list->Length() > 200)
//			{
//				_location_push_state = StateInit;
//			}
//			break;
//
//		default:
//
//			break;
//	}
//}

void  UltrasonicObstaclePercption::ObstacleLocationPushStateMachine(Ultrasonic_Packet* u_dat,ObstacleLocationPacket* p_dat)
{
	switch(_location_push_state)
	{
		case StateInit:
			if(0x50 == Command)
			{
				Init();
				_ultrasonic_location_sts = LocationDataPush;
				_location_push_state = UltrasonicDataPush_LRU;
			}
			break;

		case UltrasonicDataPush_LRU:
			time_ms_cnt = (time_ms_cnt + 1) % 3;
			Push(u_dat[11],p_dat[11]);
//			if(0 == time_ms_cnt)
//			{
				Push(p_dat[5]);
				Push(p_dat[6]);
//			}
			if(0x60 == Command)
			{
				Command = 0x70;
				_location_push_state = StateInit;
			}
			break;

		default:

			break;
	}
}

int8_t UltrasonicObstaclePercption::ObstacleLocationCalculateStateMachine()
{
	switch(_location_calculate_state)
	{
		case WaitForFrontEdgeCalculate:
			if(0x70 == Command)
			{
				_ultrasonic_location_sts = LocationCalculate;
				_location_calculate_state = FrontEdgeCalculate;
			}
			break;

		case FrontEdgeCalculate:
			EdgeFinding();
			_location_calculate_state = RearEdgeCalculate;
			break;

		case WaitForRearEdgeCalculate:
			if(0x80 == Command)
			{
				_location_calculate_state = RearEdgeCalculate;
			}
			break;

		case RearEdgeCalculate:
			ValueDistributed();
			SpaceDelete();
			Command = 0;
			_ultrasonic_location_sts = LocationFinish;
			_location_calculate_state = WaitForFrontEdgeCalculate;
			return SUCCESS;
			break;

		default:
			break;
	}
	return FAIL;
}


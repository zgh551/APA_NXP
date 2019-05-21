/*
 * ultrasonic_abstacle_percption.cpp
 *
 *  Created on: 2019年1月29日
 *      Author: zhuguohua
 */

#include "ultrasonic_obstacle_percption.h"

static Vector2d _last_point;
//static float _last_x_value;
static float _last_triangle_x_value;

UltrasonicObstaclePercption::UltrasonicObstaclePercption() {
	UltrasonicLocationStatus.setContainer(this);
	UltrasonicLocationStatus.getter(&UltrasonicObstaclePercption::getUltrasonicLocationStatus);
	UltrasonicLocationStatus.setter(&UltrasonicObstaclePercption::setUltrasonicLocationStatus);
	Init();
}

////////////////////////////////////////////////////////////////////////
LocationStatus UltrasonicObstaclePercption::getUltrasonicLocationStatus()           { return  _ultrasonic_location_sts;}
void  UltrasonicObstaclePercption::setUltrasonicLocationStatus(LocationStatus value){ _ultrasonic_location_sts = value;}
////////////////////////////////////////////////////////////////////////
UltrasonicObstaclePercption::~UltrasonicObstaclePercption() {
	delete _ultrasonic_position_list;
	delete _ultrasonic_triangle_location_list;
//	delete _left_edge_position_list;
//	delete _right_edge_position_list;
//	delete _left_fit_edge_list;
//	delete _right_fit_edge_list;

	_ultrasonic_position_list = NULL;
	_ultrasonic_triangle_location_list = NULL;
//	_left_edge_position_list = NULL;
//	_right_edge_position_list = NULL;
//	_left_fit_edge_list = NULL;
//	_right_fit_edge_list = NULL;
}

void UltrasonicObstaclePercption::Init()
{
	_ultrasonic_location_sts = LocationReady;

	_data_push_state = WaitPushStart;
	_parking_calculate_state = WaitCommandForCalculate;

	_edge_finding_state = ObstacleWaitEdge;


	_parking_position.First_Position  = Vector2d(0,0);
	_parking_position.Second_Position = Vector2d(0,0);
	_parking_position.Length = 0.0f;

	_vehicle_position.First_Position  = Vector2d(0,0);
	_vehicle_position.Second_Position = Vector2d(0,0);
	_vehicle_position.Length = 0.0f;
	// 最终输出的库位信息
	_valid_parking_edge_position.First_Position  = Vector2d(0,0);
	_valid_parking_edge_position.Second_Position = Vector2d(0,0);
	_valid_parking_edge_position.Length = 0.0f;
	// 进库后的库位中心调整
	_valid_parking_center_position.position = Vector2d(0,0);
	_valid_parking_center_position.angle    = 0;
	/******************************************/
	_current_node = NULL;//当前节点
	_last_node    = NULL;//上一节点
	_err_distance = 0.0f;

	delete _ultrasonic_position_list;
	delete _ultrasonic_triangle_location_list;
//	delete _left_edge_position_list;
//	delete _right_edge_position_list;
//	delete _left_fit_edge_list;
//	delete _right_fit_edge_list;
//
	_ultrasonic_position_list = NULL;
	_ultrasonic_triangle_location_list = NULL;
//	_left_edge_position_list = NULL;
//	_right_edge_position_list = NULL;
//	_left_fit_edge_list = NULL;
//	_right_fit_edge_list = NULL;

	_ultrasonic_position_list          = new LinkList;
	_ultrasonic_triangle_location_list = new LinkList;

//	_left_edge_position_list  = new LinkList;
//	_right_edge_position_list = new LinkList;
//
//	_left_fit_edge_list   = new LinkList;
//	_right_fit_edge_list  = new LinkList;
}

void UltrasonicObstaclePercption::Push(Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat)
{
	// 时间序列的数据
//	if(_ultrasonic_position_list->HeadNode == NULL)
//	{
		if( u_dat.Level > LEVEL_THRESHOLD )
		{
			_ultrasonic_position_list->Add(p_dat);
		}
//	}
//	else
//	{
//		if((u_dat.Level > LEVEL_THRESHOLD) && (p_dat.Position.getX() < _ultrasonic_position_list->getEndNode()->data.Position.getX()))
//		{
//			_ultrasonic_position_list->Add(p_dat);
//		}
//	}
}
void UltrasonicObstaclePercption::Push(ObstacleLocationPacket p_dat)
{
	// 无序数据
	if( (p_dat.Status == 0) && (fabs(p_dat.Position.getX() - _last_triangle_x_value) > 0.001))
	{
		_ultrasonic_triangle_location_list->Add(p_dat);
	}
	_last_triangle_x_value = p_dat.Position.getX();
}

void UltrasonicObstaclePercption::LeftPush(Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat)
{
	// 有序数据集
//	if((u_dat.Level > LEVEL_THRESHOLD) && (p_dat.Position.LengthSquare() != _left_edge_position_list->getEndNode()->data.Position.LengthSquare()))
//	{
//		_left_edge_position_list->Add(p_dat);
//	}
}

void UltrasonicObstaclePercption::RightPush(Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat)
{
	// 有序数据集
//	if((u_dat.Level > LEVEL_THRESHOLD) && (p_dat.Position.LengthSquare() != _right_edge_position_list->getEndNode()->data.Position.LengthSquare()))
//	{
//		_right_edge_position_list->Add(p_dat);
//	}
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

	if(_ultrasonic_position_list->HeadNode == NULL)
	{
		return;
	}
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
	_valid_parking_edge_position.First_Position = _valid_parking_edge_position.Second_Position;
//	_ultrasonic_position_list->Delete();
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

//uint8_t UltrasonicObstaclePercption::HighestDistributionBase(uint8_t group_number,uint16_t* group_value_array)
//{
//	uint8_t i;
//	uint8_t max_distribute_number_id;
//	uint16_t max_distribute_number_value;
//
//	for(i = 0; i < group_number; i++)
//	{
//		if(i == 0)
//		{
//			max_distribute_number_id = i;
//			max_distribute_number_value = group_value_array[i];
//		}
//		else
//		{
//			if(group_value_array[i] > max_distribute_number_value)
//			{
//				max_distribute_number_value = group_value_array[i];
//				max_distribute_number_id    = i;
//			}
//		}
//	}
//	return max_distribute_number_id;
//}

void  UltrasonicObstaclePercption::ValueDistributed()
{
	uint8_t i;

	float min_x,max_x;
	float min_y,max_y;
	uint8_t  x_group_number,y_group_number;

	if(_ultrasonic_triangle_location_list->HeadNode == NULL)
	{
		return;
	}
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

	x_group_number = (uint8_t)((max_x - min_x)/STEP_DISTANCE) + 1;
	y_group_number = (uint8_t)((max_y - min_y)/STEP_DISTANCE) + 1;

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

	_valid_parking_edge_position.Second_Position.setX(HighestDistribution(x_group_number,distribute_number_x,min_x));
	_valid_parking_edge_position.Second_Position.setY(HighestDistribution(y_group_number,distribute_number_y,min_y));

	delete []distribute_number_x;
	delete []distribute_number_y;
//	_ultrasonic_triangle_location_list->Delete();
}

//void UltrasonicObstaclePercption::ValueDistributed(LinkList *valid_list,LinkList *fit_list)
//{
//	uint8_t i,max_distribute_number_id;
//	uint8_t x_group_number,y_group_number;
//	float min_x,max_x;
//	float min_y,max_y;
//	float treshold_down,treshold_up;
//	Node* current_node;
//
//	if(valid_list->HeadNode == NULL){return;}
//	current_node = valid_list->HeadNode;
//	min_x = current_node->data.Position.getX();
//	max_x = current_node->data.Position.getX();
//	min_y = current_node->data.Position.getY();
//	max_y = current_node->data.Position.getY();
//
//	while(current_node->next != NULL)
//	{
//		min_x = current_node->data.Position.getX() < min_x ? current_node->data.Position.getX() : min_x;
//		max_x = current_node->data.Position.getX() > max_x ? current_node->data.Position.getX() : max_x;
//		min_y = current_node->data.Position.getY() < min_y ? current_node->data.Position.getY() : min_y;
//		max_y = current_node->data.Position.getY() > max_y ? current_node->data.Position.getY() : max_y;
//		current_node = current_node->next;
//	}
//	x_group_number = (uint8_t)((max_x - min_x)/STEP_DISTANCE) + 1;
//	y_group_number = (uint8_t)((max_y - min_y)/STEP_DISTANCE) + 1;
//
//	current_node = valid_list->HeadNode;
//	// create the array
//	uint16_t *distribute_number_x = new uint16_t[x_group_number];
//	uint16_t *distribute_number_y = new uint16_t[y_group_number];
//	// initialize the array
//	for(i = 0;i<x_group_number;i++)
//	{
//		distribute_number_x[i] = 0;
//	}
//	for(i = 0;i<y_group_number;i++)
//	{
//		distribute_number_y[i] = 0;
//	}
//	// distribute calculate
//	while(current_node->next != NULL)
//	{
//		for(i = 0;i<x_group_number;i++)
//		{
//			if( (current_node->data.Position.getX() >= (min_x + STEP_DISTANCE*i)) && (current_node->data.Position.getX() < (min_x + STEP_DISTANCE*(i + 1))))
//			{
//				distribute_number_x[i]++;
//				break;
//			}
//		}
//		for(i = 0;i<y_group_number;i++)
//		{
//			if( (current_node->data.Position.getY() >= (min_y + STEP_DISTANCE*i)) && (current_node->data.Position.getY() < (min_y + STEP_DISTANCE*(i + 1))))
//			{
//				distribute_number_y[i]++;
//				break;
//			}
//		}
//		current_node = current_node->next;
//	}
////	max_distribute_number_id = HighestDistributionBase(x_group_number,distribute_number_x);
//	max_distribute_number_id = HighestDistributionBase(y_group_number,distribute_number_y);
////	treshold_down = min_x + STEP_DISTANCE * max_distribute_number_id     ;
////	treshold_up   = min_x + STEP_DISTANCE *(max_distribute_number_id + 1);
//	treshold_down = min_y + STEP_DISTANCE * max_distribute_number_id     ;
//	treshold_up   = min_y + STEP_DISTANCE *(max_distribute_number_id + 1);
//	current_node = valid_list->HeadNode;
//	if(fit_list->Length() != 0)
//	{
//		fit_list->Delete();
//	}
//	while(current_node->next != NULL)
//	{
////		if((current_node->data.Position.getX() >= treshold_down) && (current_node->data.Position.getX() < treshold_up))
//		if((current_node->data.Position.getY() >= treshold_down) && (current_node->data.Position.getY() < treshold_up))
//		{
//			fit_list->Add(current_node->data);
//		}
//		current_node = current_node->next;
//	}
//	delete []distribute_number_x;
//	delete []distribute_number_y;
////	valid_list->Delete();
//}
//
//// 车辆进库，通过库位边界，测定库位的中心位置，准确的前提是车辆基本垂直进入
//void UltrasonicObstaclePercption::EdgeLineFitParkingCenterCalculate()
//{
////	float left_yaw,right_yaw,middle_yaw;
////	float left_b,right_b,middle_b;
////
////	ValueDistributed(_left_edge_position_list,_left_fit_edge_list);
////	ValueDistributed(_right_edge_position_list,_right_fit_edge_list);
////
////	_line_fit.LineFitting(_left_fit_edge_list,&left_yaw,&left_b);
////	_line_fit.LineFitting(_right_fit_edge_list,&right_yaw,&right_b);
////
////	middle_yaw = 0.5 * (left_yaw + right_yaw);
////	middle_b   = 0.5 * (left_b   + right_b  );
////
////	_valid_parking_center_position.angle       = middle_yaw;
////	_valid_parking_center_position.position.X 	= 	_left_fit_edge_list->getHeadNode()->data.Position.getX()  >
////														_right_fit_edge_list->getHeadNode()->data.Position.getX() ?
////														_left_fit_edge_list->getHeadNode()->data.Position.getX()  :
////														_right_fit_edge_list->getHeadNode()->data.Position.getX();
////
////	_valid_parking_center_position.position.Y  = tanf(middle_yaw) * _valid_parking_center_position.position.X + middle_b;
//
////	_left_fit_edge_list->Delete();
////	_right_fit_edge_list->Delete();
//}
/*
 * 运行于定时器中，5ms运行一次
 * */
void  UltrasonicObstaclePercption::DataPushStateMachine(Ultrasonic* u_dat)
{
	switch(_data_push_state)
	{
		case WaitPushStart:
			if(0x50 == Command)
			{
				Init();
				_ultrasonic_location_sts = LocationDataPush;
				_data_push_state = ParkingEdgeUltrasonicDataPush;
			}
			else if(0x55 == Command)
			{
//				if(0 != _left_edge_position_list->Length())
//				{
//					_left_edge_position_list->Delete();
//				}
//				if(0 != _right_edge_position_list->Length())
//				{
//					_left_edge_position_list->Delete();
//				}
				_ultrasonic_location_sts = LocationDataPush;
				_data_push_state = ParkingCenterUltrasonicDataPush;
			}
			break;

		case ParkingEdgeUltrasonicDataPush:
//			if( (0 == u_dat->ScheduleTimeCnt) || (14 == u_dat->ScheduleTimeCnt))
//			{
				Push(u_dat->UltrasonicPacket[11],u_dat->AbstacleGroundPositionTriangle[11]);
//			}
//			if(23 == u_dat->ScheduleTimeCnt)
//			{
				Push(u_dat->AbstacleGroundPositionTriangle[5]);
				Push(u_dat->AbstacleGroundPositionTriangle[6]);
//			}
			if( (0x60 == Command) && (getLocationListLength() > 100))
			{
				Command = 0x70;//库位计算命令
				_data_push_state = WaitPushStart;
			}
			break;

		case ParkingCenterUltrasonicDataPush:
//			if((26 == u_dat->ScheduleTimeCnt) || (12 == u_dat->ScheduleTimeCnt))
//			{
//				LeftPush(u_dat->UltrasonicPacket[10],u_dat->AbstacleGroundPositionTriangle[10]);
//			}
//			if((0 == u_dat->ScheduleTimeCnt) || (14 == u_dat->ScheduleTimeCnt))
//			{
//				RightPush(u_dat->UltrasonicPacket[11],u_dat->AbstacleGroundPositionTriangle[11]);
//			}
//			if((getLeftEdgeListLength() > 20) && (getRightEdgeListLength() > 20))
//			{
//				Command = 0x75;
//				_data_push_state = WaitPushStart;
//			}
			break;
		default:

			break;
	}
}

int8_t UltrasonicObstaclePercption::ParkingCalculateStateMachine(void)
{
	switch(_parking_calculate_state)
	{
		case WaitCommandForCalculate:
			if(0x70 == Command)
			{
				_ultrasonic_location_sts = LocationCalculate;
				_parking_calculate_state = FrontEdgeCalculate;
			}
			else if(0x75 == Command)
			{
				_ultrasonic_location_sts = LocationCalculate;
				_parking_calculate_state = ParkingCenterCalculate;
			}
			else
			{

			}
			break;

		case FrontEdgeCalculate:
			EdgeFinding();
			_parking_calculate_state = RearEdgeCalculate;
			break;

		case RearEdgeCalculate:
			ValueDistributed();
			Command = 0;
			_ultrasonic_location_sts = LocationFinish;
			_parking_calculate_state = WaitCommandForCalculate;
			return SUCCESS;
			break;

		case ParkingCenterCalculate:
//			EdgeLineFitParkingCenterCalculate();
			Command = 0;
			_ultrasonic_location_sts = LocationFinish;
			_parking_calculate_state = WaitCommandForCalculate;
			return SUCCESS;
			break;

		default:
			break;
	}
	return FAIL;
}
/***********************************************************************************************/
/************************************* 超声波避障功能  ***********************************************/
/***********************************************************************************************/
int8_t UltrasonicObstaclePercption::UltrasonicCollisionStatus(Ultrasonic *u,MessageManager *msg)
{
	uint8_t i;
	if(Reverse == msg->Gear)
	{
		for(i = 0; i < 4; i++)
		{
			if(u->UltrasonicPacket[i].Distance1 != 0.0f)
			{
				if( (u->UltrasonicPacket[i].Distance1 < 0.4) || (u->UltrasonicPacket[i].status == 16) )
				{
					return SUCCESS;
				}
			}
		}
		return FAIL;
	}
	else if(Drive == msg->Gear)
	{
		for(i = 4;i < 8;i++)
		{
			if(u->UltrasonicPacket[i].Distance1 != 0.0f)
			{
				if( (u->UltrasonicPacket[i].Distance1 < 0.4) || (u->UltrasonicPacket[i].status == 16))
				{
					return SUCCESS;
				}
			}
		}
		return FAIL;
	}
	else
	{
		return FAIL;
	}
}

void UltrasonicObstaclePercption::UltrasonicCollisionDiatance(Ultrasonic *u,MessageManager *msg)
{
	uint8_t i;
	float distance,current_distance;
	uint8_t over_detection_cnt;
	if(Reverse == msg->Gear)
	{
		distance = 0;
		over_detection_cnt = 0;
		for(i = 0; i < 4; i++)
		{
			if(Normal == u->AbstacleBodyPositionTriangle[i].Status)
			{
				current_distance = fabs(u->AbstacleBodyPositionTriangle[i].Position.getX() - _ultrasonic_obstacle_config.UltrasonicLocationArray[1].Point.getX());
				distance = current_distance < distance ? current_distance : distance;
				_obstacle_distance.status = Normal;

			}
			else if(BlindZone == u->AbstacleBodyPositionTriangle[i].Status)
			{
				distance = 0;
				_obstacle_distance.status = BlindZone;
				break;
			}
			else if(OverDetection == u->AbstacleBodyPositionTriangle[i].Status)
			{
				over_detection_cnt++;
			}
			else
			{
				distance                  = 0;
				_obstacle_distance.status = Noise;
			}
		}
	}
	else if(Drive == msg->Gear)
	{
		distance = 0;
		over_detection_cnt = 0;
		for(i = 4;i < 8;i++)
		{
			if(Normal == u->AbstacleBodyPositionTriangle[i].Status)
			{
				current_distance = fabs(u->AbstacleBodyPositionTriangle[i].Position.getX() - _ultrasonic_obstacle_config.UltrasonicLocationArray[5].Point.getX());
				distance = current_distance < distance ? current_distance : distance;
				_obstacle_distance.status = Normal;
			}
			else if(BlindZone == u->AbstacleBodyPositionTriangle[i].Status)
			{
				distance = 0;
				_obstacle_distance.status = BlindZone;
				break;
			}
			else if(OverDetection == u->AbstacleBodyPositionTriangle[i].Status)
			{
				over_detection_cnt++;
			}
			else
			{
				distance                  = 0;
				_obstacle_distance.status = Noise;
			}
		}
	}
	if(4 == over_detection_cnt)
	{
		distance                  = 3;
		_obstacle_distance.status = OverDetection;
	}
	_obstacle_distance.distance = distance;
}
/***********************************************************************************************/
uint16_t UltrasonicObstaclePercption::getPositionListLength()
{
	return (uint16_t)_ultrasonic_position_list->Length();
}

uint16_t UltrasonicObstaclePercption::getLocationListLength()
{
	return (uint16_t)_ultrasonic_triangle_location_list->Length();
}

uint16_t UltrasonicObstaclePercption::getLeftEdgeListLength()
{
//	return (uint16_t)_left_edge_position_list->Length();
}

uint16_t UltrasonicObstaclePercption::getRightEdgeListLength()
{
//	return (uint16_t)_right_edge_position_list->Length();
}


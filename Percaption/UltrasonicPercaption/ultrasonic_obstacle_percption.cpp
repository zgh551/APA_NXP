/*
 * ultrasonic_abstacle_percption.cpp
 *
 *  Created on: 2019年1月29日
 *      Author: zhuguohua
 */

#include "ultrasonic_obstacle_percption.h"

UltrasonicObstaclePercption::UltrasonicObstaclePercption() {
	UltrasonicLocationStatus.setContainer(this);
	UltrasonicLocationStatus.getter(&UltrasonicObstaclePercption::getUltrasonicLocationStatus);
	UltrasonicLocationStatus.setter(&UltrasonicObstaclePercption::setUltrasonicLocationStatus);
	Init();
}
UltrasonicObstaclePercption::~UltrasonicObstaclePercption() {
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
	_push_cnt = 0;
	/******************************************/
	_ultrasonic_position_list          = new LinkList;
	_ultrasonic_triangle_location_list = new LinkList;

	_left_edge_position_list  = new LinkList;
	_right_edge_position_list = new LinkList;

	_left_fit_edge_list   = new LinkList;
	_right_fit_edge_list  = new LinkList;
}

void UltrasonicObstaclePercption::Push(LinkList *list,Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat)
{
	// 时间序列的数据
	if(list->HeadNode == NULL)
	{
		if( u_dat.Level > LEVEL_THRESHOLD )
		{
			list->Add(p_dat);
		}
	}
	else
	{
		if((u_dat.Level > LEVEL_THRESHOLD) && (p_dat.Position.getX() < list->getEndNode()->data.Position.getX()))
		{
			list->Add(p_dat);
		}
	}
}

void UltrasonicObstaclePercption::Push(LinkList *list,ObstacleLocationPacket p_dat)
{
	// 无序数据
	if(list->getHeadNode() == NULL)
	{
		if(p_dat.Status == 0)
		{
			list->Add(p_dat);
		}
	}
	else
	{
		if( (p_dat.Status == 0) && (p_dat.Position.LengthSquare() != list->getEndNode()->data.Position.LengthSquare()) )
		{
			list->Add(p_dat);
		}
	}
}

void UltrasonicObstaclePercption::ParkingCenterPush(LinkList *list,Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat)
{
	// 有序数据集
	if(list->getEndNode() == NULL)
	{
		if( u_dat.Level > FIT_LINE_STEP_LEVEL_THRESHOLD )
		{
			list->Add(p_dat);
		}
	}
	else
	{
#if ENTER_PARK_DIRECTION == X_AXIS_ENTER
		if((u_dat.Level > FIT_LINE_STEP_LEVEL_THRESHOLD) && (p_dat.Position.getX() < list->getEndNode()->data.Position.getX()))
#elif ENTER_PARK_DIRECTION == Y_AXIS_ENTER
		if((u_dat.Level > FIT_LINE_STEP_LEVEL_THRESHOLD) && (p_dat.Position.getY() < list->getEndNode()->data.Position.getY()))
#endif
		{
			list->Add(p_dat);
		}
	}
}

void  UltrasonicObstaclePercption::EdgeFinding(LinkList *list)
{
	Node* _current_node;//当前节点
	Node* _last_node;//上一节点
	ObstacleInformationPacket parking_position_temp;
	ObstacleInformationPacket vehicle_position_temp;
	float max_y_axis_value;
	float _err_distance;

	if(list->HeadNode == NULL)
	{
		return;
	}
	_current_node = list->HeadNode;//当前节点
	_last_node    = list->HeadNode;//上一节点

	while(_current_node->next != NULL)
	{
		_err_distance = (_current_node->data.Position - _last_node->data.Position).Length();
		switch(_edge_finding_state)
		{
			case ObstacleWaitEdge:
				if(_err_distance < DISTANCE_THRESHOLD)
				{
					vehicle_position_temp.First_Position = _last_node->data.Position;
					max_y_axis_value = _current_node->data.Position.getY();
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
				else
				{
					max_y_axis_value = _current_node->data.Position.getY() > max_y_axis_value ? _current_node->data.Position.getY() : max_y_axis_value;
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
	_valid_parking_edge_position.First_Position.setX( _vehicle_position.Second_Position.getX() );
	_valid_parking_edge_position.First_Position.setY( max_y_axis_value );
	list->Delete();
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

uint8_t UltrasonicObstaclePercption::HighestDistributionBase(uint8_t group_number,uint16_t* group_value_array)
{
	uint8_t i;
	uint8_t max_distribute_number_id;
	uint16_t max_distribute_number_value;

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
	return max_distribute_number_id;
}

void  UltrasonicObstaclePercption::ValueDistributed(LinkList *valid_list)
{
	uint8_t i;
	float min_x,max_x;
	float min_y,max_y;
	uint8_t  x_group_number,y_group_number;
	Node* _current_node_triangle;//当前节点零时变量

	if(valid_list->HeadNode == NULL)
	{
		return;
	}
	_current_node_triangle = valid_list->HeadNode;
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

	_current_node_triangle = valid_list->HeadNode;

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
	valid_list->Delete();
}

void UltrasonicObstaclePercption::ValueDistributedFilter(LinkList *valid_list,LinkList *fit_list)
{
	uint8_t i,max_distribute_number_id;
	uint8_t x_group_number,y_group_number;
	float min_x,max_x;
	float min_y,max_y;
	float treshold_down,treshold_up;
	Node* current_node;

	if(valid_list->HeadNode == NULL){return;}
	current_node = valid_list->HeadNode;
	min_x = current_node->data.Position.getX();
	max_x = current_node->data.Position.getX();
	min_y = current_node->data.Position.getY();
	max_y = current_node->data.Position.getY();

	while(current_node->next != NULL)
	{
		min_x = current_node->data.Position.getX() < min_x ? current_node->data.Position.getX() : min_x;
		max_x = current_node->data.Position.getX() > max_x ? current_node->data.Position.getX() : max_x;
		min_y = current_node->data.Position.getY() < min_y ? current_node->data.Position.getY() : min_y;
		max_y = current_node->data.Position.getY() > max_y ? current_node->data.Position.getY() : max_y;
		current_node = current_node->next;
	}
	x_group_number = (uint8_t)((max_x - min_x)/FIT_LINE_STEP_DISTANCE) + 1;
	y_group_number = (uint8_t)((max_y - min_y)/FIT_LINE_STEP_DISTANCE) + 1;

	current_node = valid_list->HeadNode;
	// create the array
	uint16_t *distribute_number_x = new uint16_t[x_group_number];
	uint16_t *distribute_number_y = new uint16_t[y_group_number];
	// initialize the array
	for(i = 0;i<x_group_number;i++)
	{
		distribute_number_x[i] = 0;
	}
	for(i = 0;i<y_group_number;i++)
	{
		distribute_number_y[i] = 0;
	}
	// distribute calculate
	while(current_node->next != NULL)
	{
		for(i = 0;i<x_group_number;i++)
		{
			if( (current_node->data.Position.getX() >= (min_x + FIT_LINE_STEP_DISTANCE*i)) && (current_node->data.Position.getX() < (min_x + FIT_LINE_STEP_DISTANCE*(i + 1))))
			{
				distribute_number_x[i]++;
				break;
			}
		}
		for(i = 0;i<y_group_number;i++)
		{
			if( (current_node->data.Position.getY() >= (min_y + FIT_LINE_STEP_DISTANCE*i)) && (current_node->data.Position.getY() < (min_y + FIT_LINE_STEP_DISTANCE*(i + 1))))
			{
				distribute_number_y[i]++;
				break;
			}
		}
		current_node = current_node->next;
	}
#if ENTER_PARK_DIRECTION == X_AXIS_ENTER
	max_distribute_number_id = HighestDistributionBase(y_group_number,distribute_number_y);
	treshold_down = min_y + FIT_LINE_STEP_DISTANCE * max_distribute_number_id     ;
	treshold_up   = min_y + FIT_LINE_STEP_DISTANCE *(max_distribute_number_id + 1);
#elif ENTER_PARK_DIRECTION == Y_AXIS_ENTER
	max_distribute_number_id = HighestDistributionBase(x_group_number,distribute_number_x);
	treshold_down = min_x + FIT_LINE_STEP_DISTANCE * max_distribute_number_id     ;
	treshold_up   = min_x + FIT_LINE_STEP_DISTANCE *(max_distribute_number_id + 1);
#endif

	current_node = valid_list->HeadNode;
	if(fit_list->Length() != 0)
	{
		fit_list->Delete();
	}
	while(current_node->next != NULL)
	{
#if ENTER_PARK_DIRECTION == X_AXIS_ENTER
		if((current_node->data.Position.getY() >= treshold_down) && (current_node->data.Position.getY() < treshold_up))
#elif ENTER_PARK_DIRECTION == Y_AXIS_ENTER
		if((current_node->data.Position.getX() >= treshold_down) && (current_node->data.Position.getX() < treshold_up))
#endif
		{
			fit_list->Add(current_node->data);
		}
		current_node = current_node->next;
	}
	delete []distribute_number_x;
	delete []distribute_number_y;
	valid_list->Delete();
}

// 车辆进库，通过库位边界，测定库位的中心位置，准确的前提是车辆基本垂直进入
void UltrasonicObstaclePercption::EdgeLineFitParkingCenterCalculate()
{
	float left_x,right_y;
	ValueDistributedFilter(_left_edge_position_list,_left_fit_edge_list);
	ValueDistributedFilter(_right_edge_position_list,_right_fit_edge_list);


	_line_fit.LineFitting(_left_fit_edge_list,&_left_fit_line_packet.angle,&_left_fit_line_packet.offset);
	_line_fit.LineFitting(_right_fit_edge_list,&_right_fit_line_packet.angle,&_right_fit_line_packet.offset);

	_center_fit_line_packet.angle  = 0.5 * (_left_fit_line_packet.angle + _right_fit_line_packet.angle);
	_center_fit_line_packet.offset = 0.5 * (_left_fit_line_packet.offset + _right_fit_line_packet.offset);

	_valid_parking_center_position.angle        = _center_fit_line_packet.angle;
	left_x = _left_fit_edge_list->getHeadNode()->data.Position.getX();
	right_y = _right_fit_edge_list->getHeadNode()->data.Position.getX();
	_valid_parking_center_position.position.setX((left_x > right_y) ? left_x : right_y);
	_valid_parking_center_position.position.setY(tanf(_center_fit_line_packet.angle) * _valid_parking_center_position.position.getX() + _center_fit_line_packet.offset);

	_left_fit_edge_list->Delete();
	_right_fit_edge_list->Delete();
}
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
				if(0 != _ultrasonic_position_list->Length()){_ultrasonic_position_list->Delete();}
				if(0 != _ultrasonic_triangle_location_list->Length()){_ultrasonic_triangle_location_list->Delete();}
				_ultrasonic_location_sts = LocationDataPush;
				_data_push_state = ParkingEdgeUltrasonicDataPush;
			}
			else if(0x55 == Command)
			{
				if(0 != _left_edge_position_list->Length()){_left_edge_position_list->Delete();}
				if(0 != _right_edge_position_list->Length()){_right_edge_position_list->Delete();}
				_ultrasonic_location_sts = LocationDataPush;
				_data_push_state = ParkingCenterUltrasonicDataPush;
			}
			else{}
			break;

		case ParkingEdgeUltrasonicDataPush:
#if RUNNING_PLATFORM == Embedded_PLATFORM
//			if( (0 == u_dat->ScheduleTimeCnt) || (14 == u_dat->ScheduleTimeCnt))
//			{
				Push(_ultrasonic_position_list,u_dat->UltrasonicPacket[11],u_dat->AbstacleGroundPositionTriangle[11]);
//			}
//			if(23 == u_dat->ScheduleTimeCnt)
//			{
				Push(_ultrasonic_triangle_location_list,u_dat->AbstacleGroundPositionTriangle[5]);
				Push(_ultrasonic_triangle_location_list,u_dat->AbstacleGroundPositionTriangle[6]);
//			}
#elif RUNNING_PLATFORM == PC_PLATFORM
			_push_cnt = (_push_cnt + 1)%3;
			Push(_ultrasonic_position_list,u_dat->UltrasonicPacket[11],u_dat->AbstacleGroundPositionTriangle[11]);
			if(0 == _push_cnt)
			{
				Push(_ultrasonic_triangle_location_list,u_dat->AbstacleGroundPositionTriangle[5]);
				Push(_ultrasonic_triangle_location_list,u_dat->AbstacleGroundPositionTriangle[6]);
			}
#endif
			if(getLocationListLength() > MIN_LOCATION_NUM)
			{
				Command = 0x70;//库位计算命令
				_data_push_state = WaitPushStart;
			}
			break;

		case ParkingCenterUltrasonicDataPush:
#if RUNNING_PLATFORM == Embedded_PLATFORM
//			if((26 == u_dat->ScheduleTimeCnt) || (12 == u_dat->ScheduleTimeCnt))
//			{
				ParkingCenterPush(_left_edge_position_list,u_dat->UltrasonicPacket[10],u_dat->AbstacleGroundPositionTriangle[10]);
//			}
//			if((0 == u_dat->ScheduleTimeCnt) || (14 == u_dat->ScheduleTimeCnt))
//			{
				ParkingCenterPush(_right_edge_position_list,u_dat->UltrasonicPacket[11],u_dat->AbstacleGroundPositionTriangle[11]);
//			}
#elif RUNNING_PLATFORM == PC_PLATFORM
			ParkingCenterPush(_left_edge_position_list,u_dat->UltrasonicPacket[10],u_dat->AbstacleGroundPositionTriangle[10]);
			ParkingCenterPush(_right_edge_position_list,u_dat->UltrasonicPacket[11],u_dat->AbstacleGroundPositionTriangle[11]);
#endif
			if((_left_edge_position_list->EndNode != NULL) && (_right_edge_position_list->EndNode != NULL))
			{
				if(((_left_edge_position_list->getHeadNode()->data.Position - _left_edge_position_list->getEndNode()->data.Position).Length() > MIN_FIT_DISTANCE)  &&
				   ((_right_edge_position_list->getHeadNode()->data.Position - _right_edge_position_list->getEndNode()->data.Position).Length() > MIN_FIT_DISTANCE) &&
				   (getLeftEdgeListLength() > MIN_FIT_NUM) &&
				   (getRightEdgeListLength() > MIN_FIT_NUM))
				{
					Command = 0x75;
					_data_push_state = WaitPushStart;
				}
			}
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
				_edge_finding_state      = ObstacleWaitEdge;
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
			EdgeFinding(_ultrasonic_position_list);
			_parking_calculate_state = RearEdgeCalculate;
			break;

		case RearEdgeCalculate:
			ValueDistributed(_ultrasonic_triangle_location_list);
			Command = 0;
			_ultrasonic_location_sts = LocationFinish;
			_parking_calculate_state = WaitCommandForCalculate;
			return SUCCESS;
			break;

		case ParkingCenterCalculate:
			EdgeLineFitParkingCenterCalculate();
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
	return (uint16_t)_left_edge_position_list->Length();
}

uint16_t UltrasonicObstaclePercption::getRightEdgeListLength()
{
	return (uint16_t)_right_edge_position_list->Length();
}

////////////////////////////////////////////////////////////////////////
LocationStatus UltrasonicObstaclePercption::getUltrasonicLocationStatus()           { return  _ultrasonic_location_sts;}
void  UltrasonicObstaclePercption::setUltrasonicLocationStatus(LocationStatus value){ _ultrasonic_location_sts = value;}
////////////////////////////////////////////////////////////////////////

/*****************************************************************************/
/* FILE NAME: parallel_planning.cpp               COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the parallel parking trajectory planning                     */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 9  2019      Initial Version                 */
/* 1.0	 Guohua Zhu     January 16 2019      Add ReversedTrial Function      */
/* 1.0	 Guohua Zhu     January 17 2019      Add TransitionArc Function      */
/*****************************************************************************/
#include "parallel_planning.h"


Terminal m_ParallelPlanningTerminal;
VehilceConfig m_ParallelVehilceConfig;
VehicleBody front_trial_body,rear_trial_body;
Vector2d enter_point;
Vector2d parking_right_rear,parking_right_front;//库位边角点
AlgebraicGeometry m_AlgebraicGeometry;


ParallelPlanning::ParallelPlanning() {
	// TODO Auto-generated constructor stub
	LeftVirtualBoundary.setContainer(this);
	LeftVirtualBoundary.getter(&ParallelPlanning::getLeftVirtualBoundary);
	LeftVirtualBoundary.setter(&ParallelPlanning::setLeftVirtualBoundary);

	RightVirtualBoundary.setContainer(this);
	RightVirtualBoundary.getter(&ParallelPlanning::getRightVirtualBoundary);
	RightVirtualBoundary.setter(&ParallelPlanning::setRightVirtualBoundary);

	FrontVirtualBoundary.setContainer(this);
	FrontVirtualBoundary.getter(&ParallelPlanning::getFrontVirtualBoundary);
	FrontVirtualBoundary.setter(&ParallelPlanning::setFrontVirtualBoundary);

	RearVirtualBoundary.setContainer(this);
	RearVirtualBoundary.getter(&ParallelPlanning::getRearVirtualBoundary);
	RearVirtualBoundary.setter(&ParallelPlanning::setRearVirtualBoundary);
	/**********************************************************************/
	LatMarginMove.setContainer(this);
	LatMarginMove.getter(&ParallelPlanning::getLatMarginMove);
	LatMarginMove.setter(&ParallelPlanning::setLatMarginMove);

	RightMarginBoundary.setContainer(this);
	RightMarginBoundary.getter(&ParallelPlanning::getRightMarginBoundary);
	RightMarginBoundary.setter(&ParallelPlanning::setRightMarginBoundary);

	FrontMarginBoundary.setContainer(this);
	FrontMarginBoundary.getter(&ParallelPlanning::getFrontMarginBoundary);
	FrontMarginBoundary.setter(&ParallelPlanning::setFrontMarginBoundary);

	RearMarginBoundary.setContainer(this);
	RearMarginBoundary.getter(&ParallelPlanning::getRearMarginBoundary);
	RearMarginBoundary.setter(&ParallelPlanning::setRearMarginBoundary);

	InitParking.setContainer(this);
	InitParking.getter(&ParallelPlanning::getInitParking);
	InitParking.setter(&ParallelPlanning::setInitParking);

	EnterParking.setContainer(this);
	EnterParking.getter(&ParallelPlanning::getInitParking);
	EnterParking.setter(&ParallelPlanning::setInitParking);

	Command.setContainer(this);
	Command.getter(&ParallelPlanning::getCommand);
	Command.setter(&ParallelPlanning::setCommand);

	ConsoleState.setContainer(this);
	ConsoleState.getter(&ParallelPlanning::getConsoleState);
	ConsoleState.setter(&ParallelPlanning::setConsoleState);
	//边界内margin为正值
	_lat_margin_move       =  0.0f;
	_right_margin_boundary =  0.1f;
	_front_margin_boundary =  0.1f;
	_rear_margin_boundary  =  0.1f;
}

ParallelPlanning::~ParallelPlanning() {
	// TODO Auto-generated destructor stub
}

// _left_virtual_boundary : 定义边界内为正值 ，边界外为负值
void ParallelPlanning::Init()
{
//	VehilceConfig *m_VehilceConfig = new VehilceConfig();
//	m_VehilceConfig->EdgeRadiusUpdate(MIN_TURN_RADIUS);
//	MinParkingLength = REAR_EDGE_TO_CENTER + sqrtf(powf(m_VehilceConfig->RadiusFrontRight,2) - powf(MIN_TURN_RADIUS - LEFT_EDGE_TO_CENTER - _left_virtual_boundary,2));
//	MinParkingWidth  = LEFT_EDGE_TO_CENTER + m_VehilceConfig->RadiusRearRight - MIN_TURN_RADIUS + _left_virtual_boundary;
//	delete m_VehilceConfig;
}

void ParallelPlanning::Work(PercaptionInformation *p)
{
	switch(_parallel_planning_state)
	{
		case WaitStart:
			if(0x51 == _command)
			{
				_trial_status = 0;
				_reverse_cnt = 0;
				_parallel_planning_state = EnterParkingPointPlanning;
			}
			break;

		case EnterParkingPointPlanning:
			ReversedTrial(p);
			_command = 0x52;
			_parallel_planning_state = WaitStart;
			break;

		case null:

			break;
		default:
			break;
	}
}

void ParallelPlanning::Control(VehicleController *c)
{
	switch(_parallel_control_state)
	{
		case WaitPlanningFinish:
			break;

		default:
			break;
	}
}


void ParallelPlanning::ReversedTrial(PercaptionInformation *inf)
{
	// 车位信息发送
	m_ParallelPlanningTerminal.ParkingMsgSend(*inf,_front_margin_boundary,_rear_margin_boundary);
	/// 车辆初始位置信息
	_init_parking.Center      = Vector2d(inf->PositionX,inf->PositionY);
	_init_parking.AttitudeYaw = inf->AttitudeYaw;
	// TODO 终端信息 车辆初始位置信息
	m_ParallelPlanningTerminal.VehicleInitPositionSend(_init_parking);
	/// 车位虚拟边界计算
	_right_virtual_boundary = -inf->ParkingWidth + _right_margin_boundary;
	_front_virtual_boundary = inf->ParkingLength - _front_margin_boundary;
	_rear_virtual_boundary  = _rear_margin_boundary;
	// 车库点计算
	parking_right_rear = Vector2d(_rear_virtual_boundary,_right_virtual_boundary);
	parking_right_front = Vector2d(_front_virtual_boundary,_right_virtual_boundary);
	_parking_left_front = Vector2d(_front_virtual_boundary,0);
	// 根据车位宽度，确定车辆最终停车的横向位置
	m_ParallelVehilceConfig.EdgeRadius(MIN_TURN_RADIUS);
	MinParkingWidth  = LEFT_EDGE_TO_CENTER + m_ParallelVehilceConfig.RadiusRearRight - MIN_TURN_RADIUS + _right_margin_boundary;
	if(inf->ParkingWidth >= MinParkingWidth)//库位宽度足够
	{
		enter_point.Y = -LEFT_EDGE_TO_CENTER + _lat_margin_move;
	}
	else //库位宽度太小，调整y轴方向位置
	{
		enter_point.Y = -LEFT_EDGE_TO_CENTER + MinParkingWidth - inf->ParkingWidth;
	}

	// 根据车位长度，确定车辆最终的纵向位置
	MinParkingLength = REAR_EDGE_TO_CENTER + sqrtf(powf(m_ParallelVehilceConfig.RadiusFrontRight,2) - powf(MIN_TURN_RADIUS + enter_point.Y,2));
	if( inf->ParkingLength > (MinParkingLength + _front_margin_boundary + _rear_margin_boundary))//满足一次入库条件
	{
		enter_point.X = _rear_margin_boundary  + REAR_EDGE_TO_CENTER + (inf->ParkingLength - _front_margin_boundary - _rear_margin_boundary - LENGHT)*0.5;
		_enter_parking.Center = enter_point;
	}
	else//不满足一次入库，需多次尝试
	{
		enter_point.X = inf->ParkingLength - _front_margin_boundary - FRONT_EDGE_TO_CENTER;
		front_trial_body.Center = enter_point;
		front_trial_body.AttitudeYaw = 0.0f;

		enter_point.X = _rear_margin_boundary  + REAR_EDGE_TO_CENTER ;
		rear_trial_body.Center = enter_point;
		rear_trial_body.AttitudeYaw = 0.0f;

		m_ParallelPlanningTerminal.FrontTrialPositionSend(front_trial_body,_reverse_cnt);
		m_ParallelPlanningTerminal.RearTrialPositionSend(rear_trial_body,_reverse_cnt);
		while( (0 == _trial_status) && (_reverse_cnt < 9))
		{
			/*+--------------+----+----+----+*/
			/*+ Init Status  + 1  + 2  + 3  +*/
			/*+--------------+----+----+----+*/
			/*+    Front     + -R + +L + -R +*/
			/*+--------------+----+----+----+*/
			/*+    Rear      + +L + -R + +L +*/
			/*+--------------+----+----+----+*/
			_reverse_cnt++;
			if(	_reverse_cnt % 2 )// 1 3 5
			{
				front_trial_body.OneTrial(-MIN_TURN_RADIUS, parking_right_rear);
				rear_trial_body.OneTrial(MIN_TURN_RADIUS, parking_right_front);
				m_ParallelPlanningTerminal.FrontTrialPositionSend(front_trial_body,_reverse_cnt);
				m_ParallelPlanningTerminal.RearTrialPositionSend(rear_trial_body,_reverse_cnt);

				front_trial_body.RotationCenter(MIN_TURN_RADIUS);
				m_ParallelVehilceConfig.EdgeRadius(MIN_TURN_RADIUS);
				_trial_status = (front_trial_body.getRotation() - _parking_left_front).Length() >= m_ParallelVehilceConfig.RadiusFrontRight ? 1 : 0;
				if(_trial_status)
				{
					front_trial_body.EdgePoint();
					_enter_parking = front_trial_body;
				}
			}
			else // 2 4 6
			{
				front_trial_body.OneTrial(MIN_TURN_RADIUS, parking_right_front);
				rear_trial_body.OneTrial(-MIN_TURN_RADIUS, parking_right_rear);
				m_ParallelPlanningTerminal.FrontTrialPositionSend(front_trial_body,_reverse_cnt);
				m_ParallelPlanningTerminal.RearTrialPositionSend(rear_trial_body,_reverse_cnt);

				rear_trial_body.RotationCenter(MIN_TURN_RADIUS);
				m_ParallelVehilceConfig.EdgeRadius(MIN_TURN_RADIUS);
				_trial_status = (rear_trial_body.getRotation() - _parking_left_front).Length() >= m_ParallelVehilceConfig.RadiusFrontRight ? 1 : 0;
				if(_trial_status)
				{
					rear_trial_body.EdgePoint();
					_enter_parking = rear_trial_body;
				}
			}
		}
		if( (0 == _trial_status) && (_reverse_cnt >= 9))
		{
			m_ParallelPlanningTerminal.EnterParkingPositionSend(_enter_parking, _reverse_cnt,0);
		}
		else
		{
			m_ParallelPlanningTerminal.EnterParkingPositionSend(_enter_parking, _reverse_cnt,0x5A);
		}
	}
}

void ParallelPlanning::TransitionArc(PercaptionInformation *inf)
{
//	Line   _l_init;
//	Circle _circle_left;
//	Circle _circle_right;

	Line cr_line;

	//圆心和直线变量初始化
	_line_init.Point.X = inf->PositionX;
	_line_init.Point.Y = inf->PositionY;
	_line_init.Angle   = inf->AttitudeYaw;

	_circle_left.Center = _enter_parking.Rotation;
	_circle_left.Radius = MIN_TURN_RADIUS;

	_circle_right.Radius = MIN_TURN_RADIUS;
	// 计算初始右侧圆心位置
	m_AlgebraicGeometry.Tangent_CCL(_line_init,_circle_left,&_circle_right);
	do
	{
	// 沿右下方向移动圆心坐标
	cr_line.Point = _circle_right.Center;
	cr_line.Angle = _line_init.Angle + PI/4;
	_circle_right.Center.X = _circle_right.Center.getX() + 0.1;
	_circle_right.Center.Y = m_AlgebraicGeometry.LinearAlgebra(cr_line, _circle_right.Center.getX());
	// 重新根据右圆心坐标计算右圆半径和切点坐标
	m_AlgebraicGeometry.Tangent_CL(_line_init,&_circle_right,&_line_init_circle_right_tangent);
	// 计算左右圆之间切线的切点坐标
	m_AlgebraicGeometry.Tangent_CLC(_circle_left, _circle_right,&_line_middle,&_line_middle_circle_left_tangent,&_line_middle_circle_right_tangent);
	}
	//碰撞判定
	while(
			(_circle_right.Radius - RIGHT_EDGE_TO_CENTER) < (_parking_left_front - _circle_right.Center).Length() ||
			(_line_middle_circle_left_tangent - _line_middle_circle_right_tangent).Length() < K*1
	);
}
/**************************************************************************************************/
float ParallelPlanning::getLeftVirtualBoundary()           { return  _left_virtual_boundary;}
void  ParallelPlanning::setLeftVirtualBoundary(float value){ _left_virtual_boundary = value;}

float ParallelPlanning::getRightVirtualBoundary()           { return  _right_virtual_boundary;}
void  ParallelPlanning::setRightVirtualBoundary(float value){ _right_virtual_boundary = value;}

float ParallelPlanning::getFrontVirtualBoundary()           { return  _front_virtual_boundary;}
void  ParallelPlanning::setFrontVirtualBoundary(float value){ _front_virtual_boundary = value;}

float ParallelPlanning::getRearVirtualBoundary()           { return  _rear_virtual_boundary;}
void  ParallelPlanning::setRearVirtualBoundary(float value){ _rear_virtual_boundary = value;}
/**************************************************************************************************/
float ParallelPlanning::getLatMarginMove()           { return  _lat_margin_move;}
void  ParallelPlanning::setLatMarginMove(float value){ _lat_margin_move = value;}

float ParallelPlanning::getRightMarginBoundary()           { return  _right_margin_boundary;}
void  ParallelPlanning::setRightMarginBoundary(float value){ _right_margin_boundary = value;}

float ParallelPlanning::getFrontMarginBoundary()           { return  _front_margin_boundary;}
void  ParallelPlanning::setFrontMarginBoundary(float value){ _front_margin_boundary = value;}

float ParallelPlanning::getRearMarginBoundary()           { return  _rear_margin_boundary;}
void  ParallelPlanning::setRearMarginBoundary(float value){ _rear_margin_boundary = value;}
/**************************************************************************************************/
VehicleBody ParallelPlanning::getInitParking()                 { return  _init_parking;}
void        ParallelPlanning::setInitParking(VehicleBody value){ _init_parking = value;}

VehicleBody ParallelPlanning::getEnterParking()                 { return  _enter_parking;}
void        ParallelPlanning::setEnterParking(VehicleBody value){ _enter_parking = value;}
/**************************************************************************************************/
uint8_t ParallelPlanning::getCommand()             { return  _command;}
void    ParallelPlanning::setCommand(uint8_t value){ _command = value;}

uint8_t ParallelPlanning::getConsoleState()             { return  _console_state;}
void    ParallelPlanning::setConsoleState(uint8_t value){ _console_state = value;}

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
/* 1.0	 Guohua Zhu     January 21 2019      Add  Control State Machine      */
/* 1.0	 Guohua Zhu     January 23 2019      Add  Control Other Machine      */
/*****************************************************************************/
#include "parallel_planning.h"

Terminal m_ParallelPlanningTerminal;
AlgebraicGeometry m_AlgebraicGeometry;
VehilceConfig m_ParallelVehilceConfig;

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

	ParkingStatus.setContainer(this);
	ParkingStatus.getter(&ParallelPlanning::getParkingStatus);
	ParkingStatus.setter(&ParallelPlanning::setParkingStatus);

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
	_parking_status = 0;
//	VehilceConfig *m_VehilceConfig = new VehilceConfig();
//	m_VehilceConfig->EdgeRadiusUpdate(MIN_TURN_RADIUS);
//	MinParkingLength = REAR_EDGE_TO_CENTER + sqrtf(powf(m_VehilceConfig->RadiusFrontRight,2) - powf(MIN_TURN_RADIUS - LEFT_EDGE_TO_CENTER - _left_virtual_boundary,2));
//	MinParkingWidth  = LEFT_EDGE_TO_CENTER + m_VehilceConfig->RadiusRearRight - MIN_TURN_RADIUS + _left_virtual_boundary;
//	delete m_VehilceConfig;
}

void ParallelPlanning::Work(Percaption *p,VehicleState *s)
{
	switch(_parallel_planning_state)
	{
		case WaitStart:
			if(0x51 == _command)
			{
				_parking_status = 1;
				_trial_status = 0;
				_reverse_cnt = 0;
				_parallel_planning_state = EnterParkingPointPlanning;
			}
			break;

		case EnterParkingPointPlanning:
			ReversedTrial(p);
			if( (0 == _trial_status) && (_reverse_cnt >= 9))//fail
			{
				_command = 0;
				_parallel_planning_state = WaitStart;
			}
			else
			{
				_parallel_planning_state = FirstArcPlanning;
			}
			break;

		case FirstArcPlanning:
			TransitionArc(p,s);
			_parallel_planning_state = SteeringTurnningCalculate;
			break;

		case SteeringTurnningCalculate:
			TurnningPoint(s);
			_command = 0x60;
			_parallel_planning_state = WaitStart;
			break;

		default:
			break;
	}
}

int8_t ParallelPlanning::InitPositionAdjustMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u)
{
	switch(_adjust_state)
	{
		case InitPointFrontAdjust:
				_parallel_command.Gear          = Drive;
				_parallel_command.SteeringAngle = 0;
				_parallel_command.Acceleration  = PLANNING_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;

				_adjust_state = InitPointMove;
			break;

		case InitPointMove:
			if(msg->Gear > 0 && msg->Gear < 7)
			{
				_parallel_command.Velocity = STRAIGHT_VELOCITY;
				_parallel_command.ControlEnable.B.VelocityEnable = 1;

				_adjust_state = InitPoitArriveJudge;
			}
			break;

		case InitPoitArriveJudge:
			if(s->getPosition().X > _line_init_circle_right_turn.Point.getX())
			{
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_parallel_command.Acceleration = PLANNING_BRAKING;

				_adjust_state = WaitVehicleStop;
			}
			break;

		case WaitVehicleStop:
			if(2 == msg->WheelSpeedDirection)
			{
				_adjust_state = InitPointFrontAdjust;
				return SUCCESS;
			}
			break;

		default:

			break;
	}
	ctl->Update(_parallel_command);
	return FAIL;
}

int8_t ParallelPlanning::CurveTrajectoryMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u)
{
	float angle_vector;
	VehicleBody motion_body;
	switch(_curve_state)
	{
		case GearShift:
			_parallel_command.Gear          = Reverse;
			_parallel_command.SteeringAngle = 0;
			_parallel_command.Acceleration  = PLANNING_BRAKING;
			_parallel_command.ControlEnable.B.VelocityEnable = 0;
			_curve_state = VehicleMove;
			break;

		case VehicleMove:
			if(0x09 == msg->Gear)
			{
				_parallel_command.Velocity = CURVE_VELOCITY;
				_parallel_command.ControlEnable.B.VelocityEnable = 1;
				_curve_state = FirstTurnPoint;
			}
			break;

		case FirstTurnPoint:
			// 考虑转向角执行延迟时间
			if( (s->getPosition().getX() -_line_init_circle_right_turn.Point.getX()) < s->LinearRate * TURN_FEEDFORWARD_TIME)
			{
				_parallel_command.SteeringAngle = _line_init_circle_right_turn.SteeringAngle;
				_parallel_command.SteeringAngleRate = s->LinearRate * RK;
				_curve_state = SecondTurnPoint;
			}
			break;

		case SecondTurnPoint:
			// 根据当前车速，实时更新转向角速度
			_parallel_command.SteeringAngleRate = s->LinearRate * RK;
			if(fabsf(msg->SteeringAngle - _parallel_command.SteeringAngle ) < 1)
			{
				angle_vector = (s->getPosition() - _line_middle_circle_right_turn.Point).Angle();
			}
			// 象限点判定控制
			angle_vector = (s->getPosition() - _line_middle_circle_right_turn.Point).Angle();
			if(angle_vector > 0 && angle_vector < PI_2 )
			{
				if(m_AlgebraicGeometry.ArcLength(s->getPosition(), _line_middle_circle_right_turn.Point, _circle_right.Radius) < s->LinearRate * TURN_FEEDFORWARD_TIME)
				{
					_parallel_command.SteeringAngle = _line_middle_circle_right_turn.SteeringAngle;
					_curve_state = ThirdTurnPoint;
				}
			}
			else
			{
				_parallel_command.SteeringAngle = _line_middle_circle_right_turn.SteeringAngle;
				_curve_state = ThirdTurnPoint;
			}
			break;

		case ThirdTurnPoint:
			_parallel_command.SteeringAngleRate = s->LinearRate * RK;

			angle_vector = (s->getPosition() - _line_middle_circle_left_turn.Point).Angle();
			if(angle_vector > 0 && angle_vector < PI_2 )
			{
				if((s->getPosition() -_line_middle_circle_left_turn.Point).Length() < s->LinearRate * TURN_FEEDFORWARD_TIME)
				{
					_parallel_command.SteeringAngle = _line_middle_circle_left_turn.SteeringAngle;
					_curve_state = WaitArrive;
				}
			}
			else
			{
				_parallel_command.SteeringAngle = _line_middle_circle_left_turn.SteeringAngle;
				_curve_state = WaitArrive;
			}
			break;

		case WaitArrive:
			_parallel_command.SteeringAngleRate = s->LinearRate * RK;

//			angle_vector = (s->getPosition() - _enter_parking.Center).Angle();
//			if( (s->Yaw * _circle_left.Radius) < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R))
//			{
//				_parallel_command.Acceleration  = PLANNING_BRAKING;
//				_parallel_command.ControlEnable.B.VelocityEnable = 0;
//				_curve_state = WaitStill;
//			}
//			else if(angle_vector > 0 && angle_vector < PI_2 )
//			{
//				if( m_AlgebraicGeometry.ArcLength(s->getPosition(), _enter_parking.Center, _circle_left.Radius) < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R))
//				{
//					_parallel_command.Acceleration  = PLANNING_BRAKING;
//					_parallel_command.ControlEnable.B.VelocityEnable = 0;
//					_curve_state = WaitStill;
//				}
//			}
//			else
//			{
//				_parallel_command.Deceleration  = EMERGENCY_BRAKING;
//				_parallel_command.Acceleration  = EMERGENCY_BRAKING;
//				_parallel_command.ControlEnable.B.VelocityEnable = 0;
//				_parallel_command.ControlEnable.B.DecelerationEnable = 1;
//				_curve_state = WaitStill;
//			}

			motion_body.Center = s->getPosition();
			motion_body.AttitudeYaw = s->getYaw();
			motion_body.EdgePoint();
			if( (motion_body.getRearLeft().getX() < _rear_virtual_boundary) || (motion_body.getRearRight().getY() < _right_virtual_boundary) )
			{
				_parallel_command.Deceleration  = EMERGENCY_BRAKING;
				_parallel_command.Acceleration  = EMERGENCY_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_parallel_command.ControlEnable.B.DecelerationEnable = 1;
				_curve_state = WaitStill;
			}
			else
			{
				if( (motion_body.getRearLeft().getX() - _rear_virtual_boundary) < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R) ||
					(motion_body.getRearRight().getY() - _right_virtual_boundary) < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R))
				{
					_parallel_command.Acceleration  = PLANNING_BRAKING;
					_parallel_command.ControlEnable.B.VelocityEnable = 0;
					_curve_state = WaitStill;
				}
			}
			if( (s->Yaw * MIN_RIGHT_TURN_RADIUS) < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R))
			{
				_parallel_command.Acceleration  = PLANNING_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_curve_state = WaitStill;
			}
			break;

		case WaitStill:
			if(2 == msg->WheelSpeedDirection)
			{
				_curve_state = GearShift;
				if( s->Yaw < 0.01)
				{
					return PARKING_FINISH;
				}
				else
				{
					return SUCCESS;
				}
			}
			else
			{
//				angle_vector = (s->getPosition() - _enter_parking.Center).Angle();
//				if( (angle_vector > 0) && (angle_vector < PI_2) )
//				{
//
//				}
//				else
//				{
//					_parallel_command.Deceleration  = EMERGENCY_BRAKING;
//					_parallel_command.Acceleration  = EMERGENCY_BRAKING;
//					_parallel_command.ControlEnable.B.VelocityEnable = 0;
//					_parallel_command.ControlEnable.B.DecelerationEnable = 1;
//				}

				motion_body.Center = s->getPosition();
				motion_body.AttitudeYaw = s->getYaw();
				motion_body.EdgePoint();
				if( (motion_body.getRearLeft().getX() < _rear_virtual_boundary) || (motion_body.getRearRight().getY() < _right_virtual_boundary) || (s->Yaw < 0))
				{
					_parallel_command.Deceleration  = EMERGENCY_BRAKING;
					_parallel_command.Acceleration  = EMERGENCY_BRAKING;
					_parallel_command.ControlEnable.B.VelocityEnable = 0;
					_parallel_command.ControlEnable.B.DecelerationEnable = 1;
				}
			}
			break;
		default:

			break;
	}
	ctl->Update(_parallel_command);
	return FAIL;
}

int8_t ParallelPlanning::RightFrontTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u)
{
	VehicleBody motion_body;
	switch(_right_front_state)
	{
		case RightFrontTrialGearShift:
			_parallel_command.Gear              =  Drive;
			_parallel_command.SteeringAngle     = -MAX_STEERING_ANGLE;
			_parallel_command.SteeringAngleRate =  STEERING_RATE;
			_parallel_command.ControlEnable.B.VelocityEnable = 0;
			_right_front_state = RightFrontTrialVehicleMove;
			break;

		case RightFrontTrialVehicleMove:
			if((msg->Gear > 0 && msg->Gear <7 ) && (fabsf(msg->SteeringAngle - _parallel_command.SteeringAngle ) < 1))
			{
				_parallel_command.Velocity = CURVE_VELOCITY;
				_parallel_command.ControlEnable.B.VelocityEnable = 1;
				_parallel_command.ControlEnable.B.AccelerationEnable = 0;
				_acc_disable_cnt = 0;
				_right_front_state = RightFrontTrialDisableACC;
			}
			break;

		case RightFrontTrialDisableACC:
			_acc_disable_cnt++;
			if(_acc_disable_cnt > ACC_DISABLE_TIME)
			{
				_parallel_command.ControlEnable.B.AccelerationEnable = 1;
				_right_front_state = RightFrontTrialWaitArrive;
			}
			break;

		case RightFrontTrialWaitArrive:
			motion_body.Center = s->getPosition();
			motion_body.AttitudeYaw = s->getYaw();
			motion_body.EdgePoint();
			if(motion_body.getFrontRight().getX() > _front_virtual_boundary)
			{
				_parallel_command.Deceleration  = EMERGENCY_BRAKING;
				_parallel_command.Acceleration  = EMERGENCY_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_parallel_command.ControlEnable.B.DecelerationEnable = 1;
				_right_front_state = RightFrontTrialWaitStill;
			}
			else
			{
				if( (_front_virtual_boundary - motion_body.getFrontRight().getX()) < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R))
				{
					_parallel_command.Acceleration  = PLANNING_BRAKING;
					_parallel_command.ControlEnable.B.VelocityEnable = 0;
					_right_front_state = RightFrontTrialWaitStill;
				}
			}
			if( (s->Yaw * MIN_RIGHT_TURN_RADIUS) < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R))
			{
				_parallel_command.Acceleration  = PLANNING_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_right_front_state = RightFrontTrialWaitStill;
			}
			break;

		case RightFrontTrialWaitStill:
			if(2 == msg->WheelSpeedDirection)
			{
				_right_front_state = RightFrontTrialGearShift;
				if( s->Yaw < 0.01)
				{
					return PARKING_FINISH;
				}
				else
				{
					return SUCCESS;
				}
			}
			else
			{
				motion_body.Center = s->getPosition();
				motion_body.AttitudeYaw = s->getYaw();
				motion_body.EdgePoint();
				if( (motion_body.getFrontRight().getX() > _front_virtual_boundary)  || (s->Yaw < 0) )
				{
					_parallel_command.Deceleration  = EMERGENCY_BRAKING;
					_parallel_command.Acceleration  = EMERGENCY_BRAKING;
					_parallel_command.ControlEnable.B.VelocityEnable = 0;
					_parallel_command.ControlEnable.B.DecelerationEnable = 1;
				}
			}
			break;
		default:

			break;
	}
	ctl->Update(_parallel_command);
	return FAIL;
}

int8_t ParallelPlanning::LeftRearTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u)
{
	VehicleBody motion_body;
	switch(_left_rear_state)
	{
		case LeftRearTrialGearShift:
			_parallel_command.Gear              =  Reverse;
			_parallel_command.SteeringAngle     =  MAX_STEERING_ANGLE;
			_parallel_command.SteeringAngleRate =  STEERING_RATE;
			_parallel_command.ControlEnable.B.VelocityEnable = 0;
			_left_rear_state = LeftRearTrialVehicleMove;
			break;

		case LeftRearTrialVehicleMove:
			if((0x09 == msg->Gear) && (fabsf(msg->SteeringAngle - _parallel_command.SteeringAngle ) < 1))
			{
				_parallel_command.Velocity = CURVE_VELOCITY;
				_parallel_command.ControlEnable.B.VelocityEnable = 1;
				_parallel_command.ControlEnable.B.AccelerationEnable = 0;
				_acc_disable_cnt = 0;
				_left_rear_state = LeftRearTrialDisableACC;
			}
			break;

		case LeftRearTrialDisableACC:
			_acc_disable_cnt++;
			if(_acc_disable_cnt > ACC_DISABLE_TIME)
			{
				_parallel_command.ControlEnable.B.AccelerationEnable = 1;
				_left_rear_state = LeftRearTrialWaitArrive;
			}
			break;

		case LeftRearTrialWaitArrive:
			motion_body.Center = s->getPosition();
			motion_body.AttitudeYaw = s->getYaw();
			motion_body.EdgePoint();
			if( (motion_body.getRearLeft().getX() < _rear_virtual_boundary) || (motion_body.getRearRight().getY() < _right_virtual_boundary) )
			{
				_parallel_command.Deceleration  = EMERGENCY_BRAKING;
				_parallel_command.Acceleration  = EMERGENCY_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_parallel_command.ControlEnable.B.DecelerationEnable = 1;
				_left_rear_state = LeftRearTrialWaitStill;
			}
			else
			{
				if( (motion_body.getRearLeft().getX() - _rear_virtual_boundary) < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R) ||
					(motion_body.getRearRight().getY() - _right_virtual_boundary) < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R))
				{
					_parallel_command.Acceleration  = PLANNING_BRAKING;
					_parallel_command.ControlEnable.B.VelocityEnable = 0;
					_left_rear_state = LeftRearTrialWaitStill;
				}
			}
			if( (s->Yaw * MIN_RIGHT_TURN_RADIUS) < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R))
			{
				_parallel_command.Acceleration  = PLANNING_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_left_rear_state = LeftRearTrialWaitStill;
			}
			break;

		case LeftRearTrialWaitStill:
			if(2 == msg->WheelSpeedDirection)
			{
				_left_rear_state = LeftRearTrialGearShift;
				if( s->Yaw < 0.01)
				{
					return PARKING_FINISH;
				}
				else
				{
					return SUCCESS;
				}
			}
			else
			{
				motion_body.Center = s->getPosition();
				motion_body.AttitudeYaw = s->getYaw();
				motion_body.EdgePoint();
				if( (motion_body.getRearLeft().getX() < _rear_virtual_boundary) || (motion_body.getRearRight().getY() < _right_virtual_boundary) )
				{
					_parallel_command.Deceleration  = EMERGENCY_BRAKING;
					_parallel_command.Acceleration  = EMERGENCY_BRAKING;
					_parallel_command.ControlEnable.B.VelocityEnable = 0;
					_parallel_command.ControlEnable.B.DecelerationEnable = 1;
				}
			}
			break;

		default:

			break;
	}
	ctl->Update(_parallel_command);
	return FAIL;
}

int8_t ParallelPlanning::ParkingCompletedMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u)
{
	switch(_parking_complete_state)
	{
		case GearShiftJudge:
			if(s->getPosition().getX() < ( _parking_center_point.getX() - PARKING_CENTER_MARGIN ))
			{
				_parallel_command.Gear =  Drive;
				_parking_complete_state = FrontMoveAdjust;
			}
			else if(s->getPosition().getX() > ( _parking_center_point.getX() + PARKING_CENTER_MARGIN ))
			{
				_parallel_command.Gear =  Reverse;
				_parking_complete_state = RearMoveAdjust;
			}
			else
			{
				_parallel_command.Gear =  Praking;
				_parking_complete_state = ParkingStill;
			}
			_parallel_command.SteeringAngle     =  0;
			_parallel_command.SteeringAngleRate =  STEERING_RATE;
			_parallel_command.ControlEnable.B.VelocityEnable = 0;

			break;

		case FrontMoveAdjust:
			if((msg->Gear > 0 && msg->Gear < 7) && (fabsf(msg->SteeringAngle - _parallel_command.SteeringAngle ) < 1))
			{
				_parallel_command.Velocity = 0.2;
				_parallel_command.ControlEnable.B.VelocityEnable = 1;
				_parallel_command.ControlEnable.B.AccelerationEnable = 0;
				_acc_disable_cnt = 0;
				_parking_complete_state = FrontMoveDisableACC;
			}
			break;

		case RearMoveAdjust:
			if((0x09 == msg->Gear) && (fabsf(msg->SteeringAngle - _parallel_command.SteeringAngle ) < 1))
			{
				_parallel_command.Velocity = 0.2;
				_parallel_command.ControlEnable.B.VelocityEnable = 1;
				_parallel_command.ControlEnable.B.AccelerationEnable = 0;
				_acc_disable_cnt = 0;
				_parking_complete_state = RearMoveDisableACC;
			}
			break;

		case FrontMoveDisableACC:
			_acc_disable_cnt++;
			if(_acc_disable_cnt > ACC_DISABLE_TIME)
			{
				_parallel_command.ControlEnable.B.AccelerationEnable = 1;
				_parking_complete_state = FrontWaitArrive;
			}
			break;

		case RearMoveDisableACC:
			_acc_disable_cnt++;
			if(_acc_disable_cnt > ACC_DISABLE_TIME)
			{
				_parallel_command.ControlEnable.B.AccelerationEnable = 1;
				_parking_complete_state = RearWaitArrive;
			}
			break;

		case FrontWaitArrive:
			// 带预测的停车
			if( (s->getPosition() - _parking_center_point).Length() < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R))
			{
				_parallel_command.Acceleration  = PLANNING_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_parking_complete_state = FrontMoveStill;
			}
			// 未能按照规划停止，需紧急停车
			if( s->getPosition().getX() > _parking_center_point.getX() )
			{
				_parallel_command.Deceleration  = EMERGENCY_BRAKING;
				_parallel_command.Acceleration  = EMERGENCY_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_parallel_command.ControlEnable.B.DecelerationEnable = 1;
				_parking_complete_state = FrontMoveStill;
			}
			break;

		case RearWaitArrive:
			// 带预测的停车
			if( (s->getPosition() - _parking_center_point).Length() < ( s->LinearRate * s->LinearRate * 0.5 * PLANNING_BRAKING_R))
			{
				_parallel_command.Acceleration  = PLANNING_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_parking_complete_state = RearMoveStill;
			}
			// 未能按照规划停止，需紧急停车
			if( s->getPosition().getX() < _parking_center_point.getX() )
			{
				_parallel_command.Deceleration  = EMERGENCY_BRAKING;
				_parallel_command.Acceleration  = EMERGENCY_BRAKING;
				_parallel_command.ControlEnable.B.VelocityEnable = 0;
				_parallel_command.ControlEnable.B.DecelerationEnable = 1;
				_parking_complete_state = RearMoveStill;
			}
			break;

		case FrontMoveStill:
			if(2 == msg->WheelSpeedDirection)
			{
				_parking_complete_state = GearShiftJudge;
				return SUCCESS;
			}
			else
			{
				if( s->getPosition().getX() > _parking_center_point.getX() )
				{
					_parallel_command.Deceleration  = EMERGENCY_BRAKING;
					_parallel_command.Acceleration  = EMERGENCY_BRAKING;
					_parallel_command.ControlEnable.B.VelocityEnable = 0;
					_parallel_command.ControlEnable.B.DecelerationEnable = 1;
				}
			}
			break;

		case RearMoveStill:
			if(2 == msg->WheelSpeedDirection)
			{
				_parking_complete_state = GearShiftJudge;
				return SUCCESS;
			}
			else
			{
				if( s->getPosition().getX() > _parking_center_point.getX() )
				{
					_parallel_command.Deceleration  = EMERGENCY_BRAKING;
					_parallel_command.Acceleration  = EMERGENCY_BRAKING;
					_parallel_command.ControlEnable.B.VelocityEnable = 0;
					_parallel_command.ControlEnable.B.DecelerationEnable = 1;
				}
			}
			break;

		case ParkingStill:
			if(2 == msg->WheelSpeedDirection)
			{
				_parking_complete_state = GearShiftJudge;
				return SUCCESS;
			}
			break;

		default:

			break;
	}
	ctl->Update(_parallel_command);
	return FAIL;
}

void ParallelPlanning::Control(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u)
{
	int8_t status;
	switch(_parallel_control_state)
	{
		case WaitPlanningFinish:
			if( 0x70 == _command )
			{
				_parking_status = 2;
				_command = 0x00;
				_parallel_command.ControlEnable.R   = 0xE0;
				_parallel_command.Gear              = Praking;
				_parallel_command.SteeringAngle     = 0;
				_parallel_command.SteeringAngleRate = STEERING_RATE;
				_parallel_command.Acceleration      = PLANNING_BRAKING;
				_parallel_command.Deceleration      = 0;
				_parallel_command.Torque            = 0;
				_parallel_command.Velocity          = 0;
				ctl->Update(_parallel_command);
				_parallel_control_state = InitPointJudge;
			}
			break;

		case InitPointJudge:
			if(s->getPosition().getX() < _line_init_circle_right_turn.Point.getX())
			{
				_parallel_control_state = InitPointAdjust;
			}
			else
			{
				_parallel_control_state = CurveTrajectory;
			}
			break;

		case InitPointAdjust:
			if(SUCCESS == InitPositionAdjustMachine(ctl,msg,s,u))
			{
				_parallel_control_state = CurveTrajectory;
			}
			break;

		case CurveTrajectory:
			status = CurveTrajectoryMachine(ctl,msg,s,u);
			if(SUCCESS == status)
			{
				if(_reverse_cnt > 0)
				{
					_forward_cnt = _reverse_cnt - 1;
				}
				else
				{
					_forward_cnt = 0;
				}
				_parallel_control_state = RightFrontTrial;
			}
			else if(PARKING_FINISH == status)
			{
				_parallel_control_state = ParkingComplete;
			}
			break;

		case RightFrontTrial:
			status = RightFrontTrialMachine(ctl,msg,s,u);
			if(SUCCESS == status)
			{
				_parallel_control_state = LeftRearTrial;
			}
			else if(PARKING_FINISH == status)
			{
				_parallel_control_state = ParkingComplete;
			}
			break;

		case LeftRearTrial:
			status = LeftRearTrialMachine(ctl,msg,s,u);
			if(SUCCESS == status)
			{
				_parallel_control_state = RightFrontTrial;
			}
			else if( PARKING_FINISH == status )
			{
				_parallel_control_state = ParkingComplete;
			}
			break;

		case ParkingComplete:
			status = ParkingCompletedMachine(ctl,msg,s,u);
			if(SUCCESS == status)
			{
				_parking_status = 3;
				_parallel_control_state = WaitPlanningFinish;
			}
			break;
		default:

			break;
	}

}


void ParallelPlanning::ReversedTrial(Percaption *inf)
{
	// 车位信息发送
	m_ParallelPlanningTerminal.ParkingMsgSend(inf,_front_margin_boundary,_rear_margin_boundary);
	/// 车辆初始位置信息
//	_init_parking.Center      = Vector2d(inf->PositionX,inf->PositionY);
//	_init_parking.AttitudeYaw = inf->AttitudeYaw;
	// TODO 终端信息 车辆初始位置信息
//	m_ParallelPlanningTerminal.VehicleInitPositionSend(_init_parking);
	/// 车位虚拟边界计算
	_right_virtual_boundary = -inf->ParkingWidth  + _right_margin_boundary;
	_front_virtual_boundary =  inf->ParkingLength - _front_margin_boundary;
	_rear_virtual_boundary  = _rear_margin_boundary;
	// 车库点计算
	parking_right_rear  = Vector2d(_rear_virtual_boundary,_right_virtual_boundary);
	parking_right_front = Vector2d(_front_virtual_boundary,_right_virtual_boundary);
	_parking_left_front = Vector2d(_front_virtual_boundary,0);

	// 根据车位宽度，确定车辆最终停车的横向位置
	m_ParallelVehilceConfig.EdgeRadius(MIN_LEFT_TURN_RADIUS);
	MinParkingWidth  = LEFT_EDGE_TO_CENTER + m_ParallelVehilceConfig.RadiusRearRight - MIN_LEFT_TURN_RADIUS + _right_margin_boundary;
	if(inf->ParkingWidth >= MinParkingWidth)//库位宽度足够
	{
		enter_point.Y = -LEFT_EDGE_TO_CENTER + _lat_margin_move;
	}
	else //库位宽度太小，调整y轴方向位置
	{
		enter_point.Y = -LEFT_EDGE_TO_CENTER + MinParkingWidth - inf->ParkingWidth;
	}
	_parking_center_point = Vector2d( (_front_virtual_boundary - _rear_virtual_boundary - LENGHT)*0.5 + REAR_EDGE_TO_CENTER,enter_point.Y);

	m_ParallelPlanningTerminal.ParkingCenterPointSend(_parking_center_point);
	// 根据车位长度，确定车辆最终的纵向位置
	MinParkingLength = REAR_EDGE_TO_CENTER + sqrtf(powf(m_ParallelVehilceConfig.RadiusFrontRight,2) - powf(MIN_LEFT_TURN_RADIUS + enter_point.Y,2));
	if( inf->ParkingLength > (MinParkingLength + _front_margin_boundary + _rear_margin_boundary))//满足一次入库条件
	{
		enter_point.X = _rear_margin_boundary  + REAR_EDGE_TO_CENTER + (inf->ParkingLength - _front_margin_boundary - _rear_margin_boundary - MinParkingLength)*0.5;
		_enter_parking.Center = enter_point;
		_enter_parking.RotationCenter(MIN_LEFT_TURN_RADIUS);

		rear_trial_body.Center = enter_point;
		rear_trial_body.AttitudeYaw = 0.0f;
		m_ParallelPlanningTerminal.RearTrialPositionSend(rear_trial_body,_reverse_cnt);
	}
	else//不满足一次入库，需多次尝试
	{
		enter_point.X = inf->ParkingLength - _front_margin_boundary - FRONT_EDGE_TO_CENTER;
		front_trial_body.Center = enter_point;
		front_trial_body.AttitudeYaw = 0.0f;
		front_trial_arrary[_reverse_cnt] = enter_point;

		enter_point.X = _rear_margin_boundary  + REAR_EDGE_TO_CENTER ;
		rear_trial_body.Center = enter_point;
		rear_trial_body.AttitudeYaw = 0.0f;
		rear_trial_arrary[_reverse_cnt]  = enter_point;

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
				front_trial_body.OneTrial(-MIN_RIGHT_TURN_RADIUS, parking_right_rear);
				rear_trial_body.OneTrial(MIN_LEFT_TURN_RADIUS, parking_right_front);
				m_ParallelPlanningTerminal.FrontTrialPositionSend(front_trial_body,_reverse_cnt);
				m_ParallelPlanningTerminal.RearTrialPositionSend(rear_trial_body,_reverse_cnt);

				front_trial_body.RotationCenter(MIN_LEFT_TURN_RADIUS);
				m_ParallelVehilceConfig.EdgeRadius(MIN_LEFT_TURN_RADIUS);
				_trial_status = (front_trial_body.getRotation() - _parking_left_front).Length() >= m_ParallelVehilceConfig.RadiusFrontRight ? 1 : 0;
				if(_trial_status)
				{
					front_trial_body.EdgePoint();
					_enter_parking = front_trial_body;
				}
			}
			else // 2 4 6
			{
				front_trial_body.OneTrial(MIN_LEFT_TURN_RADIUS, parking_right_front);
				rear_trial_body.OneTrial(-MIN_RIGHT_TURN_RADIUS, parking_right_rear);
				m_ParallelPlanningTerminal.FrontTrialPositionSend(front_trial_body,_reverse_cnt);
				m_ParallelPlanningTerminal.RearTrialPositionSend(rear_trial_body,_reverse_cnt);

				rear_trial_body.RotationCenter(MIN_LEFT_TURN_RADIUS);
				m_ParallelVehilceConfig.EdgeRadius(MIN_LEFT_TURN_RADIUS);
				_trial_status = (rear_trial_body.getRotation() - _parking_left_front).Length() >= m_ParallelVehilceConfig.RadiusFrontRight ? 1 : 0;
				if(_trial_status)
				{
					rear_trial_body.EdgePoint();
					_enter_parking = rear_trial_body;
				}
			}
			front_trial_arrary[_reverse_cnt]      = front_trial_body.Center;
			rear_trial_arrary[_reverse_cnt]       = rear_trial_body.Center;
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

void ParallelPlanning::TransitionArc(Percaption *inf,VehicleState *s)
{
	Line cr_line;

	//圆心和直线变量初始化
	_line_init.Point.X = inf->PositionX;
	_line_init.Point.Y = inf->PositionY;
	_line_init.Angle   = inf->AttitudeYaw;

	_circle_left.Center = _enter_parking.Rotation;
	_circle_left.Radius = MIN_LEFT_TURN_RADIUS;

	_circle_right.Radius = MIN_RIGHT_TURN_RADIUS;
	// 计算初始右侧圆心位置
	m_AlgebraicGeometry.Tangent_CCL(_line_init,_circle_left,&_circle_right);
	do
	{
		// 沿右下方向移动圆心坐标
		cr_line.Point = _circle_right.Center;
		cr_line.Angle = _line_init.Angle - PI/4;
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
			(_line_middle_circle_left_tangent - _line_middle_circle_right_tangent).Length() < 2 * K * s->SteeringAngleCalculate(_circle_right.Radius)
	);
}

void ParallelPlanning::TurnningPoint(VehicleState *s)
{
	Vector2d Ahead;
	float ahead_angle;

	// line first point
	_line_init_circle_right_turn.SteeringAngle = s->SteeringAngleCalculate(-_circle_right.Radius);
	_ahead_distance = - K * _line_init_circle_right_turn.SteeringAngle * 0.5;
	Ahead = Vector2d(_ahead_distance,0);
	_line_init_circle_right_turn.Point = _line_init_circle_right_tangent + Ahead.rotate(_line_init.Angle);
	m_ParallelPlanningTerminal.TurnPointSend(_line_init_circle_right_turn,0);
	// turning arc:sencond point
	ahead_angle = _ahead_distance / _circle_right.Radius;
	_line_middle_circle_right_turn.Point = _circle_right.Center +
   (_line_middle_circle_right_tangent    - _circle_right.Center).rotate(-ahead_angle);
	_line_middle_circle_right_turn.SteeringAngle = 0;
	m_ParallelPlanningTerminal.TurnPointSend(_line_middle_circle_right_turn,1);
	// line:third point
	_line_middle_circle_left_turn.SteeringAngle = s->SteeringAngleCalculate(_circle_left.Radius);
	_ahead_distance = K* _line_middle_circle_left_turn.SteeringAngle * 0.5;
	Ahead = Vector2d(_ahead_distance,0);
	_line_middle_circle_left_turn.Point = _line_middle_circle_left_tangent + Ahead.rotate(_line_middle.Angle);
	m_ParallelPlanningTerminal.TurnPointSend(_line_middle_circle_left_turn,2);
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

uint8_t ParallelPlanning::getParkingStatus()             { return  _parking_status;}
void    ParallelPlanning::setParkingStatus(uint8_t value){ _parking_status = value;}

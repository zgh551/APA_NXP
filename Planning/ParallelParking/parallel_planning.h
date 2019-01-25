/*****************************************************************************/
/* FILE NAME: parallel_planning.h                 COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the parallel parking trajectory planning                     */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 9  2019      Initial Version                 */
/* 1.0	 Guohua Zhu     January 16 2019      Add ReversedTrial Function      */
/* 1.0	 Guohua Zhu     January 17 2019      Add TransitionArc Function      */
/*****************************************************************************/

#include "derivative.h"
#include "property.h"
#include "math.h"
#include "chang_an_configure.h"
#include "common_configure.h"
#include "vehilce_config.h"
#include "vehicle_body.h"
#include "Terminal.h"
#include "planning.h"
#include "algebraic_geometry.h"


#ifndef PARALLELPARKING_PARALLEL_PLANNING_H_
#define PARALLELPARKING_PARALLEL_PLANNING_H_

#define K  0.0016   //  0.8/500
#define RK 625

// 轨迹规划状态
typedef enum _ParallelPlanningState
{
	WaitStart = 0,
	EnterParkingPointPlanning,
	FirstArcPlanning,
	SteeringTurnningCalculate,
}ParallelPlanningState;

// 平行泊车控制总体状态
typedef enum _ParallelControlState
{
	WaitPlanningFinish = 0,
	InitPointJudge,       //Initial Position Judge State
	InitPointAdjust,
	CurveTrajectory,
	RightFrontTrial,
	LeftRearTrial,
	ParkingComplete
}ParallelControlState;
// 初始位置调整状态
typedef enum _InitPointAdjuatState //Initial Position Adjustment Relative State
{
	InitPointFrontAdjust,
	InitPointMove,
	InitPoitArriveJudge,
	WaitVehicleStop,
}InitPointAdjuatState;
// 曲线估计段状态
typedef enum _CurveTrajectoryState
{
	GearShift,
	VehicleMove,
	FirstTurnPoint,
	SecondTurnPoint,
	ThirdTurnPoint,
	WaitArrive,
	WaitStill
}CurveTrajectoryState;

typedef enum _RightFrontTrialState
{
	RightFrontTrialGearShift,
	RightFrontTrialVehicleMove,
	RightFrontTrialSelectTrial,
	RightFrontTrialWaitArrive,
	RightFrontTrialWaitStill
}RightFrontTrialState;

typedef enum _LeftRearTrialState
{
	LeftRearTrialGearShift,
	LeftRearTrialVehicleMove,
	LeftRearTrialSelectTrial,
	LeftRearTrialWaitArrive,
	LeftRearTrialWaitStill
}LeftRearTrialState;

typedef enum _ParkingCompleteState
{
	GearShiftJudge,
	FrontMoveAdjust,
	RearMoveAdjust,
	FrontWaitArrive,
	RearWaitArrive,
	FrontMoveStill,
	RearMoveStill,
	ParkingStill
}ParkingCompleteState;

class ParallelPlanning : public Planning
{
public:
	ParallelPlanning();
	virtual ~ParallelPlanning();

	void Init() override;
	void Work(Percaption *p,VehicleState *s) override;
	void Control(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u) override;

	/***************************************************************/
	 int8_t InitPositionAdjustMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u);
	 int8_t CurveTrajectoryMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u);
	 int8_t RightFrontTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u);
	 int8_t LeftRearTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u);
	 int8_t ParkingCompletedMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Ultrasonic *u);
	/***************************************************************/

	void ReversedTrial(Percaption *inf);

	void TransitionArc(Percaption *inf,VehicleState *s);

	void TurnningPoint(VehicleState *s);
	/***************************************************************/
	float getLeftVirtualBoundary();
	void  setLeftVirtualBoundary(float value);
	Property<ParallelPlanning,float,READ_WRITE> LeftVirtualBoundary;

	float getRightVirtualBoundary();
	void  setRightVirtualBoundary(float value);
	Property<ParallelPlanning,float,READ_WRITE> RightVirtualBoundary;

	float getFrontVirtualBoundary();
	void  setFrontVirtualBoundary(float value);
	Property<ParallelPlanning,float,READ_WRITE> FrontVirtualBoundary;

	float getRearVirtualBoundary();
	void  setRearVirtualBoundary(float value);
	Property<ParallelPlanning,float,READ_WRITE> RearVirtualBoundary;
	/***************************************************************/
	float getLatMarginMove();
	void  setLatMarginMove(float value);
	Property<ParallelPlanning,float,READ_WRITE> LatMarginMove;

	float getRightMarginBoundary();
	void  setRightMarginBoundary(float value);
	Property<ParallelPlanning,float,READ_WRITE> RightMarginBoundary;

	float getFrontMarginBoundary();
	void  setFrontMarginBoundary(float value);
	Property<ParallelPlanning,float,READ_WRITE> FrontMarginBoundary;

	float getRearMarginBoundary();
	void  setRearMarginBoundary(float value);
	Property<ParallelPlanning,float,READ_WRITE> RearMarginBoundary;

	VehicleBody getInitParking();
	void        setInitParking(VehicleBody value);
	Property<ParallelPlanning,VehicleBody,READ_WRITE> InitParking;

	VehicleBody getEnterParking();
	void        setEnterParking(VehicleBody value);
	Property<ParallelPlanning,VehicleBody,READ_WRITE> EnterParking;

	uint8_t getCommand();
	void    setCommand(uint8_t value);
	Property<ParallelPlanning,uint8_t,READ_WRITE> Command;

	uint8_t getConsoleState();
	void    setConsoleState(uint8_t value);
	Property<ParallelPlanning,uint8_t,READ_WRITE> ConsoleState;

	uint8_t getParkingStatus();
	void    setParkingStatus(uint8_t value);
	Property<ParallelPlanning,uint8_t,READ_WRITE> ParkingStatus;
protected:
	VehicleBody front_trial_body,rear_trial_body;
	Vector2d    enter_point;
	Vector2d    parking_right_rear,parking_right_front;//库位边角点

private:
	ParallelPlanningState _parallel_planning_state;
	ParallelControlState  _parallel_control_state;
	///////////////////////////////////////////////
	InitPointAdjuatState _adjust_state;
	CurveTrajectoryState _curve_state;

	RightFrontTrialState _right_front_state;
	LeftRearTrialState   _left_rear_state;

	ParkingCompleteState _parking_complete_state;
	///////////////////////////////////////////////
	uint8_t _command;
	uint8_t _console_state;
	uint8_t _reverse_cnt;
	uint8_t _forward_cnt;
	uint8_t _trial_status;
	uint8_t _parking_status;

	float _left_virtual_boundary;
	float _right_virtual_boundary;
	float _front_virtual_boundary;
	float _rear_virtual_boundary;

	float _lat_margin_move;
	float _right_margin_boundary;
	float _front_margin_boundary;
	float _rear_margin_boundary;

	Vector2d _parking_left_front;
	Vector2d _parking_center_point;

	VehicleBody _init_parking;//
	VehicleBody _enter_parking;//

	Line   _line_init;
	Line   _line_middle;
	Circle _circle_left;
	Circle _circle_right;

	// tangent point
	Vector2d _line_init_circle_right_tangent;
	Vector2d _line_middle_circle_right_tangent;
	Vector2d _line_middle_circle_left_tangent;

	// turning point
	Turn _line_init_circle_right_turn;
	Turn _line_middle_circle_right_turn;
	Turn _line_middle_circle_left_turn;

	Vector2d _right_front_trial;
	Vector2d _left_rear_trial;
	// trial point

	Vector2d front_trial_arrary[12];
	Vector2d rear_trial_arrary[12];

	float _ahead_distance;

	ControlCommand _parallel_command;
};

#endif /* PARALLELPARKING_PARALLEL_PLANNING_H_ */

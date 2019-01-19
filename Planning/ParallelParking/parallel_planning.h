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
#include "vehilce_config.h"
#include "vehicle_body.h"
#include "Terminal.h"
#include "planning.h"
#include "algebraic_geometry.h"

#ifndef PARALLELPARKING_PARALLEL_PLANNING_H_
#define PARALLELPARKING_PARALLEL_PLANNING_H_

#define K 1

typedef enum _ParallelPlanningState
{
	WaitStart = 0,
	EnterParkingPointPlanning,
	null
}ParallelPlanningState;

typedef enum _ParallelControlState
{
	WaitPlanningFinish = 0,

}ParallelControlState;

class ParallelPlanning : public Planning
{
public:
	ParallelPlanning();
	virtual ~ParallelPlanning();


	void Init() override;
	void Work(PercaptionInformation *p) override;
	void Control(VehicleController *c) override;


	void ReversedTrial(PercaptionInformation *inf);

	void TransitionArc(PercaptionInformation *inf);
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
private:
	ParallelPlanningState _parallel_planning_state;
	ParallelControlState  _parallel_control_state;
	uint8_t _command;
	uint8_t _console_state;
	uint8_t _reverse_cnt;
	uint8_t _trial_status;

	float _left_virtual_boundary;
	float _right_virtual_boundary;
	float _front_virtual_boundary;
	float _rear_virtual_boundary;

	float _lat_margin_move;
	float _right_margin_boundary;
	float _front_margin_boundary;
	float _rear_margin_boundary;

	Vector2d _parking_left_front;

	VehicleBody _init_parking;//泊车初始位置信息
	VehicleBody _enter_parking;//入库点位置信息

	Line   _line_init;
	Line   _line_middle;
	Circle _circle_left;
	Circle _circle_right;

	Vector2d _line_init_circle_right_tangent;
	Vector2d _line_middle_circle_right_tangent;
	Vector2d _line_middle_circle_left_tangent;

	Vector2d _line_init_circle_right_turn;
	Vector2d _line_middle_circle_right_turn;
	Vector2d _line_middle_circle_left_turn;
};

#endif /* PARALLELPARKING_PARALLEL_PLANNING_H_ */

/*****************************************************************************/
/* FILE NAME: algebraic_geometry.h                COPYRIGHT (c) Motovis 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the algebraic geometry aclculate method  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 16 2019      Initial Version                 */
/*****************************************************************************/

#ifndef MATH_ALGEBRAIC_GEOMETRY_H_
#define MATH_ALGEBRAIC_GEOMETRY_H_

#include "derivative.h"
#include "property.h"
#include "vector_2d.h"
#include "math.h"
#include "chang_an_configure.h"
#include "common_configure.h"

typedef struct _Line
{
	Vector2d Point;
	float Angle;
}Line;

typedef struct _Circle
{
	Vector2d Center;
	float Radius;
}Circle;

class AlgebraicGeometry {
public:
	AlgebraicGeometry();
	virtual ~AlgebraicGeometry();

	//二次方程求解
	float QuadraticEquation(float a,float b,float c);

	//过一点的直线方程
	float LinearAlgebra(Line l,float x);

	// 已知圆上两点和半径，算两点间的弧长
	float ArcLength(Vector2d a,Vector2d b,float r);
	//已知圆的半径，求与圆和直线相切圆的坐标位置
	void Tangent_CCL(Line l,Circle cl,Circle *cr);
	// 已知线段和圆心，求相切圆的半径
	void Tangent_CL(Line l,Circle *c,Vector2d *p);
	// 已知两圆心位置，求与两圆相切直线的切点位置
	void Tangent_CLC(Circle cl,Circle cr,Line *lm,Vector2d *ll,Vector2d *lr);
private:

};

#endif /* MATH_ALGEBRAIC_GEOMETRY_H_ */

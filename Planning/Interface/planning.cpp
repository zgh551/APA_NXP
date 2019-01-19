/*****************************************************************************/
/* FILE NAME: path_plannig.cpp                    COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the trajectory planning interface  					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 9 2019      Initial Version                  */
/*****************************************************************************/
#include <planning.h>

Planning::Planning() {
	// TODO Auto-generated constructor stub
	MinParkingLength.setContainer(this);
	MinParkingLength.getter(&Planning::getMinParkingLength);
	MinParkingLength.setter(&Planning::setMinParkingLength);

	MinParkingWidth.setContainer(this);
	MinParkingWidth.getter(&Planning::getMinParkingWidth);
	MinParkingWidth.setter(&Planning::setMinParkingWidth);
}

Planning::~Planning() {
	// TODO Auto-generated destructor stub
}

float Planning::getMinParkingLength()           { return  _min_parking_length;}
void  Planning::setMinParkingLength(float value){ _min_parking_length = value;}

float Planning::getMinParkingWidth()           { return  _min_parking_width;}
void  Planning::setMinParkingWidth(float value){ _min_parking_width = value;}


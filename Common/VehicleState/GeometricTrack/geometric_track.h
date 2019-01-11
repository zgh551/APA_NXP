/*
 * geometric_track.h
 *
 *  Created on: 2019��1��2��
 *      Author: zhuguohua
 */

#ifndef VEHICLESTATE_GEOMETRICTRACK_GEOMETRIC_TRACK_H_
#define VEHICLESTATE_GEOMETRICTRACK_GEOMETRIC_TRACK_H_

#include "math.h"
#include "../Interface/vehicle_state.h"
#include "chang_an_configure.h"

typedef struct _fit_ratio
{
	float a;
	float b;
}FitRatio;

class GeometricTrack :public VehicleState
{
public:
	GeometricTrack();
	virtual ~GeometricTrack();

	void Init(void);

	void VelocityUpdate(MessageManager *msg,float dt) override;
	void PulseUpdate(MessageManager *msg,float dt) override;

	float SteeringAngle2TurnningRadiusExp(float steer,float a,float b);

	float SteeringAngle2TurnningRadius(float steer,float a,float b);

	float TurnRadiusCalculate(float steering_angle);
private:
	float _last_yaw;

	uint8_t _last_rear_left_pulse;
	uint8_t _last_rear_right_pulse;
};

#endif /* VEHICLESTATE_GEOMETRICTRACK_GEOMETRIC_TRACK_H_ */

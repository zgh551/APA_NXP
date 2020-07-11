/*
 * chery_s51ev_Message.h
 *
 *  Created on: 2020年6月29日
 *      Author: zhuguohua
 */

#ifndef CANBUS_CHERYS51EV_CHERY_S51EV_MESSAGE_H_
#define CANBUS_CHERYS51EV_CHERY_S51EV_MESSAGE_H_

#include "../Interface/message_manager.h"

class CheryS51EV_Message   : public MessageManager
{
public:
	CheryS51EV_Message();
	virtual ~CheryS51EV_Message();

	void Init() override;
	void Parse(const uint32_t id,const vuint8_t *data,const vuint32_t lenght) override;

private:
	float fifo_steering_angle_array[64];
	int16_t _index;
};

#endif /* CANBUS_CHERYS51EV_CHERY_S51EV_MESSAGE_H_ */

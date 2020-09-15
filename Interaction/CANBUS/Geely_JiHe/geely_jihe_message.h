/*
 * bo_rui_message.h
 *
 *  Created on: 2019骞�3鏈�15鏃�
 *      Author: zhuguohua
 */

#ifndef CANBUS_GEELY_JIHE_MESSAGE_H_
#define CANBUS_GEELY_JIHE_MESSAGE_H_

#include "../Interface/message_manager.h"

class GeelyJiHeMessage  : public MessageManager
{
public:
	GeelyJiHeMessage();
	virtual ~GeelyJiHeMessage();

	void Init() override;
	void Parse(const uint32_t id,const vuint8_t *data,const vuint32_t lenght) override;

private:
	float fifo_steering_angle_array[64];
	int16_t _index;

};

#endif /* CANBUS_BORUI_BO_RUI_MESSAGE_H_ */

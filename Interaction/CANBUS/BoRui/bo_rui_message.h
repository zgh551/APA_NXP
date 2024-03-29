/*
 * bo_rui_message.h
 *
 *  Created on: 2019年3月15日
 *      Author: zhuguohua
 */

#ifndef CANBUS_BORUI_BO_RUI_MESSAGE_H_
#define CANBUS_BORUI_BO_RUI_MESSAGE_H_

#include "derivative.h"
#include "property.h"
#include "Interface/message_manager.h"
#include "crc_compute.h"

#define V_M_S 0.015625

class BoRuiMessage  : public MessageManager
{
public:
	BoRuiMessage();
	virtual ~BoRuiMessage();

	void Init() override;
	void Parse(const uint32_t id,const vuint8_t *data,const vuint32_t lenght) override;

private:

};

#endif /* CANBUS_BORUI_BO_RUI_MESSAGE_H_ */

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
	uint16_t _crc_err_cnt;
	uint16_t _crc_suc_cnt;

	uint8_t _can_rx_id_0x122[8];//over the speed
	uint8_t _can_rx_id_0x123[8];//over the speed
	uint8_t _can_rx_id_0x125[8];//over the speed
	uint8_t _can_rx_id_0x1B0[8];// no
	uint8_t _can_rx_id_0x26D[8];
	uint8_t _can_rx_id_0x3F1[8];
	uint8_t _can_rx_id_0x1A5[8];
	uint8_t _can_rx_id_0xA6[8];
	uint8_t _can_rx_id_0x165[8];


};

#endif /* CANBUS_BORUI_BO_RUI_MESSAGE_H_ */

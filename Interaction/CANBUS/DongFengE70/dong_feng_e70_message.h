/*
 * dong_feng_e70_message.h
 *
 *  Created on: 2019年6月20日
 *      Author: zhuguohua
 */

#ifndef CANBUS_DONGFENGE70_DONG_FENG_E70_MESSAGE_H_
#define CANBUS_DONGFENGE70_DONG_FENG_E70_MESSAGE_H_

#include "derivative.h"
#include "property.h"
#include "Interface/message_manager.h"

#define V_M_S 0.00277777777777777777777777777778

class DongFengE70Message  : public MessageManager
{
public:
	DongFengE70Message();
	virtual ~DongFengE70Message();

	void Init() override;
	void Parse(const uint32_t id,const vuint8_t *data,const vuint32_t lenght) override;

	uint8_t getVCU_APA_ControlStatus();
	void    setVCU_APA_ControlStatus(uint8_t value);
	Property<DongFengE70Message,uint8_t,READ_WRITE> VCU_APA_ControlStatus;

	uint8_t getEPS_AvailabStatus();
	void    setEPS_AvailabStatus(uint8_t value);
	Property<DongFengE70Message,uint8_t,READ_WRITE> EPS_AvailabStatus;

	uint8_t getESC_APA_EnableStatus();
	void    setESC_APA_EnableStatus(uint8_t value);
	Property<DongFengE70Message,uint8_t,READ_WRITE> ESC_APA_EnableStatus;

	uint8_t getEPB_Status();
	void    setEPB_Status(uint8_t value);
	Property<DongFengE70Message,uint8_t,READ_WRITE> EPB_Status;
private:
	uint8_t _vcu_apa_control_st;
	uint8_t _eps_availab_status;
	uint8_t _esc_apa_enable_status;
	uint8_t _epb_status;

};

#endif /* CANBUS_DONGFENGE70_DONG_FENG_E70_MESSAGE_H_ */

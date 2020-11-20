/*
 * bo_rui_controller.cpp
 *
 *  Created on: 2019楠烇拷3閺堬拷15閺冿拷
 *      Author: zhuguohua
 */

#include "geely_jihe_controller.h"
CRC8 _crc8(CRC8::eAUTOSAR);

GeelyJiHeController::GeelyJiHeController() {
	// TODO Auto-generated constructor stub

}

GeelyJiHeController::~GeelyJiHeController() {
	// TODO Auto-generated destructor stub
}

void GeelyJiHeController::Init()
{
	// EPS State
	_eps_control_state = EPS_WaitRequest;
	_eps_timeout = 0;

	// VCU State
	_vcu_control_state = VCU_WaitRequest;
	_vcu_timeout = 0;

	// ESC State
	_esc_control_state = ESC_WaitRequest;
	_esc_timeout = 0;
}
/**
 * @brief start the vehicle controller.
 * @return true if successfully started.
 */
void GeelyJiHeController::Start()
{

}

/**
 * @brief stop the vehicle controller.
 */
void GeelyJiHeController::Stop()
{

}
  /**
   * @brief update the vehicle controller.
   * @param command the control command
   * @return error_code
   */
void GeelyJiHeController::Update(ControlCommand cmd)
{

}

void GeelyJiHeController::Update(APAControlCommand cmd)
{
	APAEnable         = cmd.ControlEnable.B.APAEnable;

	Gear              = cmd.Gear;

	SteeringAngle 	  = cmd.SteeringAngle;
	SteeringAngleRate = cmd.SteeringAngleRate;

	Velocity          = cmd.Velocity;
	Distance          = cmd.Distance;
}


void GeelyJiHeController::VehicleControl(void)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.length = 8;

	// 0x190: EPS Control ID
	m_CAN_Packet.id = 0x190;
	/// Data Mapping
	m_CAN_Packet.data[0] = 0;
	m_CAN_Packet.data[1] = 0;
	m_CAN_Packet.data[2] = 0;
	m_CAN_Packet.data[3] = (eps_request_ & 0x03) << 3;
	m_CAN_Packet.data[4] = steering_angle_request_ >> 8;
	m_CAN_Packet.data[5] = steering_angle_request_;
	m_CAN_Packet.data[6] = rolling_counter_ & 0x0f;
	m_CAN_Packet.data[7] = _crc8.crcCompute(m_CAN_Packet.data, 7);
	CAN0_TransmitMsg(m_CAN_Packet);
	CAN1_TransmitMsg(m_CAN_Packet);

	// 0x191: APA system checker
	m_CAN_Packet.id = 0x191;
	/// Data Mapping
	m_CAN_Packet.data[0] = 0;
	m_CAN_Packet.data[1] = 0x80; // The apa button pressed
	m_CAN_Packet.data[2] = 0;
	m_CAN_Packet.data[3] = 0x20; // The APA status:valid standby
	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 50;
	m_CAN_Packet.data[6] = rolling_counter_ & 0x0f;
	m_CAN_Packet.data[7] = _crc8.crcCompute(m_CAN_Packet.data, 7);
	CAN0_TransmitMsg(m_CAN_Packet);
	CAN1_TransmitMsg(m_CAN_Packet);

	// 0x135: Lower speed parking control
	m_CAN_Packet.id = 0x135;
	/// Data Mapping
	m_CAN_Packet.data[0] = esc_target_acceleration_request_;
	m_CAN_Packet.data[1] = esc_acc_upper_limit_;
	m_CAN_Packet.data[2] = esc_acc_lower_limit_;
	m_CAN_Packet.data[3] = stop_distance_;
	m_CAN_Packet.data[4] = (esc_jerk_max_ & 0x7f) | ((brake_preferred_request_ & 0x01) << 7);
	m_CAN_Packet.data[5] = (esc_jerk_min_ & 0x7f) | ((driver_off_request_      & 0x01) << 7);
	m_CAN_Packet.data[6] = ((esc_prefill_request_ & 0x01) << 4) | (rolling_counter_ & 0x0f);
	m_CAN_Packet.data[7] = _crc8.crcCompute(m_CAN_Packet.data, 7);
	CAN0_TransmitMsg(m_CAN_Packet);

	// 0x136: Gear control
	m_CAN_Packet.id = 0x136;
	/// Data Mapping
	m_CAN_Packet.data[0] = ((emergency_brake_request_ & 0x01) << 7)
						 | ((standstill_request_      & 0x01) << 3)
						 | ( vlc_mode_request_        & 0x07);
	m_CAN_Packet.data[1] = vlc_shutdown_request_ & 0x03;
	m_CAN_Packet.data[2] = ((gear_enable_  & 0x03) << 3)
			    		 |  (gear_request_ & 0x07); // Gear control
	m_CAN_Packet.data[3] = 0;
	m_CAN_Packet.data[4] = 0;
	m_CAN_Packet.data[5] = 0;
	m_CAN_Packet.data[6] = rolling_counter_ & 0x0f;
	m_CAN_Packet.data[7] = _crc8.crcCompute(m_CAN_Packet.data, 7);
	CAN0_TransmitMsg(m_CAN_Packet);

	// 0x1A1: High speed acc control
	m_CAN_Packet.id = 0x1A1;
	/// Data Mapping
	m_CAN_Packet.data[0] = alod_target_acceleration_request_;
	m_CAN_Packet.data[1] = alod_jerk_max_ & 0x7f;
	m_CAN_Packet.data[2] = alod_acc_upper_limit_; // Gear control
	m_CAN_Packet.data[3] = alod_acc_lower_limit_;
	m_CAN_Packet.data[4] = (alod_mode_request_ & 0x0f) << 4;
	m_CAN_Packet.data[5] = rolling_counter_ & 0x0f;
	m_CAN_Packet.data[6] = alod_jerk_min_ & 0x7f;
	m_CAN_Packet.data[7] = _crc8.crcCompute(m_CAN_Packet.data, 7);
	CAN0_TransmitMsg(m_CAN_Packet);
}

void GeelyJiHeController::EPS_Push(void)
{
	CAN_Packet m_CAN_Packet;
	m_CAN_Packet.length = 8;

	// 0x190: EPS Control ID
	m_CAN_Packet.id = 0x190;
	/// Data Mapping
	m_CAN_Packet.data[0] = 0;
	m_CAN_Packet.data[1] = 0;
	m_CAN_Packet.data[2] = 0;
	m_CAN_Packet.data[3] = (eps_request_ & 0x03) << 3;
	m_CAN_Packet.data[4] = steering_angle_request_ >> 8;
	m_CAN_Packet.data[5] = steering_angle_request_;
	m_CAN_Packet.data[6] = rolling_counter_ & 0x0f;
	m_CAN_Packet.data[7] = _crc8.crcCompute(m_CAN_Packet.data, 7);
	CAN0_TransmitMsg(m_CAN_Packet);
}
void GeelyJiHeController::EPS_StateMachine(MessageManager& msg)
{
	switch(_eps_control_state)
	{
	case EPS_WaitRequest:
		if ( (1 == this->getAPAEnable()) && (1 == msg.getBrakePedalSts()))
		{
			_eps_timeout = 0; // timeout init
			_eps_control_state = EPS_WaitEnable;
		}
		else
		{
			eps_request_            = 0; // no request
			steering_angle_request_ = 0; // init steering angle
		}
		break;

	case EPS_WaitEnable:
		if (Enable == msg.getEPS_RequestFeedback())
		{
			_eps_control_state = EPS_Active;
		}
		else
		{
			if (_eps_timeout > 25) // 500ms timeout
			{
				_eps_control_state = EPS_Err;
			}
			else
			{
				_eps_timeout++;
				eps_request_            = 1; // eps request
				steering_angle_request_ = static_cast<int16_t>(msg.getSteeringAngle() * 10);
			}
		}
		break;

	case EPS_Active:
		if (0 == this->getAPAEnable())
		{
			_eps_timeout = 0; // timeout init
			_eps_control_state = EPS_WaitDisable;
		}
		else
		{
			eps_request_ = 2; // eps active
			steering_angle_request_ = static_cast<int16_t>(steering_angle_valid_request_ * 10);
		}

		if (1 == msg.getEPS_AbortFeedback())
		{
			if (Enable == msg.getEPS_RequestFeedback())
			{
				_eps_control_state = EPS_ResumableInterrupt; // Resumable interription
			}
			else
			{
				_eps_control_state = EPS_UnresumableInterrupt; // Unresumble interruption
			}
		}
		break;

	case EPS_WaitDisable:
		if (Disable == msg.getEPS_RequestFeedback())
		{
			_eps_control_state = EPS_WaitRequest;
		}
		else
		{
			if (_eps_timeout > 25) // 500ms timeout
			{
				_eps_control_state = EPS_Err;
			}
			else
			{
				_eps_timeout++;
				eps_request_ = 0; // no request
			}
		}
		break;

	case EPS_ResumableInterrupt:
		if (0 == msg.getEPS_AbortFeedback())
		{
			_eps_control_state = EPS_WaitRequest;
		}
		else if (Disable == msg.getEPS_RequestFeedback())
		{
			_eps_control_state = EPS_UnresumableInterrupt;
		}
		else
		{
			eps_request_ = 0; // no request
		}
		break;

	case EPS_UnresumableInterrupt:
		eps_request_ = 0; // no request
		_eps_control_state = EPS_Err;
		break;

	case EPS_Err:
		if (0 == this->getAPAEnable())
		{
			_eps_control_state = EPS_WaitRequest;
		}
		else
		{
			eps_request_ = 0; // no request
		}
		break;

	default:
		_eps_control_state = EPS_WaitRequest;
		break;
	}
}

void GeelyJiHeController::VCU_StateMachine(MessageManager& msg)
{
	switch(_vcu_control_state)
	{
	case VCU_WaitRequest:
		if ((1 == this->getAPAEnable()) && (1 == msg.getVehicleStandstill()))
		{
			_vcu_timeout = 0;
			_vcu_control_state = VCU_WaitActive;
		}
		else
		{
			gear_enable_ = 0   ; // no request
			gear_request_= None; // no request
			_vcu_control_state = VCU_WaitRequest;
		}
		break;

	case VCU_WaitActive:
		if (1 == msg.getVcuControlStatus())
		{
			_vcu_control_state = VCU_Active;
		}
		else
		{
			if (_vcu_timeout > 5) // 100ms timeout
			{
				_vcu_control_state = VCU_Err;
			}
			else
			{
				_vcu_timeout++;
				gear_enable_ = 1   ; // gear shift request
				gear_request_= None; // no request
			}
		}
		break;

	case VCU_Active:
		if (0 == this->getAPAEnable())
		{
			gear_enable_ = 2; // gear shift control active
			gear_request_= Parking; // set the parking gear
			_vcu_timeout = 0; // init the timeout count
			_vcu_control_state = VCU_ParkingArrival;
		}
		else
		{
			gear_enable_ = 2; // gear shift control active
			if (this->getGear() != gear_request_)// if the target gear change
			{
				gear_request_ = this->getGear(); //update the target gear
				_vcu_timeout  = 0;
				_vcu_control_state = VCU_Arrival;
			}
		}

		// Driver abort
		if (0 == msg.getVcuControlStatus())
		{
			if (1 == msg.getVcuDriverGearAbort())
			{
				_vcu_control_state = VCU_Err;
			}
			else if (1 == msg.getVcuEptFault())
			{
				_vcu_control_state = VCU_Err;
			}
			else
			{
				_vcu_control_state = VCU_Err;
			}
		}

		break;

	case VCU_Arrival:
		if (this->getGear() == msg.getActualGear())
		{
			_vcu_control_state = VCU_Active;
		}
		else
		{
			if (_vcu_timeout > 100)
			{
				_vcu_control_state = VCU_Err;
			}
			else
			{
				_vcu_timeout++;
			}
		}
		break;

	case VCU_ParkingArrival:
		if (this->getGear() == msg.getActualGear())
		{
			_vcu_control_state = VCU_WaitRequest;
		}
		else
		{
			if (_vcu_timeout > 100)// gear shift timeout 2s
			{
				_vcu_control_state = VCU_Err;
			}
			else
			{
				_vcu_timeout++;
			}
		}
		break;

	case VCU_Err:
		if (0 == this->getAPAEnable())
		{
			_vcu_control_state = VCU_WaitRequest;
		}
		else
		{
			gear_enable_ = 0   ; // no request
			gear_request_= None; // no request
		}
		break;

	default:
		_vcu_control_state = VCU_WaitRequest;
		break;
	}
}

void GeelyJiHeController::ESC_StateMachine(MessageManager& msg)
{
	// ESC
	switch(_esc_control_state)
	{
	case ESC_WaitRequest:
		if ((1 == this->getAPAEnable())
		&& (Available == msg.getApaAvailable())
		&& (Available == msg.getApaCddAvailable()))
		{
			_esc_control_state = ESC_WaitBrake;
		}
		else
		{
			// the request single init

			vlc_shutdown_request_    = 0; // 0x136: No Request
			standstill_request_      = 0; // 0x136: No Request
			emergency_brake_request_ = 0; // 0x136: No Request

			esc_prefill_request_     = 0; // 0x135: No Request
			driver_off_request_      = 0; // 0x135: No Request
			brake_preferred_request_ = 0; // 0x135: No Request


			// The Lower Speed initialize configure
			vlc_mode_request_        = 0; // 0x136: Off Mode (Lower Speed)
			stop_distance_           = 0;
			esc_target_acceleration_request_ = 100; // init the accelerate to 0 m/s2
			esc_acc_lower_limit_ = 140;
			esc_acc_upper_limit_ = 140;
			esc_jerk_min_ = 127;
			esc_jerk_max_ = 127;

			// The High Speed initialize configure
			alod_mode_request_       = 0; // 0x1A1: Off Mode (High Speed)
			alod_target_acceleration_request_ = 0;
			alod_acc_lower_limit_ = 0;
			alod_acc_upper_limit_ = 0;
			alod_jerk_min_ = 0;
			alod_jerk_max_ = 0;
		}
		break;

	case ESC_WaitBrake:
		if (1 == msg.getBrakePedalSts())
		{
			_esc_timeout = 0;
			_esc_control_state = ESC_WaitActive;
		}
		else
		{
			vlc_mode_request_        = 2; // 0x136: Standby Mode (Lower Speed)
			driver_off_request_      = 1; // 0x135: Request
			vlc_shutdown_request_    = 0; // 0x136: No Request
		}
		break;

	case ESC_WaitActive:
		if ((Active == msg.getApaActive())
		&&  (Active == msg.getApaCddActive()))
		{
			// The High Speed initialize configure
			alod_mode_request_       = 0; // 0x1A1: Off Mode (High Speed)
			alod_target_acceleration_request_ = 0;
			alod_acc_lower_limit_ = 0;
			alod_acc_upper_limit_ = 0;
			alod_jerk_min_ = 0;
			alod_jerk_max_ = 0;
			_esc_control_state = ESC_Active;
		}
		else
		{
			if (_esc_timeout++ > 250) // 500ms
			{
				_esc_control_state = ESC_Err;
			}
			else
			{
				vlc_mode_request_        = 3; // 0x136: On Mode (Lower Speed)
				alod_mode_request_       = 2; // 0x1A1: Standby Mode (High Speed)
			}
		}
		break;

	case ESC_Active:
		if (0 == this->getAPAEnable()) // parking finish
		{
			_esc_control_state = ESC_CloseRequest;
		}
		else
		{
			// if the actual speed great than 7.2km/h, then VLC is available
			if ((msg.getVehicleMiddleSpeed() > 2.0f)
			&& (Available == msg.getVlcAvailable())
			&& (Available == msg.getVlcCddAvailable()))
			{
				_esc_control_state = ESC_WaitAlodActive;
			}
			else
			{
				if (NoAvailable == msg.getApaAvailable()
				&& (NoAvailable == msg.getApaCddAvailable()))
				{
					_esc_control_state = ESC_EPB_Inactive;
				}
				else
				{
					if (this->getDistance() < 1.0e-6f)
					{
						stop_distance_ = 0.0f;
					}
					else
					{
						stop_distance_ = static_cast<uint8_t>(this->getDistance() * 100); // cm
					}

					// velocity is zero
					if (this->getVelocity() < 1.0e-6f)
					{
						if ((msg.getVehicleMiddleSpeed() < 1.0e-6f)
						&& (StandStill == msg.getWheelPulseDirection()))
						{
							vlc_mode_request_        = 6; // 0x136: StandStill Mode (Lower Speed)
							standstill_request_      = 1; // 0x136: Request
						}
						else
						{
							vlc_mode_request_        = 4; // 0x136: Brake Only Mode (Lower Speed)
							standstill_request_      = 1; // 0x136: Request
						}
					}
					else
					{
						if (1 == msg.getBrakePedalSts())
						{
							if (msg.getVehicleMiddleSpeed() < 1.0e-6f)
							{
								vlc_mode_request_        = 6; // 0x136: StandStill Mode (Lower Speed)
								standstill_request_      = 1; // 0x136: Request
							}
							else
							{
								vlc_mode_request_        = 4; // 0x136: Brake Only Mode (Lower Speed)
								standstill_request_      = 1; // 0x136: Request
							}
						}
						else
						{
							vlc_mode_request_        = 3; // 0x136: On Mode (Lower Speed)
							standstill_request_      = 0; // 0x136: Request
						}
					}

					// the driver off request
					driver_off_request_ = (msg.getVehicleMiddleSpeed() < 1.0e-6f) ? 1 : 0;

					// the accelerate and decelerate update, the range is [-5, 1.5]m/s2 (lower speed)
					esc_target_acceleration_request_ = (this->getTargetAcceleration() >  1.5f) ? 130:
													   (this->getTargetAcceleration() < -5.0f) ? 0  :
													    static_cast<uint8_t>((this->getTargetAcceleration() + 5) * 20);
					// lower speed accelerate confort zone lower limit range [-7, 5.75]
					esc_acc_lower_limit_ = (this->getAccLowerLimit() > 5.75f) ? 255 :
							               (this->getAccLowerLimit() < -7.0f) ? 0   :
							            	static_cast<uint8_t>((this->getAccLowerLimit() + 7) * 20);
					// lower speed accelerate confort zone upper limit range [-7, 5.75]
					esc_acc_upper_limit_ = (this->getAccUpperLimit() > 5.75f) ? 255 :
				               	   	   	   (this->getAccUpperLimit() < -7.0f) ? 0   :
										    static_cast<uint8_t>((this->getAccUpperLimit() + 7) * 20);
					// the minimum allowed jerk of target acceleration [-25.4, 0]
					esc_jerk_min_ = (this->getJerkMin() > -1.0e-6f) ? 127 :
	               	   	   	        (this->getJerkMin() < -25.4f  ) ? 0   :
							         static_cast<uint8_t>((this->getJerkMin() + 25.4f) * 5);
					esc_jerk_max_ = (this->getJerkMax() > 25.4f  ) ? 127 :
           	   	   	                (this->getJerkMax() < 1.0e-6f) ? 0   :
					                 static_cast<uint8_t>(this->getJerkMax() * 5);
				}
			}
		}
		break;

	case ESC_WaitAlodActive:
		if ((Active == msg.getVlcActive())
		&&  (Active == msg.getVlcCddActive()))
		{
			// The Lower Speed initialize configure
			vlc_mode_request_        = 2; // 0x136: StandBy Mode (Lower Speed)
			stop_distance_           = 0;
			esc_target_acceleration_request_ = 100; // init the accelerate to 0 m/s2
			esc_acc_lower_limit_ = 140;
			esc_acc_upper_limit_ = 140;
			esc_jerk_min_ = 127;
			esc_jerk_max_ = 127;
			vlc_shutdown_request_    = 0; // 0x136: No Request
			driver_off_request_      = 0; // 0x135: No Request
			_esc_control_state = ESC_AlodActive;
		}
		else
		{
			vlc_mode_request_        = 2; // 0x136: StandBy Mode (Lower Speed)
			alod_mode_request_       = 3; // 0x1A1: Active Control Mode (High Speed)
		}
		break;

	case ESC_AlodActive:
		if (msg.getVehicleMiddleSpeed() < 2.0f)
		{
			_esc_control_state = ESC_WaitActive;
		}
		else
		{
			// The High Speed initialize configure
			alod_mode_request_       = 3; // 0x1A1: Active Control Mode (High Speed)

			// the accelerate and decelerate update, the range is [-7, 5.75]m/s2 (lower speed)
			alod_target_acceleration_request_ = (this->getTargetAcceleration() > 5.75f) ? 255:
											    (this->getTargetAcceleration() < -7.0f) ? 0  :
											     static_cast<uint8_t>((this->getTargetAcceleration() + 7) * 20);
			// lower speed accelerate confort zone lower limit range [-7, 5.75]
			alod_acc_lower_limit_ = (this->getAccLowerLimit() > 5.75f) ? 255 :
					                (this->getAccLowerLimit() < -7.0f) ? 0   :
					            	 static_cast<uint8_t>((this->getAccLowerLimit() + 7) * 20);
			// lower speed accelerate confort zone upper limit range [-7, 5.75]
			alod_acc_upper_limit_ = (this->getAccUpperLimit() > 5.75f) ? 255 :
		               	   	   	    (this->getAccUpperLimit() < -7.0f) ? 0   :
								     static_cast<uint8_t>((this->getAccUpperLimit() + 7) * 20);
			// the minimum allowed jerk of target acceleration [-25.4, 0]
			alod_jerk_min_ = (this->getJerkMin() > -1.0e-6f) ? 127 :
           	   	   	         (this->getJerkMin() < -25.4f  ) ? 0   :
					          static_cast<uint8_t>((this->getJerkMin() + 25.4f) * 5);
			alod_jerk_max_ = (this->getJerkMax() > 25.4f  ) ? 127 :
   	   	   	                 (this->getJerkMax() < 1.0e-6f) ? 0   :
			                  static_cast<uint8_t>(this->getJerkMax() * 5);
		}
		break;

	case ESC_CloseRequest:
		vlc_mode_request_        = 1; // 0x136: Maneuver Finish Mode (Lower Speed)
		vlc_shutdown_request_    = 0; // 0x136: No Request
		_esc_control_state       = ESC_EPB_Inactive;
		break;

	case ESC_EPB_Inactive:
		if ((Inactive == msg.getApaActive())
		&&  (Inactive == msg.getApaCddActive()))
		{
			vlc_mode_request_        = 2; // 0x136: Standby Mode (Lower Speed)
			vlc_shutdown_request_    = 0; // 0x136: No Request
			_esc_control_state       = ESC_WaitRequest;
		}
		else
		{
			vlc_mode_request_        = 0; // 0x136: Off Mode (Lower Speed)
			vlc_shutdown_request_    = 3; // 0x136: Immediate off
		}
		break;

	case ESC_Err:
		if (0 == this->getAPAEnable())
		{
			_esc_control_state = ESC_WaitRequest;
		}
		else
		{

		}
		break;

	default:
		_esc_control_state = ESC_WaitRequest;
		break;
	}
}

void GeelyJiHeController::WorkStateMachine(MessageManager& msg)
{
	EPS_StateMachine(msg);
	VCU_StateMachine(msg);
	ESC_StateMachine(msg);
	SteeringAngleControl(0.02f, msg.getSteeringAngle());
}

void GeelyJiHeController::DataPush(void)
{
	this->VehicleControl();
	rolling_counter_++;
}

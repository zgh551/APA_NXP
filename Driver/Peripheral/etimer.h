/*
 * etimer.h
 *
 *  Created on: 2019骞�6鏈�27鏃�
 *      Author: zhuguohua
 */

#ifndef _INC_ETIMER_H_
#define _INC_ETIMER_H_

/**********************************************************************************************
* Includes
**********************************************************************************************/
#include "../System/derivative.h"

#ifdef __cplusplus
extern "C" {
#endif
/**********************************************************************************************
* Constants
**********************************************************************************************/


/**********************************************************************************************
* Macros
**********************************************************************************************/

#define LED_ON 0
#define LED_OFF 1

#define DEBOUNCEDELAYTIME 0xFFFF


/**********************************************************************************************
* Types
**********************************************************************************************/


/**********************************************************************************************
* Variables
**********************************************************************************************/


/**********************************************************************************************
* Global Variables
**********************************************************************************************/


/**********************************************************************************************
* Global Functions
**********************************************************************************************/
/*******************************************************************************
Function Name : eTimer0_Init
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer0 channel 0, 1 init for capture features
Issues        : NONE
*******************************************************************************/
void eTimer0_Init(void);

/*******************************************************************************
Function Name : eTimer1_Init
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer1 channel 0, 1 init for capture features
Issues        : NONE
*******************************************************************************/
void eTimer1_Init(void);

/*******************************************************************************
Function Name : eTimer2_Init
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer2 channel 0, 1 init for capture features
Issues        : NONE
*******************************************************************************/
void eTimer2_Init(void);

/*******************************************************************************
Function Name : eTimer0_StartInputCapture
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer0 start the input single capture with channel 0 1
Issues        : NONE
*******************************************************************************/
void eTimer0_StartInputCapture();

/*******************************************************************************
Function Name : eTimer1_StartInputCapture
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer1 start the input single capture with channel 0 5
Issues        : NONE
*******************************************************************************/
void eTimer1_StartInputCapture();

/*******************************************************************************
Function Name : eTimer2_StartInputCapture
Engineer      : Zhuguohua
Date          : June-27-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer2 start the input single capture with channel 0 2 3 4
Issues        : NONE
*******************************************************************************/
void eTimer2_StartInputCapture();

/*******************************************************************************
Function Name : eTimer1_CalculatePulse
Engineer      : Zhuguohua
Date          : June-28-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer1 calculate the pulse single frequence,period,duty
Issues        : NONE
*******************************************************************************/
void eTimer1_CalculatePulse();

/*******************************************************************************
Function Name : eTimer2_CalculatePulse
Engineer      : Zhuguohua
Date          : June-28-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer2 calculate the pulse single frequence,period,duty
Issues        : NONE
*******************************************************************************/
void eTimer2_CalculatePulse();


/*******************************************************************************
Function Name : eTimer1_OutputInit
Engineer      : Zhuguohua
Date          : July-2-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer1 variable frequency PWM mode
Issues        : NONE
*******************************************************************************/
void eTimer1_OutputInit(void);

/*******************************************************************************
Function Name : eTimer2_OutputInit
Engineer      : Zhuguohua
Date          : July-2-2019
Parameters    : NONE
Modifies      : NONE
Returns       : NONE
Notes         : eTimer2 variable frequency PWM mode
Issues        : NONE
*******************************************************************************/
void eTimer2_OutputInit(void);

void eTimer1Channel5OutputStart(void);

void eTimer1_Channel5SendWakeUp(void);
void eTimer1_Channel5Send_ID();

#ifdef __cplusplus
}
#endif



#endif /* PERIPHERAL_INC_ETIMER_H_ */

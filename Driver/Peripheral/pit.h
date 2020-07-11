 /*
  * pit.h
  *
  *  Created on: December 22 2018
  *      Author: Guohua Zhu
  */

#ifndef PIT_H_
#define PIT_H_

#include "../System/derivative.h"

#ifdef __cplusplus
extern "C" {
#endif

void PIT0_init(uint32_t);

void PIT1_init(uint32_t);

void PIT2_init(uint32_t);

void PIT_Configure();

#ifdef __cplusplus
}
#endif

#endif /* PIT_H_ */

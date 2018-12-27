/*
 * pit.h
 *
 *  Created on: Feb 25, 2016
 *      Author: B55457
 */

#ifndef PIT_H_
#define PIT_H_

#include "project.h"
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

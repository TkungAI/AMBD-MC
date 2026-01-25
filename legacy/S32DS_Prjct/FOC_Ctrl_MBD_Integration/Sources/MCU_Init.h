/*
 * MCU_Init.h
 *
 *  Created on: 2021Äê12ÔÂ9ÈÕ
 *      Author: k
 */

#ifndef MCU_INIT_H_
#define MCU_INIT_H_

#include "Cpu.h"

extern void LPIT_ISR(void);
extern void Motor_ISR (void);
extern void PDB0_ISR (void);
extern void PDB1_ISR (void);
extern void Button_ISR (void);

status_t MCU_Init (void);
status_t MCU_Start (void);

#endif /* MCU_INIT_H_ */

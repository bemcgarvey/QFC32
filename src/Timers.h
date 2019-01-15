//*******************************************************************//
// File: Timers.h                                                    //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Timer initialization and control                     //
//                                                                   //
//*******************************************************************// 

#ifndef TIMERS_H
#define	TIMERS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
    
void InitTimers(void);
void StartTimer1(void);
void StartTimer2(void);
void StartTimer3(void);
void StopTimer3(void);

extern volatile uint32_t systemTickCount;

#ifdef	__cplusplus
}
#endif

#endif	/* TIMERS_H */


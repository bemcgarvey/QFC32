//*******************************************************************//
// File: SystemInit.h                                                //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Initialize clock, pins, interrupts,                  //
//  and peripheral disable                                           //
//                                                                   //
//*******************************************************************// 

#ifndef SYSTEMINIT_H
#define	SYSTEMINIT_H

#ifdef	__cplusplus
extern "C" {
#endif

void InitSystem(void);
void InitInterruptSystem(void);
void InitExternalInt4(void);
void DisableExternalInt4(void);

#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEMINIT_H */


//*******************************************************************//
// File: Timers.c                                                    //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Timer initialization and control                     //
//                                                                   //
//*******************************************************************// 


#include <xc.h>
#include "Timers.h"

volatile uint32_t systemTickCount = 0;


void InitTimers(void) {
    T1CON = 0x10;  //Off, 1:8 prescale
    PR1 = 3000;   //1 ms
    TMR1 = 0;
    IPC1bits.T1IP = 7;
    IPC1bits.T1IS = 0;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    //Timer 2 is used by Input Capture
    T2CON = 0x30;  //Off, 1:8 prescale
    PR2 = 60000U;  //20 ms
    TMR2 = 0;
    //Timer 3 is used by Output Compare
    T3CON = 0x30;  //Off, 1:8 prescale
    PR3 = 60000U;  //20 ms
    TMR3 = 0;
    IPC3bits.T3IP = 6;
    IPC3bits.T3IS = 0;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
}

void StartTimer1(void) {
    T1CONbits.ON = 1;
}

void StartTimer2(void) {
    T2CONbits.ON = 1;
}

void StartTimer3(void) {
    T3CONbits.ON = 1;
}

void StopTimer3(void) {
    T3CONbits.ON = 0;
}



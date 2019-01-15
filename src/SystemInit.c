//*******************************************************************//
// File: SystemInit.c                                                //
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

#include <xc.h>
#include "SystemInit.h"


static void InitPins(void);
static void InitClock(void);
static void InitPMD(void);

void InitSystem(void) {
    InitClock();
    InitPins();
    InitPMD();
    InitInterruptSystem();
}


void InitInterruptSystem(void) {
    INTCONbits.MVEC = 1;   //Multi vector mode
    __builtin_set_isr_state(0); //Enable all interrupts
    __builtin_enable_interrupts();
}

void InitClock(void) {
    //Nothing required at this time
}

void InitPins(void) {
    ANSELA = 0;
    ANSELB = 0;
    LATA = 0;
    LATB = 0;
    TRISA = 0b10010;
    TRISB = 0b11101;
    SYSKEY = 0xaa996655;
    SYSKEY = 0x556699aa;
    CFGCONbits.IOLOCK = 0;
    RPA0R = 0b0101;  //OC1
    RPB1R = 0b0101;  //OC2
    RPB13R = 0b0101;  //OC4
    RPB14R = 0b0101;  //OC3
    IC4R = 0b0001;  //RB3
    IC3R = 0b0000;  //RA1
    IC1R = 0b0010;  //RA4
    IC5R = 0b0100;  //RB2
    IC2R = 0b0010;  //RB0
    INT4R = 0b0010; //RB4
    CFGCONbits.IOLOCK = 1;
    SYSKEY = 0x33333333; 
}

void InitPMD(void) {
    //Disable any unused peripherals
    SYSKEY = 0xaa996655;
    SYSKEY = 0x556699aa;
    CFGCONbits.PMDLOCK = 0;
    PMD1 = 0xffffffff;
    PMD2 = 0xffffffff;
    PMD5 = 0b101111111111111111;
    PMD6 = 0xffffffff;
    CFGCONbits.PMDLOCK = 1;
    SYSKEY = 0x33333333; 
}

void InitExternalInt4(void) {
    IPC4bits.INT4IP = 5;
    IPC4bits.INT4IS = 0;
    INTCONbits.INT4EP = 0;
    IFS0bits.INT4IF = 0;
    IEC0bits.INT4IE = 1;
}

void DisableExternalInt4(void) {
    IEC0bits.INT4IE = 0;
}
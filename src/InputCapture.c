//*******************************************************************//
// File: InputCapture.c                                              //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Initialize and control input capture modules for     //
//  aileron, elevator, rudder, mode and gain                         //
//                                                                   //
//*******************************************************************// 

#include <xc.h>
#include <stdint.h>
#include "InputCapture.h"


volatile bool ailUpdate;
volatile bool elevUpdate;
volatile bool rudUpdate;
volatile bool gainUpdate;
volatile bool modeUpdate;

void InitInputCapture(void) {
    
    //Aileron
    IC1CON = 0xA6;
    IPC1bits.IC1IP = 4;
    IPC1bits.IC1IS = 2;
    IFS0bits.IC1IF = 0;
    IEC0bits.IC1IE = 1;
    //Elevator
    IC4CON = 0xA6;
    IPC4bits.IC4IP = 4;
    IPC4bits.IC4IS = 2;
    IFS0bits.IC4IF = 0;
    IEC0bits.IC4IE = 1;
    //Rudder
    IC5CON = 0xA6;
    IPC5bits.IC5IP = 4;
    IPC5bits.IC5IS = 2;
    IFS0bits.IC5IF = 0;
    IEC0bits.IC5IE = 1;
    //Mode
    IC2CON = 0xA6;
    IPC2bits.IC2IP = 4;
    IPC2bits.IC2IS = 1;
    IFS0bits.IC2IF = 0;
    IEC0bits.IC2IE = 1;
    //Gain
    IC3CON = 0xA6;
    IPC3bits.IC3IP = 4;
    IPC3bits.IC3IS = 0;
    IFS0bits.IC3IF = 0;
    IEC0bits.IC3IE = 1;
}

void EnableAilInput(void) {
    ailUpdate = false;
    IC1CONbits.ON = 1;
}

void DisableAilInput(void) {
    IC1CONbits.ON = 0;
}

void EnableElevInput(void) {
    elevUpdate = false;
    IC4CONbits.ON = 1;
}

void DisableElevInput(void) {
    IC4CONbits.ON = 0;
}

void EnableRudInput(void){
    rudUpdate = false;
    IC5CONbits.ON = 1;
}

void DisableRudInput(void){
    IC5CONbits.ON = 0;
}

void EnableModeInput(void){
    modeUpdate = false;
    IC2CONbits.ON = 1;
}

void DisableModeInput(void) {
    IC2CONbits.ON = 0;
}

void EnableGainInput(void) {
    gainUpdate = false;
    IC3CONbits.ON = 1;
}

void DisableGainInput(void) {
    IC3CONbits.ON = 0;
}
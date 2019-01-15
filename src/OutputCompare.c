//*******************************************************************//
// File: OutputCompare.h                                             //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Initialize and control output compare modules for    //
//  aileron, elevator, rudder, and aileron2                          //
//                                                                   //
//*******************************************************************// 

#include <xc.h>
#include <stdint.h>
#include "OutputCompare.h"
#include "Data.h"

void InitOutputCompare(void) {
    OC1CON = 0x000d;    //Off, duel compare mode, TMR3
    OC1R = AIL2_OFFSET;
    OC1RS = AIL2_OFFSET + SERVO_CENTER;
    OC2CON = 0x000d;    //Off, duel compare mode, TMR3
    OC2R = ELEV_OFFSET;
    OC2RS = ELEV_OFFSET + SERVO_CENTER;
    OC3CON = 0x000d;    //Off, duel compare mode, TMR3
    OC3R = RUD_OFFSET;
    OC3RS = RUD_OFFSET + SERVO_CENTER;
    OC4CON = 0x000d;    //Off, duel compare mode, TMR3
    OC4R = AIL_OFFSET;
    OC4RS = AIL_OFFSET + SERVO_CENTER;
}

void EnableAilOutput(void) {
    OC4CONbits.ON = 1;
}

void EnableElevOutput(void) {
    OC2CONbits.ON = 1;
}

void EnableRudOutput(void) {
    OC3CONbits.ON = 1;
}

void EnableAil2Output(void) {
    OC1CONbits.ON = 1;
}

void DisableAilOutput(void) {
    IFS0bits.OC4IF = 0;
    while (IFS0bits.OC4IF == 0);
    OC4CONbits.ON = 0;
}

void DisableElevOutput(void) {
    IFS0bits.OC2IF = 0;
    while (IFS0bits.OC2IF == 0);
    OC2CONbits.ON = 0;
}

void DisableRudOutput(void) {
    IFS0bits.OC3IF = 0;
    while (IFS0bits.OC3IF == 0);
    OC3CONbits.ON = 0;
}

void DisableAil2Output(void) {
    IFS0bits.OC1IF = 0;
    while (IFS0bits.OC1IF == 0);
    OC1CONbits.ON = 0;
}
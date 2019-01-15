//*******************************************************************//
// File: Interrupts.c                                                //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Contains all the ISR's for the system                //
//                                                                   //
//*******************************************************************// 


#include <xc.h>
#include <sys/attribs.h>
#include "Timers.h"
#include "Data.h"
#include <stdint.h>
#include "IO.h"
#include "MPU6050.h"
#include "InputCapture.h"
#include "Attitude.h"

void __ISR(_TIMER_1_VECTOR, ipl7SOFT) Timer1ISR(void) {
    ++systemTickCount;
    IFS0bits.T1IF = 0;
}

void __ISR(_TIMER_3_VECTOR, ipl6SOFT) Timer3ISR(void) {
    AIL_OC = AIL_OFFSET + outputChannels.ailOut;
    ELEV_OC = ELEV_OFFSET + outputChannels.elevOut;
    RUD_OC = RUD_OFFSET + outputChannels.rudOut;
    AIL2_OC = AIL2_OFFSET + outputChannels.ail2Out;
    outputsUpdated = true;
    IFS0bits.T3IF = 0;
}

void __ISR(_INPUT_CAPTURE_1_VECTOR, ipl4SOFT) AilCaptureISR(void) {
    uint16_t value1, value2;
    value1 = IC1BUF;
    value2 = IC1BUF;
    if (value1 < value2) {
        inputChannels.ailIn = value2 - value1;
    } else {
        inputChannels.ailIn = ((60000U - value1) + value2);
    }
    ailUpdate = true;
    IFS0bits.IC1IF = 0;
}

void __ISR(_INPUT_CAPTURE_4_VECTOR, ipl4SOFT) ElevCaptureISR(void) {
    uint16_t value1, value2;
    value1 = IC4BUF;
    value2 = IC4BUF;
    if (value1 < value2) {
        inputChannels.elevIn = value2 - value1;
    } else {
        inputChannels.elevIn = ((60000U - value1) + value2);
    }
    elevUpdate = true;
    IFS0bits.IC4IF = 0;
}

void __ISR(_INPUT_CAPTURE_5_VECTOR, ipl4SOFT) RudCaptureISR(void) {
    uint16_t value1, value2;
    value1 = IC5BUF;
    value2 = IC5BUF;
    if (value1 < value2) {
        inputChannels.rudIn = value2 - value1;
    } else {
        inputChannels.rudIn = ((60000U - value1) + value2);
    }
    rudUpdate = true;
    IFS0bits.IC5IF = 0;
}

void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl4SOFT) ModeCaptureISR(void) {

    uint16_t value1, value2;
    value1 = IC2BUF;
    value2 = IC2BUF;
    if (value1 < value2) {
        inputChannels.modeIn = value2 - value1;
    } else {
        inputChannels.modeIn = ((60000U - value1) + value2);
    }
    modeUpdate = true;
    IFS0bits.IC2IF = 0;
}

void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl4SOFT) GainCaptureISR(void) {
    uint16_t value1, value2;
    value1 = IC3BUF;
    value2 = IC3BUF;
    if (value1 < value2) {
        inputChannels.gainIn = value2 - value1;
    } else {
        inputChannels.gainIn = ((60000U - value1) + value2);
    }
    gainUpdate = true;
    IFS0bits.IC3IF = 0;
}

void __ISR(_EXTERNAL_4_VECTOR, ipl5SOFT) ExternalInterrupt4ISR(void) {
    if (startTesting == 1) {
        ++startTesting;
    } else if (startTesting == 2) {
        InitAttitude();
        testing = true;
        startTesting = 0;
    } else if (testing) {
        UpdateAttitude();
    }
    ReadMPU6050Data();
    IFS0bits.INT4IF = 0;
}

void __ISR(_DMA_0_VECTOR, ipl3SOFT) DMA0Interrupt(void) {
    UpdateAttitude();
    attitudeUpdated = true;
    DCH0INTbits.CHBCIF = 0;
    IFS1bits.DMA0IF = 0;
}
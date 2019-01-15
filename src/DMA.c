//*******************************************************************//
// File: DMA.c                                                       //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: DMA functions                                        //
//                                                                   //
//*******************************************************************// 

#include <xc.h>
#include <sys/kmem.h>
#include "DMA.h"
#include "Data.h"
#include "MPU6050.h"

void InitDMA(void) {
    DMACONbits.ON = 1;
    DCH0CONbits.CHAEN = 1;
    DCH0CONbits.CHPRI = 2;
    DCH0ECON = 0;
    IPC10bits.DMA0IP = 3;
    IPC10bits.DMA0IS = 0;
    DCH0INTbits.CHBCIF = 0;
    DCH0INTbits.CHBCIE = 1;
    DCH0SSA = KVA_TO_PA((void *)&gyroData);
    DCH0DSA = KVA_TO_PA((void *)&rawData);
    DCH0SSIZ = sizeof(RawData);
    DCH0DSIZ = sizeof(RawData);
    DCH0CSIZ = sizeof(RawData);
    DCH0CONbits.CHEN = 1;
}

inline void StartDMATransfer(void) {
    DCH0ECONbits.CFORCE = 1;
}

void EnableDMAInterrupt(void) {
    DCH0INTbits.CHBCIF = 0;
    IFS1bits.DMA0IF = 0;
    IEC1bits.DMA0IE = 1;
}

void DisableDMAInterrupt(void) {
    IEC1bits.DMA0IE = 0;
}

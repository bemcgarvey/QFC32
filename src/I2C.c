//*******************************************************************//
// File: I2C.h                                                       //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: I2C support functions for EEPROM and gyro            //
//                                                                   //
//*******************************************************************// 

#include <xc.h>
#include "I2C.h"

#define EE_ADDRESS      0b10100000

void InitI2C(void) {
    I2C1CONbits.ON = 1;
    I2C1CONbits.DISSLW = 0;
    I2C1BRG = 25;
    IPC8bits.I2C1IP = 2;
    IPC8bits.I2C1IS = 0;
    IFS1bits.I2C1MIF = 0;
    IEC1bits.I2C1MIE = 0; //no interrupts for now
}

void EEPROMWrite(uint32_t address, int len, uint8_t *buff) {
    uint8_t controlByte = EE_ADDRESS;

    if (address & 0x10000) {
        controlByte |= 0b00001000;
    }
    I2CStart();
    I2CWriteByte(controlByte);
    I2CWriteByte(address >> 8);
    I2CWriteByte(address);
    while (len > 0) {
        I2CWriteByte(*buff);
        ++buff;
        --len;
    }
    I2CStop();
}

void EEPROMRead(uint32_t address, int len, uint8_t *buff) {
    uint8_t controlByte = EE_ADDRESS;

    if (address & 0x10000) {
        controlByte |= 0b00001000;
    }
    I2CStart();
    I2CWriteByte(controlByte);
    I2CWriteByte(address >> 8);
    I2CWriteByte(address);
    I2CRestart();
    I2CWriteByte(controlByte | 1);
    while (len > 1) {
        *buff = I2CReadByte(true);
        ++buff;
        --len;
    }
    *buff = I2CReadByte(false);
    I2CStop();
}

void EEAckPoll(uint32_t address) {
    uint8_t controlByte = EE_ADDRESS;

    if (address & 0x10000) {
        controlByte |= 0b00001000;
    }

    do {
        I2CStart();
        I2CWriteByte(controlByte);
    } while (I2C1STATbits.ACKSTAT == 1);
    I2CStop();
}

void I2CWriteByte(uint8_t byte) {
    IFS1bits.I2C1MIF = 0;
    I2C1TRN = byte;
    while (IFS1bits.I2C1MIF == 0);
}

uint8_t I2CReadByte(bool ack) {
    uint8_t byte;
    IFS1bits.I2C1MIF = 0;
    I2C1CONbits.RCEN = 1;
    while (IFS1bits.I2C1MIF == 0);
    byte = I2C1RCV;
    if (ack) {
        I2C1CONbits.ACKDT = 0;
    } else {
        I2C1CONbits.ACKDT = 1;
    }
    IFS1bits.I2C1MIF = 0;
    I2C1CONbits.ACKEN = 1;
    while (IFS1bits.I2C1MIF == 0);
    return byte;
}

void I2CStart(void) {
    IFS1bits.I2C1MIF = 0;
    I2C1CONbits.SEN = 1;
    while (IFS1bits.I2C1MIF == 0);
}

void I2CStop(void) {
    IFS1bits.I2C1MIF = 0;
    I2C1CONbits.PEN = 1;
    while (IFS1bits.I2C1MIF == 0);
}

void I2CRestart(void) {
    IFS1bits.I2C1MIF = 0;
    I2C1CONbits.RSEN = 1;
    while (IFS1bits.I2C1MIF == 0);
}

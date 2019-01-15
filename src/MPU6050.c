//*******************************************************************//
// File: MPU6050.h                                                   //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: MPU6050 defines and functions                        //
//                                                                   //
//*******************************************************************// 

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include <sys/attribs.h>
#include "MPU6050.h"
#include "I2C.h"
#include "IO.h"
#include "DMA.h"

typedef enum {
    ISR_STATE_READY = 0, ISR_STATE_START = 1, ISR_STATE_ADDRESSW, ISR_STATE_REG,
    ISR_STATE_RESTART, ISR_STATE_ADDRESSR, ISR_STATE_RCV, ISR_STATE_ACK,
    ISR_STATE_STOP
} I2CStates;

volatile I2CStates currentIsrState = ISR_STATE_READY;
volatile uint8_t rcvCount;
volatile uint8_t *rcvBuff;
volatile RawData gyroData;

void InitMPU6050(void) {
    WriteMPU6050Register(107, 0b10000000); //Device reset
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < 2400000); //delay 100ms
    WriteMPU6050Register(107, 0b00001001); //Clock from X gyro, disable temp sensor
    WriteMPU6050Register(25, 1); //Set sample rate divisor  8000/(n+1) or 1000/n depending on DPLF
    WriteMPU6050Register(26, 1); //Set DLPF: 0 = 8000 kHz sample, >0 = 1000 kHz
    //If scales are changed be sure to change macros in .h file
    WriteMPU6050Register(27, 0b00011000); //2000 dps gyro scale
    WriteMPU6050Register(28, 0b00001000); //4g accelerometer scale
    WriteMPU6050Register(106, 0b00000001); //Signal condition reset
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < 2400000); //delay 100ms
    WriteMPU6050Register(55, 0b10010000); //Int pin active low, push-pull, pulse, clear on any read
    WriteMPU6050Register(56, 0b00000001); //Interrupt on data ready
}

void ReadMPU6050Data(void) {
    rcvBuff = (uint8_t *) & gyroData;
    rcvCount = 0;
    currentIsrState = ISR_STATE_START;
    IFS1bits.I2C1MIF = 0;
    IEC1bits.I2C1MIE = 1; //enable interrupt
    I2C1CONbits.SEN = 1; //start
}

void WriteMPU6050Register(uint8_t regNum, uint8_t value) {
    I2CStart();
    I2CWriteByte(MPU6050_W_ADDRESS);
    I2CWriteByte(regNum);
    I2CWriteByte(value);
    I2CStop();
}

uint8_t ReadMPU6050Register(uint8_t regNum) {
    uint8_t value;
    I2CStart();
    I2CWriteByte(MPU6050_W_ADDRESS);
    I2CWriteByte(regNum);
    I2CRestart();
    I2CWriteByte(MPU6050_R_ADDRESS);
    value = I2CReadByte(false);
    I2CStop();
    return value;
}

void __ISR(_I2C_1_VECTOR, ipl2SOFT) I2C1MasterISR(void) {
    switch (currentIsrState) {
        case ISR_STATE_START:
            I2C1TRN = MPU6050_W_ADDRESS;
            currentIsrState = ISR_STATE_ADDRESSW;
            break;
        case ISR_STATE_ADDRESSW:
            if (I2C1STATbits.ACKSTAT == 0) {
                I2C1TRN = MPU6050_DATA_ACCEL;
                currentIsrState = ISR_STATE_REG;
            } else {
                I2C1CONbits.PEN = 1;
                currentIsrState = ISR_STATE_STOP;
            }
            break;
        case ISR_STATE_REG:
            if (I2C1STATbits.ACKSTAT == 0) {
                I2C1CONbits.RSEN = 1;
                currentIsrState = ISR_STATE_RESTART;
            } else {
                I2C1CONbits.PEN = 1;
                currentIsrState = ISR_STATE_STOP;
            }
            break;
        case ISR_STATE_RESTART:
            I2C1TRN = MPU6050_R_ADDRESS;
            currentIsrState = ISR_STATE_ADDRESSR;
            break;
        case ISR_STATE_ADDRESSR:
            if (I2C1STATbits.ACKSTAT == 0) {
                I2C1CONbits.RCEN = 1;
                currentIsrState = ISR_STATE_RCV;
            } else {
                I2C1CONbits.PEN = 1;
                currentIsrState = ISR_STATE_STOP;
            }
            break;
        case ISR_STATE_RCV:
            if (rcvCount % 2 == 0) {
                *(rcvBuff + 1) = I2C1RCV;
            } else {
                *rcvBuff = I2C1RCV;
                rcvBuff += 2;
            }
            ++rcvCount;
            if (rcvCount < 14) {
                I2C1CONbits.ACKDT = 0;
            } else {
                StartDMATransfer();
                I2C1CONbits.ACKDT = 1;
            }
            I2C1CONbits.ACKEN = 1;
            currentIsrState = ISR_STATE_ACK;
            break;
        case ISR_STATE_ACK:
            if (rcvCount < 14) {
                I2C1CONbits.RCEN = 1;
                currentIsrState = ISR_STATE_RCV;
            } else {
                I2C1CONbits.PEN = 1;
                currentIsrState = ISR_STATE_STOP;
            }
            break;
        case ISR_STATE_STOP:
            currentIsrState = ISR_STATE_READY;
            IEC1bits.I2C1MIE = 0; //disable I2C interrupts for now
            break;
    }
    IFS1bits.I2C1MIF = 0;
}

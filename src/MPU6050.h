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

#ifndef MPU6050_H
#define	MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "Data.h"

#define MPU6050_W_ADDRESS       0xd0
#define MPU6050_R_ADDRESS       0xd1
#define MPU6050_DATA_ACCEL      0x3b
#define MPU6050_DATA_GYRO       0x43

#define GYRO_SCALE              2000.0
#define GYRO_SCALE_DEG          (GYRO_SCALE / 32768.0)
#define GYRO_SCALE_RAD          ((GYRO_SCALE * M_PI / 180.0) / 32768.0)
#define ACCEL_SCALE             (4.0 / 32768.0)

#define SAMPLE_RATE             500
#define dt                      (1.0 / SAMPLE_RATE)  //sample time in sec
#define DT                      (1000000 / SAMPLE_RATE) //sample time in us


#ifdef	__cplusplus
extern "C" {
#endif

    extern volatile RawData gyroData;
    
    void InitMPU6050(void);
    void WriteMPU6050Register(uint8_t regNum, uint8_t value);
    uint8_t ReadMPU6050Register(uint8_t regNum);
    void ReadMPU6050Data(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MPU6050_H */


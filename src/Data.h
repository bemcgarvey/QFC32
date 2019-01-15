//*******************************************************************//
// File: Data.h                                                      //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Global variables for channel inputs and outputs      //
//                                                                   //
//*******************************************************************// 

#ifndef DATA_H
#define	DATA_H

#include <stdint.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define AIL_OFFSET      100
#define ELEV_OFFSET     5700
#define RUD_OFFSET      11300
#define AIL2_OFFSET     16900
#define SERVO_CENTER    4500
#define MILLISECOND_SCALE   3

    typedef struct {
        uint16_t ailIn;
        uint16_t elevIn;
        uint16_t rudIn;
        uint16_t modeIn;
        uint16_t gainIn;
    } InputChannels;

    typedef struct {
        uint16_t ailOut;
        uint16_t elevOut;
        uint16_t rudOut;
        uint16_t ail2Out;
    } OutputChannels;

    typedef struct {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t temp;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    } RawData;
    
    typedef struct {
        int16_t xOffset;
        int16_t yOffset;
        int16_t zOffset;
    } GyroOffsets;
    
    extern int testing;
    extern int startTesting;
    
    extern bool attitudeUpdated;
    extern bool outputsUpdated;
    extern uint32_t lastInputUpdate;
    
    extern volatile InputChannels inputChannels;
    extern OutputChannels outputChannels;
    extern volatile RawData rawData;
    extern GyroOffsets gyroOffsets;
    
    void ConvertRawData(void);
    
#ifdef	__cplusplus
}
#endif

#endif	/* DATA_H */


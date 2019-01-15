//*******************************************************************//
// File: Data.c                                                      //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Global variables for channel inputs and outputs      //
//                                                                   //
//*******************************************************************// 


#include <xc.h>
#include "Data.h"
#include "Config.h"

int testing= 0;
int startTesting = 0;

bool attitudeUpdated;
bool outputsUpdated;
uint32_t lastInputUpdate;

InputChannels volatile inputChannels __attribute__((persistent));
OutputChannels outputChannels = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
RawData volatile rawData = {0, 0, 0, 0, 0, 0};
GyroOffsets gyroOffsets = {0, 0, 0};

void ConvertRawData(void) {
    int16_t temp;
    switch (config.orientation) {
        case ORIENT_NORMAL:
            temp = rawData.accel_x;
            rawData.accel_x = -rawData.accel_y;
            rawData.accel_y = -temp;
            
            temp = rawData.gyro_x;
            rawData.gyro_x = rawData.gyro_y;
            rawData.gyro_y = temp;
            rawData.gyro_z = -rawData.gyro_z;
            break;
        case ORIENT_INVERT:
            temp = rawData.accel_x;
            rawData.accel_x = -rawData.accel_y;
            rawData.accel_y = temp;
            rawData.accel_z = -rawData.accel_z;
            
            temp = rawData.gyro_x;
            rawData.gyro_x = rawData.gyro_y;
            rawData.gyro_y = -temp;
            break;
        case ORIENT_SIDE:
            temp = rawData.accel_y;
            rawData.accel_y = rawData.accel_z;
            rawData.accel_z = rawData.accel_x;
            rawData.accel_x = -temp;
            
            temp = rawData.gyro_y;
            rawData.gyro_y = -rawData.gyro_z;
            rawData.gyro_z = -rawData.gyro_x;
            rawData.gyro_x = temp;
            break;
        case ORIENT_INVERT_SIDE:
            temp = rawData.accel_y;
            rawData.accel_y = -rawData.accel_z;
            rawData.accel_z = -rawData.accel_x;
            rawData.accel_x = -temp;
            
            temp = rawData.gyro_y;
            rawData.gyro_y = rawData.gyro_z;
            rawData.gyro_z = rawData.gyro_x;
            rawData.gyro_x = temp;
            break;
    }
    rawData.gyro_x -= gyroOffsets.xOffset;
    rawData.gyro_y -= gyroOffsets.yOffset;
    rawData.gyro_z -= gyroOffsets.zOffset;
}
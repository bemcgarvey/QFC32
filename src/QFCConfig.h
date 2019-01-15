//*******************************************************************//
// File: QFCConfig.h                                                 //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Definitions for QFC configuration                    //
//                                                                   //
//*******************************************************************// 

#ifndef QFCCONFIG_H
#define	QFCCONFIG_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

//QFC modes
#define MODE_OFF        0
#define MODE_NORMAL     1
#define MODE_AVCS       2
#define MODE_AUTO_LEVEL 3
#define MODE_AUTO_HOVER 4
#define MODE_FAILSAFE   -1
    
//Gyro and servo directions
#define DIR_NORM        1 
#define DIR_REV         -1
    
//Stick modes
#define STICK_MODE_MANUAL   0 
#define STICK_MODE_AUTO     1
    
//Enable/Disable
#define ENABLED         1
#define DISABLED        0
    
//Wing types
#define WING_NORMAL     0
#define WING_DUAL_AIL   1 
#define WING_DELTA      2

//Tail types
#define TAIL_NORMAL     0
#define TAIL_V          1
    
//Response speeds
#define SPEED_VSLOW     -2
#define SPEED_SLOW      -1
#define SPEED_NORMAL    0
#define SPEED_FAST      1
#define SPEED_VFAST     2
    
//Servo frame rates
#define HZ50            0
#define HZ65            1
#define HZ165           2
#define HZ200           3
#define HZ270           4
#define HZ333           5
    
//Orientations
#define ORIENT_NORMAL       0
#define ORIENT_INVERT       1
#define ORIENT_SIDE         2
#define ORIENT_INVERT_SIDE  3

#define NUM_MODES       3
#define PID_LIST_LEN    10
    
#define PID_TYPE_ANGLE      0
#define PID_TYPE_RATE       1

#define PID_DEFAULT    -1
    
typedef struct {
    uint8_t enabled;
    uint8_t gain;
    uint8_t deadband;
    int8_t pidNumber;
} Channel;

typedef struct {
    uint16_t min;
    uint16_t max;
    uint16_t center;
} Servo;

typedef struct {
    int8_t mode;
    int8_t stickMode;
    float maxRoll;
    float maxPitch;
    float maxYaw;
    Channel aileron;
    Channel elevator;
    Channel rudder;
} Mode;

typedef struct {
    uint8_t wingType;
    uint8_t tailType;
    int8_t differential;
    int8_t ail2Direction;
    Mode modes[NUM_MODES];
    uint8_t frameRate;
    uint8_t orientation;
    float rollOffset;
    float pitchOffset;
    float hoverPitchOffset;
    float hoverYawOffset;
    int8_t responseRate;
    int8_t ailGyroDirection;
    int8_t elevGyroDirection;
    int8_t rudGyroDirection;
    Servo ailServo;
    Servo elevServo;
    Servo rudServo;
    Servo ail2Servo;
    uint16_t msConversion;
    uint16_t crc;
} QFCConfig;

typedef struct {
    float baseKp;
    float baseKi;
    float baseKd;
    int type;
} PID;

#ifdef	__cplusplus
}
#endif

#endif	/* QFCCONFIG_H */


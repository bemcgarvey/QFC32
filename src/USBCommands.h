//*******************************************************************//
// File: USBCommands.h                                               //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Defines for USB communication                        //
//                                                                   //
//*******************************************************************// 

#ifndef USBCOMMANDS_H
#define	USBCOMMANDS_H

#define CMD_YELLOW_TOGGLE       0x80
#define CMD_BLUE_TOGGLE         0x81
#define CMD_RAW_DATA            0x82
#define CMD_ATTITUDE            0x83
#define CMD_GET_CONFIG          0x84
#define CMD_SAVE_CONFIG         0x85
#define CMD_BOOTLOAD            0x86
#define CMD_RESET               0x87
#define CMD_SET_SERVO           0x88
#define CMD_SERVO_CENTER        0x89
#define CMD_H_CALIBRATE         0x8a
#define CMD_V_CALIBRATE         0x8b
#define CMD_VERSION             0x8c
#define CMD_START_DATA          0x8d
#define CMD_STOP_DATA           0x8e
#define CMD_SERVO_ENABLE        0x8f
#define CMD_SERVO_DISABLE       0x90
#define CMD_CALIBRATION_DATA    0x91
#define CMD_GET_SERVO_CENTERS   0x92
#define CMD_GREEN_TOGGLE        0x93
#define CMD_DEFAULT_CONFIG      0x94
#define CMD_START_ATTITUDE      0x95
#define CMD_STOP_ATTITUDE       0x96
#define CMD_SAVE_PID            0x97
#define CMD_LOAD_PID            0x98
#define CMD_DEFAULT_PID         0x99
    
#define AIL_SERVO               0
#define ELEV_SERVO              1
#define RUD_SERVO               2
#define AIL2_SERVO              3

#endif	/* USBCOMMANDS_H */


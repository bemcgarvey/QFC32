//*******************************************************************//
// File: Controller.h                                                //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: PID Controller functions                             //
//                                                                   //
//*******************************************************************// 

#ifndef CONTROLLER_H
#define	CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "QFCConfig.h"

#define CONTROL_LOOP_TIME   20
#define AIL_CONTROLLER      0
#define ELEV_CONTROLLER     1
#define RUD_CONTROLLER      2

typedef struct {
    float baseKp;
    float baseKi;
    float baseKd;
    float kp;
    float ki;
    float kd;
    int center;
    int max;
    int min;
    int range;
    int direction;
    int lastOutput;
    float tolerance;
    float errorSum;
    float lastError;
    float target;
    bool enabled;
    int type;
    int deadband;
} ControllerData;

extern ControllerData controllers[3];
extern PID PIDList[PID_LIST_LEN];

#define PID_EEPROM_ADDRESS       (((CONFIG_EEPROM_ADDRESS + sizeof(QFCConfig) + EE_PAGE_SIZE) / EE_PAGE_SIZE) * EE_PAGE_SIZE)

#ifdef	__cplusplus
extern "C" {
#endif

    void InitControllers(void);
    void SetTarget(int controller, float target);
    float GetTarget(int controller);
    bool SavePIDs(void);
    bool LoadPIDs(void);
    void DefaultPIDs(void);
    void SetMasterGain(float gain);
#ifdef	__cplusplus
}
#endif

#endif	/* CONTROLLER_H */


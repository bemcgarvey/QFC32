//*******************************************************************//
// File: Controller.c                                                //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: PID Controller functions                             //
//                                                                   //
//*******************************************************************// 

#include <xc.h>

#include "Controller.h"
#include "Attitude.h"
#include "Config.h"
#include "I2C.h"
#include "Data.h"
#include <math.h>
#include <stdbool.h>
#include "AutoHoverMode.h"
#include "AutoLevelMode.h"
#include "NormalMode.h"

PID PIDList[PID_LIST_LEN];
ControllerData controllers[3];

void InitControllers() {
    int pidNum;
    Channel *pChannel;
    float gain;
    Servo *servo;
    int i;

    for (i = 0; i < 3; ++i) {
        switch (i) {
            case AIL_CONTROLLER:
                pidNum = pCurrentMode->aileron.pidNumber;
                pChannel = &(pCurrentMode->aileron);
                servo = &(config.ailServo);
                controllers[i].direction = config.ailGyroDirection;
                controllers[i].lastOutput = outputChannels.ailOut;
                break;
            case ELEV_CONTROLLER:
                pidNum = pCurrentMode->elevator.pidNumber;
                pChannel = &(pCurrentMode->elevator);
                servo = &(config.elevServo);
                controllers[i].direction = config.elevGyroDirection;
                controllers[i].lastOutput = outputChannels.elevOut;
                break;
            case RUD_CONTROLLER:
                pidNum = pCurrentMode->rudder.pidNumber;
                pChannel = &(pCurrentMode->rudder);
                servo = &(config.rudServo);
                controllers[i].direction = config.rudGyroDirection;
                controllers[i].lastOutput = outputChannels.rudOut;
                break;
        }
        if (pidNum == -1) {
            switch (pCurrentMode->mode) {
                case MODE_AUTO_LEVEL:
                    pidNum = 0; //Default angle PID
                    break;
                case MODE_AUTO_HOVER:
                    pidNum = 0; //Default angle PID
                    break;
                case MODE_NORMAL:
                case MODE_AVCS:
                    pidNum = 1; //Default rate PID
                    break;
                default:
                    pidNum = 0;
                    break;
            }
        }
        switch (pCurrentMode->mode) {
            case MODE_AUTO_LEVEL:
                AutoLevelInit();
                break;
            case MODE_AUTO_HOVER:
                AutoHoverInit();
                break;
            case MODE_NORMAL:
                NormalInit();
                break;
            case MODE_AVCS:
                break;
            default:
                break;
        }
        controllers[i].enabled = pChannel->enabled;
        if (pChannel->enabled) {
            gain = pChannel->gain / 100.0;
            controllers[i].baseKp = PIDList[pidNum].baseKp * gain;
            controllers[i].kp = controllers[i].baseKp;
            controllers[i].baseKi = PIDList[pidNum].baseKi * gain;
            controllers[i].ki = controllers[i].baseKi;
            controllers[i].baseKd = PIDList[pidNum].baseKd * gain;
            controllers[i].kd = controllers[i].baseKd;
            controllers[i].type = PIDList[pidNum].type;
            controllers[i].center = servo->center;
            controllers[i].max = servo->max;
            controllers[i].min = servo->min;
            controllers[i].range = controllers[i].max - controllers[i].center;
            if (controllers[i].center - controllers[i].min > controllers[i].range) {
                controllers[i].range = controllers[i].center - controllers[i].min;
            }
            controllers[i].target = 0.0;
            controllers[i].errorSum = controllers[i].lastError = 0;
            //TODO Set different tolerance for rate controllers than for angle?
            //FIXME Is tolerance even needed?
            controllers[i].tolerance = 0.5 * M_PI / 180.0; //0.5 degrees???
            controllers[i].deadband = (pChannel->deadband * controllers[i].range) / 100;
        }
    }
}

inline void SetTarget(int controller, float target) {
    controllers[controller].target = target;
}

inline float GetTarget(int controller) {
    return controllers[controller].target;
}

bool SavePIDs(void) {
    PID readPIDList[PID_LIST_LEN];
    uint8_t *buff = (uint8_t *) PIDList;
    int size = sizeof (PID) * PID_LIST_LEN;
    int saved = 0;
    int i;
    while (saved < size) {
        if (size - saved > EE_PAGE_SIZE) {
            i = EE_PAGE_SIZE;
        } else {
            i = size - saved;
        }
        EEPROMWrite(PID_EEPROM_ADDRESS + saved, i, buff);
        EEAckPoll(PID_EEPROM_ADDRESS + saved);
        buff += i;
        saved += i;
    }
    uint16_t crc = calcCRC((uint8_t *) PIDList, size);
    EEPROMWrite(PID_EEPROM_ADDRESS + size, 2, (uint8_t *) & crc);
    EEAckPoll(PID_EEPROM_ADDRESS + size);
    EEPROMRead(PID_EEPROM_ADDRESS, size, (uint8_t *) readPIDList);
    uint16_t readCrc;
    EEPROMRead(PID_EEPROM_ADDRESS + size, 2, (uint8_t *) & readCrc);
    return (memcmp(PIDList, readPIDList, size) == 0) && (crc == readCrc);
}

bool LoadPIDs(void) {
    int size = sizeof (PID) * PID_LIST_LEN;
    EEPROMRead(PID_EEPROM_ADDRESS, size, (uint8_t *) PIDList);
    uint16_t readCrc;
    EEPROMRead(PID_EEPROM_ADDRESS + size, 2, (uint8_t *) & readCrc);
    return readCrc == calcCRC((uint8_t *) PIDList, size);
}

void DefaultPIDs(void) {
    //TODO Fill with appropriate values
    int i;
    for (i = 0; i < PID_LIST_LEN; ++i) {
        if (i == 0) {
            PIDList[i].type = PID_TYPE_ANGLE;
            PIDList[i].baseKp = 0.8;
            PIDList[i].baseKi = 0.0;
            PIDList[i].baseKd = 0.0;
        } else {
            PIDList[i].type = PID_TYPE_RATE;
            PIDList[i].baseKp = 0.6;
            PIDList[i].baseKi = 0.03;
            PIDList[i].baseKd = 0.0;
        }
    }
}

void SetMasterGain(float gain) {
    int i;
    for (i = 0; i < 3; ++i) {
        controllers[i].kp = controllers[i].baseKp * gain;
        controllers[i].ki = controllers[i].baseKi * gain;
        controllers[i].kd = controllers[i].baseKd * gain;
    }
}
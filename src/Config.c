//*******************************************************************//
// File: Config.c                                                    //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Configuration management functions                   //
//                                                                   //
//*******************************************************************// 


#include <xc.h>
#include "Config.h"
#include "QFCConfig.h"
#include "I2C.h"
#include "IO.h"
#include "Data.h"
#include <string.h>

QFCConfig config __attribute__((persistent));
Mode *pCurrentMode;

bool LoadConfig(QFCConfig *pconfig) {
    EEPROMRead(CONFIG_EEPROM_ADDRESS, sizeof (QFCConfig), (uint8_t *) pconfig);
    pCurrentMode = &(pconfig->modes[0]);
    return CheckConfigCRC(pconfig);
}

bool CheckConfigCRC(QFCConfig *pconfig) {
    uint16_t crc;

    crc = calcCRC((uint8_t *) pconfig, sizeof (QFCConfig) - 2);
    return crc == pconfig->crc;
}

void DefaultConfig(QFCConfig *pconfig) {
    memset(pconfig, 0, sizeof (QFCConfig));
    pconfig->msConversion = MILLISECOND_SCALE;
    pconfig->ailGyroDirection = DIR_NORM;
    pconfig->elevGyroDirection = DIR_NORM;
    pconfig->rudGyroDirection = DIR_NORM;
    pconfig->ail2Direction = DIR_NORM;
    pconfig->ailServo.min = pconfig->elevServo.min = pconfig->rudServo.min
            = pconfig->ail2Servo.min = 3000;
    pconfig->ailServo.center = pconfig->elevServo.center = pconfig->rudServo.center
            = pconfig->ail2Servo.center = 4500;
    pconfig->ailServo.max = pconfig->elevServo.max = pconfig->rudServo.max
            = pconfig->ail2Servo.max = 6000;
    int i;
    for (i = 0; i < NUM_MODES; ++i) {
        pconfig->modes[i].aileron.pidNumber = -1;
        pconfig->modes[i].elevator.pidNumber = -1;
        pconfig->modes[i].rudder.pidNumber = -1;        
    }
    pconfig->crc = calcCRC((uint8_t *) pconfig, sizeof (QFCConfig) - 2);
}

bool SaveConfig(QFCConfig *pconfig) {
    QFCConfig readConfig;
    uint8_t *buff = (uint8_t *) pconfig;
    int saved = 0;
    int i;
    while (saved < sizeof (QFCConfig)) {
        if (sizeof (QFCConfig) - saved > EE_PAGE_SIZE) {
            i = EE_PAGE_SIZE;
        } else {
            i = sizeof (QFCConfig) - saved;
        }
        EEPROMWrite(CONFIG_EEPROM_ADDRESS + saved, i, buff);
        EEAckPoll(CONFIG_EEPROM_ADDRESS + saved);
        buff += i;
        saved += i;
    }
    EEPROMRead(CONFIG_EEPROM_ADDRESS, sizeof (QFCConfig), (uint8_t *) &readConfig);
    return memcmp(&config, &readConfig, sizeof(QFCConfig)) == 0;
}


static const uint16_t crc_table[16] ={
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};

uint16_t calcCRC(uint8_t *data, int len) {
    int i;
    uint16_t crc = 0;

    while (len--) {
        i = (crc >> 12) ^ (*data >> 4);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        i = (crc >> 12) ^ (*data >> 0);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        data++;
    }

    return (crc & 0xFFFF);
}
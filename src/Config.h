//*******************************************************************//
// File: Config.h                                                    //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Configuration management functions                   //
//                                                                   //
//*******************************************************************// 

#ifndef CONFIG_H
#define	CONFIG_H

#include "QFCConfig.h"
#include <stdbool.h>
#include <stdint.h>

#define CONFIG_EEPROM_ADDRESS       0x00000000

#ifdef	__cplusplus
extern "C" {
#endif

    bool LoadConfig(QFCConfig *pconfig);
    bool CheckConfigCRC(QFCConfig *pconfig);
    void DefaultConfig(QFCConfig *pconfig);
    bool SaveConfig(QFCConfig *pconfig);
    uint16_t calcCRC(uint8_t *data, int len);
    
    extern QFCConfig config;
    extern Mode *pCurrentMode;

#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */


//*******************************************************************//
// File: I2C.h                                                       //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: I2C support functions for EEPROM and gyro            //
//                                                                   //
//*******************************************************************// 

#ifndef I2C_H
#define	I2C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif

    void InitI2C(void);
    void I2CWriteByte(uint8_t byte);
    uint8_t I2CReadByte(bool ack);
    void I2CStart(void);
    void I2CStop(void);
    void I2CRestart(void);
    void EEPROMWrite(uint32_t address, int len, uint8_t *buff);
    void EEPROMRead(uint32_t address, int len, uint8_t *buff);
    void EEAckPoll(uint32_t address);
    
#define EE_PAGE_SIZE        128  //128 for 24LC1025

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */


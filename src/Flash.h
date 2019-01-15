//*******************************************************************//
// File: Flash.h                                                     //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Flash memory functions                               //
//                                                                   //
//*******************************************************************// 

#ifndef FLASH_H
#define	FLASH_H


#define FLASH_SIGNATURE_ADDRESS 0x9d000c00
#define FLASH_SIGNATURE         0x12345678

#ifdef	__cplusplus
extern "C" {
#endif

    void EraseFlashSignature(void);
    void delay_us(unsigned int us);

#ifdef	__cplusplus
}
#endif

#endif	/* FLASH_H */


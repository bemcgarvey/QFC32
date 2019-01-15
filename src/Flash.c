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

#include <xc.h>
#include "Flash.h"
#include <sys/kmem.h>
#include <sys/attribs.h>

const unsigned int countPerMicroSec = ((48000000/1000000)/2);

void delay_us(unsigned int us)
{
    unsigned int targetCount;  
    unsigned int backupCount; 
    
    backupCount = _CP0_GET_COUNT();
    targetCount = countPerMicroSec * us;      
    _CP0_SET_COUNT(0);        
    while(_CP0_GET_COUNT() < targetCount);
    _CP0_SET_COUNT(backupCount + targetCount);       	
} 

void EraseFlashSignature(void) {
    int	susp;
    
    NVMADDR = KVA_TO_PA((unsigned int)FLASH_SIGNATURE_ADDRESS);
    // Disable DMA & Disable Interrupts
	__builtin_disable_interrupts();
	susp = DMACONbits.SUSPEND;
    DMACONbits.SUSPEND = 1;
    // Enable Flash Write/Erase Operations
    NVMCON = NVMCON_WREN | 0x4004;
    NVMKEY 		= 0xAA996655;
    NVMKEY 		= 0x556699AA;
    NVMCONSET 	= NVMCON_WR;
    // Wait for WR bit to clear
    while(NVMCON & NVMCON_WR);
    // Disable Flash Write/Erase operations
    NVMCONCLR = NVMCON_WREN;  
	// Enable DMA & Enable Interrupts
    DMACONbits.SUSPEND = susp;
    __builtin_enable_interrupts();
}

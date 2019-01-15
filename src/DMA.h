//*******************************************************************//
// File: DMA.h                                                       //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: DMA functions                                        //
//                                                                   //
//*******************************************************************// 

#ifndef DMA_H
#define	DMA_H

#ifdef	__cplusplus
extern "C" {
#endif

    void InitDMA(void);
    void StartDMATransfer(void);
    void EnableDMAInterrupt(void);
    void DisableDMAInterrupt(void);

#ifdef	__cplusplus
}
#endif

#endif	/* DMA_H */


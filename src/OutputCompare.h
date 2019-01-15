//*******************************************************************//
// File: OutputCompare.h                                             //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Initialize and control output compare modules for    //
//  aileron, elevator, rudder, and aileron2                          //
//                                                                   //
//*******************************************************************// 

#ifndef OUTPUTCOMPARE_H
#define	OUTPUTCOMPARE_H

#ifdef	__cplusplus
extern "C" {
#endif

    void InitOutputCompare(void);
    void EnableAilOutput(void);
    void EnableElevOutput(void);
    void EnableRudOutput(void);
    void EnableAil2Output(void);
    void DisableAilOutput(void);
    void DisableElevOutput(void);
    void DisableRudOutput(void);
    void DisableAil2Output(void);

#ifdef	__cplusplus
}
#endif

#endif	/* OUTPUTCOMPARE_H */


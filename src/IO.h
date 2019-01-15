//*******************************************************************//
// File: IO.h                                                        //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
//                                                                   //
// Description: Port IO macros                                       //
//                                                                   //
//*******************************************************************// 

//MCU: PIC32MX270F256B 
//Compiler: XC32 1.44
//Harmony v1_07_01

#ifndef IO_H
#define	IO_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#define BlueLEDOn()     LATBSET=0x0020
#define BlueLEDOff()    LATBCLR=0x0020
#define BlueLEDToggle() LATBINV=0x0020
    
#define YellowLEDOn()     LATBSET=0x8000
#define YellowLEDOff()    LATBCLR=0x8000
#define YellowLEDToggle() LATBINV=0x8000
    
#define GreenLEDOn()     LATBSET=0x0080
#define GreenLEDOff()    LATBCLR=0x0080
#define GreenLEDToggle() LATBINV=0x0080
    
#define AIL_OC     OC4RS
#define ELEV_OC    OC2RS
#define RUD_OC     OC3RS
#define AIL2_OC    OC1RS
    
/*
#define RxAIL_IC   IC1
#define RxELEV_IC  IC4
#define RxRUD_IC   IC5
#define RxMODE_IC  IC2
#define RxGAIN_IC  IC3
 */

#ifdef	__cplusplus
}
#endif

#endif	/* IO_H */


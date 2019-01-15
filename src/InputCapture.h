//*******************************************************************//
// File: InputCapture.h                                              //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Initialize and control input capture modules for     //
//  aileron, elevator, rudder, mode and gain                         //
//                                                                   //
//*******************************************************************// 

#ifndef INPUTCAPTURE_H
#define	INPUTCAPTURE_H

#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif

void InitInputCapture(void);
void EnableAilInput(void);
void DisableAilInput(void);
void EnableElevInput(void);
void DisableElevInput(void);
void EnableRudInput(void);
void DisableRudInput(void);
void EnableModeInput(void);
void DisableModeInput(void);
void EnableGainInput(void);
void DisableGainInput(void);

extern volatile bool ailUpdate;
extern volatile bool elevUpdate;
extern volatile bool rudUpdate;
extern volatile bool gainUpdate;
extern volatile bool modeUpdate;

#ifdef	__cplusplus
}
#endif

#endif	/* INPUTCAPTURE_H */


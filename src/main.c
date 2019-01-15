//*******************************************************************//
// File: main.c                                                    //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
//                                                                   //
// Description: Main program                                         //
//                                                                   //
//*******************************************************************// 

//MCU: PIC32MX270F256B 
//Compiler: XC32 1.44
//Harmony v1_07_01

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include <stddef.h>                     // Defines NULL
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes

#include "IO.h"
#include "SystemInit.h"
#include "Timers.h"
#include "InputCapture.h"
#include "I2C.h"
#include "Data.h"
#include "MPU6050.h"
#include "Config.h"
#include "Flash.h"
#include "Attitude.h"
#include <string.h>
#include "Controller.h"
#include "math.h"
#include "OutputCompare.h"
#include "DMA.h"
#include "AutoHoverMode.h"
#include "AutoLevelMode.h"
#include "OffMode.h"
#include "NormalMode.h"

//TODO add bootloader as loadable
//TODO add flash signature as loadable or variable?

static void USBMain(void);
static void setLEDs(int mode);
static void WDTOMain(void);

int i;
int x, y, z;
int ailCount, elevCount, rudCount;
int ailTotal, elevTotal, rudTotal;
int gainCount;
int lastMode;
bool gainActive;
int samples;

int main(void) {
    InitSystem();
    if (RCONbits.POR == 0) {
        if (RCONbits.WDTO == 1 || RCONbits.BOR == 1) {
            //Watchdog timeout or brownout  Go to emergency main loop
            WDTOMain();
        }
    }
    RCONCLR = 0b10011; //clear POR, BOR and WDTO flags
    //Initialize persistent data since it won't get done at startup
    //We want the values to hold through reset in case of a WDTO
    memset((void *) &inputChannels, 0, sizeof (inputChannels));
    InitTimers();
    InitInputCapture();
    InitOutputCompare();
    InitI2C();
    if (!LoadConfig(&config)) {
        DefaultConfig(&config);
        SaveConfig(&config);
        YellowLEDOn();
        BlueLEDOn();
        GreenLEDOn();
        USBMain();
        while (true);
    }
    if (!LoadPIDs()) {
        DefaultPIDs();
        SavePIDs();
        YellowLEDOn();
        GreenLEDOn();
        USBMain();
        while (true);
    }
    InitDMA();
    StartTimer1();
    StartTimer2();
    systemTickCount = 0;
    while (systemTickCount < 4000) {
        if ((systemTickCount / 300) % 2 == 0) {
            YellowLEDOn();
        } else {
            YellowLEDOff();
        }
    }
    YellowLEDOff();
    EnableAilInput();
    EnableElevInput();
    EnableRudInput();
    EnableModeInput();
    EnableGainInput();
    InitExternalInt4();
    InitMPU6050();
    samples = 0;
    ailCount = elevCount = rudCount = 0;
    gainCount = 0;
    gainActive = false;
    x = y = z = 0;
    ailTotal = elevTotal = rudTotal = 0;
    systemTickCount = 0;
    while (systemTickCount < 3000) {
        if (DCH0INTbits.CHBCIF == 1) {
            DCH0INTbits.CHBCIF = 0;
            ConvertRawData();
            x += rawData.gyro_x;
            y += rawData.gyro_y;
            z += rawData.gyro_z;
            ++samples;
        }
        if (ailUpdate) {
            ailTotal += inputChannels.ailIn;
            ++ailCount;
            ailUpdate = false;
        }
        if (elevUpdate) {
            elevTotal += inputChannels.elevIn;
            ++elevCount;
            elevUpdate = false;
        }
        if (rudUpdate) {
            rudTotal += inputChannels.rudIn;
            ++rudCount;
            rudUpdate = false;
        }
        if (gainUpdate) {
            ++gainCount;
            gainUpdate = false;
        }
        if ((systemTickCount / 100) % 2 == 0) {
            BlueLEDOn();
        } else {
            BlueLEDOff();
        }
    }
    BlueLEDOff();
    gyroOffsets.xOffset = x / samples;
    gyroOffsets.yOffset = y / samples;
    gyroOffsets.zOffset = z / samples;
    //These center values are in raw units - not adjusted for milliseconds
    config.ailServo.center = config.ail2Servo.center
            = config.elevServo.center = config.rudServo.center
            = 1500 * MILLISECOND_SCALE;
    if (ailCount > 10) {
        config.ailServo.center = ailTotal / ailCount;
        config.ail2Servo.center = config.ailServo.center;
    }
    if (elevCount > 10) {
        config.elevServo.center = elevTotal / elevCount;
    }
    if (rudCount > 10) {
        config.rudServo.center = rudTotal / rudCount;
    }
    if (gainCount > 10) {
        gainActive = true;
    }
    //update config crc to reflect new servo centers
    config.crc = calcCRC((uint8_t *) & config, sizeof (QFCConfig) - 2);
    //Get initial attitude
    while (DCH0INTbits.CHBCIF == 0);
    DCH0INTbits.CHBCIF = 0;
    InitAttitude();
    //Check for USB connection
    U1PWRCbits.USBPWR = 1;
    delay_us(30000);
    if (U1OTGSTATbits.VBUSVD == 1) {
        U1PWRCbits.USBPWR = 0;
        DisableAilInput();
        DisableElevInput();
        DisableRudInput();
        DisableModeInput();
        DisableGainInput();
        DisableExternalInt4();
        USBMain();
        while (true);
    }
    U1PWRCbits.USBPWR = 0;

    if (ailCount > 10) { //if (inputChannels.ailIn > 0) {
        outputChannels.ailOut = inputChannels.ailIn;
        EnableAilOutput();
    }
    if (elevCount > 10) { //if (inputChannels.elevIn > 0) {
        outputChannels.elevOut = inputChannels.elevIn;
        EnableElevOutput();
    }
    if (rudCount > 10) { //if (inputChannels.rudIn > 0) {
        outputChannels.rudOut = inputChannels.rudIn;
        EnableRudOutput();
    }
    //Get current mode
    if (inputChannels.modeIn > 5000) {
        lastMode = 2;
    } else if (inputChannels.modeIn > 4000) {
        lastMode = 1;
    } else {
        lastMode = 0;
    }
    pCurrentMode = &(config.modes[lastMode]);
    setLEDs(pCurrentMode->mode);
    InitControllers();
    StartTimer3();
    //Start attitude tracking
    EnableDMAInterrupt();
    //TODO Enable and test watchdog timer
    attitudeUpdated = false;
    outputsUpdated = false;
    lastInputUpdate = systemTickCount;
    WDTCONbits.WDTWINEN = 0;
    WDTCONbits.ON = 1;
    WDTCONbits.WDTCLR = 1;
    while (true) {
        if (modeUpdate) {
            modeUpdate = 0;
            int currentMode;
            if (inputChannels.modeIn > 5000) {
                currentMode = 2;
            } else if (inputChannels.modeIn > 4000) {
                currentMode = 1;
            } else {
                currentMode = 0;
            }
            if (currentMode != lastMode) {
                lastMode = currentMode;
                pCurrentMode = &(config.modes[currentMode]);
                setLEDs(pCurrentMode->mode);
                InitControllers();
            }
        }
        if (gainUpdate) {
            gainUpdate = 0;
            float gain;
            int offset;
            offset = inputChannels.gainIn - (1000 * MILLISECOND_SCALE);
            if (offset < 0)
                offset = 0;
            gain = offset / (500.0 * MILLISECOND_SCALE);
            SetMasterGain(gain);
        }
        if (ailUpdate || elevUpdate || rudUpdate) {
            lastInputUpdate = systemTickCount;
            switch (pCurrentMode->mode) {
                case MODE_OFF:
                    OffMode();
                    break;
                case MODE_AUTO_LEVEL:
                    if (pCurrentMode->stickMode == STICK_MODE_MANUAL) {
                        AutoLevelManual();
                    } else {
                        AutoLevelAuto();
                    }
                    break;
                case MODE_AUTO_HOVER:
                    if (pCurrentMode->stickMode == STICK_MODE_MANUAL) {
                        AutoHoverManual();
                    } else {
                        AutoHoverAuto();
                    }
                    break;
                case MODE_AVCS:
                    break;
                case MODE_NORMAL:
                    if (pCurrentMode->stickMode == STICK_MODE_MANUAL) {
                        NormalManual();
                    } else {
                        NormalAuto();
                    }
                    break;
            }
        }
        if (systemTickCount - lastInputUpdate > 200) {
            outputChannels.ailOut = config.ailServo.center;
            outputChannels.ail2Out = config.ail2Servo.center;
            outputChannels.elevOut = config.elevServo.center;
            outputChannels.rudOut = config.rudServo.center;
        }
        if (attitudeUpdated && outputsUpdated) {
            WDTCONbits.WDTCLR = 1;
            attitudeUpdated = false;
            outputsUpdated = false;
        }
    }
    return -1;
}

void setLEDs(int mode) {
    YellowLEDOff();
    BlueLEDOff();
    GreenLEDOff();
    switch (mode) {
        case MODE_NORMAL:
            BlueLEDOn();
            break;
        case MODE_AVCS:
            BlueLEDOn();
            GreenLEDOn();
            break;
        case MODE_AUTO_LEVEL:
            GreenLEDOn();
            break;
        case MODE_AUTO_HOVER:
            YellowLEDOn();
            GreenLEDOn();
            break;
        case MODE_FAILSAFE:
            YellowLEDOn();
            GreenLEDOn();
            BlueLEDOn();
            break;
    };
}

void USBMain(void) {
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize(NULL);
    while (true) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();
    }
}

void WDTOMain(void) {
    int count = 0;
    if (!CheckConfigCRC(&config)) {
        InitI2C();
        LoadConfig(&config);
    }
    InitTimers();
    InitInputCapture();
    InitOutputCompare();
    StartTimer2();
    EnableAilInput();
    EnableElevInput();
    EnableRudInput();
    EnableAilOutput();
    EnableElevOutput();
    EnableRudOutput();
    EnableAil2Output();
    StartTimer3();
    YellowLEDOn();
    GreenLEDOn();
    BlueLEDOn();
    while (true) {
        outputChannels.ailOut = inputChannels.ailIn;
        //TODO Adjust ail2 output based on direction
        outputChannels.ail2Out = inputChannels.ailIn;
        outputChannels.elevOut = inputChannels.elevIn;
        outputChannels.rudOut = inputChannels.rudIn;
        ++count;
        if (count >= 1000000) {
            count = 0;
            if (RCONbits.BOR == 1)
                GreenLEDToggle();
            if (RCONbits.WDTO == 1)
                BlueLEDToggle();
        }
    }
}
/*******************************************************************************
 End of File
 */


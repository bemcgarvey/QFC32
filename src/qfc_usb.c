//*******************************************************************//
// File: qfc_usb.c                                                    //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
//                                                                   //
// Description: USB event handler functions                          //
//                                                                   //
//*******************************************************************// 

//MCU: PIC32MX270F256B 
//Compiler: XC32 1.44
//Harmony v1_07_01

#include <stdint.h>
#include <xc.h>

#include "system_definitions.h"
#include "qfc_usb.h"
#include "IO.h"
#include "Data.h"
#include "Config.h"
#include "USBCommands.h"
#include "Version.h"
#include "SystemInit.h"
#include "MPU6050.h"
#include "QFCConfig.h"
#include "OutputCompare.h"
#include "Timers.h"
#include "InputCapture.h"
#include "Flash.h"
#include "Attitude.h"
#include "Controller.h"
#include <string.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

/* Receive data buffer */
uint8_t receiveDataBuffer[64];

/* Transmit data buffer */
uint8_t transmitDataBuffer[64];

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

QFC_USB_DATA appData;
int ailCount, elevCount, rudCount;
int ailTotal, elevTotal, rudTotal;
bool centering = false;
bool horizontalCalibrate = false;
bool verticalCalibrate = false;
uint32_t startTime;
Vector calibrateData;
int calibrateCount;
int verticalSign;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void ProcessUSBCommand(void) {
    int len;
    int page;
    int remain;
    int done;
    bool tx;
    uint8_t *p;
    bool valid;
    int16_t data;

    tx = false;
    switch (appData.receiveDataBuffer[0]) {
        case CMD_YELLOW_TOGGLE:
            YellowLEDToggle();
            break;
        case CMD_BLUE_TOGGLE:
            BlueLEDToggle();
            break;
        case CMD_GREEN_TOGGLE:
            GreenLEDToggle();
            break;
        case CMD_VERSION:
            if (appData.hidDataTransmitted) {
                appData.transmitDataBuffer[0] = CMD_VERSION;
                memcpy(&appData.transmitDataBuffer[1], &VersionNumber, sizeof (uint32_t));
                strcpy(&transmitDataBuffer[5], VersionString);
                tx = true;
            }
            break;
        case CMD_RAW_DATA:
            if (appData.hidDataTransmitted) {
                appData.transmitDataBuffer[0] = CMD_RAW_DATA;
                while (DCH0INTbits.CHBCIF == 0);
                DCH0INTbits.CHBCIF = 0;
                ConvertRawData();
                memcpy(&transmitDataBuffer[1], (void *) &rawData, sizeof (rawData));
                tx = true;
            }
            break;
        case CMD_START_ATTITUDE:
            startTesting = 1;
            InitExternalInt4();
            break;
        case CMD_STOP_ATTITUDE:
            DisableExternalInt4();
            testing = false;
            startTesting = 0;
            break;
        case CMD_ATTITUDE:
            if (appData.hidDataTransmitted) {
                appData.transmitDataBuffer[0] = CMD_ATTITUDE;
                memcpy(&transmitDataBuffer[1], (void *) &attitude, sizeof (attitude));
                tx = true;
            }
            break;
        case CMD_GET_CONFIG:
            if (appData.hidDataTransmitted) {
                appData.transmitDataBuffer[0] = CMD_GET_CONFIG;
                page = appData.receiveDataBuffer[1];
                appData.transmitDataBuffer[1] = page;
                remain = sizeof (QFCConfig) - (page * 62);
                p = (uint8_t *) & config;
                p += (page * 62);
                if (remain > 62) {
                    len = 62;
                } else {
                    len = remain;
                }
                memcpy(&appData.transmitDataBuffer[2], p, len);
                tx = true;
            }
            break;
        case CMD_SAVE_CONFIG:
            page = appData.receiveDataBuffer[1];
            done = page * 62;
            remain = sizeof (QFCConfig) - done;
            p = (uint8_t *) & config;
            p += done;
            if (remain > 62) {
                len = 62;
            } else {
                len = remain;
            }
            memcpy(p, &appData.receiveDataBuffer[2], len);
            if (done + len == sizeof (QFCConfig)) {
                __builtin_disable_interrupts();
                valid = SaveConfig(&config);
                __builtin_enable_interrupts();
                if (appData.hidDataTransmitted) {
                    appData.transmitDataBuffer[0] = CMD_SAVE_CONFIG;
                    appData.transmitDataBuffer[1] = valid;
                    tx = true;
                }
            }
            break;
        case CMD_DEFAULT_CONFIG:
            DefaultConfig(&config);
            SaveConfig(&config);
            break;
        case CMD_SAVE_PID:
            page = appData.receiveDataBuffer[1];
            done = page * 62;
            remain = sizeof(PID) * PID_LIST_LEN - done;
            p = (uint8_t *) PIDList;
            p += done;
            if (remain > 62) {
                len = 62;
            } else {
                len = remain;
            }
            memcpy(p, &appData.receiveDataBuffer[2], len);
            if (done + len == sizeof(PID) * PID_LIST_LEN) {
                __builtin_disable_interrupts();
                valid = SavePIDs();
                __builtin_enable_interrupts();
                if (appData.hidDataTransmitted) {
                    appData.transmitDataBuffer[0] = CMD_SAVE_CONFIG;
                    appData.transmitDataBuffer[1] = valid;
                    tx = true;
                }
            }
            break;
        case CMD_LOAD_PID:
            if (appData.hidDataTransmitted) {
                appData.transmitDataBuffer[0] = CMD_LOAD_PID;
                page = appData.receiveDataBuffer[1];
                appData.transmitDataBuffer[1] = page;
                remain = sizeof(PID) * PID_LIST_LEN - (page * 62);
                p = (uint8_t *) PIDList;
                p += (page * 62);
                if (remain > 62) {
                    len = 62;
                } else {
                    len = remain;
                }
                memcpy(&appData.transmitDataBuffer[2], p, len);
                tx = true;
            }
            break;
        case CMD_DEFAULT_PID:
            DefaultPIDs();
            SavePIDs();
            break;
        case CMD_START_DATA:
            InitExternalInt4();
            break;
        case CMD_STOP_DATA:
            startTesting = 0;
            testing = false;
            DisableExternalInt4();
            break;
        case CMD_BOOTLOAD:
            EraseFlashSignature();
        case CMD_RESET:
            SYSKEY = 0xAA996655;
            SYSKEY = 0x556699AA;
            RSWRSTSET = 1;
            unsigned int dummy;
            dummy = RSWRST;
            while(1);
            break;
        case CMD_SERVO_ENABLE:
            switch (appData.receiveDataBuffer[1]) {
                case AIL_SERVO:
                    outputChannels.ailOut = config.ailServo.center;
                    EnableAilOutput();
                    break;
                case ELEV_SERVO:
                    outputChannels.elevOut = config.elevServo.center;
                    EnableElevOutput();
                    break;
                case RUD_SERVO:
                    outputChannels.rudOut = config.rudServo.center;
                    EnableRudOutput();
                    break;
                case AIL2_SERVO:
                    outputChannels.ail2Out = config.ail2Servo.center;
                    EnableAil2Output();
                    break;
            }
            StartTimer3();
            break;
        case CMD_SERVO_DISABLE:
            switch (appData.receiveDataBuffer[1]) {
                case AIL_SERVO:
                    DisableAilOutput();
                    break;
                case ELEV_SERVO:
                    DisableElevOutput();
                    break;
                case RUD_SERVO:
                    DisableRudOutput();
                    break;
                case AIL2_SERVO:
                    DisableAil2Output();
                    break;
            }
            StopTimer3();
            break;
        case CMD_SET_SERVO:
            switch (appData.receiveDataBuffer[1]) {
                case AIL_SERVO:
                    outputChannels.ailOut =
                            (*((uint16_t *) & appData.receiveDataBuffer[2]));
                    break;
                case ELEV_SERVO:
                    outputChannels.elevOut =
                            (*((uint16_t *) & appData.receiveDataBuffer[2]));
                    break;
                case RUD_SERVO:
                    outputChannels.rudOut =
                            (*((uint16_t *) & appData.receiveDataBuffer[2]));
                    break;
                case AIL2_SERVO:
                    outputChannels.ail2Out =
                            (*((uint16_t *) & appData.receiveDataBuffer[2]));
                    break;
            }
            break;
        case CMD_SERVO_CENTER:
            EnableAilInput();
            EnableElevInput();
            EnableRudInput();
            ailTotal = elevTotal = rudTotal = 0;
            ailCount = elevCount = rudCount = 0;
            startTime = systemTickCount;
            centering = true;
            BlueLEDOn();
            break;
        case CMD_H_CALIBRATE:
            calibrateData.roll = 0;
            calibrateData.pitch = 0;
            calibrateData.yaw = 0;
            calibrateCount = 0;
            startTime = systemTickCount;
            horizontalCalibrate = true;
            BlueLEDOn();
            InitExternalInt4();
            break;
        case CMD_V_CALIBRATE:
            verticalSign = 0;
            calibrateData.roll = 0;
            calibrateData.pitch = 0;
            calibrateData.yaw = 0;
            calibrateCount = 0;
            startTime = systemTickCount;
            verticalCalibrate = true;
            BlueLEDOn();
            InitExternalInt4();
            break;
        case CMD_CALIBRATION_DATA:
            if (appData.hidDataTransmitted) {
                memcpy(&appData.transmitDataBuffer[1], &calibrateData, sizeof(calibrateData));
                tx = true;
            }
            break;
        case CMD_GET_SERVO_CENTERS:
            if (appData.hidDataTransmitted) {
                appData.transmitDataBuffer[0] = CMD_GET_SERVO_CENTERS;
                appData.transmitDataBuffer[1] = config.ailServo.center;
                appData.transmitDataBuffer[2] = config.ailServo.center >> 8;
                appData.transmitDataBuffer[3] = config.elevServo.center;
                appData.transmitDataBuffer[4] = config.elevServo.center >> 8;
                appData.transmitDataBuffer[5] = config.rudServo.center;
                appData.transmitDataBuffer[6] = config.rudServo.center >> 8;
                appData.transmitDataBuffer[7] = config.ail2Servo.center;
                appData.transmitDataBuffer[8] = config.ail2Servo.center >> 8;
                tx = true;
            }
            break;
        default:
            break;
    }
    if (tx) {
        appData.hidDataTransmitted = false;
        USB_DEVICE_HID_ReportSend(USB_DEVICE_HID_INDEX_0,
                &appData.txTransferHandle, appData.transmitDataBuffer, 64);
    }
}

USB_DEVICE_HID_EVENT_RESPONSE APP_USBDeviceHIDEventHandler
(
        USB_DEVICE_HID_INDEX iHID,
        USB_DEVICE_HID_EVENT event,
        void * eventData,
        uintptr_t userData
        ) {
    USB_DEVICE_HID_EVENT_DATA_REPORT_SENT * reportSent;
    USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED * reportReceived;

    /* Check type of event */
    switch (event) {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_SENT
             * pointer type containing details about the report that was
             * sent. */
            reportSent = (USB_DEVICE_HID_EVENT_DATA_REPORT_SENT *) eventData;
            if (reportSent->handle == appData.txTransferHandle) {
                // Transfer progressed.
                appData.hidDataTransmitted = true;
            }

            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_RECEIVED
             * pointer type containing details about the report that was
             * received. */

            reportReceived = (USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED *) eventData;
            if (reportReceived->handle == appData.rxTransferHandle) {
                // Transfer progressed.
                appData.hidDataReceived = true;
            }

            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* For now we just accept this request as is. We acknowledge
             * this request using the USB_DEVICE_HID_ControlStatus()
             * function with a USB_DEVICE_CONTROL_STATUS_OK flag */

            USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* Save Idle rate recieved from Host */
            appData.idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*) eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData.usbDevHandle, & (appData.idleRate), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function driver returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;
        default:
            // Nothing to do.
            break;
    }
    return USB_DEVICE_HID_EVENT_RESPONSE_NONE;
}

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    switch (event) {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Host has de configured the device or a bus reset has happened.
             * Device layer is going to de-initialize all function drivers.
             * Hence close handles to all function drivers (Only if they are
             * opened previously. */

            appData.deviceConfigured = false;
            appData.state = QFC_USB_STATE_WAIT_FOR_CONFIGURATION;
            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* Set the flag indicating device is configured. */
            appData.deviceConfigured = true;

            /* Save the other details for later use. */
            appData.configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData)->configurationValue;

            /* Register application HID event handler */
            USB_DEVICE_HID_EventHandlerSet(USB_DEVICE_HID_INDEX_0, APP_USBDeviceHIDEventHandler, (uintptr_t) & appData);

            /* Update the LEDs */


            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch on green and orange, switch off red */

            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */

            USB_DEVICE_Attach(appData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available */
            USB_DEVICE_Detach(appData.usbDevHandle);
            break;

            /* These events are not used in this demo */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void QFC_USB_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = QFC_USB_STATE_INIT;
    appData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    appData.deviceConfigured = false;
    appData.txTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    appData.rxTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    appData.hidDataReceived = false;
    appData.hidDataTransmitted = true;
    appData.receiveDataBuffer = &receiveDataBuffer[0];
    appData.transmitDataBuffer = &transmitDataBuffer[0];
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void QFC_USB_Tasks(void) {

    /* Check if device is configured.  See if it is configured with correct
     * configuration value  */

    switch (appData.state) {
        case QFC_USB_STATE_INIT:

            /* Open the device layer */
            appData.usbDevHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);
            if (appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle, APP_USBDeviceEventHandler, 0);

                appData.state = QFC_USB_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case QFC_USB_STATE_WAIT_FOR_CONFIGURATION:
            if (appData.deviceConfigured == true) {
                /* Device is ready to run the main task */
                appData.hidDataReceived = false;
                appData.hidDataTransmitted = true;
                appData.state = QFC_USB_STATE_MAIN_TASK;

                /* Place a new read request. */
                USB_DEVICE_HID_ReportReceive(USB_DEVICE_HID_INDEX_0,
                        &appData.rxTransferHandle, appData.receiveDataBuffer, 64);
            }
            break;

        case QFC_USB_STATE_MAIN_TASK:
            if (horizontalCalibrate) {
                while (DCH0INTbits.CHBCIF == 0);
                DCH0INTbits.CHBCIF = 0;
                ConvertRawData();
                calibrateData.roll += AccelRoll();
                calibrateData.pitch += AccelPitch();
                ++calibrateCount;
                if (((systemTickCount - startTime) / 200) % 2 == 0) {
                    BlueLEDOn();
                } else {
                    BlueLEDOff();
                }
                if (systemTickCount - startTime > 2000) {
                    DisableExternalInt4();
                    horizontalCalibrate = false;
                    calibrateData.roll /= calibrateCount;
                    calibrateData.pitch /= calibrateCount;
                    BlueLEDOff();
                }
            }

            if (verticalCalibrate) {
                while (DCH0INTbits.CHBCIF == 0);
                DCH0INTbits.CHBCIF = 0;
                ConvertRawData();
                calibrateData.yaw += AccelYaw();
                verticalSign += rawData.accel_z;
                calibrateData.pitch += AccelPitch();
                ++calibrateCount;
                if (((systemTickCount - startTime) / 200) % 2 == 0) {
                    BlueLEDOn();
                } else {
                    BlueLEDOff();
                }
                if (systemTickCount - startTime > 2000) {
                    DisableExternalInt4();
                    verticalCalibrate = false;
                    calibrateData.yaw /= calibrateCount;
                    calibrateData.pitch /= calibrateCount;
                    if (verticalSign < 0) {
                        calibrateData.pitch = (-M_PI / 2.0) + calibrateData.pitch;
                    } else {
                        calibrateData.pitch = (M_PI / 2.0) - calibrateData.pitch;
                    }
                    BlueLEDOff();
                }
            }

            if (centering) {
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
                if (((systemTickCount - startTime) / 200) % 2 == 0) {
                    BlueLEDOn();
                } else {
                    BlueLEDOff();
                }
                if (systemTickCount - startTime > 2000) {
                    DisableAilInput();
                    DisableElevInput();
                    DisableRudInput();
                    centering = false;
                    if (ailCount > 0) {
                        config.ailServo.center = (ailTotal / ailCount);
                        config.ail2Servo.center = config.ailServo.center;
                    }
                    if (elevCount > 0) {
                        config.elevServo.center = (elevTotal / elevCount);
                    }
                    if (rudCount > 0) {
                        config.rudServo.center = (rudTotal / rudCount);
                    }
                    config.crc = calcCRC((uint8_t *) & config, sizeof (QFCConfig) - 2);
                    BlueLEDOff();
                }
            }
            if (!appData.deviceConfigured) {
                /* Device is not configured */
                appData.state = QFC_USB_STATE_WAIT_FOR_CONFIGURATION;
            } else if (appData.hidDataReceived) {
                /* Look at the data the host sent, to see what
                 * kind of application specific command it sent. */
                ProcessUSBCommand();
                appData.hidDataReceived = false;
                /* Place a new read request. */
                USB_DEVICE_HID_ReportReceive(USB_DEVICE_HID_INDEX_0,
                        &appData.rxTransferHandle, appData.receiveDataBuffer, 64);
            }
        case QFC_USB_STATE_ERROR:
            break;
        default:
            break;
    }
}


/*******************************************************************************
 End of File
 */

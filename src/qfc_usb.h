//*******************************************************************//
// File: qfc_usb.h                                                   //
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

#ifndef _QFC_USB_H
#define _QFC_USB_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "usb/usb_chapter_9.h"
#include "usb/usb_device.h"

/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* Application is initializing */
    QFC_USB_STATE_INIT,

    /* Application is waiting for configuration */
    QFC_USB_STATE_WAIT_FOR_CONFIGURATION,

    /* Application is running the main tasks */
    QFC_USB_STATE_MAIN_TASK,

    /* Application is in an error state */
    QFC_USB_STATE_ERROR
            
} QFC_USB_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    QFC_USB_STATES state;

      /* Device layer handle returned by device layer open function */
    USB_DEVICE_HANDLE  usbDevHandle;

    /* Recieve data buffer */
    uint8_t * receiveDataBuffer;

    /* Transmit data buffer */
    uint8_t * transmitDataBuffer;

    /* Device configured */
    bool deviceConfigured;

    /* Send report transfer handle*/
    USB_DEVICE_HID_TRANSFER_HANDLE txTransferHandle;

    /* Receive report transfer handle */
    USB_DEVICE_HID_TRANSFER_HANDLE rxTransferHandle;

    /* Configuration value selected by the host*/
    uint8_t configurationValue;

    /* HID data received flag*/
    bool hidDataReceived;

    /* HID data transmitted flag */
    bool hidDataTransmitted;

     /* USB HID current Idle */
    uint8_t idleRate;

} QFC_USB_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony Demo application initialization routine

  Description:
    This routine initializes Harmony Demo application.  This function opens
    the necessary drivers, initializes the timer and registers the application
    callback with the USART driver.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void QFC_USB_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void QFC_USB_Tasks ( void );



extern const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[1];
extern const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor;



#endif /* _QFC_USB_H */
/*******************************************************************************
 End of File
 */


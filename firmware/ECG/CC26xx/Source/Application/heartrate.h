/*******************************************************************************
  Filename:       heartRate.h
  Revised:        $Date: 2014-04-17 15:59:23 -0700 (Thu, 17 Apr 2014) $
  Revision:       $Revision: 38222 $

  Description:    This file contains the Heart Rate sample application
                  definitions and prototypes.

  Copyright 2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
*******************************************************************************/

#ifndef HEARTRATE_H
#define HEARTRATE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
   
/*********************************************************************
 * MACROS
 */
   
// Internal events for RTOS application.
// Heart Rate events.
#define HEARTRATE_MEAS_EVT                              0x0001
#define HEARTRATE_MEAS_PERIODIC_EVT                     0x0002

// Key Press events.
#define HEARTRATE_KEY_CHANGE_EVT                        0x0010

// Peripheral State Change Event.
#define HEARTRATE_STATE_CHANGE_EVT                      0x0020

#define HEARTRATE_FALL_DOWN_EVT                         0x0040   
   
   
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Heart Rate.
 */
extern void HeartRate_createTask(void);
extern void HeartRate_sendEvent(UArg arg);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HEARTRATE_H */

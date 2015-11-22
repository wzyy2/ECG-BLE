/*******************************************************************************
  Filename:       heartRate.c
  Revised:        $Date: 2015-07-20 11:31:07 -0700 (Mon, 20 Jul 2015) $
  Revision:       $Revision: 44370 $

  Description:    This file contains the Heart Rate sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2011 - 2015 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "gatt_profile_uuid.h"
#include "heartrateservice.h"
#include "devinfoservice.h"
#include "battservice.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_key.h"
#include "Board.h"

#include "heartrate.h"
#include "ecg.h"
   
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Fast advertising interval in 625us units.
#define DEFAULT_FAST_ADV_INTERVAL                       32

// Duration of fast advertising duration in ms.
#define DEFAULT_FAST_ADV_DURATION                       30000

// Slow advertising interval in 625us units.
#define DEFAULT_SLOW_ADV_INTERVAL                       1600

// Slow advertising duration in ms (set to 0 for continuous advertising).
#define DEFAULT_SLOW_ADV_DURATION                       0

// How often to perform heart rate periodic event.
#define DEFAULT_HEARTRATE_PERIOD                        40

// Whether to enable automatic parameter update request when a connection is 
// formed.
#define DEFAULT_ENABLE_UPDATE_REQUEST                   FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update 
// request is enabled.
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL               32

// Maximum connection interval (units of 1.25ms) if automatic parameter update 
// request is enabled.
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL               32

// Slave latency to use if automatic parameter update request is enabled.
#define DEFAULT_DESIRED_SLAVE_LATENCY                   0

// Supervision timeout value (units of 10ms) if automatic parameter update 
// request is enabled.
#define DEFAULT_DESIRED_CONN_TIMEOUT                    120

// Task configuration
#define HEARTRATE_TASK_PRIORITY                         1
#define HEARTRATE_TASK_STACK_SIZE                       644

#define HEARTRATE_MEAS_LEN                              20
   
// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         2

/*********************************************************************
 * TYPEDEFS
 */
   
// App event passed from profiles.
typedef struct 
{
  appEvtHdr_t hdr;  // Event header
} heartRateEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages.
static ICall_EntityID selfEntity;

// HeartRate_semaphore globally used to post events to the application thread.
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct measPerClock;

static Clock_Struct battPerClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct HeartRate_task;
Char HeartRate_taskStack[HEARTRATE_TASK_STACK_SIZE];

// Profile state parameter.
static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP Profile - Name attribute for SCAN RSP data (max size = 31 bytes).
static uint8_t scanData[] =
{
  // Complete name.
  0x07,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'E',
  'C',
  'G',
  '-',
  'C',
  'H'
};

//GAP - Advertisement data (max size = 31 bytes)
static uint8_t advertData[] = 
{ 
  // Flags
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // Service UUIDs
  0x03,
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(HEARTRATE_SERV_UUID),
  HI_UINT16(HEARTRATE_SERV_UUID)
};

// Device name attribute value.
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "ECG-CH";

// GAP connection handle.
static uint16_t gapConnHandle;

// Advertising user-cancelled state.
static bool advCancelled = FALSE;

static bool is_enable = false;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

// Task functions and message processing.
static void HeartRate_init(void);
static void HeartRate_taskFxn(UArg a0, UArg a1);
static void HeartRate_processStackMsg(ICall_Hdr *pMsg);
static void HeartRate_processGattMsg(gattMsgEvent_t *pMsg);
static void HeartRate_processAppMsg(heartRateEvt_t *pMsg);
static uint8_t HeartRate_enqueueMsg(uint8_t event, uint8_t state);
static void HeartRate_measPerTask(void);
static void HeartRate_measNotify(void);
static bool HeartRate_toggleAdvertising(void);
void HeartRate_fallNotify(void);

// Events and callbacks for profiles and keys.
static void HeartRate_serviceCB(uint8_t event);
static void HeartRate_heartRateEvt(uint8_t event);
static void HeartRate_keyPressHandler(uint8_t keys);
static void HeartRate_handleKeys(uint8_t shift, uint8_t keys);
static void HeartRate_stateChangeCB(gaprole_States_t newState);
static void HeartRate_stateChangeEvt(gaprole_States_t pEvent);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks.
static gapRolesCBs_t heartRatePeripheralCB =
{
  HeartRate_stateChangeCB  // Profile State Change Callbacks.
};

// Bond Manager Callbacks.
static const gapBondCBs_t heartRateBondCB =
{
  NULL,                    // Passcode callback.
  NULL                     // Pairing state callback.
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HeartRate_createTask
 *
 * @brief   Task creation function for the Heart Rate.
 *
 * @param   none
 *
 * @return  none
 */
void HeartRate_createTask(void)
{
  Task_Params taskParams;
    
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = HeartRate_taskStack;
  taskParams.stackSize = HEARTRATE_TASK_STACK_SIZE;
  taskParams.priority = HEARTRATE_TASK_PRIORITY;
  
  Task_construct(&HeartRate_task, HeartRate_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      HeartRate_init
 *
 * @brief   Initialization function for the Heart Rate application thread.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
void HeartRate_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the DB Address till CC2650 board gets its own IEEE address.
  //uint8_t bdAddress[B_ADDR_LEN] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22 };
  //HCI_EXT_SetBDADDRCmd(bdAddress);
    
  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
  
  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&measPerClock, HeartRate_sendEvent,
                      DEFAULT_HEARTRATE_PERIOD, 0, false,
                      HEARTRATE_MEAS_PERIODIC_EVT);

#ifdef DEBUG_BUILD
  // Initialize keys on SRF06.
  Board_initKeys(HeartRate_keyPressHandler);
#endif
  
  // Setup the GAP
  //GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);
  
  // Setup the GAP Peripheral Role Profile.
  {
#if ADVERTISE_WHEN_NOT_CONNECTED
    uint8_t initial_advertising_enable = TRUE;
#else
    // Press right key to initiate advertising and measurement.
    uint8_t initial_advertising_enable = FALSE;
#endif //ADVERTISE_WHEN_NOT_CONNECTED

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE.
    uint16_t gapRole_AdvertOffTime = 0;
      
    uint8_t enable_update_request  = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desired_min_interval  = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desired_max_interval  = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desired_conn_timeout  = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters.
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                          &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                          &gapRole_AdvertOffTime);
    
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof (scanData),
                          scanData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData),
                          advertData);
    
    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t), 
                          &enable_update_request);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), 
                          &desired_min_interval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), 
                          &desired_max_interval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t), 
                          &desired_slave_latency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t), 
                          &desired_conn_timeout);
  }
  
  // Set the GAP Characteristics.
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
  
  // Setup the GAP Bond Manager.
  {
    uint32 passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = FALSE;
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t bonding = FALSE;
    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof (uint32),
                             &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof (uint8_t),
                             &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof (uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof (uint8_t),
                             &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof (uint8_t),
                             &bonding);
  }
  
  // Initialize GATT attributes.
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
  
  // Add device info service.
  //DevInfo_AddService();
  // Add heart rate service.
  HeartRate_AddService(GATT_ALL_SERVICES);  
  
  // Register for Heart Rate service callback.
  HeartRate_Register(&HeartRate_serviceCB);
  
  // Start the Device.
  GAPRole_StartDevice(&heartRatePeripheralCB);
  
  // Start the Bond Manager.
  GAPBondMgr_Register((gapBondCBs_t *)&heartRateBondCB);
  
  // Register with GAP for HCI/Host messages
  //GAP_RegisterForMsgs(selfEntity);
  
  // Register for GATT local events and ATT Responses pending for transmission
  //GATT_RegisterForMsgs(selfEntity);  
}

/*********************************************************************
 * @fn      HeartRate_taskFxn
 *
 * @brief   Application task entry point for the Heart Rate.
 *
 * @param   none
 *
 * @return  none
 */     
static void HeartRate_taskFxn(UArg a0, UArg a1)
{
  // Initialize the application.
  HeartRate_init();
  
  // Application main loop.
  for(;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a 
    // message is queued to the message receive queue of the thread or when the
    // ICall_signal() function is called on the thread's respective semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);
    
    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;
      
      if (ICall_fetchServiceMsg(&src, &dest, 
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message.
          HeartRate_processStackMsg((ICall_Hdr *)pMsg);
        }
                
        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }
    
    // If RTOS queue is not empty, process app message.
    while (!Queue_empty(appMsgQueue))
    {
      heartRateEvt_t *pMsg = (heartRateEvt_t*)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message.
        HeartRate_processAppMsg(pMsg);
        
        // Free the space from the message.
        ICall_free(pMsg);
      }
    }
    
    // Heart rate service periodic task.
    if (events & HEARTRATE_MEAS_PERIODIC_EVT)
    {
      events &= ~HEARTRATE_MEAS_PERIODIC_EVT;
      
      HeartRate_measPerTask();
    } else if (events & HEARTRATE_FALL_DOWN_EVT) {
      events &= ~HEARTRATE_FALL_DOWN_EVT;   
      HeartRate_fallNotify();
    }
    
  }
}

/*********************************************************************
 * @fn      HeartRate_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HeartRate_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      HeartRate_processGattMsg((gattMsgEvent_t *)pMsg);
      break;
    
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      HeartRate_processGattMsg
 *
 * @brief   Process GATT messages.
 *
 * @param   pMsg - pointer the the GATT message.
 *
 * @return  none
 */
static void HeartRate_processGattMsg(gattMsgEvent_t *pMsg)
{
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      HeartRate_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */                       
static void HeartRate_processAppMsg(heartRateEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case HEARTRATE_STATE_CHANGE_EVT:
      HeartRate_stateChangeEvt((gaprole_States_t)pMsg->hdr.state);
      break;
      
    case HEARTRATE_KEY_CHANGE_EVT:
      HeartRate_handleKeys(0, pMsg->hdr.state);
      break;
      
    case HEARTRATE_MEAS_EVT:  
      HeartRate_heartRateEvt(pMsg->hdr.state);
      break;
    
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      HeartRate_keyPressHandler
 *
 * @brief   Key event handler function.
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void HeartRate_keyPressHandler(uint8_t keys)
{
  // Enqueue the event.
  HeartRate_enqueueMsg(HEARTRATE_KEY_CHANGE_EVT, keys);
}

/*********************************************************************
 * @fn      HeartRate_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void HeartRate_handleKeys(uint8_t shift, uint8_t keys)
{
  // Up key.
  if (keys & KEY_UP)
  {
    // Set simulated measurement flag index.
  }
  
  // Left key.
  if (keys & KEY_LEFT)
  {
    // If not in a connection, toggle advertising on and off.
    if(gapProfileState != GAPROLE_CONNECTED)
    {  
      // Set fast advertising interval for user-initiated connections.
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL);
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL);
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION);

      // Toggle GAP advertisement status.      
      // Set flag if advertising was cancelled.
      if (HeartRate_toggleAdvertising() == FALSE)
      {
        advCancelled = TRUE;
      }
    }
  }
}

/*********************************************************************
 * @fn      HeartRate_toggleAdvertising
 *
 * @brief   Toggle advertising state.
 *
 * @param   none
 *
 * @return  status - TRUE if advertising, FALSE otherwise.
 */
static bool HeartRate_toggleAdvertising(void)
{
  uint8_t advState;
  
  // Find the current GAP advertisement status.
  GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &advState);
  
  // Get the opposite state.
  advState = !advState;
  
  // Change the GAP advertisement status to opposite of current status.
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), 
                       &advState);  
  
  return advState;
}

/*********************************************************************
 * @fn      HeartRate_measNotify
 *
 * @brief   Prepare and send a heart rate measurement notification.
 *
 * @return  none
 */
static void HeartRate_measNotify(void)
{
  attHandleValueNoti_t heartRateMeas;

  if(fifo8_status(&ecg_data)) {
    
    heartRateMeas.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_NOTI,
                                         HEARTRATE_MEAS_LEN, NULL);
    if (heartRateMeas.pValue != NULL)
    {
      uint8_t *p = heartRateMeas.pValue;
      
      // Build heart rate measurement structure from simulated values.
      //*p++ = flags;
      for(int i = 0; i < 10; i++) {
        if(fifo8_status(&ecg_data)) {
          uint16_t val = fifo8_get(&ecg_data);
          *p++ = LO_UINT16(val);
          *p++ = HI_UINT16(val);       
        }
      }
      
      heartRateMeas.len = (uint8)(p - heartRateMeas.pValue);
      
      // Send notification.
      if (HeartRate_MeasNotify(gapConnHandle, &heartRateMeas) != SUCCESS)
      {
        GATT_bm_free((gattMsg_t *)&heartRateMeas, ATT_HANDLE_VALUE_NOTI);
      }
      
    }
  }
}

void HeartRate_fallNotify(void)
{
  attHandleValueNoti_t Falldown;

    Falldown.pValue = GATT_bm_alloc(gapConnHandle, ATT_HANDLE_VALUE_NOTI,
                                         2, NULL);
    if (Falldown.pValue != NULL)
    {
      uint8_t *p = Falldown.pValue;
      
      
      // Get Fall-Down value
      HeartRate_GetParameter(HEARTRATE_COMMAND, p);
      static uint8_t total = 0;
      
      *p++ = total++;
      *p++ = total++;
      
      Falldown.len = (uint8)(p - Falldown.pValue);
      
      // Send notification.
      if (HeartRate_FallNotify(gapConnHandle, &Falldown) != SUCCESS)
      {
        GATT_bm_free((gattMsg_t *)&Falldown, ATT_HANDLE_VALUE_NOTI);
      }
     
    }
}


/*********************************************************************
 * @fn      HeartRate_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void HeartRate_stateChangeCB(gaprole_States_t newState)
{
  // Enqueue the event.
  HeartRate_enqueueMsg(HEARTRATE_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      HeartRate_stateChangeEvt
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void HeartRate_stateChangeEvt(gaprole_States_t newState)
{
  // If no change to the GAP Role state has occurred
  if (gapProfileState == newState)
  {
    return; 
  }
  
  // If connected
  if (newState == GAPROLE_CONNECTED)
  {
    // Get connection handle.
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
  }
  // If disconnected
  else if (gapProfileState == GAPROLE_CONNECTED && 
           newState != GAPROLE_CONNECTED)
  {
    // Stop periodic measurement of heart rate.
    HeartRate_heartRateEvt(HEARTRATE_MEAS_NOTI_DISABLED);
    
    if (newState == GAPROLE_WAITING_AFTER_TIMEOUT)
    {
      // Link loss timeout-- use fast advertising
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL);
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL);
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION);
    }
    else
    {
      // Else use slow advertising
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL);
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL);
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION);
    }

    // Enable advertising.
    HeartRate_toggleAdvertising();
  }
  // If advertising stopped
  else if (gapProfileState == GAPROLE_ADVERTISING && 
            newState == GAPROLE_WAITING)
  {
    // If advertising stopped by user
    if (advCancelled)
    {
      // Disable advertising.
      advCancelled = FALSE;
    }
    // Else if fast advertising switch to slow
    else if (GAP_GetParamValue(TGAP_GEN_DISC_ADV_INT_MIN) == DEFAULT_FAST_ADV_INTERVAL)
    { 
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL);
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL);
      GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION);
      
      // Enable advertising.
      HeartRate_toggleAdvertising();
    }
#if ADVERTISE_WHEN_NOT_CONNECTED
    else
    {
      // Test mode: continue advertising.
      HeartRate_toggleAdvertising();
    }
#endif //ADVERTISE_WHEN_NOT_CONNECTED
  }
#if ADVERTISE_WHEN_NOT_CONNECTED  
  else if (newState == GAPROLE_WAITING_AFTER_TIMEOUT)
  {
    // Test mode: continue advertising.
    HeartRate_toggleAdvertising();      
  }
#endif //ADVERTISE_WHEN_NOT_CONNECTED
  // If started
  else if (newState == GAPROLE_STARTED)
  {
    // Set the system ID from the bd addr.
    uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);
    
    // Shift three bytes up.
    systemId[7] = systemId[5];
    systemId[6] = systemId[4];
    systemId[5] = systemId[3];
    
    // Set middle bytes to zero.
    systemId[4] = 0;
    systemId[3] = 0;
    
    // Pass systemId to the Device Info service.
    //DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
  }
  
  // Update GAP profile state.
  gapProfileState = newState;
}

/*********************************************************************
 * @fn      HeartRate_serviceCB
 *
 * @brief   Callback function for heart rate service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HeartRate_serviceCB(uint8_t event)
{
  // Enqueue the message.
  HeartRate_enqueueMsg(HEARTRATE_MEAS_EVT, event);
}

/*********************************************************************
 * @fn      HeartRate_heartRateEvt
 *
 * @brief   event handler for heart rate service callbacks.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void HeartRate_heartRateEvt(uint8_t event)
{
  if (event == HEARTRATE_MEAS_NOTI_ENABLED)
  {
    // If connected start periodic measurement.
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      is_enable = true;
      Util_startClock(&measPerClock);
      ecg_SwitchEvt(ECG_ENABLE_EVT);
    }
  }
  else if (event == HEARTRATE_MEAS_NOTI_DISABLED)
  {
    // Stop periodic measurement.
    is_enable = false;
    Util_stopClock(&measPerClock);
    ecg_SwitchEvt(ECG_DISABLE_EVT);
  }
}


/*********************************************************************
 * @fn      HeartRate_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   none
 *
 * @return  none
 */
void HeartRate_sendEvent(UArg arg)
{ 
  // Store the event.
  events |= arg;
  
  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HeartRate_perTask
 *
 * @brief   Perform a periodic heart rate measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void HeartRate_measPerTask(void)
{
  if (gapProfileState == GAPROLE_CONNECTED && is_enable)
  {
    // Get Mutex for ecg data
    Semaphore_pend(data_mutex, BIOS_WAIT_FOREVER);
    
    // Send heart rate measurement notification.
    //HeartRate_fallNotify();
    HeartRate_measNotify();
    HeartRate_measNotify();
    //HeartRate_measNotify();
    //HeartRate_measNotify();

    // Realse Mutex
    Semaphore_post(data_mutex);
   
    // Restart timer.
    Util_startClock(&measPerClock);
  }
}

/*********************************************************************
 * @fn      HeartRate_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  TRUE or FALSE
 */
static uint8_t HeartRate_enqueueMsg(uint8_t event, uint8_t state)
{
  heartRateEvt_t *pMsg;
  
  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(heartRateEvt_t)))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    
    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }
  
  return FALSE;
}


/*********************************************************************
*********************************************************************/

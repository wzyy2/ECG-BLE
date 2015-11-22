#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/BIOS.h>

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
#include "ADS1x9x.h"
#include "util.h"
#include "board_key.h"
#include "Board.h"
#include "bsp_spi.h"
#include "heartrate.h"
#include "ecg.h"

#define BUF_SIZE 1000

struct FIFO8 ecg_data;
uint16_t __buf[BUF_SIZE];

Semaphore_Handle appSem = NULL;
Semaphore_Handle data_mutex = NULL;

Task_Struct ecg_task;
Char ecg_taskStack[1024];

// events flag for internal application events.
static uint16_t events;

void ecg_entry(UArg a0, UArg a1);

void ecg_clockHandler(UArg arg);

void ecg_PerTask();

void ecg_createTask(void)
{
  Task_Params taskParams;
    
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = ecg_taskStack;
  taskParams.stackSize = 1024;
  taskParams.priority = 2;
  
  Task_construct(&ecg_task, ecg_entry, &taskParams, NULL);
}

void ecg_init()
{
  appSem = Semaphore_create(0, NULL, NULL);
  
  Semaphore_Params params;
  Semaphore_Params_init(&params);
  params.mode = Semaphore_Mode_BINARY_PRIORITY;
  params.mode = Semaphore_Mode_BINARY;
  data_mutex = Semaphore_create(1, &params, NULL);
  
  fifo8_init(&ecg_data, BUF_SIZE, __buf);
  
  ADS1x9x_PowerOn_Init();
}

void ecg_entry(UArg a0, UArg a1)
{
  ecg_init();

  while(1)
  {
    if (Semaphore_pend(appSem, BIOS_WAIT_FOREVER))
    {
      if (events & ECG_AD_EVT)
      {
        events &= ~ECG_AD_EVT;
        // Get Mutex for ecg data
        Semaphore_pend(data_mutex, BIOS_WAIT_FOREVER);        
        receive_entry();
        // Realse Mutex
        Semaphore_post(data_mutex);        
      }           
      else if (events & ECG_ENABLE_EVT)
      {
        events &= ~ECG_ENABLE_EVT;
        // Get Mutex for ecg data
        Semaphore_pend(data_mutex, BIOS_WAIT_FOREVER);           
        fifo8_init(&ecg_data, BUF_SIZE, __buf);
        // Realse Mutex 
        Semaphore_post(data_mutex);    
        
        enter_continuous();
        
      }     
      else if (events & ECG_DISABLE_EVT)
      {
        events &= ~ECG_DISABLE_EVT;
        enter_nothing();       
      }   
      
    }
  }
}

void ecg_SwitchEvt(uint16_t arg)
{
  // Store the event.
  events |= arg;
    
  // Wake up the application.
  Semaphore_post(appSem);
}

void ecg_StartAD(uint16_t arg)
{
  // Store the event.
  events |= arg;
    
  // Wake up the application.
  Semaphore_post(appSem);
}
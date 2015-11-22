#include <string.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/BIOS.h>

#include <ti/drivers/PIN/PINCC26XX.h>
#include <inc/hw_memmap.h>
#include <driverlib/aux_wuc.h>
#include <inc/hw_adi.h>
#include <inc/hw_adi_4_aux.h>
#include <inc/hw_aux_anaif.h> // Will change name to anaif
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>
#include <inc/hw_aux_evctl.h>


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
#include "heartrate.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"
#include "ADXL335.h"
#include "util.h"

#define SAMPLECOUNT 8
#define SAMPLETYPE uint16_t
#define SAMPLESIZE sizeof(SAMPLETYPE)

#ifdef DEBUG_BUILD
#define ALS_X   IOID_25 //aux5 //
#define ALS_Y  IOID_26 //aux4
#define ALS_Z  IOID_27 //aux3
#else
#define ALS_X   IOID_7 //aux5 //
#define ALS_Y  IOID_8 //aux4
#define ALS_Z  IOID_9 //aux3
#endif


#define ALS_AUX_X   ADC_COMPB_IN_AUXIO5 //aux5
#define ALS_AUX_Y  ADC_COMPB_IN_AUXIO4 //aux4
#define ALS_AUX_Z  ADC_COMPB_IN_AUXIO3 //aux3

// Analog light sensor pins
const PIN_Config alsPins[] = {
  ALS_X      | PIN_INPUT_DIS | PIN_GPIO_OUTPUT_DIS ,
  ALS_Y      | PIN_INPUT_DIS | PIN_GPIO_OUTPUT_DIS ,
  ALS_Z      | PIN_INPUT_DIS | PIN_GPIO_OUTPUT_DIS ,  
  PIN_TERMINATE
};

static PIN_Handle pinHandle;
static PIN_State  pinState;

Task_Struct Adc_task;
Char Adc_taskStack[512];
void Adc_entry(UArg a0, UArg a1);


// auxIo, see MUX3 Register (Offset = 3h) [reset = X]
// auxIo, sensor controller engine IO, will map to M3's IO automatilcally
// for DIO23, auxIO7, the value should be 80h, 0x80
uint16_t OneShotADC(uint8_t auxIo)
{
  static __root uint16_t retval = 0xABBA;
  
  // Enable clock for ADC digital and analog interface (not currently enabled in driver)
  AUXWUCClockEnable(AUX_WUC_MODCLKEN0_ANAIF_M|AUX_WUC_MODCLKEN0_AUX_ADI4_M);
  // Connect AUX IO7 (DIO23) as analog input. Light sensor on SmartRF06EB
  AUXADCSelectInput(auxIo);
  
  // Set up ADC
  AUXADCEnableSync(AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL);
  
  // Disallow STANDBY mode while using the ADC.
  Power_setConstraint(Power_SB_DISALLOW);
  
  // Trigger ADC sampling
  AUXADCGenManualTrigger();
  
  retval = AUXADCReadFifo();
  
  // Disable ADC
  AUXADCDisable();
  // Allow STANDBY mode again
  Power_releaseConstraint(Power_SB_DISALLOW);
  
  return retval;
}

void Adc_createTask(void)
{
  Task_Params taskParams;
  
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = Adc_taskStack;
  taskParams.stackSize = 512;
  taskParams.priority = 3;
  
  Task_construct(&Adc_task, Adc_entry, &taskParams, NULL);
}

void Adc_init()
{
  // Set up pins
  pinHandle = PIN_open(&pinState, alsPins);
}

int valx,valz,valy;
int32_t Svm;
int32_t maxSvm;

void Adc_entry(UArg a0, UArg a1)
{
  uint8_t p = 0;
  uint8_t total = 0;
  Adc_init();
//1800 1560  1100   x  y  z
// 1800 1200      

  while(1)
  {
    HeartRate_GetParameter(HEARTRATE_COMMAND, &p);
    if( p == 0) {
      valx = OneShotADC(ALS_AUX_X) * 43 / 33;
      valy = OneShotADC(ALS_AUX_Y) * 43 / 33;
      valz = OneShotADC(ALS_AUX_Z) * 43 / 33;
      ADXL335_setVal(valx, valy, valz);
      Svm = ADXL335_Filter();
      if(total < 8) {
        total++;
        maxSvm = 0;
      }
      maxSvm = max(Svm, maxSvm);
      

      if( _ADXL335_IsDown( Svm ) ) {
        p = 1;
        HeartRate_SetParameter(HEARTRATE_COMMAND, 1, &p);
      };

      Task_sleep(5);
    } else {
      Task_sleep(1000);
    }
      
    
  }
}
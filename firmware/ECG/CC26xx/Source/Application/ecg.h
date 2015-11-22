#ifndef ECG_H
#define ECG_H
#include "fifo.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/BIOS.h>

#define ECG_PERIODIC_EVT                     0x0001
#define ECG_ENABLE_EVT                     0x0002
#define ECG_DISABLE_EVT                     0x0004
#define ECG_AD_EVT                     0x0008

extern struct FIFO8 ecg_data;
extern Semaphore_Handle data_mutex;

void ecg_createTask(void);
void ecg_SwitchEvt(uint16_t arg);
void ecg_StartAD(uint16_t arg);

#endif
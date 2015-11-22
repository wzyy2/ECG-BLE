#include "ADXL335.h"
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>
#include <inc/hw_aux_evctl.h>
#include <iir.h>

#define FIRSTLIMITMG		2*1000

#define AdcChannel_X	1
#define AdcChannel_Y	2
#define AdcChannel_Z	3
#define AdcPrecision	12

#define AXISSVM			3


enum TaskEnum
{
  Task_Init,
  Task_EdgeP,
  Task_PeakWidthP,
  Task_PeakP,
  Task_PeakWait,
  Task_PeakWidthN,
  Task_PeakN,
  Task_DownBackDelay,
  Task_DownBack
};

#define Def_EdgePLimit			110						
#define Def_PeakPLimit			700					
#define Def_WidthPLimitMs		60					
#define Def_PeakNLimit			500					
#define Def_WidthNLimit			180						
#define Def_PeakNWaveLimit		50						
#define Def_PPLimit				1600					
#define Def_DownBackDelay		80						
#define Def_DownBackWidthMs		1200					
#define Def_RemoveCountMax		8						

//--------------------------------------------------------
struct Axis
{
  uint32_t AdcGroupId;
  int32_t AdcValue;
  int32_t mg;
  
  double Filter_HighPass[6];
};

//--------------------------------------------------------
struct _ADXL335
{
  struct Axis sAxis[3];
  
  bool bIsBusy;
  bool bIsAdcFinished;
  
  int32_t LastSvm;
  double Svm_HighPass[6];
  double Svm_LowPass[6];
  bool bIsDown;
  
  uint32_t _Task;
  
  uint32_t _EdgeLimitTimer;
  int32_t _LastWave[2];
  int32_t _PeakStartValue;
  uint32_t _PeakNOverLimitCounter;
  
  int32_t _Max;
  int32_t _Min;
  
  int32_t DownHoldValue;
  int32_t DownCycleLimitTimer;				 
  uint32_t RemoveCounter;						 
  
};
struct _ADXL335 _sADXL335;
#define this _sADXL335


int32_t ADXL335_Filter( void )
{
  uint32_t i;
  int32_t temp;
  struct Axis *psAxi = &this.sAxis[0];
  

  for(i=0; i<3; i++ )
  {
    //高通滤波
    temp = IIR_LP2( IIR2_1Hz_300Hz, &psAxi->Filter_HighPass[0], psAxi->AdcValue );
    temp = psAxi->AdcValue - temp;													//è???á÷
    
    
    psAxi->mg = temp;
    psAxi++;
  }
  
  
  temp = sqrt( (double)(this.sAxis[0].mg*this.sAxis[0].mg + this.sAxis[1].mg*this.sAxis[1].mg + this.sAxis[2].mg*this.sAxis[2].mg) );
  
  
  temp = temp - IIR_LP2( IIR2_1Hz_300Hz, &this.Svm_HighPass[0], temp );
  
  
  temp = IIR_LP2( IIR2_25Hz_300Hz, &this.Svm_LowPass[0], temp );
  
  
  if( this.LastSvm > 0)
  {
    if( this.LastSvm < temp )
    {
      
      IIR_LP2_Balance( &this.Svm_HighPass[0] );
      IIR_LP2_Balance( &this.Svm_LowPass[0] );
    }
  }
  this.LastSvm = temp;

  
  return temp;
}


bool _ADXL335_IsDown( int32_t SvmWave )
{
  bool bIsDowned = FALSE;
  int32_t value;
  
  this._Max = max(this._Max, SvmWave);
  this._Min = min(this._Min, SvmWave);
  
  // check if have activity
  if (this.DownCycleLimitTimer > 0)               
  {
    if( --this.DownCycleLimitTimer == 0 )
    {
      this.RemoveCounter = 0;
    }
  }
  
  switch (this._Task)
  {
  case Task_Init:
    this._LastWave[0] = this._LastWave[1] = SvmWave;
    this._Task = Task_EdgeP;
    break;
    
  case Task_EdgeP:     //正向边沿
    if ( (this._LastWave[1] > this._LastWave[0]) && (SvmWave > this._LastWave[1]) )
    {
      value = max(this._LastWave[1] - this._LastWave[0], SvmWave - this._LastWave[1]);
      if (value > Def_EdgePLimit)
      {
        this._PeakStartValue = this._LastWave[0];
        this._EdgeLimitTimer = Def_WidthPLimitMs / 5;           //200Hz: 5mS
        this._Max = -65536;
        this._Min = 65536;
        this._Task = Task_PeakWidthP;
        break;
      }
    }
    break;
    
  case Task_PeakWidthP:     //正阈值
    if (--this._EdgeLimitTimer == 0)
    {
      //没有效幅度
      this._Task = Task_EdgeP;
      break;
    }
    
    value = SvmWave - this._PeakStartValue;
    if (value > Def_PeakPLimit)
    {
      //有有效幅度
      this._Task = Task_PeakP;
    }
    break;
    
  case Task_PeakP:
    if (--this._EdgeLimitTimer == 0)
    {
      // 幅度太宽
      this._Task = Task_EdgeP;
      break;
    }
    value = SvmWave - this._PeakStartValue;
    if (value < Def_PeakPLimit)
    {
      // 有效脉冲小于宽度， 有效 ，转负沿
      this._EdgeLimitTimer = Def_WidthNLimit / 5;           //200Hz: 5mS
      this._Task = Task_PeakWidthN;
      
      if (this.DownCycleLimitTimer > 0)
      {
        this.RemoveCounter++;
      }
    }
    break;
  // 负阈值
  case Task_PeakWidthN:   
    if (--this._EdgeLimitTimer == 0)
    {
      
      this.DownCycleLimitTimer = Def_DownBackWidthMs * 2 / 5;
      this._Task = Task_EdgeP;
      break;
    }
    
    value = -1 * SvmWave;
    if (value > Def_PeakNLimit)
    {
      
      this._EdgeLimitTimer = Def_WidthNLimit / 5;           //200Hz: 5mS
      this._Task = Task_PeakN;
    }
    break;
    
  case Task_PeakN:
    if (--this._EdgeLimitTimer == 0)
    {
      value = this._Max - this._Min;
      if (value > Def_PPLimit)
      {
        
        if (this.DownCycleLimitTimer > 0)
        {
          this.DownCycleLimitTimer += Def_DownBackWidthMs;      //?3?ó
        }
        IIR_LP2_Balance( &this.Svm_HighPass[0] );
        IIR_LP2_Balance( &this.Svm_LowPass[0] );
        
        this._EdgeLimitTimer = Def_DownBackDelay / 5;           //200Hz: 5mS
        this._Task = Task_DownBackDelay;
        break;
      }
    }
    
    value = -1 * SvmWave;
    if (value < Def_PeakNLimit)
    {
      if (++this._PeakNOverLimitCounter >= Def_PeakNWaveLimit / 5)
      {
        
        this.DownCycleLimitTimer = Def_DownBackWidthMs * 2 / 5;
        this._Task = Task_EdgeP;
      }
    }
    else {
      this._PeakNOverLimitCounter = 0;
    }
    break;
    
  case Task_DownBackDelay:
    if (--this._EdgeLimitTimer == 0)
    {
      this.DownHoldValue = SvmWave;                       
      this._EdgeLimitTimer = Def_DownBackWidthMs / 5;           //200Hz: 5mS
      this._Task = Task_DownBack;
    }
    break;
  // 跌后检测  
  case Task_DownBack: 
    value = this.DownHoldValue - SvmWave;
    value = abs(value);                              			//
    this.DownHoldValue = SvmWave;
    if (value > Def_PeakPLimit/4)
    {
      
      this._Task = Task_EdgeP;
    }
    else
    {
      if (--this._EdgeLimitTimer == 0)
      {
        if (this.DownCycleLimitTimer == 0)
        {
          bIsDowned = TRUE;                                
        }
        else {
          if (this.RemoveCounter <= Def_RemoveCountMax)
          {
            bIsDowned = TRUE;                             
          }
        }
        this.DownCycleLimitTimer = Def_DownBackWidthMs * 2 / 5;
        this._Task = Task_EdgeP;
      }
    }
    break;
  }
  
  this._LastWave[0] = this._LastWave[1];
  this._LastWave[1] = SvmWave;
  
  return bIsDowned;
}

//--------------------------------------------------------
//Up
//--------------------------------------------------------
void _ADXL335_Up( void )
{
  
}


void ADXL335_AdcStart( void )
{
  this.bIsBusy = TRUE;
  _ADXL335_Up();
}


bool ADXL335_IsBusy( void )
{
  return this.bIsBusy;
}


bool ADXL335_IsDown( void )
{
  return this.bIsDown;
}

void ADXL335_Init( void )
{ 
  //this.sAxis[0].AdcGroupId = ADC_Register( AdcChannel_X, AdcPrecision, _ADXL335_CallBackX );
  //this.sAxis[1].AdcGroupId = ADC_Register( AdcChannel_Y, AdcPrecision, _ADXL335_CallBackY );
  //this.sAxis[2].AdcGroupId = ADC_Register( AdcChannel_Z, AdcPrecision, _ADXL335_CallBackZ );
}

void ADXL335_setVal( uint16_t x, uint16_t y, uint16_t z)
{ 
  this.sAxis[0].AdcValue = x;
  this.sAxis[1].AdcValue = y;
  this.sAxis[2].AdcValue = z;
}


void ADXL335_MainLoop( void )
{
  if( this.bIsAdcFinished )
  {
    this.bIsAdcFinished = FALSE;
    
    this.bIsDown = _ADXL335_IsDown( ADXL335_Filter() );
    
    this.bIsBusy = FALSE;
  }
}

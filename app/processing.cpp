#include "processing.h"

#define FILTERORDER 161

short CoeffBuf_50Hz_Notch[FILTERORDER] = {
/* Coeff for Notch @ 50Hz @ 500 SPS*/
      -47,   -210,    -25,    144,     17,     84,    249,     24,   -177,
      -58,   -144,   -312,    -44,    191,     78,    185,    357,     42,
     -226,   -118,   -248,   -426,    -61,    243,    134,    290,    476,
       56,   -282,   -169,   -352,   -549,    -70,    301,    177,    392,
      604,     60,   -344,   -200,   -450,   -684,    -66,    369,    191,
      484,    749,     44,   -420,   -189,   -535,   -843,    -32,    458,
      146,    560,    934,    -16,   -532,    -89,   -600,  -1079,     72,
      613,    -50,    614,   1275,   -208,   -781,    308,   -642,  -1694,
      488,   1141,  -1062,    642,   3070,  -1775,  -3344,   9315,  19005,
     9315,  -3344,  -1775,   3070,    642,  -1062,   1141,    488,  -1694,
     -642,    308,   -781,   -208,   1275,    614,    -50,    613,     72,
    -1079,   -600,    -89,   -532,    -16,    934,    560,    146,    458,
      -32,   -843,   -535,   -189,   -420,     44,    749,    484,    191,
      369,    -66,   -684,   -450,   -200,   -344,     60,    604,    392,
      177,    301,    -70,   -549,   -352,   -169,   -282,     56,    476,
      290,    134,    243,    -61,   -426,   -248,   -118,   -226,     42,
      357,    185,     78,    191,    -44,   -312,   -144,    -58,   -177,
       24,    249,     84,     17,    144,    -25,   -210,    -47
};

void ECG_ProcessCurrSample(short *CurrAqsSample, short *FilteredOut)
{

    static unsigned short ECG_bufStart=0, ECG_bufCur = FILTERORDER-1, ECGFirstFlag = 1;
    static short ECG_Pvev_DC_Sample, ECG_Pvev_Sample;/* Working Buffer Used for Filtering*/
    static short ECG_WorkingBuff[2 * FILTERORDER];
    short *CoeffBuf;

    short temp1, temp2, ECGData;


    /* Count variable*/
    unsigned short Cur_Chan;
    short FiltOut;
//	short FilterOut[2];
    CoeffBuf = CoeffBuf_50Hz_Notch;					// filter option is 50Hz Notch & 0.5-150 Hz Band

    if  ( ECGFirstFlag )								// First Time initialize static variables.
    {
        for ( Cur_Chan =0 ; Cur_Chan < FILTERORDER; Cur_Chan++)
        {
            ECG_WorkingBuff[Cur_Chan] = 0;
        }
        ECG_Pvev_DC_Sample = 0;
        ECG_Pvev_Sample = 0;
        ECGFirstFlag = 0;
    }
    temp1 = NRCOEFF * ECG_Pvev_DC_Sample;				//First order IIR
    ECG_Pvev_DC_Sample = (CurrAqsSample[0]  - ECG_Pvev_Sample) + temp1;
    ECG_Pvev_Sample = CurrAqsSample[0];
    temp2 = ECG_Pvev_DC_Sample >> 2;
    ECGData = (short) temp2;

    /* Store the DC removed value in Working buffer in millivolts range*/
    ECG_WorkingBuff[ECG_bufCur] = ECGData;
    ECG_FilterProcess(&ECG_WorkingBuff[ECG_bufCur],CoeffBuf,(short*)&FiltOut);
    /* Store the DC removed value in ECG_WorkingBuff buffer in millivolts range*/
    ECG_WorkingBuff[ECG_bufStart] = ECGData;

    //FiltOut = ECGData[Cur_Chan];

    /* Store the filtered out sample to the LeadInfo buffer*/
    FilteredOut[0] = FiltOut ;//(CurrOut);
    //FilteredOut[0] = ECGData;

    ECG_bufCur++;
    ECG_bufStart++;
    if ( ECG_bufStart  == (FILTERORDER-1))
    {
        ECG_bufStart=0;
        ECG_bufCur = FILTERORDER-1;
    }

    return ;
}

/*********************************************************************************************************/
/*********************************************************************************************************
** Function Name : ECG_FilterProcess()                                  								**
** Description	  :                                                         							**
** 				The function process one sample filtering with 161 ORDER    							**
** 				FIR multiband filter 0.5 t0 150 Hz and 50/60Hz line nose.   							**
** 				The function supports compile time 50/60 Hz option          							**
**                                                                          							**
** Parameters	  :                                                         							**
** 				- WorkingBuff		- In - input sample buffer              							**
** 				- CoeffBuf			- In - Co-eficients for FIR filter.     							**
** 				- FilterOut			- Out - Filtered output                 							**
** Return 		  : None                                                    							**
*********************************************************************************************************/

void ECG_FilterProcess(short * WorkingBuff, short * CoeffBuf, short* FilterOut)
{
    short i, Val_Hi, Val_Lo;
    int s;

    s=( *WorkingBuff--)**CoeffBuf++; // Load first operand -unsigned mult
    for ( i = 0; i < FILTERORDER/10; i++)
    {
        s+=(*WorkingBuff--)*(*CoeffBuf++); // Load first operand -unsigned mult
    }

    /*
    Val_Hi = (((s&0xffff0000)>>16) << 1); //Val_Hi = s << 1;
    Val_Lo = (s&0xffff) >> 15;// Val_Lo = s >> 15;
    Val_Lo &= 0x01;
    */

    *FilterOut = s;
}

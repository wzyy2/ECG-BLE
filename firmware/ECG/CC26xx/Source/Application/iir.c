#include "iir.h"

const double IIR2_1Hz_300Hz[] = {
  1.00, -1.970382898374199, 0.970815130700386,
  0.000108058081547, 0.000216116163094, 0.000108058081547,
};

const double IIR2_25Hz_300Hz[] = {
  1.00, -1.279632424997809, 0.477592250072517,
  0.049489956268677, 0.098979912537354, 0.049489956268677,
};

//-------------------------------------------------
//Filter LowPass Order 2
//-------------------------------------------------
double  IIR_LP2( const double *pTable, double *pBuffer, double Xn )
{
  //YÒÆ¶¯.0-2
  pBuffer[2] = pBuffer[1];
  pBuffer[1] = pBuffer[0];
  
  //X
  pBuffer[5] = pBuffer[4];
  pBuffer[4] = pBuffer[3];
  pBuffer[3] = Xn;
  
  pBuffer[0] =
    (pBuffer[3] + pBuffer[5]) * pTable[3] +
      pBuffer[4] * pTable[4] -
	pBuffer[1] * pTable[1] -
          pBuffer[2] * pTable[2];
  
  return pBuffer[0];
}

//-------------------------------------------------
//Balance LowPass
//-------------------------------------------------
void IIR_LP2_Balance( double *pBuffer )
{
  pBuffer[0] = pBuffer[1] = pBuffer[2] = pBuffer[4] = pBuffer[5] = pBuffer[3];
}

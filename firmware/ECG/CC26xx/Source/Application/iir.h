#ifndef _IIR_FILTER_H__
#define _IIR_FILTER_H__

double  IIR_LP2( const double *pTable, double *pBuffer, double Xn );
void IIR_LP2_Balance( double *pBuffer );

extern const double IIR2_1Hz_300Hz[];
extern const double IIR2_25Hz_300Hz[];


#endif
#ifndef IIR_H
#define IIR_H

/****************************************************************/
/* Constants*/
/****************************************************************/

#define MAX_PEAK_TO_SEARCH 				5
#define MAXIMA_SEARCH_WINDOW			40
#define MINIMUM_SKIP_WINDOW				50

#define RESP_MAX_PEAK_TO_SEARCH 			5
#define RESP_MAXIMA_SEARCH_WINDOW			8
#define RESP_MINIMUM_SKIP_WINDOW			80

#define RESP_SAMPLING_RATE				100
#define RESP_TWO_SEC_SAMPLES  			2 * RESP_SAMPLING_RATE

#define SAMPLING_RATE					250
#define TWO_SEC_SAMPLES  				2 * SAMPLING_RATE

/*threshold = 0.7 * maxima*/
#define QRS_THRESHOLD_FRACTION	0.7

#define MAXCHAN						2
#define FILTERORDER 				161

#define TRUE	1
#define FALSE	0

#define MUSCLE_ARTIFACT_FILTER		1
#define NOTCHFILTERSEL				1		// 0 - 50 Hz Notch filter
                                            // 1 - 60 Hz Notch filter

/* DC Removal Numerator Coeff*/
#define NRCOEFF (0.992)

/****************************************************************/
/* Global functions*/
/****************************************************************/
void ECG_ProcessCurrSample(short *CurrAqsSample, short *FilteredOut);
void ECG_FilterProcess(short * WorkingBuff, short * CoeffBuf, short* FilterOut);
void ADS1x9x_Filtered_ECG(void);

#endif // IIR_H


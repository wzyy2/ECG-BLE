#ifndef __ADXL335H
#define __ADXL335H

#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/BIOS.h>



#define max(a,b) ( ((a)>(b)) ? (a):(b) )
#define min(a,b) ( ((a)>(b)) ? (b):(a) )

	//--------------------------------------------------------
	//����һ�β���
	//--------------------------------------------------------
	extern void ADXL335_AdcStart( void );

	//--------------------------------------------------------
	//������
	//--------------------------------------------------------
	extern bool ADXL335_IsBusy( void );
	
	//--------------------------------------------------------
	//��?��1���騦��
	//--------------------------------------------------------
	extern bool ADXL335_IsDown( void );

	//--------------------------------------------------------
	//3?��??��
	//--------------------------------------------------------
	extern void ADXL335_Init( void );

	//--------------------------------------------------------
	//ѭ��
	//--------------------------------------------------------
	extern void ADXL335_MainLoop( void );
        
        extern bool _ADXL335_IsDown( int32_t SvmWave );
        extern int32_t ADXL335_Filter( void );
        extern void ADXL335_setVal( uint16_t x, uint16_t y, uint16_t z);
        extern bool _ADXL335_IsDown( int32_t SvmWave );
#endif

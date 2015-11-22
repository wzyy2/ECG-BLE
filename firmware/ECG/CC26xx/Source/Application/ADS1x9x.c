

/****************************************************************************************************************************************************
*	ADS1x9x.c  - Provides access to ADS1x9x ECG Data Converter.																					*
* 		Functions: 
* 				1. ADS1x9x_Init() : Initiazation of ADS1292																						*
*				2. ADS1x9x_Default_Reg_Init(): 							                                                  													*
*				3. 								                                                  													*
*												                                                  													*
*****************************************************************************************************************************************************/
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/SPI.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/BIOS.h>
#include "ADS1x9x.h"
#include "bsp_spi.h"
#include "ecg.h"
#include "fifo.h"

/**************************************************************************************************************************************************
*	        Prototypes									                                                  										  *
**************************************************************************************************************************************************/

/**************************************************************************************************************************************************
*	        Global Variables										                                  											  *
**************************************************************************************************************************************************/
unsigned char ADC_Read_data[16];
unsigned char ADS129x_SPI_cmd_Flag=0, ADS129x_SPI_data_Flag=0,  SPI_Send_count=0, SPI_Tx_Count = 0,SPI_Tx_buf[12];
unsigned char SPI_Rx_Data_Flag = 0,  SPI_Rx_buf[12], SPI_Rx_Count=0, SPI_Rx_exp_Count=0 ;
unsigned char ECG_Data_rdy;
long ADS1x9x_ECG_Data_buf[6];

struct ADS1x9x_state ECG_Recoder_state;
unsigned char ECGRecorder_data_Buf[256], Recorder_head,Recorder_tail;
//extern struct ECGPage ECGPageBuf2_data;
//extern unsigned char ECGRecorder_ACQdata_Buf[128];
unsigned char Store_data_rdy;
//unsigned char bankFlag = 0;
//unsigned char ECG_recorderReadyFlag = 0;
//unsigned short sampleCount = 0;

#define DELAY_COUNT 2

/* ADS1x9x Register values*/

unsigned char 	ADS1x9xRegVal[16] = {
  
  //Device ID read Ony
  0x00,
  //CONFIG1
  0x02,
  //CONFIG2
  0xE0,
  //LOFF
  0xF0,
  //CH1SET (PGA gain = 6)
  0x00,
  //CH2SET (PGA gain = 6)
  0x00,
  //RLD_SENS (default)
  0x2C,
  //LOFF_SENS (default)
  0x0F,    
  //LOFF_STAT
  0x00,
  //RESP1
  0xEA,
  //RESP2
  0x03,
  //GPIO
  0x0C 
};		
unsigned char 	ADS1x9xR_Default_Register_Settings[15] = {
  
  //Device ID read Ony
  0x00,
  //CONFIG1
  0x01,
  //CONFIG2
  0xE0,
  //LOFF
  0xF0,
  //CH1SET (PGA gain = 6)
  0x00,
  //CH2SET (PGA gain = 6)
  0x00,
  //RLD_SENS (default)
  0x2C,
  //LOFF_SENS (default)
  0x0F,    
  //LOFF_STAT
  0x00,
  //RESP1
  0xEA,
  //RESP2
  0x03,
  //GPIO
  0x0C 
};		
unsigned char 	ADS1x9x_Default_Register_Settings[15] = {
  
  //Device ID read Ony
  0x00,
  //CONFIG1
  0x01,
  //CONFIG2
  0xA0, //E0
  //LOFF
  0xF0,
  //CH1SET (PGA gain = 12)
  0x60,
  //CH2SET (PGA gain = 6)
  0x00,
  //RLD_SENS (default)
  0x2C,
  //LOFF_SENS (default)
  0x0F,    
  //LOFF_STAT
  0x00,
  //RESP1
  0x02,
  //RESP2
  0x03,
  //GPIO
  0x0C 
};		

/***********************************************************************************************************
*	        Variables for the SPI Interaction                                                          	   *
***********************************************************************************************************/


/**********************************************************************************************************
*	External Variables																			          *
**********************************************************************************************************/
#define ADS_START                   IOID_6          
#define ADS_RESET                   IOID_5        
#define ADS_DRDY                    IOID_3        
#define ADS_MISO                     IOID_2          /* RF1.20 */
#define ADS_MOSI                     IOID_1          /* RF1.18 */
#define ADS_CLK                      IOID_0          /* RF1.16 */
#define ADS_CSN                      IOID_4          /* RF1.14 */

PIN_Config ecgPinsCfg[] =
{
  ADS_START    | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_HIGH  |  PIN_PUSHPULL ,
  ADS_RESET        | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_HIGH  |  PIN_PUSHPULL ,
  ADS_DRDY    | PIN_INPUT_EN  |  PIN_PULLUP,
  ADS_CSN        | PIN_GPIO_OUTPUT_EN  | PIN_GPIO_HIGH  |  PIN_PUSHPULL ,
  PIN_TERMINATE
};

PIN_State  ecgPins;
PIN_Handle hecgPins;

void drdy_isr(PIN_Handle handle, PIN_Id pinId);

static void blocking_delay(uint32_t usDelay)
{
  while(usDelay--)
  {
    /* 32 NOPs == 1 usecs */
    for (int i=0; i < 8; ++i) {
      asm("nop");
    }
    
  }  
}

/**********************************************************************************************************
* ADS1x9x_Clock_Select																					  *
* Input : 0 - external																					  *
* 		: 1 - internal					                                         						  *
**********************************************************************************************************/
void ADS1x9x_Clock_Select(unsigned char clock_in)
{
  
  if (clock_in == 1)
  {
    //P2OUT |= (enum PORT2_ADC_CONTROL)ADC_CLK_SEL;	// Choose internal clock input
  }
  else
  {
    //P2OUT &= ~(enum PORT2_ADC_CONTROL)ADC_CLK_SEL;	// Choose external clock input
  }
  
}
/**********************************************************************************************************
* ADS1x9x_Reset			 			                                         						  	  *
**********************************************************************************************************/
void ADS1x9x_Reset(void)
{
  PIN_setOutputValue(hecgPins, ADS_RESET , 1);
  //P8OUT |= (enum PORT8_ADC_CONTROL)ADC_RESET;		// Set High
  /* Provide suficient dealy*/
  Task_sleep(2);	// Wait 1 mSec
  PIN_setOutputValue(hecgPins, ADS_RESET , 0);    
  //P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_RESET;	// Set to low
  Task_sleep(2);				    // Wait 1 mSec
  PIN_setOutputValue(hecgPins, ADS_RESET , 1);
  //P8OUT |= (enum PORT8_ADC_CONTROL)ADC_RESET;		// Set High
  Task_sleep(2);
}

/**********************************************************************************************************
* ADS1x9x_Disable_Start						                                          					  *
**********************************************************************************************************/
void ADS1x9x_Disable_Start(void)
{
  PIN_setOutputValue(hecgPins, ADS_START , 0);
  //P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_START;	// Set to LOW
  Task_sleep(1);   					// Small Delay to settle   
}
/*********************************************************************************************************/
/**********************************************************************************************************
* ADS1x9x_Disable_Start						                                          					  *
**********************************************************************************************************/
void ADS1x9x_Enable_Start(void)
{
  PIN_setOutputValue(hecgPins, ADS_START , 1);
  //P8OUT |= (enum PORT8_ADC_CONTROL)ADC_START;		// Set to High
  Task_sleep(1);   					// Small Delay to settle   
}
/*********************************************************************************************************/
/**********************************************************************************************************
* Set_ADS1x9x_Chip_Enable																                  *
**********************************************************************************************************/
void Set_ADS1x9x_Chip_Enable (void)
{
  /* ADS1x9x CS is Active low*/
  PIN_setOutputValue(hecgPins, ADS_CSN , 0);
  //P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_CS;		// Set to LOW
}
/**********************************************************************************************************
* Clear_ADS1x9x_Chip_Enable						                                          			  *
**********************************************************************************************************/

void Clear_ADS1x9x_Chip_Enable (void)
{
  blocking_delay(10);	
  /* ADS1x9x CS is Active low*/
  PIN_setOutputValue(hecgPins, ADS_CSN , 1);
  //P8OUT |= (enum PORT8_ADC_CONTROL)ADC_CS;		// Set to High
}
/**********************************************************************************************************
* Init_ADS1x9x_DRDY_Interrupt												                                          *
**********************************************************************************************************/
void Init_ADS1x9x_DRDY_Interrupt (void)
{
  // Register ISR
  PIN_registerIntCb(hecgPins, drdy_isr);
  PIN_setConfig(hecgPins, PIN_BM_IRQ, ADS_DRDY | PIN_IRQ_DIS);
  
  //P1DIR &= ~0x02;
  //P1REN |= BIT1;                            	// Enable P1.1 internal resistance
  //P1OUT |= BIT1;                            	// Set P1.1 as pull-Up resistance
  //P1IES |= BIT1;                           		// P1.1 Lo/Hi edge
  //P1IFG &= ~BIT1;                           	// P1.1 IFG cleared
  //P1IE &= ~BIT1;                             	// P1.1 interrupt disabled
  
}
/**********************************************************************************************************
* Enable_ADS1x9x_DRDY_Interrupt												                                          *
**********************************************************************************************************/
void Enable_ADS1x9x_DRDY_Interrupt (void)
{
  PIN_setConfig(hecgPins, PIN_BM_IRQ, ADS_DRDY | PIN_IRQ_NEGEDGE);
  //P1IFG &= ~BIT1;                           	// P1.1 IFG cleared
  //P1IE |= BIT1;                             	// P1.1 interrupt enabled
}
/**********************************************************************************************************
* Disable_ADS1x9x_DRDY_Interrupt												                                          *
**********************************************************************************************************/
void Disable_ADS1x9x_DRDY_Interrupt (void)
{
  PIN_setConfig(hecgPins, PIN_BM_IRQ, ADS_DRDY | PIN_IRQ_DIS);
  //P1IFG &= ~BIT1;                           	// P1.1 IFG cleared
  //P1IE &= ~BIT1;                             	// P1.1 interrupt disabled
}

/**********************************************************************************************************
* Set_GPIO														                                          *
**********************************************************************************************************/
void Set_GPIO(void)
{
  // Open pin structure for use
  hecgPins = PIN_open(&ecgPins, ecgPinsCfg);
  PIN_setOutputValue(hecgPins, ADS_CSN , 1);
  PIN_setOutputValue(hecgPins, ADS_START , 0);
  PIN_setOutputValue(hecgPins, ADS_RESET , 0);
  // Register ISR
  //PIN_registerIntCb(hecgPins, buttonHwiFxn);
  // Configure interrupt
  //PIN_setConfig(hSbpPins, PIN_BM_IRQ, Board_KEY_UP | PIN_IRQ_NEGEDGE);
  // Enable wakeup
  //PIN_setConfig(hSbpPins, PINCC26XX_BM_WAKEUP, Board_KEY_UP|PINCC26XX_WAKEUP_NEGEDGE);  
  
  /*
  P2SEL = 0x00;                            
  P2DIR |= 0x8F;
  P2OUT |= (enum PORT2_ADC_CONTROL)POW_CE;                            
  P8DIR |= 0x07;       
  P8OUT &= 0xF8;
  P8OUT |= (enum PORT8_ADC_CONTROL)ADC_CS;		// Set RESET, START to Low and CS to High
  P8OUT |= (enum PORT8_ADC_CONTROL)ADC_RESET;	// Set RESET, START to Low and CS to High
  P2OUT = 0x03;
  */
  //dataCnt = 0;
  
}  

/**********************************************************************************************************
* Set_UCB0_SPI													                                          *
**********************************************************************************************************/
void Set_UCB0_SPI(void)
{
  bspSpiOpen();
  /*
  P3SEL |= BIT2+BIT1+BIT0;  				// Set SPI peripheral bits
  P3DIR |= BIT0+BIT2;						// Clock and DOUT as output
  P3DIR &= ~BIT1;                         	// Din as input 
  UCB0CTL1 |= UCSWRST;               		// Enable SW reset
  UCB0CTL0 |= UCMSB+UCMST+UCSYNC;			//[b0]   1 -  Synchronous mode 
  //[b2-1] 00-  3-pin SPI
  //[b3]   1 -  Master mode
  //[b4]   0 - 8-bit data
  //[b5]   1 - MSB first
  //[b6]   0 - Clock polarity low.
  //[b7]   1 - Clock phase - Data is captured on the first UCLK edge and changed on the following edge.
  
  UCB0CTL1 |= UCSSEL__ACLK;               	// ACLK
  UCB0BR0 = 24;                             // 1 MHz
  UCB0BR1 = 0;                              //
  UCB0CTL1 &= ~UCSWRST;              		// Clear SW reset, resume operation
  */
}  

/**********************************************************************************************************
* ADS1x9x_SPI_Command_Data						                                          *
**********************************************************************************************************/
void ADS1x9x_SPI_Command_Data(unsigned char Data)
{
  Set_ADS1x9x_Chip_Enable();	
  Clear_ADS1x9x_Chip_Enable();
  Set_ADS1x9x_Chip_Enable();
  
  bspSpiWrite(&Data, 1);      // Send the data sitting at the pointer DATA to the TX Buffer
  
  blocking_delay(10);	
  
}

/**********************************************************************************************************
* Init_ADS1x9x_Resource						                                          *
**********************************************************************************************************/

void Init_ADS1x9x_Resource(void)
{
  Set_GPIO();										// Initializes ADS1x9x's input control lines
  Set_UCB0_SPI();									// Initialize SPI regs.
  //Set_DMA_SPI();	   								// Initialize DMA regs for SPI.
  
}
/**********************************************************************************************************
*	        ADS1x9x Control Registers      				                                  *
**********************************************************************************************************/
/**********************************************************************************************************
* Wake_Up_ADS1x9x						                                          						  *
**********************************************************************************************************/
void Wake_Up_ADS1x9x (void)
{ 
  ADS1x9x_SPI_Command_Data (WAKEUP);                   // Send 0x02 to the ADS1x9x                                                      
}

/**********************************************************************************************************
* Put_ADS1x9x_In_Sleep						                                          					  *
**********************************************************************************************************/
void Put_ADS1x9x_In_Sleep (void)
{
  ADS1x9x_SPI_Command_Data (STANDBY);                 // Send 0x04 to the ADS1x9x
}
/**********************************************************************************************************
* Soft_Reset_ADS1x9x					                                          						  *
**********************************************************************************************************/

void Soft_Reset_ADS1x9x (void)
{
  ADS1x9x_SPI_Command_Data (RESET);                   // Send 0x06 to the ADS1x9x
}
/**********************************************************************************************************
* Soft_Start_ReStart_ADS1x9x			                                          						  *
**********************************************************************************************************/

void Soft_Start_ReStart_ADS1x9x (void)
{
  ADS1x9x_SPI_Command_Data (START);                  // Send 0x08 to the ADS1x9x
  Clear_ADS1x9x_Chip_Enable ();                                                       
}
/**********************************************************************************************************
* Hard_Start_ReStart_ADS1x9x			                                          						  *
**********************************************************************************************************/

void Hard_Start_ReStart_ADS1x9x(void)
{
  PIN_setOutputValue(hecgPins, ADS_START , 1);
  //P8OUT |= (enum PORT8_ADC_CONTROL)ADC_START;			// Set Start pin to High
}

/**********************************************************************************************************
* Soft_Start_ADS1x9x					                                          						  *
**********************************************************************************************************/

void Soft_Start_ADS1x9x (void)
{
  ADS1x9x_SPI_Command_Data (START);                   // Send 0x0A to the ADS1x9x
}

/**********************************************************************************************************
* Soft_Stop_ADS1x9x					                                          						  *
**********************************************************************************************************/

void Soft_Stop_ADS1x9x (void)
{
  ADS1x9x_SPI_Command_Data (STOP);                   // Send 0x0A to the ADS1x9x
}

/**********************************************************************************************************
* Hard_Stop_ADS1x9x					                                          						  *
**********************************************************************************************************/

void Hard_Stop_ADS1x9x (void)
{
  PIN_setOutputValue(hecgPins, ADS_START , 0);
  //P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_START;		// Set Start pin to Low
  Task_sleep(1);
}

/**********************************************************************************************************
* Soft_Start_ReStart_ADS1x9x			                                          						  *
**********************************************************************************************************/

void Stop_Read_Data_Continuous (void)
{
  ADS1x9x_SPI_Command_Data(SDATAC);					// Send 0x11 to the ADS1x9x
}

/**********************************************************************************************************
* Start_Read_Data_Continuous			                                          						  *
**********************************************************************************************************/

void Start_Read_Data_Continuous (void)
{
  ADS1x9x_SPI_Command_Data (RDATAC);					// Send 0x10 to the ADS1x9x
}


/**********************************************************************************************************
* Start_Data_Conv_Command			                                          						  *
**********************************************************************************************************/

void Start_Data_Conv_Command (void)
{
  ADS1x9x_SPI_Command_Data (START);					// Send 0x08 to the ADS1x9x
}

/**********************************************************************************************************
* Initialize ADS1x9x						                                          *
**********************************************************************************************************/
void Init_ADS1x9x (void)
{
  ADS1x9x_Reset();
  ADS1x9x_Disable_Start();
  ADS1x9x_Enable_Start();
}
/*********************************************************************************************************/

/**********************************************************************************************************
* enable_ADS1x9x_Conversion													                          *
**********************************************************************************************************/
void enable_ADS1x9x_Conversion (void)
{
  Start_Read_Data_Continuous ();		//RDATAC command
  
  Hard_Start_ReStart_ADS1x9x();
  
}
/*********************************************************************************************************/

/*********************************************************************************************************
* ADS1x9x_Reg_Write																	                 *
**********************************************************************************************************/

void ADS1x9x_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{ 
  
  switch (READ_WRITE_ADDRESS)
  {
  case 1:
    DATA = DATA & 0x87;
    break;
  case 2:
    DATA = DATA & 0xFB;
    DATA |= 0x80;
    
    break;
  case 3:
    DATA = DATA & 0xFD;
    DATA |= 0x10;
    
    break;
  case 7:
    DATA = DATA & 0x3F;
    break;
  case 8:
    DATA = DATA & 0x5F;
    break;
  case 9:
    DATA |= 0x02;
    break;
  case 10:
    DATA = DATA & 0x87;
    DATA |= 0x01;
    break;
  case 11:
    DATA = DATA & 0x0F;
    break;
    
  default:
    
    break;
    
  }
  SPI_Tx_buf[0] = READ_WRITE_ADDRESS | WREG;
  SPI_Tx_buf[1] = 0;						// Write Single byte
  SPI_Tx_buf[2] = DATA;					// Write Single byte
  Set_ADS1x9x_Chip_Enable();
  
  blocking_delay(10);	
  
  bspSpiWrite(SPI_Tx_buf, 1);
  bspSpiWrite(SPI_Tx_buf + 1, 1);
  bspSpiWrite(SPI_Tx_buf + 2, 1);
  /*
  UCB0TXBUF = SPI_Tx_buf[0];              // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );			// USCI_B0 TX buffer ready?
  i = UCB0RXBUF;							// Read Rx buf
  
  UCB0TXBUF = SPI_Tx_buf[1];              // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );			// USCI_B0 TX buffer ready?
  i = UCB0RXBUF;
  UCB0TXBUF = SPI_Tx_buf[2];              // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );			// USCI_B0 TX buffer ready?
  i = UCB0RXBUF;
  */
}
/*********************************************************************************************************
* ADS1x9x_Reg_Read																	                 *
**********************************************************************************************************/
unsigned char ADS1x9x_Reg_Read(unsigned char Reg_address)
{
  unsigned char retVal;

  SPI_Tx_buf[0] = Reg_address | RREG;
  SPI_Tx_buf[1] = 0;							// Read number of bytes - 1
  SPI_Tx_buf[2] = 0;	
  
  Set_ADS1x9x_Chip_Enable();					// Set chip select to low
  
  bspSpiWrite(SPI_Tx_buf, 1);
  bspSpiWrite(SPI_Tx_buf + 1, 1);
  bspSpiRead(SPI_Tx_buf, 1);
  
  //bspSpiWriteRead(SPI_Tx_buf, 2, 1);
  
  retVal = SPI_Tx_buf[0];  
  /* 
  UCB0TXBUF = SPI_Tx_buf[0];                  // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );				// USCI_B0 TX buffer ready?
  UCB0TXBUF = SPI_Tx_buf[1];                  // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );				// USCI_B0 TX buffer ready?
  retVal = UCB0RXBUF;							// Read RX buff
  UCB0TXBUF = 0x00;                           // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );				// USCI_B0 TX buffer ready?
  retVal = UCB0RXBUF;							// Read RX buff
  */
  
  Clear_ADS1x9x_Chip_Enable();				// Disable chip select
  return 	retVal;
}
/**********************************************************************************************************
*	        ADS1x9x default Initialization          				                  					  *
**********************************************************************************************************/

void ADS1x9x_Default_Reg_Init(void)
{
  
  unsigned char Reg_Init_i;
  Set_ADS1x9x_Chip_Enable();
  blocking_delay(10);	
  Clear_ADS1x9x_Chip_Enable();
  
  if ((ADS1x9xRegVal[0] & 0X20) == 0x20)
  {
    for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
    {
      ADS1x9x_Reg_Write(Reg_Init_i,ADS1x9xR_Default_Register_Settings[Reg_Init_i]);
    }
  }
  else
  {
    for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
    {
      ADS1x9x_Reg_Write(Reg_Init_i,ADS1x9x_Default_Register_Settings[Reg_Init_i]);
    }
  }
  
}

/**********************************************************************************************************
*	        ADS1x9x_Read_All_Regs          				                  					  *
**********************************************************************************************************/

void ADS1x9x_Read_All_Regs(unsigned char ADS1x9xeg_buf[])
{
  unsigned char Regs_i;
  Set_ADS1x9x_Chip_Enable();
  blocking_delay(10);	
  Clear_ADS1x9x_Chip_Enable();
  
  for ( Regs_i = 0; Regs_i < 12; Regs_i++)
  {
    ADS1x9xeg_buf[Regs_i] = ADS1x9x_Reg_Read(Regs_i);
    
  }
  
}

#define max(a,b) ( ((a)>(b)) ? (a):(b) )
#define min(a,b) ( ((a)>(b)) ? (b):(a) )

int maxval = 0;
int minval = 0;


/*********************************************************************************************************
**********************************************************************************************************/
void ADS1191_Parse_data_packet(void)
{
  unsigned char ECG_Chan_num;
  
  for (ECG_Chan_num = 0; ECG_Chan_num < 2; ECG_Chan_num++)
  {
    ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[2*ECG_Chan_num]; 	// Get MSB 8 bits 
    ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
    ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[2*ECG_Chan_num+1];				// Get LSB 8 bits
  }
  fifo8_put(&ecg_data, ADS1x9x_ECG_Data_buf[1]); 
  maxval = max(maxval, (int16_t)ADS1x9x_ECG_Data_buf[1]);
  minval = min(minval, (int16_t)ADS1x9x_ECG_Data_buf[1]);
  if((int16_t)ADS1x9x_ECG_Data_buf[1] > 100) {
    ADS1x9x_ECG_Data_buf[0] = 0;
    blocking_delay(1);
  }
      
  /*    
  switch (ECG_Recoder_state.state)
  {
    
  case IDLE_STATE:
    break;
  case DATA_STREAMING_STATE:
    {
      for (ECG_Chan_num = 0; ECG_Chan_num < 2; ECG_Chan_num++)
      {
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[2*ECG_Chan_num]; 	// Get MSB 8 bits 
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[2*ECG_Chan_num+1];				// Get LSB 8 bits
      }
      
      ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] << 8;								// to make compatable with 24 bit devices
    }
    break;
    
    
  case ACQUIRE_DATA_STATE:
    
  case ECG_RECORDING_STATE:
    {
      unsigned char *ptr;
      ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;
      *ptr++ = SPI_Rx_buf[0];				// Store status 
      *ptr++ = SPI_Rx_buf[1];				// Store status 
      //			if ((SPI_Rx_buf[2] & 0x80 ) == 0x80)// CH0[15-8] = MSB ( 16Bit device)
      //			*ptr++ = 0xFF;						// CH0[23-16] = 0xFF ( 16Bit device) sign
      //			else
      //			*ptr++ = 0;							// CH0[23-16] = 0 ( 16Bit device)
      *ptr++ = SPI_Rx_buf[2];				// CH0[15-8] = MSB ( 16Bit device)
      *ptr++ = SPI_Rx_buf[3];				// CH0[7-0] = LSB ( 16Bit device)
      *ptr++ = 0;
      //			if ((SPI_Rx_buf[2] & 0x80 ) == 0x80)// CH0[15-8] = MSB ( 16Bit device)
      //			*ptr++ = 0xFF;						// CH0[23-16] = 0xFF ( 16Bit device) sign
      //			else
      //			*ptr++ = 0;							// CH0[23-16] = 0 ( 16Bit device)
      *ptr++ = SPI_Rx_buf[2];				// CH0[15-8] = MSB ( 16Bit device)
      *ptr++ = SPI_Rx_buf[3];				// CH0[7-0] = LSB ( 16Bit device)
      *ptr++ = 0;
      Recorder_head ++;					// Increment Circuler buffer pointer
      
      if (Recorder_head == 32)			// Check for circuler buffer depth.
        Recorder_head = 0;				// Rest once it reach to MAX
    }
    break;
    
  default:
    break;
    
  }
  */
}
/*********************************************************************************************************
**********************************************************************************************************/
void ADS1192_Parse_data_packet(void)
{
  unsigned char ECG_Chan_num;
  
  for (ECG_Chan_num = 0; ECG_Chan_num < 3; ECG_Chan_num++)
  {
    ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[2*ECG_Chan_num];	// Get MSB Bits15-bits8
    ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
    ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[2*ECG_Chan_num+1];				// Get LSB Bits7-bits0
  }
  ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] << 8;				// to make compatable with 24 bit devices  
  
  /*    
  switch (ECG_Recoder_state.state)
  {
  
case IDLE_STATE:
    break;
  case DATA_STREAMING_STATE:
    {
      for (ECG_Chan_num = 0; ECG_Chan_num < 3; ECG_Chan_num++)
      {
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[2*ECG_Chan_num];	// Get MSB Bits15-bits8
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[2*ECG_Chan_num+1];				// Get LSB Bits7-bits0
      }
      ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] << 8;				// to make compatable with 24 bit devices
    }
    break;
    
    
  case ACQUIRE_DATA_STATE:
    
  case ECG_RECORDING_STATE:
    {
      unsigned char *ptr;
      
      ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;
      *ptr++ = SPI_Rx_buf[0];				// Store status 
      *ptr++ = SPI_Rx_buf[1];				// Store status 
      
      //			if ((SPI_Rx_buf[2] & 0x80 ) == 0x80)// CH0[15-8] = MSB ( 16Bit device)
      //			*ptr++ = 0xFF;						// CH0[23-16] = 0xFF ( 16Bit device) sign
      //			else
      //			*ptr++ = 0;							// CH0[23-16] = 0 ( 16Bit device)
      *ptr++ = SPI_Rx_buf[2];				// CH0[15-8] = MSB ( 16Bit device)
      *ptr++ = SPI_Rx_buf[3];				// CH0[7-0] = LSB ( 16Bit device)
      *ptr++ = 0;
      
      //			if ((SPI_Rx_buf[4] & 0x80 ) == 0x80)// CH1[15-8] = MSB ( 16Bit device)
      //			*ptr++ = 0xFF;						// CH1[23-16] = 0xFF ( 16Bit device) sign
      //			else
      //			*ptr++ = 0;							// CH1[23-16] = 0 ( 16Bit device)
      *ptr++ = SPI_Rx_buf[4];				// CH1[15-8] = MSB ( 16Bit device)
      *ptr++ = SPI_Rx_buf[5];				// CH1[7-0] = LSB ( 16Bit device)
      *ptr++ = 0;
      Recorder_head ++;					// Increment Circuler buffer pointer
      
      if (Recorder_head == 32)			// Check for circuler buffer depth.
        Recorder_head = 0;				// Rest once it reach to MAX
    }
    break;
    
  default:
    break;
    
  } */
  
}
/*********************************************************************************************************
**********************************************************************************************************/
void ADS1291_Parse_data_packet(void)
{
  unsigned char ECG_Chan_num;
  
  switch (ECG_Recoder_state.state)
  {		
  case DATA_STREAMING_STATE:
    {
      for (ECG_Chan_num = 0; ECG_Chan_num < 2; ECG_Chan_num++)
      {
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[3*ECG_Chan_num];	// Get Bits23-bits16
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+1];				// Get Bits15-bits8
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+2];				// Get Bits7-bits0
      }
    }
    break;
    
    
  case ACQUIRE_DATA_STATE:
    
  case ECG_RECORDING_STATE:
    {
      unsigned char *ptr;
      
      ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;
      
      *ptr++ = SPI_Rx_buf[0];				// Store status 
      *ptr++ = SPI_Rx_buf[1];				// Store status 
      //SPI_Rx_buf[2] is always 0x00 so it is discarded
      
      *ptr++ = SPI_Rx_buf[3];				// CH0[23-16] = MSB ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[4];				// CH0[15-8] = MID ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[5];				// CH0[7-0] = LSB ( 24 Bit device)
      
      *ptr++ = SPI_Rx_buf[3];				// CH1[23-16] = Ch0 to mentain uniformality
      *ptr++ = SPI_Rx_buf[4];				// CH1[15-8] =  Ch0 to mentain uniformality
      *ptr++ = SPI_Rx_buf[5];				// CH1[7-0] =  Ch0 to mentain uniformality
      
      Recorder_head++;					// Increment Circuler buffer pointer
      if (Recorder_head == 32)			// Check for circuler buffer depth.
        Recorder_head = 0;				// Rest once it reach to MAX
    }
    
    break;
    
  default:
    break;
    
  }
  
}
/*********************************************************************************************************
**********************************************************************************************************/
void ADS1292x_Parse_data_packet(void)
{
  unsigned char ECG_Chan_num;
  switch (ECG_Recoder_state.state)
  {		
  case DATA_STREAMING_STATE:
    {
      for (ECG_Chan_num = 0; ECG_Chan_num < 3; ECG_Chan_num++)
      {
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[3*ECG_Chan_num];
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+1];
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+2];
      }
    }
    break;
    
    
  case ACQUIRE_DATA_STATE:
    
  case ECG_RECORDING_STATE:
    {
      unsigned char *ptr;
      
      ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;
      *ptr++ = SPI_Rx_buf[0];				// Store status 
      *ptr++ = SPI_Rx_buf[1];				// Store status 
      //SPI_Rx_buf[2] is always 0x00 so it is discarded
      
      *ptr++ = SPI_Rx_buf[3];				// CH0[23-16] = MSB ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[4];				// CH0[15-8] = MID ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[5];				// CH0[7-0] = LSB ( 24 Bit device)
      
      *ptr++ = SPI_Rx_buf[6];				// CH1[23-16] = MSB ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[7];				// CH1[15-8] = MID ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[8];				// CH1[7-0] = LSB ( 24 Bit device)
      
      Recorder_head ++;					// Increment Circuler buffer pointer
      if (Recorder_head == 32)			// Check for circuler buffer depth.
        Recorder_head = 0;				// Rest once it reach to MAX
      
    }
    
    break;
    
  default:
    break;
  }
}

/*********************************************************************************************************
**********************************************************************************************************/

void ADS1x9x_Parse_data_packet(void)
{
  
  switch( ADS1x9xRegVal[0] & 0x03)
  {
  case ADS1191_16BIT:
    {
      ADS1191_Parse_data_packet();
    }		
    
    break;
    
  case ADS1192_16BIT:
    {
      ADS1192_Parse_data_packet();
    }		
    
    break;
    
  case ADS1291_24BIT:
    {
      ADS1291_Parse_data_packet();
    }			
    
    break;
    
  case ADS1292_24BIT:
    {
      ADS1292x_Parse_data_packet();
      
    }			
    break;
  }
  ECG_Data_rdy = 1;
  //SPI_Rx_exp_Count = 1;
  
}

/*********************************************************************************************************/
/*********************************************************************************************************/
/**********************************************************************************************************
*	        ADS1x9x_PowerOn_Init          				                  					  			  *
***********************************************************************************************************/
void ADS1x9x_PowerOn_Init(void)
{
  Init_ADS1x9x_Resource();
  
  ADS1x9x_Reset();
  
  Task_sleep(5);
  
  Init_ADS1x9x_DRDY_Interrupt();

  ADS1x9x_Clock_Select(1);		// Set internal clock
  Task_sleep(5);
  ADS1x9x_Disable_Start();
  ADS1x9x_Enable_Start();
  
  Hard_Stop_ADS1x9x();
  
  
  Start_Data_Conv_Command();
  Soft_Stop_ADS1x9x();
  Task_sleep(5);
  Stop_Read_Data_Continuous();					// SDATAC command
  Task_sleep(5);

  //enter_continuous();
 
  ADS1x9x_Read_All_Regs(ADS1x9xRegVal);
  ADS1x9x_Default_Reg_Init();
  ADS1x9x_Read_All_Regs(ADS1x9xRegVal);
  Set_Device_out_bytes();
  
}

void enter_continuous()
{
  ADS1x9x_Disable_Start();
  ADS1x9x_Enable_Start();
  
  ADS1x9x_Disable_Start();					// Disable START (SET START to high)
  
  Set_ADS1x9x_Chip_Enable();					// CS = 0
  blocking_delay(5);
  Clear_ADS1x9x_Chip_Enable();				// CS = 1
  
  blocking_delay(5);
  
  Set_ADS1x9x_Chip_Enable();				// CS =0
  blocking_delay(5);				    
  Start_Read_Data_Continuous();			//RDATAC command
  blocking_delay(5);
  Enable_ADS1x9x_DRDY_Interrupt();		// Enable DRDY interrupt
  ADS1x9x_Enable_Start();				// Enable START (SET START to high)  
}

void enter_nothing()
{
  Disable_ADS1x9x_DRDY_Interrupt();		// Disable DRDY interrupt
  Stop_Read_Data_Continuous();			// SDATAC command  
  ADS1x9x_Disable_Start();
}

/**********************************************************************************************************/
// Echo character
void receive_entry(void)
{
  
  //while (!(UCB0IFG&UCTXIFG));             // USCI_B0 TX buffer ready?
  //while(SPI_Rx_Count != SPI_Rx_exp_Count) {
  bspSpiRead(SPI_Rx_buf, 1); 
  bspSpiRead(SPI_Rx_buf + 1, 1); 
  bspSpiRead(SPI_Rx_buf + 2, 1);
  bspSpiRead(SPI_Rx_buf + 3, 1);
    //SPI_Rx_Count++;
  //}
  ADS1x9x_Parse_data_packet();
  
  
  /*
  switch(__even_in_range(UCB0IV,4))
  {
case 0:break;                             // Vector 0 - no interrupt
case 2:                                   // Vector 2 - RXIFG
  
  //while (!(UCB0IFG&UCTXIFG));             // USCI_B0 TX buffer ready?
  
  SPI_Rx_buf[SPI_Rx_Count] = UCB0RXBUF;
  SPI_Rx_Count++;
  if ( SPI_Rx_Count == SPI_Rx_exp_Count)
  {
  UCB0IE &= ~UCRXIE;                 // Disable USCI_B0 RX interrupt
  ADS1x9x_Parse_data_packet();
}
               else 
  {
  
  UCB0TXBUF = 0; 					// To get Next byte.
}
  break;
case 4:break;                             // Vector 4 - TXIFG
  
  default: break;
}
  */
}

// Port 1 interrupt service routine
void drdy_isr(PIN_Handle handle, PIN_Id pinId)
{
  extern Semaphore_Handle appSem;
  SPI_Rx_Count=0;
  ecg_StartAD(ECG_AD_EVT);
  
  /*
  if ( P1IFG &= BIT1)
  {
  P1IFG &= ~BIT1;                 // Clear P1.1 IFG i.e Data RDY interrupt status
  SPI_Rx_Count = UCB0RXBUF; 		// Dummy Read
  SPI_Rx_Count=0;
  
  UCB0TXBUF = 0;
  UCB0IE |= UCRXIE;               // Enable USCI_B0 RX interrupt
}
  */
}
/**********************************************************************************************************
* Set_Device_out_bytes						                                          					  *
* 		: Selects number of sample to be recieved from device based device ID							
**********************************************************************************************************/

void Set_Device_out_bytes(void)
{
  switch( ADS1x9xRegVal[0] & 0x03)
  {
    
  case ADS1191_16BIT:
    SPI_Rx_exp_Count=4;		// 2 byte status + 2 bytes CH0 data
    
    break;
    
  case ADS1192_16BIT:	
    SPI_Rx_exp_Count=6;		// 2 byte status + 2 bytes ch1 data + 2 bytes CH0 data
    
    break;
    
  case ADS1291_24BIT:
    SPI_Rx_exp_Count=6;		// 3 byte status + 3 bytes CH0 data
    
    break;
    
  case ADS1292_24BIT:
    SPI_Rx_exp_Count=9;		// 3 byte status + 3 bytes ch1 data + 3 bytes CH0 data
    
    break;
    
  }
}

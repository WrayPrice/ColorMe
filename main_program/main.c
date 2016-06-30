/* 
 * File:   main.c
 * Author: nathen
 *
 * Created on June 21, 2016, 7:26 PM
 */

/* Standard includes. */
#include <stdio.h>


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "p24FJ128GA010.h"
#include "spi.h"

/*---------------------------------------------------------------------------------------------*/
//Macros

#define NumOfLeds 1  //done for simplicity
#define StartVal 255 //set everything to high for simplicity
#define T0H 3        //represents time high for a code 0
#define T0L 9        // represents time low for a code 0
#define T1H 6        // represents time high for a code 1
#define T1L 6        // represents time low for a code 1
#define reset 80     //time for reset
#define MYSPI _LATA0 // represents the pin chosen for the SPI bus (SDO1)
#define IOMYSPI _TRISA0 //represents the I/0 register for the spi bus-neede to set pin as an output by writing a 0


#include <xc.h>

// CONFIG2
#pragma config POSCMOD = XT             // Primary Oscillator Select (XT Oscillator mode selected)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSC2/CLKO/RC15 functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Clock switching and Fail-Safe Clock Monitor are disabled)
#pragma config FNOSC = FRCPLL           // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))
#pragma config IESO = OFF               // Internal External Switch Over Mode (IESO mode (Two-Speed Start-up) disabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = ON              // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx2               // Comm Channel Select (Emulator/debugger uses EMUC2/EMUD2)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)


 
 
static void prvSetupHardware( void );

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );

/*-----------------------------------------------------------*/
//Prototypes

void TestTask(void *p);
void OutputTask(void *p);

/*-----------------------------------------------------------*/
//Strucs

struct LEDParam
{
	int Green;
	int Red;
	int Blue;
}; 


/*------------------------------------------------------------*/
xQueueHandle Global_Queue_Handle = 0;
/*
 * Create the demo tasks then start the scheduler.
 */
int main( void )
{
    /*********************SPI Setup****************************************/
    
    IFS0bits.SPI1IF = 0; // Clear the Interrupt flag-from SPI datasheet pg. 15
    IEC0bits.SPI1IE = 0; // Disable the interrupt-from SPI datasheet pg. 15

    
    //SPIxSTAT:
    SPI1STATbits.SPIEN=0;  //disabling the SPI Bus (bit 15)
    // bit 14 = unimplemented
    SPI1STATbits.SPISIDL = 0; //set Continues module operation in Idle mode (bit 13)
    //bits 12-11 = unimplemented
    SPI1STATbits.SPIBEC = 0; //setting Number of SPI transfers pending to 0 (bits 10-8)
    SPI1STATbits.SRMPT = 0; //Shift register is not empty; read as ?0? (in enhancement mode) (bit 7)
    SPI1STATbits.SPIROV = 0; //overflow bit flag 3; 0 = no overflow has occurred (bit 6)
    SPI1STATbits.SRXMPT = 0; //FIFO EMPTY BIT FLAG; Receive FIFO is not empty? = 0; (bit 5)
    SPI1STATbits.SISEL = 0; //Buffer Interrupt Mode bits; 000= Interrupt 
                            //when the last data in the receive buffer is read, 
                            //and as a result, the buffer is empty (bits 4-2)
    SPI1STATbits.SPITBF = 0; //Transmit buffer; 0 = Transmit started, SPIxTXB is empty (bit 1)
    SPI1STATbits.SPIRBF = 0; //SPIx Receive Buffer Full Status bit; 0 = Receive is not complete, SPIxRXB is empty (bit 0)
    
    //SPIxCON1:
    //bits 15-13 = unimplemented
    SPI1CON1bits.DISSCK = 1; //Internal SPI clock is disabled, the pin functions as an I/O (bit 12)
    SPI1CON1bits.DISSDO = 0; //SDOx pin is controlled by the module (bit 11)
    //MODE16 not defined--want 0, default is 0 (bit 10)
    AD1CON2bits.SMPI = 0; // Data Input Sample Phase bit; 0 = Input data is sampled at the middle of data output time (bit 9)
    SPI1CON1bits.CKE = 0; //SPIx Clock Edge Select bit; Serial output data changes on transition from Idle clock state to active clock state (bit 8)
    SPI1CON1bits.SSEN = 0; //Slave Select Enable bit; SSx pin is not used by module; pin is controlled by port function (bit 7)
    SPI1CON1bits.CKP = 0; // Clock Polarity Select bit; 0 = Idle state for clock is a low level; active state is a high level (bit 6)
    SPI1CON1bits.MSTEN = 1; //Master Mode Enable bit; 1 = Master mode (bits 5)
    SPI1CON1bits.SPRE = 6; // Secondary Prescale bits (Master mode); 5 dec = 101 bin= Secondary prescale 4:1 (bits 4-2)
    SPI1CON1bits.PPRE = 3; // Primary Prescale bits (Master mode); 3 dec = 11 bin = Primary prescale 1:1 (bits 1-0)
    
    //SPIxCON2:
    SPI1CON2bits.FRMEN = 0; //Framed SPIx Support bit; 0 = Framed SPIx support is disabled (bit 15)
    SPI1CON2bits.SPIFSD = 0; //SPIx Frame Sync Pulse Direction Control bit; Frame sync pulse output (master) (bit 14)
    SPI1CON2bits.SPIFPOL = 1; //SPIx Frame Sync Pulse Polarity bit; 1 = Frame sync pulse is active-high (bit 13)
     //(bits 12-3 = unimplemented)
    SPI1CON2bits.SPIFE = 0; //SPIx Frame Sync Pulse Edge Select bit; 0 = Frame sync pulse precedes the first bit clock (bit 1)
    SPI1CON2bits.SPIBEN = 0; //Enhanced Buffer Enable bit; 0 = Enhanced buffer is disabled (Legacy mode) (bit 0)
    
    //OSCTUN:
    //bits 15-6 = unimplemented
    OSCTUNbits.TUN = 0x20; // 32= most Minimum frequency deviation *****may use this later****
    
   IFS0bits.SPI1IF = 0; // Clear the Interrupt flag-from SPI datasheet pg. 15
    
    //MAY NOT WANT TO ENABLE UNTIL PORT HAS BEEN SET UP FOR SPI
    SPI1STATbits.SPIEN = 1;  //Enabling the SPI Bus (bit 15)
    
   
     
	/*************************END SPI Bus config*********************************************/
    
    /* Configure any hardware required for this demo. */
    IOMYSPI=0;
	//prvSetupHardware();

    
    //Global_Queue_Handle = xQueueCreate(3,sizeof(int));
    

	/* Create the test tasks defined within this file. */
	xTaskCreate( TestTask, (signed char *) "output_task", 1024, NULL, 1, NULL );
	/* Finally start the scheduler. */
	vTaskStartScheduler();

	/* Will only reach here if there is insufficient heap available to start
	the scheduler. */
	return 0;
}
/*-----------------------------------------------------------*/
void TestTask(void *p)
{   
    while (1)
    {
        int i;
        //reset signal
        for (i=0;i<36;i++)
        {
            while(SPI1STATbits.SPITBF == 1);
            SPI1BUF=0x00;
        }
        
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0xCC;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0xCC;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0xCC;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0xCC;
        
        
        vTaskDelay(100);
        
                for (i=0;i<36;i++)
        {
            while(SPI1STATbits.SPITBF == 1);
            SPI1BUF=0x00;
        }
        
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0xCC;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0xCC;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0xCC;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0xCC;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF=0x88;
        
        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}

void OutputTask(void *p)
{
 
    #include <stdio.h>

#define NumOfLeds 1  //done for simplicity
#define reset 80     //time for reset

typedef struct
{
	//int green;
	//int red;
	//int blue;
	int color[3]; //green, red, blue
}Lednum;


int main ()
{
	Lednum lednum[NumOfLeds]={0};
    int BitEnc=0;
	int LedIter, ColorIter, BitIter;
	int counter=0;
	//
	lednum[0].color[0]=200; //green
	lednum[0].color[1]=1; //red
	lednum[0].color[2]=2; //blue
	for(LedIter=0;LedIter<NumOfLeds;LedIter++)
	{
		for (ColorIter=0;ColorIter<3;ColorIter++)
		{
			 for (BitIter=0;BitIter<4;BitIter++)
			 {
				 BitEnc = lednum[LedIter].color[ColorIter] >> (6-2*BitIter) & 0x03;
				 printf("BitEnc is: %d\n",BitEnc);
				 counter++;
				 switch (BitEnc)
				 {
					 case 0:
						 printf("Outut 10001000\n");
						 break;
					 case 1:
						 printf("Outut 10001100\n");
						 break;
					 case 2:
						 printf("Outut 11001000\n");
						 break;
					 case 3:
						 printf("Outut 11001100\n");
						 break;
				 }
			}
		}
	}
	printf("\nCounter is: %d\n", counter);
	return 0;
}

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    vTaskDelete(NULL);
}


static void prvSetupHardware( void )
{
	TRISA = 0;
	PORTA = 0;
}
/*-----------------------------------------------------------*/


void vApplicationIdleHook( void )
{
	/* Schedule the co-routines from within the idle task hook. */
	
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}


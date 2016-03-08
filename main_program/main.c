/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the 
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt 
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt 
 * timing.  The maximum measured jitter time is latched in the usMaxJitter 
 * variable, and displayed on the LCD by the 'Check' as described below.  
 * The fast interrupt is configured and handled in the timer_test.c source 
 * file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the LCD directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of 
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting 
 * for messages - waking and displaying the messages as they arrive.  The LCD
 * task is defined in lcd.c.  
 * 
 * "Check" task -  This only executes every three seconds but has the highest 
 * priority so is guaranteed to get processor time.  Its main function is to 
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write "FAIL #n" to the LCD (via the LCD task).  If all the demo tasks are 
 * executing with their expected behaviour then the check task writes the max
 * jitter time to the LCD (again via the LCD task), as described above.
 */

/* Standard includes. */
#include <stdio.h>


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "p24FJ128GA010.h"

/*---------------------------------------------------------------------------------------------*/
//Macros

#define NumOfLeds 1  //done for simplicity
#define StartVal 255 //set everything to high for simplicity
#define T0H 3        //represents time high for a code 0
#define T0L 9        // represents time low for a code 0
#define T1H 6        // represents time high for a code 1
#define T1L 6        // represents time low for a code 1
#define reset 80     //time for reset


/*---------------------------------------------------------------------------------------------*/
 int CONFIG2 __attribute__((space(prog), address(0x157FC))) = 0x7BFD ;
//_CONFIG2(
//    POSCMOD_XT &         // Primary Oscillator Select (XT Oscillator mode selected)
//    OSCIOFNC_OFF &       // Primary Oscillator Output Function (OSC2/CLKO/RC15 functions as CLKO (FOSC/2))
//    FCKSM_CSDCMD &       // Clock Switching and Monitor (Clock switching and Fail-Safe Clock Monitor are disabled)
//    FNOSC_PRIPLL &       // Oscillator Select (Primary Oscillator with PLL module (HSPLL, ECPLL))
//    IESO_OFF             // Internal External Switch Over Mode (IESO mode (Two-Speed Start-up) disabled)
//);
 int CONFIG1 __attribute__((space(prog), address(0x157FE))) = 0x3F7F ;
//_CONFIG1(
//    WDTPS_PS32768 &      // Watchdog Timer Postscaler (1:32,768)
//    FWPSA_PR128 &        // WDT Prescaler (Prescaler ratio of 1:128)
//    WINDIS_ON &          // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
//    FWDTEN_OFF &         // Watchdog Timer Enable (Watchdog Timer is disabled)
//    ICS_PGx2 &           // Comm Channel Select (Emulator/debugger uses EMUC2/EMUD2)
//    GWRP_OFF &           // General Code Segment Write Protect (Writes to program memory are allowed)
//    GCP_OFF &            // General Code Segment Code Protect (Code protection is disabled)
//    JTAGEN_OFF           // JTAG Port Enable (JTAG port is disabled)
//);
/*-----------------------------------------------------------*/

/*
 * Setup the processor ready for the demo.
 */
static void prvSetupHardware( void );

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );

/*-----------------------------------------------------------*/
//Prototypes

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
	/* Configure any hardware required for this demo. */
    _TRISA0=0;
	prvSetupHardware();
    
    Global_Queue_Handle = xQueueCreate(3,sizeof(int));
    

	/* Create the test tasks defined within this file. */
	xTaskCreate( OutputTask, (signed char *) "output_task", 1024, NULL, 1, NULL );
	/* Finally start the scheduler. */
	vTaskStartScheduler();

	/* Will only reach here if there is insufficient heap available to start
	the scheduler. */
	return 0;
}
/*-----------------------------------------------------------*/
void OutputTask(void *p)
{
 
    struct LEDParam LedNum[NumOfLeds];
	int LedIter;
	int MaskingIter;
	LedNum[0].Green = StartVal-125;  //some value for green
	LedNum[0].Red = StartVal-180;    //some value for red
	LedNum[0].Blue = StartVal-200;   //some value for blue
    
    
    while(1)
    {
     
        for (LedIter=0; LedIter < NumOfLeds; LedIter++)
        {  //begin Led iteration loop
            for (MaskingIter=0; MaskingIter < 8; MaskingIter++)
            {
              //begin masking iteration loop for Green
                if ((0x80 >> MaskingIter) & LedNum[0].Green)  
				//if statement says: if a 0 is found in temp; produce pulse for a 0,
				//otherwise produce a pulse for a 1
                {
				_LATA0=1;  //use ra0  port = in; lat = out; tris = tristate
				vTaskDelay (T0H);
				_LATA0=0;
				vTaskDelay (T0L);
                }
			else
                {
				_LATA0=1;
				vTaskDelay (T1H);
				_LATA0=0;
				vTaskDelay (T1L);
                }
            }
            //begin iteration for RED
            for (MaskingIter=0; MaskingIter < 8; MaskingIter++)
            {
			//begin masking iteration loop for Red
                if ((0x80 >> MaskingIter) & LedNum[0].Red)  
				//if statement says: if a 0 is found in temp; produce pulse for a 0,
				//otherwise produce a pulse for a 1
                {
				_LATA0=1;
				vTaskDelay (T0H);
				_LATA0=0;
				vTaskDelay (T0L);
                }
			else
                {
				_LATA0=1;
				vTaskDelay (T1H);
				_LATA0=0;
				vTaskDelay (T1L);
                }
		}
		//begin masking iteration for blue
            for (MaskingIter=0; MaskingIter < 8; MaskingIter++)
            {
			//begin masking iteration loop for Green
                if ((0x80 >> MaskingIter) & LedNum[0].Blue)  
				//if statement says: if a 0 is found in temp; produce pulse for a 0,
				//otherwise produce a pulse for a 1
                {
				_LATA0=1;
				vTaskDelay (T0H);
				_LATA0=0;
				vTaskDelay (T0L);
                }
			else
                {
				_LATA0=1;
				vTaskDelay (T1H);
				_LATA0=0;
				vTaskDelay (T1L);
                }
            }
        }        
        
    }
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


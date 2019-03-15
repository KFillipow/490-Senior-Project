/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 UART - PC Echo with 12MHz BRCLK
 *
 * Description: This demo echoes back characters received via a PC serial port.
 * SMCLK/DCO is used as a clock source and the device is put in LPM0
 * The auto-clock enable feature is used by the eUSCI and SMCLK is turned off
 * when the UART is idle and turned on when a receive edge is detected.
 * Note that level shifter hardware is needed to shift between RS232 and MSP
 * voltage levels.
 *
 *               MSP432P401
 *             -----------------
 *            |                 |
 *            |                 |
 *            |                 |
 *       RST -|     P1.3/UCA0TXD|----> PC (echo)
 *            |                 |
 *            |                 |
 *            |     P1.2/UCA0RXD|<---- PC
 *            |                 |
 *
 *******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "stdarg.h"
#include <math.h>
#include <complex.h>
#include <arm_math.h>
#include <arm_common_tables.h>


//![Simple UART Config]
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 
 */
 

#define SAMPLE_NUM			8192 // 4096
#define CR              0x0D
#define LF              0x0A
//#define ARMBITREVINDEXTABLE1024_TABLE_LENGTH ((uint16_t)1800)
#define ARMBITREVINDEXTABLE2048_TABLE_LENGTH ((uint16_t)3808)
#define ARMBITREVINDEXTABLE4096_TABLE_LENGTH ((uint16_t)4032)
float32_t resultsBuffer[8192]; //4092
float32_t magnitudeBuffer[SAMPLE_NUM/2];
static volatile uint16_t resPos;
volatile char						disADC; //changed all flags to volatile
volatile char						send_data;
volatile char						write_data;
volatile char           begin_sample;
volatile char						do_fft;
char mag_string[20];
uint64_t   current_smclk;
uint64_t   current_hsmclk;
uint64_t   current_mclk;
uint32_t   result; //possibly change to volatile once try to implement main loop again
uint16_t   uart_out_count;
uint32_t   fftSize = 4096; //2048
uint32_t   ifftFlag = 0;
uint32_t   doBitReverse = 1;

//static float32_t testOutput[SAMPLE_NUM/2];

const arm_cfft_instance_f32 fft_settings = {
	4096, twiddleCoef_4096, armBitRevIndexTable4096, ARMBITREVINDEXTABLE4096_TABLE_LENGTH
}; //all 1024 stuff originally

 

const Timer_A_UpModeConfig upModeConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,            // SMCLK Source @ 12MHz
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // Divide clock source by 1
        3000,                                // 3000 Period (4KHz)
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0
        TIMER_A_DO_CLEAR                     // Clear Counter
};

 
const Timer_A_CompareModeConfig compareConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR1
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_SET_RESET,               // Toggle output but
        3000                                        // 3000 (4KHz) Period
}; 
 
const eUSCI_UART_Config uartConfig =
{

		EUSCI_A_UART_CLOCKSOURCE_ACLK,          // ACLK Clock Source @32.768KHz
        3,                                     // BRDIV = 78
        0,                                       // UCxBRF = 2
        132,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
};

void UART_OutChar(char data){
  while((UCA0IFG&0x02) == 0);
  UCA0TXBUF = data;
}

void UART_OutUDec(uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART_OutUDec(n/10);
    n = n%10;
  }
  UART_OutChar(n+'0'); /* n is between 0 and 9 */
}

void UART_OutString(char *pt){
  while(*pt){
    UART_OutChar(*pt);
    pt++;
  }
}

void reverse(char *str, int len) 
{ 
    int i=0, j=len-1, temp; 
    while (i<j) 
    { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; j--; 
    } 
} 
  
 // Converts a given integer x to string str[].  d is the number 
 // of digits required in output. If d is more than the number 
 // of digits in x, then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) 
    { 
        str[i++] = (x%10) + '0'; 
        x = x/10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  
// Converts a floating point number to string. 
void ftoa(float n, char *res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) 
    { 
        res[i] = '.';  // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter is needed 
        // to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
} 



int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();
	resPos = 0;
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
	CS_setExternalClockSourceFrequency(32768, 48000000);
	MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 1);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 1);
    CS_startHFXT(false);
    CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ); //originally 32KHz
    //SMCLK for Timer A
    //HSMCLK for ADC
    //ACLK for UART
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_2);
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);

    //MCLK is at 48MHz
    //HSMCLK is at 24MHz
    //SMCLK is as 6MHz
    //ACLK is at 32.768KHz
    current_smclk = CS_getSMCLK();
    current_hsmclk = CS_getHSMCLK();
    current_mclk = CS_getMCLK();
	/* Setting DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initializing ADC (HSMCLK/1/2) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_HSMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_2,
            0);

	//right now HSMCLK is at 64KHz

    /* Configuring GPIOs (5.5 A0) */
    //MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
            GPIO_PIN7 | GPIO_PIN6 | GPIO_PIN5 | GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Configuring ADC Memory */
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_EXTPOS_VREFNEG_EXTNEG,
    ADC_INPUT_A0, false);

    /* Configuring Timer_A in continuous mode and sourced from SMCLK */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upModeConfig);

    /* Configuring Timer_A0 in CCR1 to trigger at 3000 (4KHz) */
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig);

    /* Configuring the sample trigger to be sourced from Timer_A0  and setting it
     * to automatic iteration after it is triggered*/
    MAP_ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false);

    /* Enabling the interrupt when a conversion on channel 1 is complete and
     * enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT0);
    MAP_ADC14_enableConversion();
		//REF_A_setReferenceVoltage(REF_A_VREF2_5V);
		//REF_A_enableReferenceVoltage();

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    //MAP_Interrupt_enableMaster();

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    //![Simple UART Example]
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);
    /* Enabling interrupts */
    //MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();   
    //![Simple UART Example]
    UART_OutString("Init");
    UART_OutChar(CR);
    UART_OutChar(LF);
    disADC = 0;
    send_data = 0;
    uart_out_count = 0;
    write_data = 0;
    begin_sample = 0;
    do_fft = 0;
		/* Starting the Timer */
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
    while(1)
    {

    }
}

void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);
    if (status & ADC_INT0)
{
		result = MAP_ADC14_getResult(ADC_MEM0);
		if(result > 8600 && begin_sample == 0 && send_data == 0 && do_fft == 0){
			send_data = 0;
			begin_sample = 1;
			resPos = 0; //index
			uart_out_count = 0; //index
			//write_data = 1;
		}
		else if(begin_sample == 1 && resPos < 8192){ //4096
			resultsBuffer[resPos] = result;
			resPos++;
		}
		else if(begin_sample == 1 && resPos >= 8192){ //4096
			begin_sample = 0;
			send_data = 1;
		}
		else if(send_data == 1 && uart_out_count < 8192) //4096
		{
			UART_OutUDec(resultsBuffer[uart_out_count]);
			UART_OutChar(CR);
			UART_OutChar(LF);
			uart_out_count++;
		}
		else if(send_data == 1 && uart_out_count >= 8192){ //4096
			send_data = 0;
			do_fft = 1;
			
		}

		else if(do_fft == 1){
			UART_OutString("FFT");
			UART_OutChar(CR);
		    UART_OutChar(LF);
			do_fft = 0;
			arm_cfft_f32(&fft_settings, resultsBuffer, ifftFlag, doBitReverse);
			arm_cmplx_mag_f32(resultsBuffer, magnitudeBuffer, fftSize);
			int mag_ind = 1;
			for(mag_ind = 1; mag_ind < 4096; mag_ind = mag_ind+1){ //2048
				ftoa(magnitudeBuffer[mag_ind],mag_string,8);
				UART_OutString(mag_string);
				UART_OutChar(CR);
				UART_OutChar(LF);
				//mag index originally 1024
			}
			UART_OutString("DONE");
			UART_OutChar(CR);
		    UART_OutChar(LF);
			do_fft = 0;
			begin_sample = 0;
		}
	}			
} 



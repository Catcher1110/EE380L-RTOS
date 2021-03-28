// Lab01.c
// Runs on LM4F120/TM4C123
// Jian Chu(jc86537) Huang Huang(hh26752)
// Febu 4, 2019
// Lab 1
// TA: Kishore Punniyamurthy (MW 1:30-3:00pm)
// Last version: Febu 13, 2019

// hardware connections

// **********wide.hk ST7735R with ADXL345 accelerometer *******************
// Silkscreen Label (SDC side up; LCD side down) - Connection
// VCC  - +3.3 V
// GND  - Ground
// !SCL - PA2 Sclk SPI clock from microcontroller to TFT or SDC
// !SDA - PA5 MOSI SPI data from microcontroller to TFT or SDC
// DC   - PA6 TFT data/command
// RES  - PA7 TFT reset
// CS   - PA3 TFT_CS, active low to enable TFT
// *CS  - (NC) SDC_CS, active low to enable SDC
// MISO - (NC) MISO SPI data from SDC to microcontroller
// SDA  � (NC) I2C data for ADXL345 accelerometer
// SCL  � (NC) I2C clock for ADXL345 accelerometer
// SDO  � (NC) I2C alternate address for ADXL345 accelerometer
// Backlight + - Light, backlight connected to +3.3 V

#include <stdio.h>
#include <stdint.h>
#include "ST7735.h"
#include "PLL.h"
#include "..//inc/tm4c123gh6pm.h"
#include "UART.h"
#include "Timer2.h"
#include "ADC.h"
#include "OS.h"
#include "ADCSWTrigger.h"
#include <stdlib.h>

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void OutCRLF(void);           // Go to a new line for UART
void UserTask(void);          // Define the user task
int IsString(char *pstr, char *psub); 
// Determine whether these two expression are equal
void LCD_display(void);       // Task 1 for the lab01, LCD display
void ADC_Periodic(void);      // Task 2 for the lab01, collect ADC value periodicly
void ADC_SoftTrigger(void);   // Task 3 for the lab01, collect ADC value by software trigger

// User task, read the ADC value and print it out
// Inputs: None
// Output: None
// here we comment it for measuring the time
void UserTask(void){
//	int32_t data;
//	data = ADC0_InSeq3();
//	UART_OutString("\n\rADC data =");
//  UART_OutUDec(data);
//  OutCRLF();
//	UART_OutUDec(COUNTER);
}

//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

// ---------------------IsString---------------------
// Output whether the string equals to the specific one
// Input:  String, Standard String
// Output: 0 for not equal or 1 for equal
int IsString(char *pstr, char *psub){
	while(*pstr || *psub){
		if (*pstr != *psub){
			return 0;
		}else{
			// point to the next character
			pstr++;
			psub++;
		}
	}
	return 1;
}

// LCD display function, let user to manipulate the LCD
void LCD_display(void){
  char string[30];
  uint32_t n;
  OutCRLF();
  ST7735_InitR(INITR_REDTAB); // Initialize the LCD
  while(1){
		int device;  
		int line;
		char value[30];  
		while(1){
			UART_OutString("Which Device you want to use(top/bottom/-1 for EXIT): ");
			UART_InString(string,29); // Read user input
			if (IsString(string, "top")){
				// User choose the top display
				device = 0;	
				break;
			}else if (IsString(string, "bottom")){
				// User choose the bottom display
				device = 1;
				break;
			}else if (IsString(string, "-1")){
				// User choose to exit the function
				return;
			}
			OutCRLF();
			// If user's input is wrong, it will continue ask for input
		}
		
		while(1){
			OutCRLF();
			UART_OutString("Which line you want to use(0-3): ");
			n = UART_InUDec(); // Read user input for which line
			if(n <=3){
				line = n;
				break;
			}
		}
		OutCRLF();
		
		UART_OutString("What String you want to show(in 30 char): ");
		UART_InString(string,29); // Read user input
		OutCRLF();
		
		UART_OutString("What number you want to show: ");
		UART_InNum(value, 29);   // Read user input
		OutCRLF();
		
		UART_OutString("See the LCD!"); OutCRLF();

		ST7735_Message(device, line, string, value);
		// Display the message from the above
  }
}

// Collect the ADC data periodicly
// Inputs: None
// Output: None
void ADC_Periodic(void){
  volatile uint32_t delay;
  UART_Init();              // initialize UART
  OutCRLF();
  ADC0_InitSWTriggerSeq3_Ch9();   // initialize ADC0 with channel 9
  OS_AddPeriodicThread(&UserTask, 8000000, 2);
  // Add user task into the thread, set the priority 2, period 800000(10Hz)
  EnableInterrupts();
  while(1){
    WaitForInterrupt();
  }
}

// Collect the ADC by software trigger
// Inputs: None
// Output: None
void ADC_SoftTrigger(void){
  uint32_t buffer[100];
  uint32_t channelNum;
  uint32_t fs;
  uint32_t numberOfSamples;
  int i = 0;
  // Read User Inputs for the program                 
  OutCRLF();
  UART_OutString("Which channel you want to use(0~11):"); 
  channelNum = UART_InUDec();
  OutCRLF();
  UART_OutString("What frequency you want to use(100~10000Hz):");
  fs = UART_InUDec();
  OutCRLF();
  UART_OutString("How many samples you want to collect?:");
  numberOfSamples = UART_InUDec();
  OutCRLF();
  SYSCTL_RCGCGPIO_R |= 0x00000020;         // activate port F
  ADC_Collect(channelNum, fs, buffer,numberOfSamples);
  // Output the String by UART
  while(buffer[i]){
	UART_OutUDec(buffer[i]);
	UART_OutString(" ");
	i++;
  }
  SYSCTL_RCGCTIMER_R |= 0x00;
}

// main function for Lab 1
int main(void){
  char string[30];  				
  PLL_Init(Bus80MHz);       // set system clock to 80 MHz
  UART_Init();              // initialize UART
	EnableInterrupts();		// enable interrupts
	while(1){
		OutCRLF();
		UART_OutString("Which function you want to use?(LCD, ADCH, ADCS):");
		UART_InString(string,29);
		// User make choices for what function to use
		if(IsString(string, "LCD")){
			LCD_display();        // Display the LCD function
		}else if(IsString(string, "ADCH")){
			ADC_Periodic();       // Display the periodic ADC sampling
		}else if(IsString(string, "ADCS")){
			ADC_SoftTrigger();    // Display the software trigger ADC sampling
		}
	}
}

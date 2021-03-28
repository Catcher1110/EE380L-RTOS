//*****************************************************************************
//
// Lab4.c - user programs, File system, stream data onto disk
//*****************************************************************************

// Jonathan W. Valvano 3/7/17, valvano@mail.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file to do Lab 4
// as long as the basic functionality is simular
// 1) runs on your Lab 2 or Lab 3
// 2) implements your own eFile.c system with no code pasted in from other sources
// 3) streams real-time data from robot onto disk
// 4) supports multiple file reads/writes
// 5) has an interpreter that demonstrates features
// 6) interactive with UART input, and switch input

// LED outputs to logic analyzer for OS profile 
// PF1 is preemptive thread switch
// PF2 is periodic task
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task 
// PF4 is SW1 button input

// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) connected to PB0
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground 

// ESP8266
// PB1 Reset
// PD6 Uart Rx <- Tx ESP8266
// PD7 Uart Tx -> Rx ESP8266

// Free pins (debugging)
// PF3, PF2, PF1 (color LED)
// PD3, PD2, PD1, PD0, PC4

#include <stdio.h>
#include <stdint.h>
#include "os.h"
#include "../inc/tm4c123gh6pm.h"
#include "ST7735.h"
#include "UART2.h"
#include <string.h> 
#include "cmdline.h"
#include "diskio.h"
#include "ff.h"
#include "loader.h"
#include "heap.h"
#include "PWM.h"
#include "PLL.h"

unsigned long NumCreated;   // number of foreground threads created

#define TIMESLICE 2*TIME_1MS  // thread switch time in system time units

//static FATFS g_sFatFs;
//FIL Handle,Handle2;
//FRESULT MountFresult;
//FRESULT Fresult;
unsigned char buffer[512];

void DelayWait10ms(uint32_t n);

#define PD0  (*((volatile unsigned long *)0x40007004))
#define PD1  (*((volatile unsigned long *)0x40007008))
#define PD2  (*((volatile unsigned long *)0x40007010))
#define PD3  (*((volatile unsigned long *)0x40007020))

void PortD_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x08;       // activate port D
  while((SYSCTL_PRGPIO_R&0x08)==0){};      
  GPIO_PORTD_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0x0F;   // disable alt funct on PD3-0
  GPIO_PORTD_DEN_R |= 0x0F;     // enable digital I/O on PD3-0
  GPIO_PORTD_PCTL_R = ~0x0000FFFF;
  GPIO_PORTD_AMSEL_R &= ~0x0F;;      // disable analog functionality on PD
}
  
//************SW1Push*************
// Called when SW1 Button pushed
// background threads execute once and return
void SW1Push(void){

}
//************SW2Push*************
// Called when SW2 Button pushed
// background threads execute once and return
void SW2Push(void){

}
 
//******** IdleTask  *************** 
// foreground thread, runs when no other work needed
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
unsigned long Idlecount=0;
void IdleTask(void){ 
  while(1) { 
    Idlecount++;        // debugging 
  }
}


//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

void BackspacePreprocessString(char* string) {
	int index = 0;
	int offset = 0;
	while (string[index+offset]) {
		string[index] = string[index+offset];
		// PuTTY config generates 0x7F for backspace
		if (string[index+offset] == 0x7F) {
			index--;
			offset+=2;
		}
		else {
			index++;
		}
	}
	string[index] = string[index+offset];
	UART_OutString("\r\n");
}

//******** Interpreter **************
// foreground thread, accepts input from UART port, outputs to UART port
// inputs:  none
// outputs: none
void Interpreter(void) {
	char string[80];
	//eFile_Init();
  while(1){
    OutCRLF(); UART_OutString(">");
    UART_InString(string,79);
		BackspacePreprocessString(string);
		if(CmdLineProcess(string) == -1) {
			//UART_OutString("\r\ncommand not recognized");
		}
  }
}


#define PF2     (*((volatile uint32_t *)0x40025010))
#define PF3     (*((volatile uint32_t *)0x40025020))

void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= 0x20; // activate port F
  while((SYSCTL_PRGPIO_R&0x20)==0){}; // allow time for clock to start 
	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
	GPIO_PORTF_CR_R |= 0x0C;
  GPIO_PORTF_DIR_R |= 0x0C;    // (c) make PF4 out (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x0C;  //     disable alt funct on PF0, PF4
  GPIO_PORTF_DEN_R |= 0x0C;     //     enable digital I/O on PF4
  GPIO_PORTF_PCTL_R &= ~0x0000FF00; //  configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x0C;  //    disable analog functionality on PF4
  GPIO_PORTF_PUR_R |= 0x0C;     //     enable weak pull-up on PF4
}
void DelayWait10mss(uint32_t n){uint32_t volatile time;
  while(n){
    time = 727240*2/91;  // 10msec
    while(time){
      time--;
    }
    n--;
  }
}

 uint16_t cnt = 1275;
int mode = 0;
//*******************lab 4 main **********
int main(void){        // lab 4 real main
	int direction = 1;
	uint32_t Power = 5;
	PLL_Init();               // bus clock at 80 MHz
 // int direction = 0;   // forward
  PortF_Init();
	DC_Init();
 // Servo_Init(25000,0);  
  DelayWait10mss(500);	
   Servo_Deg_Duty(271);
	 DelayWait10mss(500);
	Servo_Deg_Duty(89); 
  while(1){
	DelayWait10mss(500);	
  Servo_Duty(1275);
	DelayWait10mss(500);
	Servo_Duty(2375);
	DCR_Motor(direction,0,Power);		
	DCL_Motor(direction,0,Power);	
	//DCR_Motor(1,2000);		
	//DCL_Motor(1,2000);
	}
  /*OS_Init();           // initialize, disable interrupts
	ST7735_InitR(INITR_REDTAB);
	UART_Init();
  Heap_Init();
	PortF_Init();
	PortD_Init();
  //*******attach background tasks***********
  OS_AddSW1Task(&SW1Push,2);    // PF4, SW1
  OS_AddSW2Task(&SW2Push,3);   // PF0
	
  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&Interpreter,128,3); 
  NumCreated += OS_AddThread(&IdleTask,128,7);  // runs when nothing useful to do
  //OS_Addprocess();
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
*/}

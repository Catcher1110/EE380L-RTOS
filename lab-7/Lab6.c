// PeriodMeasure.c
// Runs on LM4F120/TM4C123
// Use Timer0A in 24-bit edge time mode to request interrupts on the rising
// edge of PB6 (T0CCP0), and measure period between pulses.
// Daniel Valvano
// May 5, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Example 7.2, Program 7.2

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// external signal connected to PB6 (T0CCP0) (trigger on rising edge)
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "PLL.h"
#include "os.h"
#include "UART2.h"
#include "PWM.h"
#include <string.h>
//#include "cmdline.h"
#include "can0.h"
//#include "ADCT3ATrigger.h"
//#include "ST7735.h"


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define PC6      	(*((volatile uint32_t *)0x40006100))
#define PC7       (*((volatile uint32_t *)0x40006200))	
#define DISTANCE_TO_POWER_CONVERSION_FACTOR 10
#define DISTANCE_TO_POWER_DIFFERENCE_CONVERSION_FACTOR 50
#define DISTANCE_TO_DIRECTION_CONVERSION_FACTOR 2
#define PERIOD10MS 12500  // 800ns units
#define POWERMIN 5
#define POWERMID 5000
#define POWERMAX (PERIOD10MS-100)
#define SideFactor 1.2
#define SidePower 1.3

uint32_t Period;              // (1/clock) units
uint32_t First;               // Timer0A first edge
int32_t Done;                 // set each rising
// max period is (2^24-1)*12.5ns = 209.7151ms
// min period determined by time to run ISR, which is about 1us

// Subroutine to wait 5 usec
// Inputs: None
// Outputs: None
void DelayWait5us(void){
	uint32_t volatile time;
  time = 400/12;  // 5usec
  while(time){
    time--; // 12 cycles
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

//void BackspacePreprocessString(char* string) {
//	int index = 0;
//	int offset = 0;
//	while (string[index+offset]) {
//		string[index] = string[index+offset];
//		// PuTTY config generates 0x7F for backspace
//		if (string[index+offset] == 0x7F) {
//			index--;
//			offset+=2;
//		}
//		else {
//			index++;
//		}
//	}
//	string[index] = string[index+offset];
//	UART_OutString("\r\n");
//}


////******** Interpreter **************
//// foreground thread, accepts input from UART port, outputs to UART port
//// inputs:  none
//// outputs: none
//void Interpreter(void) {
//	char string[80];
//  while(1){
//    OutCRLF(); UART_OutString(">");
//    UART_InString(string,79);
//		BackspacePreprocessString(string);
//		if(CmdLineProcess(string) == -1) {
//			UART_OutString("\r\ncommand not recognized");
//		}
//  }
//}

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

uint8_t XmtData[8];
uint8_t RcvData[8];
uint32_t RcvCount=0;
uint8_t sequenceNum=0;  

void CANTask(void){
  XmtData[0] = PF0<<1;  // 0 or 2
  XmtData[1] = PF4>>2;  // 0 or 4
  XmtData[2] = 0;       // unassigned field
  XmtData[3] = sequenceNum;  // sequence count
  CAN0_SendData(XmtData);
  sequenceNum++;
}
uint32_t DistanceToPowerConverter(int LRdata){
	uint32_t dis = LRdata * DISTANCE_TO_POWER_CONVERSION_FACTOR ;
	return dis;
}
uint32_t DistanceToPowerDifferenceConverter(int data){
	if (data<= 10)return 0;
	uint32_t diff = data * DISTANCE_TO_POWER_DIFFERENCE_CONVERSION_FACTOR; 
	return diff;
}
int DistanceToDirectionConverter(int data){
	if (data<= 10)return 0;
	int deg = data / DISTANCE_TO_DIRECTION_CONVERSION_FACTOR;
  return deg;
}

int IRdata1=0,IRdata2=0,LRdata=0,LastLR = 0;
int Ldata=20,Rdata=20;
int Direction = 0, ServoDeg = 0, ServoDir = 0 ;
int Directionlast=1;
int *LastDirection = &Directionlast;
uint32_t PowerRef = 2000, PowerDiff;
void DelayWait10ms(uint32_t n){uint32_t volatile time;
  while(n){
    time = 727240*2/91;  // 10msec
    while(time){
      time--;
    }
    n--;
  }
}
void WaitForRelease(void){
  while((PC6!=0x40)&&(PC7!=0x80)){};  // wait for switch touch
  DelayWait10ms(2); // debounce
  while((PC6==0x40)||(PC7==0x80)){};  // wait for both release
  DelayWait10ms(200); // debounce
}
void CANTest(void){
	while(1){
	  while((PC6!=0x40)&&(PC7!=0x80)){
    if(CAN0_GetMailNonBlock(RcvData)){
			IRdata1= RcvData[0]*256 + RcvData[1];
			IRdata2= RcvData[2]*256 + RcvData[3];
			LRdata = RcvData[4]*256 + RcvData[5];
			if(LRdata != LastLR){
				PowerRef = DistanceToPowerConverter(LRdata);
			}
			if(IRdata2-IRdata1>150){
			  ServoDir = 1;
				PowerRef = 500;
			  ServoDeg = DistanceToDirectionConverter(IRdata2-IRdata1);
				Servo_Deg_Duty(ServoDir,ServoDeg);
			  /*PowerDiff = DistanceToPowerDifferenceConverter(IRdata2 - IRdata1);
				DCL_Motor(Direction, LastDirection, PowerRef + PowerDiff);
			  DCR_Motor(Direction, LastDirection,  PowerRef);*/
				PowerDiff = POWERMID;
				DCL_Motor(Direction, LastDirection, PowerRef + PowerDiff);
			  DCR_Motor(Direction, LastDirection,  PowerRef);
			}
			else if(IRdata1-IRdata2 >150) {
				ServoDir = 0;
				PowerRef = 500;
			  ServoDeg = DistanceToDirectionConverter((IRdata2-IRdata1)*-1);
			  /*PowerDiff = DistanceToPowerDifferenceConverter((IRdata2 - IRdata1)*-1);
				DCL_Motor(Direction, LastDirection, PowerRef);
			  DCR_Motor(Direction, LastDirection, PowerRef + PowerDiff);*/ 
				Servo_Deg_Duty(ServoDir,ServoDeg);
				PowerDiff = POWERMID;
				DCL_Motor(Direction, LastDirection, PowerRef );
			  DCR_Motor(Direction, LastDirection,  PowerRef+ PowerDiff);
			}
			
			/*if (Ldata<10){
				PowerRef = SideFactor*Ldata;
				PowerDiff = SidePower*(10-Ldata);
				DCL_Motor(Direction, LastDirection, PowerRef + PowerDiff);
			  DCR_Motor(Direction, LastDirection, PowerRef);
			} else if (Rdata<10){
				PowerRef = SideFactor*Rdata;
				PowerDiff = SidePower*(10-Rdata);
				DCR_Motor(Direction, LastDirection, PowerRef + PowerDiff);
			  DCL_Motor(Direction, LastDirection, PowerRef);			
			}*/
			else{
				ServoDeg = 0;
				Servo_Deg_Duty(ServoDir,ServoDeg);
				DCR_Motor(Direction, LastDirection, PowerRef);
				DCL_Motor(Direction, LastDirection, PowerRef);
			}
    }		
	}	
		Direction = 1;
		DCL_Motor(Direction, LastDirection, POWERMID);
		DCR_Motor(Direction, LastDirection, POWERMID);
		WaitForRelease();
		Direction = 0;
		DCL_Motor(Direction, LastDirection, POWERMIN);
		DCR_Motor(Direction, LastDirection, POWERMIN);
  }
		
}

//uint32_t MinusAbs(uint32_t a, uint32_t b);
//uint32_t IRDistance1, IRDistance2; // 
//extern uint32_t ADC0value;
//extern uint32_t ADC1value;
//void IRThread(void){
//	while(1){
//	OS_Sleep(1000);
//	IRDistance1 = 0.375*670670/(ADC0value-40);
//	IRDistance2 = 0.375*670670/(ADC1value-40);
//	UART_OutString("Channel 0: ");
//	UART_OutUDec(IRDistance1);	
//  OutCRLF();
//	UART_OutString("Channel 3: ");
//	UART_OutUDec(IRDistance2);
//	OutCRLF();
//	}
//}

////---------------------Timer0----------------------------
//extern uint32_t Timer0First,Timer0Done,Timer0Pulse;
//#define ECHO0       (*((volatile uint32_t *)0x40005100)) // using bits [9:2]
//#define ECHO0HIGH 0x40 // 0100 0000
////------------Timer0_StartPing------------
//// start Ping))) ultrasonic distance measurement
//// 1) Make PB6 GPIO output
//// 2) 5us Pulse output on PB6 GPIO output
//// 3) Make PB6 input capture Timer input
//// Input: none
//// Output: none
//void Timer0_StartPing(void){
//  GPIO_PORTB_AFSEL_R &= ~0x40;     // disable alt funct on PB6
//  GPIO_PORTB_PCTL_R = GPIO_PORTB_PCTL_R&0xF0FFFFFF;
//  GPIO_PORTB_DIR_R |= 0x40;        // make PB6 output
//  Timer0Done = 0;
//  ECHO0 = ECHO0HIGH;
//  DelayWait5us();
//  ECHO0 = 0;
//  GPIO_PORTB_DIR_R &= ~0x40;       // make PB6 in
//  GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x07000000;
//  TIMER0_ICR_R = TIMER_ICR_CAECINT;// clear timer0A capture match flag
//}

////------------ PF_Init ------------
//// Initialize the PF Port
//// Input: none
//// Output: none
void PF_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x20;       // activate port F
                                   // allow time to finish activating
  while((SYSCTL_PRGPIO_R&0x20) == 0){};
  GPIO_PORTF_LOCK_R = 0x4C4F434B;  // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0xFF;          // allow changes to PF4-0
  GPIO_PORTF_DIR_R = 0x0E;         // make PF3-1 output (PF3-1 built-in LEDs)
  GPIO_PORTF_AFSEL_R = 0;          // disable alt funct
  GPIO_PORTF_DEN_R = 0x1F;         // enable digital I/O on PF4-0
  GPIO_PORTF_PUR_R = 0x11;         // enable pullup on inputs
  GPIO_PORTF_PCTL_R = 0x00000000;
  GPIO_PORTF_AMSEL_R = 0;          // disable analog functionality on PF
}
void Bumper_Init(void){       

	SYSCTL_RCGCGPIO_R |= 0x04;      // (a) activate clock for port C
  //SYSCTL_RCGC2_R |= 0x00000004; 
	//while((SYSCTL_PRGPIO_R&0x0004) == 0){};// ready?
  GPIO_PORTC_DIR_R &= ~0xC0;      // (c) make PC6 and PC7 in 
  GPIO_PORTC_AFSEL_R &= ~0xC0;    //     disable alt funct on PC6 and PC7
  GPIO_PORTC_DEN_R |= 0xC0;       //     enable digital I/O on PC6 and PC7
  GPIO_PORTC_PCTL_R &= ~0xFF000000; //  configure PC6 and PC7 as GPIO
  GPIO_PORTC_AMSEL_R &= ~0xC0;    //    disable analog functionality on PC6 and PC7
  //GPIO_PORTC_PUR_R |= 0xC0;     //     enable weak pull-up on PC6 and PC7
  //GPIO_PORTC_IS_R &= ~0xC0;     // (d) PC6 and PC7 is edge-sensitive
  //GPIO_PORTC_IBE_R &= ~0xC0;    //     PC6 and PC7 is not both edges
  //GPIO_PORTC_IEV_R |= 0xC0;     //     PC6 and PC7 rising edge event
  //GPIO_PORTC_ICR_R = 0xC0;      // (e) clear flag6 and 7 
  //GPIO_PORTC_IM_R |= 0xC0;      // (f) arm interrupt on PC6 and PC7
  //NVIC_PRI0_R = (NVIC_PRI0_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  //NVIC_EN0_R = 0x00000002;      // (h) enable interrupt 2 in NVIC
  //EnableInterrupts();           // (i) Enable global Interrupt flag (I)

}
int main(void){
  uint32_t NumCreated = 0;	
  OS_Init();             // 80 MHz clock
	PF_Init();
  CAN0_Open();
	Bumper_Init();
	//Timer0_Init();
	UART_Init();
	DC_Init();
	NumCreated += OS_AddThread(&IdleTask,128,7);
	NumCreated += OS_AddThread(&CANTest,128,3);
	OS_Launch(TIME_2MS);
}

// os.c
// Runs on LM4F120/TM4C123
// Jian Chu(jc86537) Huang Huang(hh26752)
// April 10, 2019
// OS file for LAB 6&7
// TA: Kishore Punniyamurthy (MW 1:30-3:00pm)
// Last version: April 10, 2019

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

   Programs 4.4 through 4.12, section 4.2

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

#include <stdint.h>
#include <stdio.h>
#include "os.h"
#include "PLL.h"
#include "..//inc/tm4c123gh6pm.h"
#include "UART2.h"
//#include "ST7735.h"

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority
#define SW1       0x10                      // on the left side of the Launchpad board
#define SW2       0x01                      // on the right side of the Launchpad board

#define block 1
#define nonblock 0

// function definitions in osasm.s
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
int32_t StartCritical(void);
void EndCritical(int32_t primask);
void StartOS(void);
void Scheduler(void);

#define NUMTHREADS  10        // maximum number of threads
#define STACKSIZE   512      // number of 32-bit words in stack
int Id = 0;
Sema4Type BlockPt;

struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
	struct tcb *previous;
	int ID;        // thread id
	int StateSleep;// record the time the thread need to sleep
	int Priority;  // priority assigned to the thread
	int Active;    // 1 for active, 0 for inactive 
	int blockflag; // 0 for nonblock, 1 for block
	Sema4Type *StateBlock;
};

typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
tcbType *NextPt;

int32_t Stacks[NUMTHREADS][STACKSIZE];

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14 LR
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

#define TCBSIZE 10
uint32_t volatile *TCBPutPt; // put next
uint32_t volatile *TCBGetPt; // get next
uint32_t static TCBFifo[TCBSIZE];
Sema4Type TCBCurrentSize;

// ******** TCB_Wait ************
// decrement semaphore 
// Just For TCB
// input:  pointer to a counting semaphore
// output: none
void TCB_Wait(Sema4Type *semaPt){
  uint32_t status = StartCritical();
	while(semaPt->Value <= 0){
	  OS_EnableInterrupts();
		OS_Suspend();
		OS_DisableInterrupts();
	}
	semaPt->Value = semaPt->Value - 1;
	EndCritical(status);
}

// ******** TCB_Signal ************
// increment semaphore 
// Just For TCB
// input:  pointer to a counting semaphore
// output: none
void TCB_Signal(Sema4Type *semaPt){
  uint32_t status = StartCritical();
	semaPt->Value = semaPt->Value + 1;
	EndCritical(status);
}

// ******** TCB_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int TCB_Fifo_Put(unsigned long data){
  // This TCB Fifo can never be overlapped	
  *(TCBPutPt) = data;	
	TCBPutPt++;	
  if(TCBPutPt == &TCBFifo[TCBSIZE]){
		TCBPutPt = &TCBFifo[0]; // wrap
  }
	TCB_Signal(&TCBCurrentSize);
  return 0;
}

// ******** TCB_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long TCB_Fifo_Get(void){
  uint32_t data; 
	TCB_Wait(&TCBCurrentSize); // block if empty 
	data = *(TCBGetPt); 
	TCBGetPt++;
  if(TCBGetPt == &TCBFifo[TCBSIZE]){ 
		TCBGetPt = &TCBFifo[0]; // wrap
  } 
	return data;
}

// ******** TCB_Fifo_Init ************
// Initialize the TCB Fifo to be empty
// Inputs: size
// Outputs: none 
void TCB_Fifo_Init(void){
  int i;
	TCBGetPt = &TCBFifo[0];
	TCBPutPt = &TCBFifo[0];
	OS_InitSemaphore(&TCBCurrentSize, 0); 
	for(i = 0; i < TCBSIZE; i++){
	  TCB_Fifo_Put(i);
	}
}

// ******** TCB_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long TCB_Fifo_Size(void){	
	int RoomLeft = TCBSIZE - TCBCurrentSize.Value;
	return RoomLeft;
}

// *********************************************************
// *********************************************************
// ***************** OS FUNCTION PART **********************
// *********************************************************
// *********************************************************
uint32_t SleepMatrix[TCBSIZE];
//******** OS_AddThread ***************
// add three foregound threads to the scheduler
// Inputs: three pointers to a void/void foreground tasks
// Outputs: 1 if successful, 0 if this thread can not be added
int OS_AddThread(void(*task)(void), 
   unsigned long stackSize, unsigned long priority){
	int Id;
  if(TCBCurrentSize.Value == TCBSIZE){
		int32_t status;
		status = StartCritical();
		Id = TCB_Fifo_Get();
		tcbs[Id].ID = Id;
		tcbs[Id].StateSleep = 0;
		tcbs[Id].next = &tcbs[Id];     // points to itself
		tcbs[Id].previous = &tcbs[Id]; // points to itself
		tcbs[Id].Priority = priority;  // assign priority
		tcbs[Id].Active = 1;
		tcbs[Id].blockflag = 0;
		tcbs[Id].StateBlock = NULL;
		SetInitialStack(Id); 
		Stacks[Id][STACKSIZE-2] = (int32_t)(task); // PC
		RunPt = &tcbs[Id];       // thread Id will run first
		EndCritical(status);
		return 1;               // successful
	}else if(TCBCurrentSize.Value >= 1){
	  int32_t status;
		int currentID;
		status = StartCritical();
		Id = TCB_Fifo_Get();
		currentID = RunPt->ID;
		while(tcbs[currentID].Active != 1){
		  currentID = (*tcbs[currentID].next).ID;
		}
		tcbs[Id].ID = Id;
		tcbs[Id].StateSleep = 0;
		tcbs[Id].Priority = priority;  // assign priority
		tcbs[currentID].next->previous = &tcbs[Id];
		tcbs[Id].next = tcbs[currentID].next;
		tcbs[Id].previous = &tcbs[currentID];
		tcbs[currentID].next = &tcbs[Id];
		tcbs[Id].Active = 1;
		tcbs[Id].blockflag = 0;
		tcbs[Id].StateBlock = NULL;
    SetInitialStack(Id);
		Stacks[Id][STACKSIZE-2] = (int32_t)(task); // PC
	  EndCritical(status);
    return 1;               // successful
	}else if(TCBCurrentSize.Value == 0){
	  return 0;
	}          // fail
	return 0;
}

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
void (*PeriodicTask1)(void);   // user function
void (*PeriodicTask2)(void);   // user function
int TimerFlag = 0;
int OS_AddPeriodicThread(void(*task)(void), 
   unsigned long period, uint32_t priority){
	if(TimerFlag == 0){
	SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER1
  PeriodicTask1 = task;          // user function
  TIMER1_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAILR_R = period-1;    // 4) reload value
  TIMER1_TAPR_R = 0;            // 5) bus clock resolution
  TIMER1_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
  TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|priority<<13;
	NVIC_EN0_R = 1<<21;      // 9) enable IRQ 21 in NVIC
  TIMER1_CTL_R = 0x00000001;    // 10) enable TIMER1A
	TimerFlag = 1;
	return 1;
  }else{
	SYSCTL_RCGCTIMER_R |= 0x04;   // 0) activate timer2
  PeriodicTask2 = task;          // user function
  TIMER2_CTL_R = 0x00000000;    // 1) disable timer2A during setup
  TIMER2_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER2_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER2_TAILR_R = period-1;    // 4) reload value
  TIMER2_TAPR_R = 0;            // 5) bus clock resolution
  TIMER2_ICR_R = 0x00000001;    // 6) clear timer2A timeout flag
  TIMER2_IMR_R = 0x00000001;    // 7) arm timeout interrupt
	NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|priority<<29;
  NVIC_EN0_R = 1<<23;           // 9) enable IRQ 23 in NVIC
  TIMER2_CTL_R = 0x00000001;    // 10) enable timer2A
  return 1;
}
}

// ***************** Timer1A_Handler ****************
// Handler for interrupt, increse the counter and execute user task
// Inputs:  None
// Outputs: None
uint32_t Timecount1 = 0;
void Timer1A_Handler(void){
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER0A timeout
  (*PeriodicTask1)();                // execute user task
	//Timecount1++;
}

void Timer2A_Handler(void){
	// GPIO_PORTF_DATA_R ^= 0x04;        // toggle the LED
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER0A timeout
  (*PeriodicTask2)();                // execute user task
	// GPIO_PORTF_DATA_R ^= 0x04;        // toggle the LED
}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
#define PF4                     (*((volatile uint32_t *)0x40025040))
#define PF0                      (*((volatile unsigned long *)0x40025004))
volatile static unsigned long State4;      // previous
volatile static unsigned long State0;      // previous
void (*SW1Task)(void);   // user function 
void (*SW2Task)(void);   // user function 
int sw1_active=0;
int sw2_active=0;
void SW_Init(){

	SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  while((SYSCTL_PRGPIO_R & 0x00000020) == 0){};
	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
	GPIO_PORTF_CR_R |= (0x11);  // 2b) enable commit for PF4 and PF0

  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4 and 0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x11;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000F000F; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R |= 0x11;     //     PF4 is both edges
	GPIO_PORTF_ICR_R = (SW1|SW2);      // (e) clear flag4
  GPIO_PORTF_IM_R |= (SW1|SW2);      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC  
};

int OS_AddSW1Task(void(*task)(void), unsigned long priority){
	SW_Init();
  SW1Task = task;           // user function 
  State4 = PF4;                      // initial switch state
	sw1_active=1;
	return 1;
}


int OS_AddSW2Task(void(*task)(void), unsigned long priority){		
	SW_Init();
  SW2Task = task;           // user function 
  State0 = PF0;                      // initial switch state
	sw2_active=1;
	return 1;
}
	
void GPIOPortF_Handler(void){
	GPIO_PORTF_IM_R &= ~0x11;     // disarm interrupt on PF4 
	if(GPIO_PORTF_RIS_R == 0x10){
		if(State4&&sw1_active){    // 0x10 means it was previously released
    (*SW1Task)();  // execute user task
		}
	}
	if(GPIO_PORTF_RIS_R == 0x01) {
		if(State0&&sw2_active){    // 0x10 means it was previously released
    (*SW2Task)();  // execute user task
		}	
	}

	State4 = PF4;
	State0 = PF0;
	GPIO_PORTF_ICR_R |= 0x11;
	GPIO_PORTF_IM_R |= 0x11; //enable interrupt	
}

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void){
	return RunPt->ID;
}


// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
	int currentID;
	int nextID;
	int previousID;
	OS_DisableInterrupts();
	// Get the next thread to run
	// Scheduler();
	// double link for the previous and next thread
   currentID = RunPt->ID;
	 nextID = tcbs[currentID].next->ID;
	 previousID = tcbs[currentID].previous->ID;
	tcbs[nextID].previous = &tcbs[previousID];
	tcbs[previousID].next = &tcbs[nextID];
	tcbs[currentID].Active = 0;
	// remove the thread from the TCB Fifo
	TCB_Fifo_Put(currentID);
	OS_EnableInterrupts();
	// trigger pendsv
	OS_Suspend();
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){
	uint32_t status = StartCritical();
	int currentID = RunPt->ID;
	RunPt->StateSleep = sleepTime;
	tcbs[currentID].Active = 0;
  SleepMatrix[currentID] = sleepTime;	
	EndCritical(status);
  OS_Suspend();
}

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
	Scheduler();
  NVIC_INT_CTRL_R = 0x10000000;
}

// ******** Scheduler ************
// Get the next thread to run
// input:  none
// output: none
void Scheduler(void){
	int StartID; // Start ID for the Searching Circle
	int NewID;   // Next Thread to be run
	int HighPriority; // The highest priority among all available threads
	int status = StartCritical();
	int currentID = RunPt->ID;
	int NextID = tcbs[currentID].next->ID; // Temp variable
	// Find next active thread
	while(tcbs[NextID].Active != 1 || tcbs[NextID].blockflag !=0){
	  NextID = tcbs[NextID].next->ID;
	}

	StartID = NextID; // Start from the first available thread
	// Record the id and priority of this thread
	NewID = NextID;
	HighPriority = tcbs[NewID].Priority;
	// Find the active thread with highest priority to run
	NextID = tcbs[NextID].next->ID;
	while(NextID != StartID){
		if(tcbs[NextID].blockflag == 0 && tcbs[NextID].Active == 1){
	    if(tcbs[NextID].Priority < HighPriority){
		      HighPriority = tcbs[NextID].Priority;
			    NewID = NextID;
		  }
		}
		NextID = tcbs[NextID].next->ID;
	}
	NextPt = &tcbs[NewID];	
	EndCritical(status);
}

int SysCounter = 0;
int SWCounter;
int TimeSlice;
uint32_t Counter = 0;
uint32_t lasttime;

// ******** Systick_Handler ************
// triggerred every systick interrupt
// decrease the sleep time for each thread
// trigger PendSV to get context switch
// input:  none
// output: none
void SysTick_Handler(void){
	int i;
  int32_t status  = StartCritical();	
	for(i = 0; i < TCBSIZE; i++){
	  if(SleepMatrix[i] > TimeSlice){
		  SleepMatrix[i] -= TimeSlice;
		}else if(SleepMatrix[i] == TimeSlice){
		  SleepMatrix[i] = 0;
			tcbs[i].Active = 1;
			tcbs[i].StateSleep = 0;
		}else{}
	}
	Scheduler();

  Counter++;   // Count systicks
	SWCounter++; // Count the systick based on SW

	NVIC_INT_CTRL_R = 0x10000000; //PendSV trigger

	EndCritical(status);
		
}

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 80 MHz PLL
// input:  none
// output: none
void OS_Init(void) {
  OS_DisableInterrupts();
  PLL_Init(Bus80MHz);                // set processor clock to 80 MHz  
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0000000; // priority 6
	// Use PendSV to trigger a context switch
	NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00F00000; // priority 7	
	TCB_Fifo_Init();
}

///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(uint32_t  theTimeSlice){
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
	TimeSlice = theTimeSlice/80000;
  StartOS();                   // start on the first task
}

// *********************************************************
// *********************************************************
// ***************** SEMAPHORE PART ************************
// *********************************************************
// *********************************************************

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
  semaPt->Value = value;
}

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
#if nonblock
void OS_Wait(Sema4Type *semaPt){
  uint32_t status = StartCritical();
	while(semaPt->Value <= 0){
	  EnableInterrupts();
		OS_Suspend();
		DisableInterrupts();
	}
	semaPt->Value = semaPt->Value - 1;
	EndCritical(status);
}
#endif

#if block
void OS_Wait(Sema4Type *semaPt){
  uint32_t status = StartCritical();
	semaPt->Value = semaPt->Value - 1;
	while(semaPt->Value < 0){
		RunPt->blockflag = 1;
		RunPt->StateBlock = semaPt;
		OS_EnableInterrupts();
		OS_Suspend();
		OS_DisableInterrupts();
	}
	EndCritical(status);
}
#endif

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
#if nonblock
void OS_Signal(Sema4Type *semaPt){
  uint32_t status = StartCritical();
	semaPt->Value = semaPt->Value + 1;
	EndCritical(status);
}
#endif

#if block
void OS_Signal(Sema4Type *semaPt){
  long status;
	int currentID;
	int nextID;
	int CID;
	int HighPriority;
	int NewID;
	status = StartCritical();
	// DisableInterrupts();
	semaPt->Value = semaPt->Value + 1;
	if (semaPt->Value<=0){
		currentID = RunPt->ID;
		nextID = tcbs[currentID].next->ID;
		while(tcbs[nextID].StateBlock != semaPt){
		  nextID = tcbs[nextID].next->ID;
		}
		CID = nextID; // The first thread blocked by this sema4
		HighPriority = tcbs[CID].Priority; // record the priority
		
		NewID = CID;
		nextID = tcbs[nextID].next->ID;
		while(nextID != CID){
		  if((tcbs[nextID].StateBlock == semaPt) && (tcbs[nextID].Priority < HighPriority)){
			  HighPriority = tcbs[nextID].Priority;
				NewID = nextID;
			}
			nextID = tcbs[nextID].next->ID;
		}

		tcbs[NewID].StateBlock = NULL;
		tcbs[NewID].blockflag = 0;
		
		if(HighPriority < tcbs[currentID].Priority){
		  OS_Suspend();
		}
	}
	EndCritical(status);
}
#endif

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
#if nonblock
void OS_bWait(Sema4Type *semaPt){
  uint32_t status = StartCritical();
	while(semaPt->Value == 0){
	  EnableInterrupts();
		OS_Suspend();
		DisableInterrupts();
	}
	semaPt->Value = 0;
	EndCritical(status);
}
#endif

#if block
void OS_bWait(Sema4Type *semaPt){
  uint32_t status = StartCritical();
	semaPt->Value = semaPt->Value - 1;
	if (semaPt->Value < 0){
		RunPt->StateBlock = semaPt;
		RunPt->blockflag = 1;
		OS_EnableInterrupts();
		OS_Suspend();
		OS_DisableInterrupts();
	}
	EndCritical(status);
}
#endif

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
#if nonblock
void OS_bSignal(Sema4Type *semaPt){
  long status;
	status = StartCritical();
	semaPt->Value = 1;
	EndCritical(status);
}
#endif

#if block
void OS_bSignal(Sema4Type *semaPt){
  long status;
	status = StartCritical();
	semaPt->Value = semaPt->Value + 1;
	if (semaPt->Value<=0){
		int currentID = RunPt->ID;
		int nextID = tcbs[currentID].next->ID;
		// int previousID = (*tcbs[currentID].previous).ID;
		while(tcbs[nextID].StateBlock != semaPt){
			nextID = tcbs[nextID].next->ID;
		}
		tcbs[nextID].StateBlock = NULL;
		RunPt->blockflag = 0;
	}
	//semaPt->Value = 1;
	EndCritical(status);
}
#endif

// *********************************************************
// *********************************************************
// ******************* OS FIFO PART ************************
// *********************************************************
// *********************************************************
#define FIFOSIZE 256
uint32_t volatile *PutPt; // put next
uint32_t volatile *GetPt; // get next
uint32_t static Fifo[FIFOSIZE];
Sema4Type CurrentSize;
Sema4Type FIFOmutex;
uint32_t LostData;

// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
void OS_Fifo_Init(unsigned long size){
	PutPt = &Fifo[0];
	GetPt = &Fifo[0];
	OS_InitSemaphore(&CurrentSize, 0); 
	OS_InitSemaphore(&FIFOmutex, 1);
	LostData = 0;
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data){
	// The FIFO is full
	if(CurrentSize.Value == FIFOSIZE){
	  LostData++;
		return 0;
	} 
	*(PutPt) = data; 
	PutPt++;	
  if(PutPt == &Fifo[FIFOSIZE]){
		PutPt = &Fifo[0]; // wrap
  }
	OS_Signal(&CurrentSize);
  return 1;
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void){
  uint32_t data; 
	OS_Wait(&CurrentSize); // block if empty 
	OS_Wait(&FIFOmutex); 
	data = *(GetPt); 
	GetPt++;
  if(GetPt == &Fifo[FIFOSIZE]){ 
		GetPt = &Fifo[0]; // wrap
  }
  OS_Signal(&FIFOmutex); 
	return data;
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void){
	int RoomLeft;
	if(FIFOmutex.Value == 0){
	  return 0;
	}
	RoomLeft = FIFOSIZE - CurrentSize.Value;
	if( RoomLeft > 0){
		return RoomLeft;
	}else if(RoomLeft == 0){
		return 0;
	}
	return 0;
}

// *********************************************************
// *********************************************************
// ***************** OS Mailbox PART ***********************
// *********************************************************
// *********************************************************
uint32_t Mail; // shared data
Sema4Type BoxFree; //semaphore
Sema4Type DataValid;  //semaphore

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
	OS_InitSemaphore(&BoxFree, 1);
	OS_InitSemaphore(&DataValid, 0);
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data){
  OS_Wait(&BoxFree);
	Mail = data;
	OS_Signal(&DataValid);
}

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void){
  uint32_t theData;
	OS_Wait(&DataValid);
	theData = Mail; // read mail
	OS_Signal(&BoxFree);
	return theData;
}


// *********************************************************
// *********************************************************
// ******************* OS TIME PART ************************
// *********************************************************
// *********************************************************
// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void){
	int32_t status = StartCritical();
  uint32_t TimePart2 = NVIC_ST_CURRENT_R&0xFFFFFF;
	unsigned long CurrentTime = (Counter+1) * TIME_2MS - TimePart2;
	EndCritical(status);
	return CurrentTime;
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop){
	return stop - start;
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
	Counter = 0;
  NVIC_ST_CURRENT_R = 0x000000;
}

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void){
	int32_t status = StartCritical();
  uint32_t TimePart2 = NVIC_ST_CURRENT_R&0xFFFFFF;
	unsigned long CurrentTime = Counter * 2 + 2 - TimePart2 * 12.5 / 1000000;
	EndCritical(status);
	return CurrentTime;
}

// ******** OS_SWClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_SWClearMsTime(void){
  SWCounter = 0;
}

// ******** OS_SWMsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_SWMsTime(void){
	int32_t status = StartCritical();
  uint32_t TimePart2 = NVIC_ST_CURRENT_R&0xFFFFFF;
	unsigned long CurrentTime = SWCounter * 2 + 2 - TimePart2 * 12.5 / 1000000;
	EndCritical(status);
	return CurrentTime;
}

// *********************************************************
// *********************************************************
// ******************* PING PART ***************************
// *********************************************************
// *********************************************************
#define TIMER_TAMR_TACMR        0x00000004  // GPTM TimerA Capture Mode
#define TIMER_TAMR_TAMR_CAP     0x00000003  // Capture mode
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_CTL_TAEVENT_POS   0x00000000  // Positive edge
#define TIMER_CTL_TAEVENT_NEG   0x00000004  // Negative edge
#define TIMER_CTL_TAEVENT_BOTH  0x0000000C  // Both edges
#define TIMER_IMR_CAEIM         0x00000004  // GPTM CaptureA Event Interrupt
                                            // Mask
#define TIMER_ICR_CAECINT       0x00000004  // GPTM CaptureA Event Interrupt
                                            // Clear
#define TIMER_TAILR_TAILRL_M    0x0000FFFF  // GPTM TimerA Interval Load
                                            // Register Low

// TIMER0 is echo PB6
#define ECHO0       (*((volatile uint32_t *)0x40005100)) // using bits [9:2]
#define ECHO0HIGH 0x40 // 0100 0000
// TIMER4 is echo PD4
#define ECHO4       (*((volatile uint32_t *)0x40007040))
#define ECHO4HIGH 0x10 // 0001 0000
// TIMER3 is echo PB2
#define ECHO3       (*((volatile uint32_t *)0x40005010))
#define ECHO3HIGH 0x04  // 0000 0100
// TIMER5 is echo PD6
#define ECHO5       (*((volatile uint32_t *)0x40007100))
#define ECHO5HIGH 0x40  // 0100 0000


//---------------------Timer0----------------------------
uint32_t Timer0First,Timer0Done,Timer0Pulse;
//------------Timer0_Init------------
// Initialize Timer0A in edge time mode to request interrupts on
// the both edges of PB6 (T0CCP0).  The interrupt service routine
// acknowledges the interrupt records the time.
// PB7 GPIO output
// Input: none
// Output: none
void Timer0_Init(void){
  Timer0First = Timer0Done = Timer0Pulse = 0;
  SYSCTL_RCGCTIMER_R |= 0x01;// activate timer0
  SYSCTL_RCGCGPIO_R |= 0x02; // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
  GPIO_PORTB_DIR_R &= ~0x40;       // make PB6 in
  GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6
  GPIO_PORTB_DEN_R |= 0x40;        // enable PB6 as T0CCP0
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0x00FFFFFF)+0x07000000;
	GPIO_PORTB_AMSEL_R &= ~0x40;     // disable analog functionality on PB6
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
  TIMER0_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
  TIMER0_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);   // 24-bit capture         
  TIMER0_CTL_R |= TIMER_CTL_TAEVENT_BOTH;// configure for both edges
  TIMER0_TAILR_R = TIMER_TAILR_M;  // max start value
  TIMER0_TAPR_R = 0xFF;            // activate prescale, creating 24-bit
  TIMER0_IMR_R |= TIMER_IMR_CAEIM; // enable capture match interrupt
  TIMER0_ICR_R = TIMER_ICR_CAECINT;// clear timer0A capture match flag
	TIMER0_CTL_R |= TIMER_CTL_TAEN;  // enable timer0A 16-b, +edge timing, interrupts
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // top 3 bits, priority 2
  NVIC_EN0_R = 1<<19;        // enable interrupt 19 in NVIC
}

//------------Timer0_Read------------
// read ultrasonic distance measurement
// Input: none
// Output: 0 if not ready, pulse width in 12.5ns time if ready
uint32_t Timer0_Read(void){
  if(Timer0Done){
    return Timer0Pulse;
  }
  return 0;
}
void OutCRLF(void);

//-------------- Timer0A_Handler -----------
// triggerred by both eages, calculate the time slice
// data saved in the global variable Timer0Pulse
// input None
// output None
void Timer0A_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A capture match
  if(ECHO0 == ECHO0HIGH){ // first
    Timer0First = TIMER0_TAR_R;
  }else{
    Timer0Pulse = (Timer0First - TIMER0_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
    Timer0Done = 1;
//		UART_OutUDec(Timer0Pulse);
//		OutCRLF();
  }
}

////---------------------Timer3----------------------------
//uint32_t Timer3First,Timer3Done,Timer3Pulse;
////------------Timer0_Init------------
//// Initialize Timer0A in edge time mode to request interrupts on
//// the both edges of PB6 (T0CCP0).  The interrupt service routine
//// acknowledges the interrupt records the time.
//// PB7 GPIO output
//// Input: none
//// Output: none
//void Timer3_Init(void){
//  Timer3First = Timer3Done = Timer3Pulse = 0;
//  SYSCTL_RCGCTIMER_R |= 0x08;// activate timer3
//  SYSCTL_RCGCGPIO_R |= 0x02; // activate port B
//  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
//  GPIO_PORTB_DIR_R &= ~0x04;       // make PB2 in
//  GPIO_PORTB_AFSEL_R |= 0x04;      // enable alt funct on PB2
//  GPIO_PORTB_DEN_R |= 0x04;        // enable PB2 as T0CCP0
//	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00000700;
//	GPIO_PORTB_AMSEL_R &= ~0x04;     // disable analog functionality on PB2
//  TIMER3_CTL_R &= ~TIMER_CTL_TAEN; // disable timer3A during setup
//  TIMER3_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
//  TIMER3_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);   // 24-bit capture         
//  TIMER3_CTL_R |= TIMER_CTL_TAEVENT_BOTH;// configure for both edges
//  TIMER3_TAILR_R = TIMER_TAILR_M;  // max start value
//  TIMER3_TAPR_R = 0xFF;            // activate prescale, creating 24-bit
//  TIMER3_IMR_R |= TIMER_IMR_CAEIM; // enable capture match interrupt
//  TIMER3_ICR_R = TIMER_ICR_CAECINT;// clear timer3A capture match flag
//	TIMER3_CTL_R |= TIMER_CTL_TAEN;  // enable timer3A 16-b, +edge timing, interrupts
//  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x40000000; // top 3 bits, priority 2
//  NVIC_EN1_R = 1<<(35-32);        // enable interrupt 35 in NVIC
//}

////------------Timer3_Read------------
//// read ultrasonic distance measurement
//// Input: none
//// Output: 0 if not ready, pulse width in 12.5ns time if ready
//uint32_t Timer3_Read(void){
//  if(Timer3Done){
//    return Timer3Pulse;
//  }
//  return 0;
//}
//void OutCRLF(void);

////-------------- Timer3A_Handler -----------
//// triggerred by both eages, calculate the time slice
//// data saved in the global variable Timer3Pulse
//// input None
//// output None
//void Timer3A_Handler(void){
//  TIMER3_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer3A capture match
//  if(ECHO3 == ECHO3HIGH){ // first
//    Timer3First = TIMER3_TAR_R;
//  }else{
//    Timer3Pulse = (Timer3First - TIMER3_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
//    Timer3Done = 1;
//  }
//}

////---------------------Timer4----------------------------
//uint32_t Timer4First,Timer4Done,Timer4Pulse;
////------------Timer0_Init------------
//// Initialize Timer0A in edge time mode to request interrupts on
//// the both edges of PB6 (T0CCP0).  The interrupt service routine
//// acknowledges the interrupt records the time.
//// PB7 GPIO output
//// Input: none
//// Output: none
//void Timer4_Init(void){
//  Timer4First = Timer4Done = Timer4Pulse = 0;
//  SYSCTL_RCGCTIMER_R |= 0x10;// activate timer4
//  SYSCTL_RCGCGPIO_R |= 0x08; // activate port D
//  while((SYSCTL_PRGPIO_R&0x0008) == 0){};// ready?
//  GPIO_PORTD_DIR_R &= ~0x10;       // make PD4 in
//  GPIO_PORTD_AFSEL_R |= 0x10;      // enable alt funct on PD4
//  GPIO_PORTD_DEN_R |= 0x10;        // enable PD4 as T4CCP0
//	GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFF0FFFF)+0x00070000;
//	GPIO_PORTD_AMSEL_R &= ~0x10;     // disable analog functionality on PD4
//  TIMER4_CTL_R &= ~TIMER_CTL_TAEN; // disable timer4A during setup
//  TIMER4_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
//  TIMER4_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);   // 24-bit capture         
//  TIMER4_CTL_R |= TIMER_CTL_TAEVENT_BOTH;// configure for both edges
//  TIMER4_TAILR_R = TIMER_TAILR_M;  // max start value
//  TIMER4_TAPR_R = 0xFF;            // activate prescale, creating 24-bit
//  TIMER4_IMR_R |= TIMER_IMR_CAEIM; // enable capture match interrupt
//  TIMER4_ICR_R = TIMER_ICR_CAECINT;// clear timer4A capture match flag
//	TIMER4_CTL_R |= TIMER_CTL_TAEN;  // enable timer4A 16-b, +edge timing, interrupts
//  NVIC_PRI17_R = (NVIC_PRI17_R&0xFF00FFFF)|0x00400000; // priority 2
//  NVIC_EN2_R = 1<<(70-64);        // enable interrupt 70 in NVIC
//}

////------------Timer4_Read------------
//// read ultrasonic distance measurement
//// Input: none
//// Output: 0 if not ready, pulse width in 12.5ns time if ready
//uint32_t Timer4_Read(void){
//  if(Timer4Done){
//    return Timer4Pulse;
//  }
//  return 0;
//}
//void OutCRLF(void);

////-------------- Timer4A_Handler -----------
//// triggerred by both eages, calculate the time slice
//// data saved in the global variable Timer4Pulse
//// input None
//// output None
//void Timer4A_Handler(void){
//  TIMER4_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer3A capture match
//  if(ECHO4 == ECHO4HIGH){ // first
//    Timer4First = TIMER4_TAR_R;
//  }else{
//    Timer4Pulse = (Timer4First - TIMER4_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
//    Timer4Done = 1;
//  }
//}

////---------------------Timer5----------------------------
//uint32_t Timer5First,Timer5Done,Timer5Pulse;
////------------Timer0_Init------------
//// Initialize Timer0A in edge time mode to request interrupts on
//// the both edges of PB6 (T0CCP0).  The interrupt service routine
//// acknowledges the interrupt records the time.
//// PB7 GPIO output
//// Input: none
//// Output: none
//void Timer5_Init(void){
//  Timer5First = Timer5Done = Timer5Pulse = 0;
//  SYSCTL_RCGCTIMER_R |= 0x20;// activate timer5
//  SYSCTL_RCGCGPIO_R |= 0x08; // activate port D
//  while((SYSCTL_PRGPIO_R&0x0008) == 0){};// ready?
//  GPIO_PORTD_DIR_R &= ~0x40;       // make PD6 in
//  GPIO_PORTD_AFSEL_R |= 0x40;      // enable alt funct on PD6
//  GPIO_PORTD_DEN_R |= 0x40;        // enable PD6 as T5CCP0
//	GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xF0FFFFFF)+0x07000000;
//	GPIO_PORTD_AMSEL_R &= ~0x40;     // disable analog functionality on PD6
//  TIMER5_CTL_R &= ~TIMER_CTL_TAEN; // disable timer5A during setup
//  TIMER5_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
//  TIMER5_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);   // 24-bit capture         
//  TIMER5_CTL_R |= TIMER_CTL_TAEVENT_BOTH;// configure for both edges
//  TIMER5_TAILR_R = TIMER_TAILR_M;  // max start value
//  TIMER5_TAPR_R = 0xFF;            // activate prescale, creating 24-bit
//  TIMER5_IMR_R |= TIMER_IMR_CAEIM; // enable capture match interrupt
//  TIMER5_ICR_R = TIMER_ICR_CAECINT;// clear timer5A capture match flag
//	TIMER5_CTL_R |= TIMER_CTL_TAEN;  // enable timer5A 16-b, +edge timing, interrupts
//  NVIC_PRI23_R = (NVIC_PRI23_R&0x00FFFFFF)|0x40000000; // top 3 bits, priority 2
//  NVIC_EN2_R = 1<<(92-64);        // enable interrupt 92 in NVIC
//}

////------------Timer5_Read------------
//// read ultrasonic distance measurement
//// Input: none
//// Output: 0 if not ready, pulse width in 12.5ns time if ready
//uint32_t Timer5_Read(void){
//  if(Timer5Done){
//    return Timer5Pulse;
//  }
//  return 0;
//}
//void OutCRLF(void);

////-------------- Timer5A_Handler -----------
//// triggerred by both eages, calculate the time slice
//// data saved in the global variable Timer5Pulse
//// input None
//// output None
//void Timer5A_Handler(void){
//  TIMER5_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer5A capture match
//  if(ECHO5 == ECHO5HIGH){ // first
//    Timer5First = TIMER5_TAR_R;
//  }else{
//    Timer5Pulse = (Timer5First - TIMER5_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
//    Timer5Done = 1;
////		UART_OutUDec(Timer0Pulse);
////		OutCRLF();
//  }
//}

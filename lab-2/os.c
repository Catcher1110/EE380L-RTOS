// os.c
// Runs on LM4F120/TM4C123
// Jian Chu(jc86537) Huang Huang(hh26752)
// March 11, 2019
// Lab 3
// TA: Kishore Punniyamurthy (MW 1:30-3:00pm)
// Last version: March 14, 2019

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
#include "UART.h"

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
#define STACKSIZE   128      // number of 32-bit words in stack
int Id = 0;
Sema4Type BlockPt;

struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
	struct tcb *previous;
	int IdThread;
	int StateSleep;
	int Priority; // This is for Lab3
	int Active;
	int blockflag;
	Sema4Type *StateBlock;
	// int StateBlock; // This is for Lab3
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
tcbType *NextPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
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

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
  (*semaPt).Value = value;
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
	while((*semaPt).Value <= 0){
	  EnableInterrupts();
		OS_Suspend();
		DisableInterrupts();
	}
	(*semaPt).Value = (*semaPt).Value - 1;
	EndCritical(status);
}
#endif

#if block
void OS_Wait(Sema4Type *semaPt){
  uint32_t status = StartCritical();
	// DisableInterrupts();
	(*semaPt).Value = (*semaPt).Value - 1;
	while((*semaPt).Value < 0){
		(*RunPt).blockflag = 1;
		(*RunPt).StateBlock = semaPt;
		OS_Suspend();
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
	(*semaPt).Value = (*semaPt).Value + 1;
	EndCritical(status);
}
#endif

#if block
void OS_Signal(Sema4Type *semaPt){
  long status;
	status = StartCritical();
	// DisableInterrupts();
	(*semaPt).Value = (*semaPt).Value + 1;
	if ((*semaPt).Value<=0){
		int currentID = (*RunPt).IdThread;
		int nextID = (*tcbs[currentID].next).IdThread;
		while(tcbs[nextID].StateBlock != semaPt){
		  nextID = (*tcbs[nextID].next).IdThread;
		}
		int CID = nextID;
		nextID = (*tcbs[nextID].next).IdThread;
		
		int HighPriority = tcbs[CID].Priority;
		int NewID = CID;
		while(nextID != CID){
		  if((tcbs[nextID].StateBlock == semaPt) && (tcbs[nextID].Priority < HighPriority)){
			  HighPriority = tcbs[nextID].Priority;
				NewID = nextID;
			}
			nextID = (*tcbs[nextID].next).IdThread;
		}
//		if (tcbs[nextID].StateBlock == semaPt){}
//		else {
//			while(tcbs[nextID].StateBlock != semaPt && nextID != CID){
//			nextID = (*tcbs[nextID].next).IdThread;}
//			}	
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
	while((*semaPt).Value == 0){
	  EnableInterrupts();
		OS_Suspend();
		DisableInterrupts();
	}
	(*semaPt).Value = 0;
	EndCritical(status);
}
#endif

#if block
void OS_bWait(Sema4Type *semaPt){
  uint32_t status = StartCritical();
	// DisableInterrupts();
	(*semaPt).Value = (*semaPt).Value - 1;
	if ((*semaPt).Value < 0){
		(*RunPt).StateBlock = semaPt;
		(*RunPt).blockflag = 1;
		OS_Suspend();
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
	(*semaPt).Value = 1;
	EndCritical(status);
}
#endif

#if block
void OS_bSignal(Sema4Type *semaPt){
  long status;
	status = StartCritical();
	//DisableInterrupts();
	(*semaPt).Value = (*semaPt).Value + 1;
	if ((*semaPt).Value<=0){
		int currentID = (*RunPt).IdThread;
		int nextID = (*tcbs[currentID].next).IdThread;
		// int previousID = (*tcbs[currentID].previous).IdThread;
		while(tcbs[nextID].StateBlock != semaPt){
			nextID = (*tcbs[nextID].next).IdThread;
		}
		tcbs[nextID].StateBlock = NULL;
		(*RunPt).blockflag = 0;
	}
	
	//(*semaPt).Value = 1;
	EndCritical(status);
}
#endif

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
	while((*semaPt).Value <= 0){
	  EnableInterrupts();
		OS_Suspend();
		DisableInterrupts();
	}
	(*semaPt).Value = (*semaPt).Value - 1;
	EndCritical(status);
}

// ******** TCB_Signal ************
// increment semaphore 
// Just For TCB
// input:  pointer to a counting semaphore
// output: none
void TCB_Signal(Sema4Type *semaPt){
  uint32_t status = StartCritical();
	(*semaPt).Value = (*semaPt).Value + 1;
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
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void TCB_Fifo_Init(void){
  TCBGetPt = &TCBFifo[0];
	TCBPutPt = &TCBFifo[0];
	OS_InitSemaphore(&TCBCurrentSize, 0); 
	for(int i = 0; i < TCBSIZE; i++){
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
		tcbs[Id].IdThread = Id;
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
		status = StartCritical();
		Id = TCB_Fifo_Get();
		int currentID = (*RunPt).IdThread;
		while(tcbs[currentID].Active != 1){
		  currentID = (*tcbs[currentID].next).IdThread;
		}
		tcbs[Id].IdThread = Id;
		tcbs[Id].StateSleep = 0;
		tcbs[Id].Priority = priority;  // assign priority
		(*tcbs[currentID].next).previous = &tcbs[Id];
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
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
void (*PeriodicTask1)(void);   // user function
void (*PeriodicTask2)(void);   // user function
int TimerFlag = 0;
int OS_AddPeriodicThread(void(*task)(void), 
   unsigned long period, unsigned long priority){
	if(TimerFlag == 0){
	SYSCTL_RCGCTIMER_R |= 0x01;   // 0) activate timer0
  PeriodicTask1 = task;          // user function
  TIMER0_CTL_R = 0x00000000;    // 1) disable timer0A during setup
  TIMER0_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER0_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = period-1;    // 4) reload value
  TIMER0_TAPR_R = 0;            // 5) bus clock resolution
  TIMER0_ICR_R = 0x00000001;    // 6) clear timer0A timeout flag
  TIMER0_IMR_R = 0x00000001;    // 7) arm timeout interrupt
	switch(priority){
		case 0:
			NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x00000000; break; // 8) set priority 0
		case 1:
			NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x20000000; break; // 8) set priority 1
		case 2:
			NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; break; // 8) set priority 2
		case 3:
			NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x60000000; break; // 8) set priority 3
		case 4:
			NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x80000000; break; // 8) set priority 4
		case 5:
			NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0xA0000000; break; // 8) set priority 4
		case 6:
			NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0xC0000000; break; // 8) set priority 6
		case 7:
			NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0xE0000000; break; // 8) set priority 7
	}
// interrupts enabled in the main program after all devices initialized
// vector number 39, interrupt number 23
  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
  TIMER0_CTL_R = 0x00000001;    // 10) enable timer0A
	
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
	switch(priority){
		case 0:
			NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x00000000; break; // 8) set priority 0
		case 1:
			NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x20000000; break; // 8) set priority 1
		case 2:
			NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x40000000; break; // 8) set priority 2
		case 3:
			NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x60000000; break; // 8) set priority 3
		case 4:
			NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x80000000; break; // 8) set priority 4
		case 5:
			NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0xA0000000; break; // 8) set priority 5
		case 6:
			NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0xC0000000; break; // 8) set priority 6
		case 7:
			NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0xE0000000; break; // 8) set priority 7
	}
// interrupts enabled in the main program after all devices initialized
// vector number 39, interrupt number 23
  NVIC_EN0_R = 1<<23;           // 9) enable IRQ 23 in NVIC
  TIMER2_CTL_R = 0x00000001;    // 10) enable timer2A
  return 1;
}
}

// ***************** Timer0A_Handler ****************
// Handler for interrupt, increse the counter and execute user task
// Inputs:  None
// Outputs: None
void Timer0A_Handler(void){
	// GPIO_PORTF_DATA_R ^= 0x04;        // toggle the LED
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER0A timeout
  (*PeriodicTask1)();                // execute user task
	// GPIO_PORTF_DATA_R ^= 0x04;        // toggle the LED
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
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
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
  tcbType currentTcb = *RunPt;
	return currentTcb.IdThread;
}


// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
int Num=0;
void OS_Kill(void){
	DisableInterrupts();
	// Get the next thread to run
	// Scheduler();
	// double link for the previous and next thread
  int currentID = (*RunPt).IdThread;
	int nextID = (*tcbs[currentID].next).IdThread;
	int previousID = (*tcbs[currentID].previous).IdThread;
	tcbs[nextID].previous = &tcbs[previousID];
	tcbs[previousID].next = &tcbs[nextID];
	tcbs[currentID].Active = 0;
	// remove the thread from the TCB Fifo
	TCB_Fifo_Put(currentID);
	Num++;
	EnableInterrupts();
	// trigger pendsv
  //NVIC_INT_CTRL_R = 0x10000000;
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
	(*RunPt).StateSleep = sleepTime;
	int currentID = (*RunPt).IdThread;
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
	EnableInterrupts();
  NVIC_INT_CTRL_R = 0x10000000;
	DisableInterrupts();
}

void Scheduler(void){
	int status = StartCritical();
	int currentID = (*RunPt).IdThread;
	int NextID = (*tcbs[currentID].next).IdThread;
	// Find next active thread
	while(tcbs[NextID].Active != 1 || tcbs[NextID].blockflag !=0){
	  NextID = (*tcbs[NextID].next).IdThread;
	}
	int StartID = NextID; // Start ID for the Circle
	int NewID = NextID;  // Next Thread to be run
	int HighPriority = tcbs[NewID].Priority;
	NextID = (*tcbs[NextID].next).IdThread;
	while(NextID != StartID){
		if(tcbs[NextID].blockflag == 0){
	    if(tcbs[NextID].Active == 1 && tcbs[NextID].Priority < HighPriority){
			  //if(NextID != currentID){
		      HighPriority = tcbs[NextID].Priority;
			    NewID = NextID;
			  //}
		  }
		}
		NextID = (*tcbs[NextID].next).IdThread;
	}
	NextPt = &tcbs[NewID];	
	EndCritical(status);
}

#define PE0  (*((volatile unsigned long *)0x40024004))
int SysCounter = 0;
int SWCounter;
int TimeSlice;
int Counter = 0;
uint32_t lasttime;
void SysTick_Handler(void){
  int32_t status  = StartCritical();
	
	for(int i = 0; i < TCBSIZE; i++){
	  if(SleepMatrix[i] > TimeSlice){
		  SleepMatrix[i] -= TimeSlice;
		}else if(SleepMatrix[i] == TimeSlice){
		  SleepMatrix[i] = 0;
			tcbs[i].Active = 1;
			tcbs[i].StateSleep = 0;
		}else{}
	}
	// PE0 ^= 0x01;       // heartbeat
	//DisableInterrupts();
	Scheduler();
//	EnableInterrupts();
//	SysCounter++;
//	NVIC_INT_CTRL_R = 0x10000000;
  Counter++; 
	SWCounter++;
	SysCounter++;
//	if(SysCounter == 5000){
//	  UART_OutString("10 Second");
//	}
//	if(SysCounter%500 == 250){
//		uint32_t time = OS_Time();
//		int output = OS_TimeDifference(lasttime, time);
//	  UART_OutUDec(output);
//		UART_OutString("\n\r\n\r");
//		lasttime = time;
//	}
	NVIC_INT_CTRL_R = 0x10000000;
	// PE0 ^= 0x01;       // heartbeat
	//EnableInterrupts();
	EndCritical(status);
		
}


// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 50 MHz PLL
// input:  none
// output: none
void OS_Init(void){
  OS_DisableInterrupts();
  PLL_Init(Bus80MHz);         // set processor clock to 80 MHz
	NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0E00000; 
	// systick priority 6, pendsv priority 7
	NVIC_ST_RELOAD_R = 160000-1;// reload value
  NVIC_ST_CTRL_R = 0x07;     // enable SysTick with core clock and interrupts
	TCB_Fifo_Init();
	
}

void SysTick_Init(uint32_t period){
	long sr;
  sr = StartCritical();
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0F00000; // priority 6
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
  EndCritical(sr);
}
///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(uint32_t theTimeSlice){
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
	NVIC_ST_CURRENT_R = 0;      // any write to current clears it
// NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
	NVIC_ST_CTRL_R = 0x07;
	TimeSlice = theTimeSlice/80000;
  StartOS();                   // start on the first task
}

#define FIFOSIZE 32
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
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
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
	if(FIFOmutex.Value == 0){
	  return 0;
	}
	int RoomLeft = FIFOSIZE - CurrentSize.Value;
	if( RoomLeft > 0){
		return RoomLeft;
	}else if(RoomLeft == 0){
		return 0;
	}
	return 0;
}

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
	// uint32_t TimePart1 = Counter;
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

#define JITTERSIZE 64
long SW1MaxJitter;             // largest time jitter between interrupts in usec

unsigned long const SW1JitterSize=JITTERSIZE;
unsigned long SW1JitterHistogram[JITTERSIZE]={0,};

void OS_SW1Jitter(unsigned long period){
	unsigned static long LastTime;  // time at previous ADC sample
	unsigned long thisTime;         // time at current ADC sample
	long jitter;                    // time between measured and expected, in us
	static uint8_t ignore = 0;

	thisTime = OS_Time();       // current time, 12.5 ns

	ignore++;        // calculation finished
	if(ignore>1){    // ignore timing of first interrupt
		unsigned long diff = OS_TimeDifference(LastTime,thisTime);
		if(diff>period){
			jitter = (diff-period+4)/8;  // in 0.1 usec
		}else{
			jitter = (period-diff+4)/8;  // in 0.1 usec
		}
		if(jitter > SW1MaxJitter){
			SW1MaxJitter = jitter; // in usec
		}       // jitter should be 0
		if(jitter >= SW1JitterSize){
			jitter = JITTERSIZE-1;
		}
		SW1JitterHistogram[jitter]++; 
	}
	LastTime = thisTime;
}

unsigned long const SW2JitterSize=JITTERSIZE;
long SW2MaxJitter;             // largest time jitter between interrupts in usec
unsigned long SW2JitterHistogram[JITTERSIZE]={0,};

void OS_SW2Jitter(unsigned long period){
	unsigned static long LastTime;  // time at previous ADC sample
	unsigned long thisTime;         // time at current ADC sample
	long jitter;                    // time between measured and expected, in us
	static uint8_t ignore = 0;

	thisTime = OS_Time();       // current time, 12.5 ns

	ignore++;        // calculation finished
	if(ignore>1){    // ignore timing of first interrupt
		unsigned long diff = OS_TimeDifference(LastTime,thisTime);
		if(diff>period){
			jitter = (diff-period+4)/8;  // in 0.1 usec
		}else{
			jitter = (period-diff+4)/8;  // in 0.1 usec
		}
		if(jitter > SW2MaxJitter){
			SW2MaxJitter = jitter; // in usec
		}       // jitter should be 0
		if(jitter >= SW2JitterSize){
			jitter = JITTERSIZE-1;
		}
		SW2JitterHistogram[jitter]++; 
	}
	LastTime = thisTime;
}

// OS.c
// Runs on LM4F120/TM4C123
// Provide a function that adds a periodic thread for ADC
// Jian Chu(jc86537) Huang Huang(hh26752)
// Febu 4, 2019
// Lab 1
// TA: Kishore Punniyamurthy (MW 1:30-3:00pm)
// Last version: Febu 13, 2019
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

extern uint32_t COUNTER;      // global variable counter
void (*PeriodicTask)(void);   // user function

// ***************** OS_AddPeriodicThread ****************
// Add a thread to generate periodic interrupt, using TIMER0
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq), 32 bits
//					priority is the value to be specified in the NVIC for this thread
// Outputs: COUNTER
int OS_AddPeriodicThread(void(*task)(void), uint32_t period, uint32_t priority){
	SYSCTL_RCGCTIMER_R |= 0x01;   // 0) activate timer0
  PeriodicTask = task;          // user function
  TIMER0_CTL_R = 0x00000000;    // 1) disable timer0A during setup
  TIMER0_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER0_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = period-1;    // 4) reload value
  TIMER0_TAPR_R = 0;            // 5) bus clock resolution
  TIMER0_ICR_R = 0x00000001;    // 6) clear timer0A timeout flag
  TIMER0_IMR_R = 0x00000001;    // 7) arm timeout interrupt
	switch(priority){
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
  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
  TIMER0_CTL_R = 0x00000001;    // 10) enable timer0A

  SYSCTL_RCGCGPIO_R |= 0x00000020;         // activate port F
  GPIO_PORTF_DIR_R |= 0x04;                // make PF2 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x04;             // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04;                // enable digital I/O on PF2
                                           // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;                  // disable analog functionality on PF
  GPIO_PORTF_DATA_R &= ~0x04;              // turn off LED
	
	return COUNTER;
}

// ***************** OS_ClearPeriodicTime ****************
// Reset the counter to 0
// Inputs:  None
// Outputs: None
void OS_ClearPeriodicTime(void){
	COUNTER = 0;
}

// ***************** OS_ReadPeriodicTime ****************
// Read the counter
// Inputs:  None
// Outputs: Counter
uint32_t OS_ReadPeriodicTime(void){
	return COUNTER;
}

// ***************** Timer0A_Handler ****************
// Handler for interrupt, increse the counter and execute user task
// Inputs:  None
// Outputs: None
void Timer0A_Handler(void){
	GPIO_PORTF_DATA_R ^= 0x04;        // toggle the LED
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER0A timeout
	COUNTER++;												// increse counter when interrupt
  (*PeriodicTask)();                // execute user task
	GPIO_PORTF_DATA_R ^= 0x04;        // toggle the LED
}

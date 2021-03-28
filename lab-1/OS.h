// filename **********OS.H***********
// Operating System for Labs 01
// Runs on LM4F120/TM4C123
// Jian Chu(jc86537) Huang Huang(hh26752)
// Febu 4, 2019
// Lab 1
// TA: Kishore Punniyamurthy (MW 1:30-3:00pm)
// Last version: Febu 13, 2019
uint32_t COUNTER = 0; // global variable counter

// ***************** OS_AddPeriodicThread ****************
// Add a thread to generate periodic interrupt, using TIMER0
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq), 32 bits
//          priority is the value to be specified in the NVIC for this thread
// Outputs: COUNTER
int OS_AddPeriodicThread(void(*task)(void), uint32_t period, uint32_t priority);

// ************ OS_ClearPeriodicTime ****************
// Reset the counter to 0
// Inputs: None
// Output: None
void OS_ClearPeriodicTime(void);

// ************ OS_ReadPeriodicTime ******************
// Read the counter of the thread
// Inputs: None
// Output: counter value
uint32_t OS_ReadPeriodicTime(void);

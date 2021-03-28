// ADC.h
// Runs on LM4F120/TM4C123
// Provide a function that adds a periodic thread for ADC
// Jian Chu(jc86537) Huang Huang(hh26752)
// Febu 4, 2019
// Lab 1
// TA: Kishore Punniyamurthy (MW 1:30-3:00pm)
// Last version: Febu 13, 2019


// Initialize the ADC collect status
uint32_t ADC_collect_status=1;
// *********** ADC0_InitTimer2ATriggerSeq3 ******
// Initialize the ADC0 with timer2A, store the data in sequence 3
// Inputs:  channelNum the channel which we want to use
//          period the interrupt period
// Output:  None
void ADC0_InitTimer2ATriggerSeq3(uint8_t channelNum, uint32_t period);

// *************** ADC0Seq3_Handler **************
// Handler function for ADC0Seq3 interrupt
// Inputs:  None
// Output:  None
void ADC0Seq3_Handler(void);

// **************** ADC_In *******************
// Collect the data from the ADC
// Inputs:  None
// Output:  None
uint32_t ADC_In(void);

// ***************** ADC_Collect ****************
// Collect the ADC value through specific channel, frequency
// and store the samples in specific buffer
// Inputs:  channelNum is the channel number (0-11)
//          fs is the frequency of the collect
//          buffer is the location stored the data
//          numberOfSamples is the number of samples we want to store
// Outputs: None
void ADC_Collect(uint32_t channelNum, uint32_t fs, void(*task));

//****************** ADC_Status ******************
// Reture the status of the ADC conversion
// Input: None
// Output: 1 for the complete, 0 for otherwise
int ADC_Status(void);

void ADC_Init(uint8_t channelNum);



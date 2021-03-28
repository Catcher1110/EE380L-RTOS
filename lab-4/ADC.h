/**
 * @file      ADC.h
 * @brief     Provide functions that initialize ADC0
 * @details   Runs on LM4F120/TM4C123.
 * Provide functions that initialize ADC0. 
 * SS0 triggered by Timer0, and calls a user function.
 * SS3 triggered by software and trigger a conversion, wait for it to finish, and return the result.
 * @version   V1.0
 * @author    Valvano
 * @copyright Copyright 2017 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      March 9, 2017

 ******************************************************************************/


/**
 * @details  This initialization function sets up the ADC 
 * Any parameters not explicitly listed below are not modified.
 * @note Max sample rate: <=125,000 samples/second<br>
 Sequencer 0 priority: 1st (highest)<br>
 Sequencer 1 priority: 2nd<br>
 Sequencer 2 priority: 3rd<br>
 Sequencer 3 priority: 4th (lowest)<br>
 SS3 triggering event: software trigger<br>
 SS3 1st sample source: programmable using variable 'channelNum' [0:11]<br>
 SS3 interrupts: enabled but not promoted to controller
 * @param  channelNum Channel from 0 to 11
 * @return none
 * @brief  Initialize ADC sequencer 3
 */
void ADC_Init(unsigned char channelNum);


/**
 * @details  Busy-wait Analog to digital conversion on SS3
 * @param  none
 * @return 12-bit result of ADC conversion
 * @brief  Sample ADC sequencer 3
 */
unsigned long ADC_In(void);

/**
 * @details Sample the ADC using Timer0 hardware triggering using SS0.
 * This function initiates the sampling, does not wait for completion.
 * Task is user function to process or send data .
 * @param  channelNum Channel from 0 to 11
 * @param  fs sampling rate in Hz
 * @param  task pointer to a user function to process the ADC data
 * @return 0 on error, 1 if OK
 * @brief  Timer Analog to digital conversion on SS0
 */
int ADC_Collect(unsigned int channelNum, unsigned int fs, void(*task)(unsigned long));

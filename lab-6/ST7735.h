/**
 * @file      ST7735.h
 * @brief     Low level drivers for the ST7735
 * @details   Runs on LM4F120/TM4C123. 
 * Low level drivers for the ST7735 160x128 LCD based off of
 * the file described above. This version coexists with the SDC
 * 
 * @version   V1.0
 * @author    Valvano
 * @copyright Copyright 2017 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      March 9, 2017

 ******************************************************************************/

/***************************************************
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/


/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2017

 Copyright 2017 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) unconnected
// Data/Command (pin 4) connected to PA6 (GPIO)
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

#ifndef _ST7735H_
#define _ST7735H_

// some flags for ST7735_InitR()
enum initRFlags{
  none,
  INITR_GREENTAB,
  INITR_REDTAB,
  INITR_BLACKTAB
};



// Color definitions
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0xF800
#define ST7735_RED     0x001F
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0xFFE0
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0x07FF
#define ST7735_WHITE   0xFFFF

// Initialization for ST7735B screens
void ST7735_InitB(void);

// Initialization for ST7735R screens (green or red tabs)
// ST7735_InitR(INITR_GREENTAB);
void ST7735_InitR(enum initRFlags option);

//void setAddrWindow(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1); // function is now private

//void pushColor(unsigned short color); // function is now private

void ST7735_DrawPixel(short x, short y, unsigned short color);

void ST7735_DrawFastVLine(short x, short y, short h, unsigned short color);

void ST7735_DrawFastHLine(short x, short y, short w, unsigned short color);

void ST7735_FillScreen(unsigned short color);

void ST7735_FillRect(short x, short y, short w, short h, unsigned short color);

unsigned short ST7735_Color565(unsigned char r, unsigned char g, unsigned char b);

unsigned short ST7735_SwapColor(unsigned short x);

void ST7735_DrawBitmap(short x, short y, const unsigned short *image, short w, short h);

void ST7735_DrawCharS(short x, short y, char c, short textColor, short bgColor, unsigned char size);

void ST7735_DrawChar(short x, short y, char c, short textColor, short bgColor, unsigned char size);

//------------ST7735_OutString------------
// String draw function.  
// 16 rows (0 to 15) and 21 characters (0 to 20)
// Requires (11 + size*size*6*8) bytes of transmission for each character
// Input: x         columns from the left edge (0 to 20)
//        y         rows from the top edge (0 to 15)
//        pt        pointer to a null terminated string to be printed
//        textColor 16-bit color of the characters
// bgColor is Black and size is 1
// Output: number of characters printed
unsigned long ST7735_OutString(unsigned short x, unsigned short y, char *pt, short textColor);
//------------ST7735_Message------------
// String draw and number output.  
// Input: device  0 is on top, 1 is on bottom
//        line    row from top, 0 to 7 for each device
//        pt      pointer to a null terminated string to be printed
//        value   signed integer to be printed
void ST7735_Message (unsigned long d, unsigned long l, char *pt, long value);

void ST7735_SetRotation(unsigned char m);

void ST7735_InvertDisplay(int i);

#endif

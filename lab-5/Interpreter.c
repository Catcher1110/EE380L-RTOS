// *************Interpreter.c**************
// EE445M/EE380L.6 Lab 4 solution
// high level OS functions
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 3/9/17, valvano@mail.utexas.edu
#include "ST7735.h"
#include "os.h"
#include "UART.h"
#include "efile.h"
#include "eDisk.h"
#include <string.h> 
#include <stdio.h>


void Interpreter(void) {
	int format;
	char string[30];
  UART_Init();
	UART_OutString("What you want to do(0 format, 1 directory, 2 print, 3 delete): ");
	format = UART_InUDec();
	switch(format){
	  case 0:
			eFile_Format();break;
		case 1:
			eFile_Directory(&UART_OutChar);break;
		case 2:
		  UART_OutString("Which file you want to print? ");
		  UART_InString(string,29);
			//eFile_Print(string);break;
		case 3:
			UART_OutString("Which file you want to delete? ");
		  UART_InString(string,29);
			eFile_Delete(string);break;
	}
}


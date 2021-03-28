/***************************************************
Modified by Jian Chu and Huang Huang for EE 380L, Spring 2019
****************************************************/

//*****************************************************************************
//
// cmdline.c - Functions to help with processing command lines.
//
// Copyright (c) 2007-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.0.1.11577 of the Tiva Utility Library.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup cmdline_api
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "cmdline.h"
#include "UART2.h"
#include "ST7735.h"
#include "os.h"
#include "efile.h"

//*****************************************************************************
//
// Defines the maximum number of arguments that can be parsed.
//
//*****************************************************************************
#ifndef CMDLINE_MAX_ARGS
#define CMDLINE_MAX_ARGS        8
#endif

//*****************************************************************************
//
// An array to hold the pointers to the command line arguments.
//
//*****************************************************************************
static char *g_ppcArgv[CMDLINE_MAX_ARGS + 1];
int i;
//*****************************************************************************
//
//! Process a command line string into arguments and execute the command.
//!
//! \param pcCmdLine points to a string that contains a command line that was
//! obtained by an application by some means.
//!
//! This function will take the supplied command line string and break it up
//! into individual arguments.  The first argument is treated as a command and
//! is searched for in the command table.  If the command is found, then the
//! command function is called and all of the command line arguments are passed
//! in the normal argc, argv form.
//!
//! The command table is contained in an array named <tt>g_psCmdTable</tt>
//! containing <tt>tCmdLineEntry</tt> structures which must be provided by the
//! application.  The array must be terminated with an entry whose \b pcCmd
//! field contains a NULL pointer.
//!
//! \return Returns \b CMDLINE_BAD_CMD if the command is not found,
//! \b CMDLINE_TOO_MANY_ARGS if there are more arguments than can be parsed.
//! Otherwise it returns the code that was returned by the command function.
//
//*****************************************************************************
int
CmdLineProcess(char *pcCmdLine)
{
    char *pcChar;
    uint_fast8_t ui8Argc;
    bool bFindArg = true;
    tCmdLineEntry *psCmdEntry;

    //
    // Initialize the argument counter, and point to the beginning of the
    // command line string.
    //
    ui8Argc = 0;
    pcChar = pcCmdLine;

    //
    // Advance through the command line until a zero character is found.
    //
    while(*pcChar)
    {
        //
        // If there is a space, then replace it with a zero, and set the flag
        // to search for the next argument.
        //
        if(*pcChar == ' ')
        {
            *pcChar = 0;
            bFindArg = true;
        }

        //
        // Otherwise it is not a space, so it must be a character that is part
        // of an argument.
        //
        else
        {
            //
            // If bFindArg is set, then that means we are looking for the start
            // of the next argument.
            //
            if(bFindArg)
            {
                //
                // As long as the maximum number of arguments has not been
                // reached, then save the pointer to the start of this new arg
                // in the argv array, and increment the count of args, argc.
                //
                if(ui8Argc < CMDLINE_MAX_ARGS)
                {
                    g_ppcArgv[ui8Argc] = pcChar;
                    ui8Argc++;
                    bFindArg = false;
                }

                //
                // The maximum number of arguments has been reached so return
                // the error.
                //
                else
                {
                    return(CMDLINE_TOO_MANY_ARGS);
                }
            }
        }

        //
        // Advance to the next character in the command line.
        //
        pcChar++;
    }

    //
    // If one or more arguments was found, then process the command.
    //
    if(ui8Argc)
    {
        //
        // Start at the beginning of the command table, to look for a matching
        // command.
        //
        psCmdEntry = &g_psCmdTable[0];

        //
        // Search through the command table until a null command string is
        // found, which marks the end of the table.
        //
        while(psCmdEntry->pcCmd)
        {
            //
            // If this command entry command string matches argv[0], then call
            // the function for this command, passing the command line
            // arguments.
            //
            if(!strcmp(g_ppcArgv[0], psCmdEntry->pcCmd))
            {
                return(psCmdEntry->pfnCmd(ui8Argc, g_ppcArgv));
            }

            //
            // Not found, so advance to the next entry.
            //
            psCmdEntry++;
        }
    }

    //
    // Fall through to here means that no matching command was found, so return
    // an error.
    //
    return(CMDLINE_BAD_CMD);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

// Used for UART commands
char HelpSD[] = "SD card commands";
void OutCRLF(void);

void CommandSD(uint_fast8_t ui8Argc, char *g_ppcArgv[]) {
    int32_t result;
		char ch;
		int j = 0;
	
		if (ui8Argc < 2) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd");
        return;    
    }
    
    if (strcmp(g_ppcArgv[1],"format") == 0) {
			// Doesn't care if file in use
			result = eFile_Format();
    }
		else if (strcmp(g_ppcArgv[1],"init") == 0){
		  result = eFile_Init();
		}
		else if (strcmp(g_ppcArgv[1],"close") == 0){
		  result = eFile_Close();
		}
		else if (strcmp(g_ppcArgv[1],"block")==0){
			j = 0;
			if(eFile_WOpen(g_ppcArgv[2])) {
				ST7735_OutString(0, 1, "ERROR: FILE NOT FOUND", ST7735_WHITE);
				return;
			}
			else {
				while(j < 1000) {;
					eFile_Write('a');
					j++;
				}
			}
			eFile_WClose();
		}
    else if (strcmp(g_ppcArgv[1],"dir") == 0) {
      // call directory function
			result = eFile_Directory(&UART_OutString);
    }
		else if (strcmp(g_ppcArgv[1],"touch") == 0) {
      if (ui8Argc < 3) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd touch");
        return;    
			}
			
			eFile_Create(g_ppcArgv[2]);
    }
		else if (strcmp(g_ppcArgv[1],"write") == 0) {
      if (ui8Argc < 4) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd write");
        return;    
			}
			
			j = 0;
			if(eFile_WOpen(g_ppcArgv[2])) {
				ST7735_OutString(0, 1, "ERROR: FILE NOT FOUND", ST7735_WHITE);
				return;
			}
			else {
        // call write function
				while(g_ppcArgv[3][j]) {
					eFile_Write(g_ppcArgv[3][j]);
					j++;
				}
			}
			eFile_WClose();
				
    }
		else if (strcmp("print", g_ppcArgv[1]) == 0) {
			if (ui8Argc < 3) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd print");
        return;    
			}
			// call read open function
			result = eFile_ROpen(g_ppcArgv[2]);
			if(result) {
				ST7735_OutString(0, 1, "File not found", ST7735_WHITE);
				return;
			}
			while(1){
        // break if read the end of the file
				if(eFile_ReadNext(&ch)){
				  break;
				}
				UART_OutChar(ch);
			}
			eFile_RClose();
    }
		else if (strcmp(g_ppcArgv[1],"rm") == 0) {
			if (ui8Argc < 3) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd rm");
        return;    
			}
			result = eFile_Delete(g_ppcArgv[2]);
			// Call delete file
			// error if no file with that name
			if(result) {
				UART_OutString("ERROR: NO FILE NAMED "); UART_OutString(g_ppcArgv[2]);
			}
    }
		else if (strcmp(g_ppcArgv[1],"help") == 0) {
			OutCRLF();
			UART_OutString("format\r\ndirectory\r\nprint\r\n\t1)FILENAME\r\ndelete\r\n\t1)FILENAME");
			OutCRLF();
    }
    else {
        UART_OutString("command sd argument not recognized");
    }
}

// Command Table as defined by Tivaware
tCmdLineEntry g_psCmdTable[] = {
		{ "sd", CommandSD, HelpSD}
};

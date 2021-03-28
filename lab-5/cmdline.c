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
//#include "efile.h"
#include "ff.h"
#include "loader.h"

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
char HelpLoad[] = "Load Process commands";
void OutCRLF(void);
FIL Handle,Handle2;
FRESULT MountFresult;
FRESULT Fresult;
UINT successfulreads, successfulwrites;
static FATFS g_sFatFs;

static const ELFSymbol_t symtab[] = {
{ "ST7735_Message", ST7735_Message }
};
ELFEnv_t env = { symtab, 1 };

void CommandSD(uint_fast8_t ui8Argc, char *g_ppcArgv[]) {
    int32_t result;
		char ch;
		int j = 0;
	
		if (ui8Argc < 2) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd");
        return;    
    }
    
    if (strcmp(g_ppcArgv[1],"mount") == 0) {
			MountFresult = f_mount(&g_sFatFs, "", 0);
      if(MountFresult){
			  OutCRLF();
        UART_OutString("Mount ERROR");
        return;
			}
    }
		else if (strcmp(g_ppcArgv[1],"close") == 0){
		  Fresult = f_close(&Handle);
		}
		else if (strcmp(g_ppcArgv[1],"touch") == 0) {
      if (ui8Argc < 3) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd touch");
        return;    
			}	
      Fresult = f_open(&Handle, g_ppcArgv[2], FA_CREATE_NEW);
			if(Fresult == FR_OK){
			  OutCRLF();
        UART_OutString("created file successful");
        return;
			}
			else if(Fresult == FR_EXIST){
			  OutCRLF();
        UART_OutString("file already exist");
			}
			else{
			  OutCRLF();
        UART_OutString("created file fail");
        return;
			}
			//eFile_Create(g_ppcArgv[2]);
    }
		else if (strcmp(g_ppcArgv[1],"open") == 0) {
      if (ui8Argc < 3) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd open");
        return;    
			}	
      Fresult = f_open(&Handle, g_ppcArgv[2], FA_OPEN_EXISTING);
			if(Fresult == FR_OK){
			  OutCRLF();
        UART_OutString("opened file successful");
        return;
			}else{
			  OutCRLF();
        UART_OutString("cannot open file. ERROR: ");
				UART_OutUDec(Fresult);
        return;
			}
			//eFile_Create(g_ppcArgv[2]);
    }
		else if (strcmp(g_ppcArgv[1],"write") == 0) {
      if (ui8Argc < 4) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd write");
        return;    
			}
			
			j = 0;
			Fresult = f_open(&Handle, g_ppcArgv[2], FA_WRITE);
			if(Fresult != FR_OK) {
				OutCRLF();
        UART_OutString("WRITE ERROR: ");
				UART_OutUDec(Fresult);
				ST7735_OutString(0, 1, "ERROR: FILE NOT FOUND", ST7735_WHITE);
				return;
			}
			else {
				Fresult = f_lseek(&Handle, Handle.fsize);
				while(g_ppcArgv[3][j]) {
					Fresult = f_write(&Handle, &g_ppcArgv[3][j], 1, &successfulwrites);
					if(Fresult != FR_OK){
					  OutCRLF();
						UART_OutString("WRITE ERROR: ");
						UART_OutUDec(Fresult);
					}
					j++;
				}
			}
			Fresult = f_close(&Handle);
    }
		else if (strcmp(g_ppcArgv[1],"print") == 0) {
			if (ui8Argc < 3) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd print");
        return;    
			}		
			Fresult = f_open(&Handle, g_ppcArgv[2], FA_READ);
			if(Fresult != FR_OK) {
				OutCRLF();
				UART_OutString("OPEN FILE ERROR: ");
				UART_OutUDec(Fresult);
				ST7735_OutString(0, 1, "File not found", ST7735_WHITE);
				return;
			}
			while(1){
				Fresult = f_read(&Handle, &ch, 1, &successfulreads);
				if((Fresult == FR_OK) && (successfulreads == 1)){
					UART_OutChar(ch);
				}
				else if((Fresult == FR_OK) && (successfulreads != 1)){
				  break;
				}
				else{
				  OutCRLF();
				  UART_OutString("READ FILE ERROR: ");
				  UART_OutUDec(Fresult);
				  break;
				}
			}
			Fresult = f_close(&Handle);
    }
		else if (strcmp(g_ppcArgv[1],"load") == 0) {
			// UART_OutString("Load here");
			result = exec_elf(g_ppcArgv[2], &env);
			if(result == 2){
				UART_OutString("Process Full");
			}
    }
		else if (strcmp(g_ppcArgv[1],"rm") == 0) {
			if (ui8Argc < 3) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command sd rm");
        return;    
			}		
			Fresult = f_unlink(g_ppcArgv[2]);
			// Call delete file
			// error if no file with that name
			if(Fresult != FR_OK) {
				UART_OutString("CLOSE ERROR: NO FILE NAMED "); UART_OutString(g_ppcArgv[2]);
			}
			// error if any files currently open
			//UART_OutString("ERROR: FILES CURRENTLY OPEN. CLOSE ALL FILES AND TRY AGAIN");
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
//    { "os", CommandOS, HelpOS },
//		{	"echo", CommandEcho, HelpEcho},
		{ "sd", CommandSD, HelpSD}
//		{ "load", CommandLoad, HelpLoad}
};

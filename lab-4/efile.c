// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk 
// Jonathan W. Valvano 3/9/17 modified by Jian Chu & Huang Huang 3/31/19

#include <string.h>
#include "edisk.h"
#include "UART2.h"
#include <stdio.h>
#include <stdint.h>
#include "ST7735.h"
#include <stdlib.h>

#define SUCCESS 0
#define FAIL 1
#define ALLOCATION_SIZE 4096
#define DIRECTORY_SIZE 15
#define NAME_SIZE 10

const uint16_t directory_block = 0x0000;
const uint16_t allocation_block = ALLOCATION_SIZE;
uint16_t freespace = 258;
typedef struct {
	char name[NAME_SIZE];    // name for the file
	uint16_t starter;        // starter block for the file
	uint16_t size;           // size of the file in byte
	uint16_t open_flag;      // a flag the record whether it is opened
	uint16_t current_block;  // the last block the file used
	uint16_t current_byte;   // the next byte location the file will use
}DIRECT;

BYTE wbuffer[512];
int readtag=1; // 1 can be open, 0 already has a file opened

struct ReadBuffer{
  BYTE buffer[512];  // buffer to record the block
	int blockid;       // the current block it read
	int byteid;        // the current byte it read
	int last_block;    // the last block the file uesd
	int last_byte;     // the bext byte the file will use 
};
struct ReadBuffer readbuffer;

struct DIRECT{
	char name[10];
	int starter;
	int size;
	int open_flag;
	int current_block;
	int current_byte;
};

// ------------- directory_init ------------
// initialize the directory
// set all the files NULL or 0
void directory_init(DIRECT *directory){
	int i;
	for (i =0; i<DIRECTORY_SIZE;i++){
		strcpy(directory[i].name," ");
		directory[i].starter = NULL;
		directory[i].size = 0;
		directory[i].open_flag = 0;
		directory[i].current_block = NULL;
		directory[i].current_byte = NULL;
	}
}

int16_t allocation[ALLOCATION_SIZE];
DIRECT directory[DIRECTORY_SIZE];


// -------------- allocation_init -----------
// initialize the allocation table
// Give all the freespace a magic number
// Give the first block the number 0 (directory)
void allocation_init(int16_t *allocation){
	int i;
	for (i =0; i<ALLOCATION_SIZE ;i++){
		allocation[i] = freespace;
	}
	allocation[0]=0;

}

//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
int eFile_Init(void){ // initialize file system
  // initialize the SD card
	if (!eDisk_Init(0))
	{
		// read the allocation table from the SD card
		if (!eDisk_Read(0, (uint8_t *)&allocation, allocation_block, 1))
		{
			// read the directory from SD card
			if (!eDisk_Read(0, (uint8_t *)&directory, directory_block, 1))
			{
				ST7735_OutString(0, 1, "Initilization success", ST7735_Color565(255, 0, 0));
				return SUCCESS;
			}
			ST7735_OutString(0, 2, "Initilization fail 1", ST7735_Color565(255, 0, 0));
			return FAIL;}		
		ST7735_OutString(0, 3, "Initilization fail 2", ST7735_Color565(255, 0, 0));
		return FAIL;
	}	
	ST7735_OutString(0, 4, "Initilization fail 3", ST7735_Color565(255, 0, 0));
  return FAIL;
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void){ // erase disk, add format
	eDisk_Init(0);
	directory_init(directory);   // get the empty directory
	allocation_init(allocation); // get the empty allocation table
	eDisk_Write(0,(uint8_t *) &directory, directory_block,1);  
	// write the empty directory to SD card
	eDisk_Write(0,(uint8_t *) &allocation, allocation_block,1);
	// write the empty allocation table to SD card
	ST7735_OutString(0, 0, "format success", ST7735_Color565(255, 0, 0));
  return SUCCESS;   // OK
}


//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create( char name[]){  // create new file, make it empty 
  int i = 0;
	int starter = 1;
	int j=0;
	// search the file with the same name
	for (j=0;j<DIRECTORY_SIZE;j++){
		if (strcmp(directory[j].name,name)==0)
		{
		  ST7735_OutString(0, 0, "File already exits", ST7735_Color565(255, 0, 0));
			UART_OutString("File already exists \n\r");
			return FAIL;	
		}		
	}
	// search the directory to store the file name
	for (i=0;i<DIRECTORY_SIZE;i++){
		if (directory[i].starter==NULL)
		{break;}
		if (i==DIRECTORY_SIZE-1){ // the SD card is full now
		  ST7735_OutString(0, 0, "disk full", ST7735_Color565(255, 0, 0));
			return FAIL;	
		}		
	}
	// search the allocation table to store the file
	strcpy(directory[i].name, name);
	for (starter=0;starter<ALLOCATION_SIZE;starter++){
		if (allocation[starter] == freespace)
		{break;}
		if (starter==ALLOCATION_SIZE-1){
			ST7735_OutString(0, 0, "disk full", ST7735_Color565(255, 0, 0));
			return FAIL;	
		}
	}
	// record the file information in the directory
	allocation[starter] = 0;
	directory[i].starter = starter;
	directory[i].current_block = starter;
	directory[i].current_byte = 0;
	ST7735_OutString(0, 0, "create success", ST7735_Color565(255, 0, 0));
	// write the allocation table and directory back to SD card
  if ((!eDisk_Write(0, (uint8_t *)&allocation, ALLOCATION_SIZE-1, 1))&&(!eDisk_Write(0, (uint8_t *)&directory, DIRECTORY_SIZE, 1)))
  {
		ST7735_OutString(0, 0, "write back success", ST7735_Color565(255, 0, 0));
		return SUCCESS;
	}
	return FAIL;	     
}

//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WOpen(char name[]){      // open a file for writing 
	int fileid = 0;
	DRESULT ReadStatus;
	int location_block; // current location (block)

  if(readtag != 1){return FAIL;} // check whether there is a file opened for read
	for(fileid = 0; fileid < DIRECTORY_SIZE; fileid++){
	  if(!strcmp(directory[fileid].name, name)){break;} // find the file
		if(fileid == DIRECTORY_SIZE-1){ // do not find the file
		  ST7735_OutString(0, 3, "WOpen: no such file", ST7735_Color565(255, 0, 0));
	    return FAIL;
		}
	}
	// set the open flag for the file
	directory[fileid].open_flag = 1;
	// set the readtag
	readtag=0;
	ST7735_OutString(0, 3, "WOpen: open success", ST7735_Color565(255, 0, 0));
	
	// if(readtag ==1){ST7735_OutString(0, 0, "Write: no open file", ST7735_Color565(255, 0, 0));
	// return FAIL;} // return FAIL if there is no file opened
  
	// get the last block the file used
	location_block = directory[fileid].current_block;
  // read the last block into the buffer
	ReadStatus = eDisk_Read(0, wbuffer, location_block, 1);
	if(ReadStatus == FAIL){ST7735_OutString(0, 0, "Write: read file", ST7735_Color565(255, 0, 0));return FAIL;}
  return SUCCESS;
}

//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Write(char data){
	int opened_fileid = 0; // ID of the opened file;
	int location_block; // current location (block)
	int location_byte; // current sector
  int next_block = 0; // data will write in this block
	DRESULT WriteStatus;
	DRESULT ReadStatus;
	// check whether there is a file opened
	if(readtag ==1){ST7735_OutString(0, 0, "Write: no open file", ST7735_Color565(255, 0, 0));
	return FAIL;} // return FAIL if there is no file opened
	while(directory[opened_fileid].open_flag != 1){opened_fileid++;} // get the ID of opened file
	location_block = directory[opened_fileid].current_block; // get the last block of the file
	location_byte = directory[opened_fileid].current_byte;   // get the next block the file will use
  // if it write to the end of the block, find another available block
	if(location_byte == 512){
		WriteStatus = eDisk_Write(0, wbuffer, location_block, 1);
	  while(allocation[next_block] != freespace){
			if(next_block == ALLOCATION_SIZE){return FAIL;} // all blocks are used, cannot write
		  next_block++;
		}
		allocation[location_block] = next_block;
		allocation[next_block] = 0;
		location_block = next_block;
		location_byte = 0;
		directory[opened_fileid].current_byte = location_byte;
		directory[opened_fileid].current_block = location_block;
		ReadStatus = eDisk_Read(0, wbuffer, location_block, 1);
  }
	// record the data into the buffer, increse the size of the file
  wbuffer[location_byte] = data;
	location_byte++;
	directory[opened_fileid].size++;
	directory[opened_fileid].current_byte = location_byte;
  return SUCCESS; 
}


//---------- eFile_Close-----------------
// Deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently open)
int eFile_Close(void){
	DRESULT WriteDirStatus, WriteAllStatus;
	// write back the directory
	WriteDirStatus = eDisk_Write(0, (uint8_t *)&directory, 0, 1);
	if(WriteDirStatus){ST7735_OutString(0, 0, "Close write fail", ST7735_Color565(255, 0, 0));
	return FAIL;}
	// write back the allocation table
	WriteAllStatus = eDisk_Write(0, (uint8_t *)&allocation, ALLOCATION_SIZE-1, 1);
	if(WriteAllStatus){return FAIL;}
  return SUCCESS;     
}

//---------- eFile_WClose-----------------
// close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WClose(void){ // close the file for writing
	DRESULT WriteStatus;
	int opened_fileid;
	int location_block;
	if(readtag == 1){ST7735_OutString(0, 11, "no open file", ST7735_Color565(255, 0, 0));
	return FAIL;} // no file opened
	// search the file opened
	for(opened_fileid = 0;opened_fileid < DIRECTORY_SIZE;opened_fileid++){
	  if(directory[opened_fileid].open_flag == 1){
		  break;
		}
		if(opened_fileid == DIRECTORY_SIZE-1){
		  ST7735_OutString(0, 4, "Write Close: No opended file", ST7735_Color565(255, 0, 0));
		  return FAIL;
		}
	}
	// close the file
	directory[opened_fileid].open_flag = 0;
  // get the last block the file used 
	location_block = directory[opened_fileid].current_block;
	// write back the last block
	WriteStatus = eDisk_Write(0, wbuffer, location_block, 1);
	// set the readtag
	readtag = 1;
  return WriteStatus;     
}

//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM 
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)
int eFile_ROpen( char name[]){      // open a file for reading 
	int fileid;
	DRESULT ReadStatus;
	// check whether there is a file opened
  if(readtag != 1){ST7735_OutString(0, 2, "multiple open", ST7735_Color565(255, 0, 0));
	return FAIL;} // already opened a file
	// check whether the file exist
	for(fileid = 0; fileid < DIRECTORY_SIZE; fileid++){
	  if(!strcmp(directory[fileid].name, name)){break;}
		if(fileid == DIRECTORY_SIZE-1){
	    ST7735_OutString(0, 5, "ReadOpen:No file", ST7735_Color565(255, 0, 0));
			return FAIL;
		}
	}
	// get the file information
	directory[fileid].open_flag = 1;
	readtag=0;
	readbuffer.blockid = directory[fileid].starter;
	readbuffer.byteid = 0;
	readbuffer.last_block = directory[fileid].current_block;
	readbuffer.last_byte = directory[fileid].current_byte;
  // read the first block of the file
	ReadStatus = eDisk_Read(0, readbuffer.buffer, readbuffer.blockid, 1);
	if(ReadStatus){return FAIL;}
  return SUCCESS;    
}

//---------- eFile_ReadNext-----------------
// retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int eFile_ReadNext( char *pt){       // get next byte 
	DRESULT ReadStatus;
	// check whether the file is opened
  if(readtag == 1){ST7735_OutString(0, 11, "no open file", ST7735_Color565(255, 0, 0));
	return FAIL;}
	// stop until the end of the file
	if(readbuffer.blockid == readbuffer.last_block && readbuffer.byteid == readbuffer.last_byte){
    ST7735_OutString(0, 10, "ReadNext", ST7735_Color565(255, 0, 0));
	  return FAIL; // end of file
	}
	// read until the end of the block, change to the next block
  if(readbuffer.byteid == 512){
	  readbuffer.blockid = allocation[readbuffer.blockid];
		ReadStatus = eDisk_Read(0, (uint8_t *)&readbuffer.buffer, readbuffer.blockid, 1);
		readbuffer.byteid = 0;
	}
	// read the next char
	*pt = readbuffer.buffer[readbuffer.byteid];

	readbuffer.byteid++;

  return SUCCESS; 
}

    
//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void){ // close the file for writing
  if(readtag != 0){return FAIL;} // no file opened
	readtag = 1;
  return SUCCESS;
}

//---------- eFile_Directory-----------------
// Display the directory with filenames and sizes
// Input: pointer to a function that outputs ASCII characters to display
// Output: none
//         0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_Directory(void(*fp)(char *)){   
  int id = 0;
	//int i;
	char name[10];
	char nr[] = "\n\r";
	char backspace[] = "  ";
	char sizestr[6];
	int size;
	for(id = 0; id<DIRECTORY_SIZE;id++){
	  if(directory[id].name[0] != ' '){
    strcpy(name, directory[id].name);
		// convert the int to char
		size = directory[id].size;
		sizestr[0] = size/100000 + 48;
		sizestr[1] = (size/10000)%10 + 48;
		sizestr[2] = (size/1000)%10 + 48;
		sizestr[3] = (size/100)%10 + 48;
		sizestr[4] = (size/10)%10 + 48;
		sizestr[5] = size%10 + 48;
    // print the information 
		fp(name);
		fp(backspace);
		fp(sizestr);
		fp(nr);
		}
	}
  return SUCCESS;
}

//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Delete( char name[]){  // remove this file 
	int deleteid=0;
	int current_num=0;
	int i;
	uint8_t starter;
	// check whether there is a opened file
	if(readtag != 1){ST7735_OutString(0, 10, "file is open", ST7735_Color565(255, 0, 0));
	return FAIL;} // already file opened
	// find the file to be deleted
	for (i=0;i<DIRECTORY_SIZE;i++){
		if (strcmp(directory[i].name, name)==0)
		{deleteid=i; break;}
		if (i == DIRECTORY_SIZE-1){
		return FAIL;}
	}
	// reset the allocation table
	starter=directory[deleteid].starter;
	while(allocation[starter]!=0){
		current_num=allocation[starter];
		allocation[current_num]=freespace;
		starter++;
	}
	allocation[starter]=freespace;
	// reset the directory
	strcpy(directory[deleteid].name," ");
	directory[deleteid].starter = NULL;
	directory[deleteid].size = 0;
	directory[deleteid].open_flag = 0;
	directory[deleteid].current_block = NULL;
	directory[deleteid].current_byte = NULL;

  return SUCCESS;    // restore directory back to flash
}

int StreamToFile=0;                // 0=UART, 1=stream to file

int eFile_RedirectToFile(char *name){
  eFile_Create(name);              // ignore error if file already exists
	//UART_OutString("Create Success");
  if(eFile_WOpen(name)) return 1;  // cannot open file
	//UART_OutString("WOpen Success");
  StreamToFile = 1;
  return 0;
}

int eFile_EndRedirectToFile(void){
  StreamToFile = 0;
  if(eFile_WClose()) return 1;    // cannot close file
  return 0;
}

int fputc (int ch, FILE *f) { 
  if(StreamToFile){
    if(eFile_Write(ch)){          // close file on error
       eFile_EndRedirectToFile(); // cannot write to file
       return 1;                  // failure
    }
    return 0; // success writing
  }

   // regular UART output
  UART_OutChar(ch);
  return 0; 
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);            // echo
  return ch;
}

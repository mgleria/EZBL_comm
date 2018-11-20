/*******************************************************************************
  Microchip Easy Bootloader Library (EZBL) Image Transfer Utility (Console)

  Summary:
    EZBL v2.xx console based file upload utility without external dependencies. 
    Intended for execution on Linux or POSIX systems acting as the host for 
    Application firmware update with an EZBL based Bootloader node.

  Copyright (C) 2018 Microchip Technology Inc.

  MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:  You may use this software, and any
  derivatives created by any person or entity by or on your behalf, exclusively
  with Microchip's products.  Microchip and its licensors retain all ownership
  and intellectual property rights in the accompanying software and in all
  derivatives here to.

  This software and any accompanying information is for suggestion only.  It
  does not modify Microchip's standard warranty for its products.  You agree
  that you are solely responsible for testing the software and determining its
  suitability.  Microchip has no obligation to modify, test, certify, or
  support the software.

  THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
  EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
  WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
  PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP'S PRODUCTS,
  COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

  IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT
  (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT LIABILITY,
  INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
  EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF
  ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWSOEVER CAUSED, EVEN IF
  MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
  TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
  CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
  FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

  MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
  TERMS.
*******************************************************************************/
#include "stdafx.h"

#include <cstdarg>      // va_arg/va_start/va_end/va_list
#include <stdio.h>      // printf(), fprintf()
#include <stdlib.h>     // atol()
#include <string.h>     // strerror()
#include <time.h>       // clock_gettime()
#include <errno.h>      // errno


FILE * file_open(const char *filePath, const char *mode);
unsigned long long NOW_64(void);
int LogOpen(const char *logFilePath, int argc, const char *argv[]);
int LogClose(void);
size_t LogData(const char *typeStr, long streamOffset, const void *data, size_t dataCount);
void Logf(const char *format, ...);
void LogError(const char *format, ...);


static FILE *logFile = 0;
static FILE *logFile2 = 0;
static unsigned long long logStart;


int LogOpen(const char *logFilePath, int argc, const char *argv[])
{
    if(memcmp(logFilePath, "stderr", 6) == 0)
        logFile2 = stderr;
    else if(memcmp(logFilePath, "stdout", 6) == 0)
        logFile2 = stdout;
    else
    {
        LogClose();
        logFile = file_open(logFilePath, "wct"); // "w" = Open for writing, overwrite file if already exists; "c" = Commit contents to disk when flush() or _flushall() is called; "t" = Text mode; 
        if(logFile == 0)
        {
	        fprintf(stderr, "Unable to open log file for writing: %s\n", logFilePath);
	        return 0;
	    }
    }
	logStart = NOW_64();

    // Start by printing the arguments used to launch this process, if provided
    if(logFile)
    {
        for(int i = 0; i < argc; i++)
            fprintf(logFile, "%s ", argv[i]);
        fprintf(logFile, "\n\n");
    }
    if(logFile2)
    {
        for(int i = 0; i < argc; i++)
            fprintf(logFile2, "%s ", argv[i]);
        fprintf(logFile2, "\n\n");
    }
		
    return 1;
}

int LogClose(void)
{
    if(!logFile)
        return 0;

    fflush(logFile);
    fclose(logFile);
    logFile = 0;
    return 1;
}


//void LogError(const wchar_t *format, ...)
//{
//    FILE *logPtr;
//	va_list args;
//    unsigned long long timestamp = NOW_64() - logStart;
//
//    fflush(stderr);
//    fflush(stdout);
//
//	va_start(args, format); // Initialize variable arguments list
//
//    fwprintf(stderr, L"\n");
//    vfwprintf(stderr, format, args);
//
//	// Generate output log string
//    for(int fileCnt = 0; fileCnt < 2; fileCnt++)
//    {
//        if(fileCnt == 0)
//            logPtr = logFile;
//        else if(fileCnt == 1)
//            logPtr = logFile2;
//        else
//            logPtr = 0;
//        if(!logPtr)
//            continue;
//
//        fwprintf(logPtr, L"\n%2lu.%06u: ", (unsigned long)(timestamp / 1000000u), (unsigned int)(timestamp % 1000000u));
//        vfwprintf(logPtr, format, args);
//
//    }
//    va_end(args);			// Clean up variable arguments list (does nothing in GNU C)
//
//    fflush(stderr);
//    fflush(stdout);
//}


void LogError(const char *format, ...)
{
    FILE *logPtr;
	va_list args;
    unsigned long long timestamp = NOW_64() - logStart;

    fflush(stderr);
    fflush(stdout);

	va_start(args, format); // Initialize variable arguments list

    fprintf(stderr, "\n");
    vfprintf(stderr, format, args);

	// Generate output log string
    for(int fileCnt = 0; fileCnt < 2; fileCnt++)
    {
        if(fileCnt == 0)
            logPtr = logFile;
        else if(fileCnt == 1)
            logPtr = logFile2;
        else
            logPtr = 0;
        if(!logPtr)
            continue;

        fprintf(logPtr, "\n%2lu.%06u: ", (unsigned long)(timestamp / 1000000u), (unsigned int)(timestamp % 1000000u));
        vfprintf(logPtr, format, args);
        fflush(logPtr);
    }
    va_end(args);			// Clean up variable arguments list (does nothing in GNU C)

    fflush(stderr);
    fflush(stdout);
}


//void Logf(const wchar_t *format, ...)
//{
//    unsigned long long timestamp = NOW_64() - logStart;
//	va_list args;
//    FILE *logPtr;
//
//	// Generate output log string
//	va_start(args, format); // Initialize variable arguments list
//    for(int fileCnt = 0; fileCnt < 2; fileCnt++)
//    {
//        if(fileCnt == 0)
//            logPtr = logFile;
//        else if(fileCnt == 1)
//            logPtr = logFile2;
//        else
//            logPtr = 0;
//        if(!logPtr)
//            continue;
//
//        fwprintf(logPtr, L"\n%2lu.%06u: ", (unsigned long)(timestamp / 1000000u), (unsigned int)(timestamp % 1000000u));
//        vfwprintf(logPtr, format, args);
//    }
//    va_end(args);			// Clean up variable arguments list (does nothing in GNU C)
//}

/**
 * Writes only to the logFile/logFile2 pointers, not stdout or stderr (unless LogOpen("stdout")/LogOpen("stderr") was called).
 */
void Logf(const char *format, ...)
{
    unsigned long long timestamp = NOW_64() - logStart;
	va_list args;
    FILE *logPtr;

    fflush(stderr);
    fflush(stdout);

	// Generate output log string
	va_start(args, format); // Initialize variable arguments list
    for(int fileCnt = 0; fileCnt < 2; fileCnt++)
    {
        if(fileCnt == 0)
            logPtr = logFile;
        else if(fileCnt == 1)
            logPtr = logFile2;
        else
            logPtr = 0;
        if(!logPtr)
            continue;

        fprintf(logPtr, "\n%2lu.%06u: ", (unsigned long)(timestamp / 1000000u), (unsigned int)(timestamp % 1000000u));
        vfprintf(logPtr, format, args);
        fflush(logPtr);
    }
    va_end(args);			// Clean up variable arguments list (does nothing in GNU C)

    fflush(stderr);
    fflush(stdout);
}


/**
 * Debug display of ASCII and hex decoded human readable forms of data passing through TX and RX (or whatever) streams.
 *
 * @param *type Pointer to null terminated string indicating the directionality, ex: "RX" or "TX".
 *
 * @return Original value of dataCount
 */
size_t LogData(const char *typeStr, long streamOffset, const void *data, size_t dataCount)
{
    static unsigned int bytesSinceLastFlush = 0;
	const int dataCharsOutPerLine = 16;
	unsigned long long timestamp;
	unsigned char *readPtr;;
    char printables[dataCharsOutPerLine];
    FILE *logPtr;
    size_t count;

    timestamp = NOW_64() - logStart;
    bytesSinceLastFlush += dataCount;
    for(int fileCnt = 0; fileCnt < 2; fileCnt++)
    {
        if(fileCnt == 0)
            logPtr = logFile;
        else if(fileCnt == 1)
            logPtr = logFile2;
        else
            logPtr = 0;
        if(!logPtr)
            continue;

        readPtr = (unsigned char*)data;
        count = dataCount;
        
	    fprintf(logPtr, "%2lu.%06u: %s %2d @ %d:\n", (unsigned long)(timestamp / 1000000u), (unsigned int)(timestamp % 1000000u), typeStr, count, streamOffset);
	    while(count >= dataCharsOutPerLine)
	    {
            for(size_t i = 0; i < sizeof(printables); i++)
                printables[i] = ((unsigned char)readPtr[i] < 0x20u) || ((unsigned char)readPtr[i] > 0x7Eu) ? '.' : readPtr[i];

            fprintf(logPtr, "    %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X    %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n", 
                             readPtr[0], readPtr[1], readPtr[2], readPtr[3], readPtr[4], readPtr[5], readPtr[6], readPtr[7], 
                             readPtr[8], readPtr[9], readPtr[10], readPtr[11], readPtr[12], readPtr[13], readPtr[14], readPtr[15], 
                             printables[0], printables[1], printables[2], printables[3], printables[4], printables[5], printables[6], printables[7], 
                             printables[8], printables[9], printables[10], printables[11], printables[12], printables[13], printables[14], printables[15]);
	        count -= dataCharsOutPerLine;
	        readPtr += dataCharsOutPerLine;
	    }
    	
	    if(count)
	    {
	        memset(printables, ' ', sizeof(printables));
	        fprintf(logPtr, "    ");
	        for(size_t i = 0; i < dataCharsOutPerLine; i++)
	        {
                if(i < count)
                {
                    fprintf(logPtr, "%02X ", readPtr[i]);
                    printables[i] = ((unsigned char)readPtr[i] < 0x20u) || ((unsigned char)readPtr[i] > 0x7Eu) ? '.' : readPtr[i];
                }
                else
                    fprintf(logPtr, "   ");
            }
            fprintf(logPtr, "   %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n", 
                             printables[0], printables[1], printables[2], printables[3], printables[4], printables[5], printables[6], printables[7], 
                             printables[8], printables[9], printables[10], printables[11], printables[12], printables[13], printables[14], printables[15]);
	    }

    }

    if(bytesSinceLastFlush >= 256u)
    {
        fflush(logPtr);
        bytesSinceLastFlush = 0;
    }

    return dataCount;
}

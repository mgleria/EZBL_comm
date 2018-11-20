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

/**
 * To build this utility with gcc (ex: using Cygwin), try:
 *      gcc -o ezbl_comm ezbl_update.cpp logger.cpp
 */

#include "stdafx.h"

#include <cstdarg>      // va_arg/va_start/va_end/va_list
#include <stdio.h>      // printf(), fprintf()
#include <stdlib.h>     // atol()
#include <string.h>     // strerror()
#include <time.h>       // clock_gettime()
#include <errno.h>      // errno
#include "mcp2221_dll_um.h" // MCP2221A API error code definitions, ex: E_ERR_I2C_BUSY and E_ERR_TIMEOUT



#if defined(_WIN32)
    #include <io.h>

    // Windows Sleep() implementation with 1us resolution instead of 1ms
    void usleep(__int64 usec)   
    { 
        HANDLE hTimer; 
        LARGE_INTEGER delay100ns; 

        delay100ns.QuadPart = -(10u*usec); // Convert to 100 nanosecond tick count where negative values are time relative to timer creation
        hTimer = CreateWaitableTimer(NULL, TRUE, NULL);
        SetWaitableTimer(hTimer, &delay100ns, 0, NULL, NULL, 0);
        WaitForSingleObject(&hTimer, (DWORD)(1u + usec/1000u));
        CloseHandle(hTimer);
    }

    void ioctl(int fileDesc, int something, int i2cSlaveAddr)
    {
    }
    
    HANDLE InitializeI2CPort(int vid, int pid, int instanceNum, int SCLFrequency, int mcpMilliTimeout, int mcpRetries); // ezbl_mcp2221_i2c.cpp
    const char * GetMCP2221ErrorString(int errorCode);                                                                  // ezbl_mcp2221_i2c.cpp
    int GetMCP2221DeviceCount(unsigned int vid, unsigned int pid);                                                      // ezbl_mcp2221_i2c.cpp

    #define fileno(x)   _fileno(x)
#else
    #include <unistd.h>         // usleep()
    #include <sys/termios.h>    // termios struct, cfsetospeed(), tcsetattr(), etc.
    #include <sys/ioctl.h>      // ioctl()
    //#include <linux/i2c-dev.h>  // I2C_SLAVE?    

    #define Mcp2221_Close(comHandle)                                                    fclose((comHandle))
    #define Mcp2221_I2cCancelCurrentTransfer(x)                                         0
    #define GetMCP2221ErrorString(x)                                                    "EZBL doesn't support the MCP2221 I2C API on this OS"
    #define Mcp2221_I2cRead(comHandle, bytesToRead, slaveAddr, use7bitAddr, destPtr)    ((fread((destPtr), 1, (bytesToRead), (comHandle)) == (bytesToRead)) ? 0 : -1)
    #define Mcp2221_I2cWrite(comHandle, bytesToWrite, slaveAddr, use7bitAddr, dataPtr)  ((fwrite((dataPtr), 1, (bytesToWrite), (comHandle)) == (bytesToWrite)) ? 0 : -1)
#endif



#if !defined(I2C_SLAVE)
#define I2C_SLAVE   0x0703
#endif


typedef enum _EZBL_ERROR_CODES
{
    EZBL_ERROR_LOCAL_SUCCESS        =   0,      // 0x0000 ezbl_comm.exe process return value: Operation completed successfully
    EZBL_ERROR_SUCCESS              =   1,      // 0x0001 Bootloader: Operation completed successfully (ezbl_comm.exe will not return this value)
    EZBL_ERROR_SUCCESS_VER_GAP      =   2,      // 0x0002 Operation completed successfully, but the programmed image did not have an APPID_VER_MAJOR.APPID_VER_MINOR field that was +1 (minor code) from the existing application.
    EZBL_ERROR_ALREADY_INSTALLED    =   3,      // 0x0003 Offered firmware image already matches the existing target firmware
    EZBL_ERROR_COM_READ_TIMEOUT     = -20,      // 0xFFEC Bootloader signaled communications timeout while waiting for image data
    EZBL_ERROR_IMAGE_MALFORMED      = -21,      // 0xFFEB Bootloader rejected firmware as malformed or of unsupported type. Possible communications error.
    EZBL_ERROR_BOOTID_HASH_MISMATCH = -22,      // 0xFFEA Bootloader rejected firmware as incompatible
    EZBL_ERROR_APPID_VER_MISMATCH   = -23,      // 0xFFE9 Bootloader rejected firmware as out of the required programming order
    EZBL_ERROR_HARD_VERIFY_ERROR    = -24,      // 0xFFE8 Bootloader read-back verification failure
    EZBL_ERROR_SOFT_VERIFY_ERROR    = -25,      // 0xFFE7 Bootloader read-back verification mismatch in reserved address range
    EZBL_ERROR_IMAGE_CRC            = -26,      // 0xFFE6 Bootloader computed CRC mismatch with CRC contained in firmware image. Probable communications error.
    EZBL_ERROR_IMAGE_REJECTED       = -27,      // 0xFFE5 Bootloader or running application rejected the offered image
    EZBL_ERROR_CUSTOM_MESSAGE       = -28,      // 0xFFE4 Custom bootloader response message follows
    EZBL_ERROR_LOCAL_PARAMETERS     = -1000,    // 0xFC18 ezbl_comm.exe input parameter error
    EZBL_ERROR_LOCAL_MEMORY         = -1001,    // 0xFC17 ezbl_comm.exe could not allocate memory
    EZBL_ERROR_LOCAL_FILE_RD        = -1002,    // 0xFC16 ezbl_comm.exe error reading input artifact
    EZBL_ERROR_LOCAL_COM            = -1003,    // 0xFC15 ezbl_comm.exe error opening communications port
    EZBL_ERROR_LOCAL_COM_RD         = -1004,    // 0xFC14 ezbl_comm.exe communications port read failure
    EZBL_ERROR_LOCAL_COM_RD_TIMEOUT = -1005,    // 0xFC13 ezbl_comm.exe communications port data read timeout
    EZBL_ERROR_LOCAL_COM_WR         = -1006,    // 0xFC12 ezbl_comm.exe communications port write failure
    EZBL_ERROR_LOCAL_COM_WR_TIMEOUT = -1007,    // 0xFC11 ezbl_comm.exe communications port data write timeout
} EZBL_ERROR_CODES;


int EZBLBootload(const char *srcFilePath, const char *comPath, long baud, int slaveI2CAddr, long milliTimeout);
FILE * file_open(const char *filePath, const char *mode);
FILE * com_open(const char *comPath, long baud, int slaveI2CAddr, long milliTimeout);
int com_close(FILE *comFile);
int com_read(void *destPtr, size_t size, size_t nmemb, FILE *srcFile, long milliTimeout);
int com_write(const void *srcData, size_t size, size_t nmemb, FILE *destFile, long milliTimeout);
unsigned long long NOW_64(void);

int LogOpen(const char *logFilePath, int argc, const char *argv[]);
int LogClose(void);
size_t LogData(const char *typeStr, long streamOffset, const void *data, size_t dataCount);
void Logf(const char *format, ...);
void LogError(const char *format, ...);

static int i2cMode;                     // 0 == UART, 1 == I2C
static int baudSpecified = 0;           // Flag indicating if -baud= parameter provided (allows I2C default baud of 400000 vs 115200 UART default)
static int i2cSlaveAddr;
static unsigned short vid = 0x04D8u;    // Microchip USB Vendor ID (for MCP2221 I2C in Windows only)
static unsigned short pid = 0x00DDu;    // MCP2221 I2C USB Product ID (for MCP2221 I2C in Windows only)
static int mcpMilliTimeout = 100;       // MCP2221 I2C USB Product ID (for MCP2221 I2C in Windows only)
static int mcpRetries = 0;              // MCP2221 I2C USB Product ID (for MCP2221 I2C in Windows only)
static int disableFlowControl = 0;      // Experimental feature enabled when -no_flow passed on command line
static unsigned long dataTXOffset;      // PC TX byte count tracking for debug logging
static unsigned long dataRXOffset;      // PC RX byte count tracking for debug logging


int main(int argc, const char *argv[])
{
    char *srcPath = 0;       // Ex: "./ex_app_led_blink.production.bl2"
    char *comPath = 0;       // Ex: "/dev/ttyS20"
    int baud = 115200;          // Ex: 115200 (default if unspecified)
    int slaveI2CAddr = 0x60;    // Ex: 0x60   (default if unspecified)
    long milliTimeout = 1100;   // Ex: 1100   (1100 millisecond timeout between work progressing)
    int retCode;
    const char *logPath = 0;
    
	// Decode option parameters
    for(int i = 1; i < argc; i++)
    {
		if(memcmp(argv[i], "-artifact=", 10) == 0)
			srcPath = (char*)&argv[i][10];
		else if(memcmp(argv[i], "-timeout=", 9) == 0)
			sscanf(&argv[i][9], "%i", &milliTimeout);
		else if(memcmp(argv[i], "-com=", 5) == 0)
			comPath = (char*)&argv[i][5];
		else if(memcmp(argv[i], "-baud=", 6) == 0)
        {
		    sscanf(&argv[i][6], "%i", &baud);// Communications baud rate
            baudSpecified = 1;
        }
		else if(memcmp(argv[i], "-i2c_addr=", 10) == 0)
            sscanf(&argv[i][10], "%i", &slaveI2CAddr);
		else if(memcmp(argv[i], "-slave_address=", 15) == 0)    // Legacy synonym for -i2c_addr=
            sscanf(&argv[i][15], "%i", &slaveI2CAddr);
		else if(memcmp(argv[i], "-vid=", 5) == 0)
            sscanf(&argv[i][5], "%i", &vid);
		else if(memcmp(argv[i], "-pid=", 5) == 0)
            sscanf(&argv[i][5], "%i", &pid);
		else if(strcmp(argv[i], "-no_flow") == 0)
            disableFlowControl = 1;
		else if(memcmp(argv[i], "-mcp_retries=", 13) == 0)
            sscanf(&argv[i][13], "%i", &mcpRetries);
		else if(memcmp(argv[i], "-mcp_timeout=", 13) == 0)
            sscanf(&argv[i][13], "%i", &mcpMilliTimeout);
		else if(memcmp(argv[i], "-log", 4) == 0)
        {
            if(argv[i][4] == '=')
                logPath = &argv[i][5];
            LogOpen(logPath ? logPath : "stdout", argc, argv);
        }
#if defined(_WIN32)
		else if(memcmp(argv[i], "-enum", 5) == 0)
		{
		    TCHAR strBuf[32768];
            int dwSize;
            int devCount = 0;
            if(strcmp(argv[i], "-enum=all") == 0)
			    dwSize = (int)QueryDosDevice(NULL, strBuf, sizeof(strBuf)/sizeof(strBuf[0]));
            else
		        dwSize = EnumCOMPorts(strBuf, sizeof(strBuf)/sizeof(strBuf[0]));
            for(int j = 0; j < dwSize; j++)
            {
                printf("%ls%ls", (devCount++ ? "\n" : ""), &strBuf[j]);                  // Supress final newline when listing
		        j += wcsnlen(&strBuf[j], sizeof(strBuf)/sizeof(strBuf[0]) - j);
            }
            return EZBL_ERROR_LOCAL_SUCCESS;    // EZBL_ERROR_LOCAL_SUCCESS == 0
        }
#endif
		else if((i == argc - 1) && (argv[i][0] != '-')) // Implicit source .bl2 file path as the last parameter on the command line
			srcPath = (char*)argv[i];
		else
			LogError("Unrecognized parameter: %s. Ignoring.\n", argv[i]);
	}
    
    
    if((srcPath == 0) || (comPath == 0))
    {
        LogError(
               "Microchip EZBL v2.10 Image Transfer Utility\n"
               "Copyright (C) 2018 Microchip Technology, Inc.\n"
               "\n"
               "Transfers a .bl2 firmware image file to an EZBL bootloader using EZBL's software flow control\n"
               "implementation (non-XON/OFF).\n"
               "\n"
               "Usage:\n"
               "    ezbl_comm -com=path [-baud=bps] [-i2c_addr=target] [-timeout=ms] [-log=log.txt] \"firmware.bl2\"\n"
               "\n"
               "Parameters:\n"
               "    -artifact   Source .bl2 file containing the firmware image for programming.\n"
               "                The '-artifact=' prefix can be ommitted if the filename is provided last.\n"
               "    -com        System dependent target communications file path.\n"
               "                For Windows MCP2221 I2C use, specify \"I2C\", \"I2C1\", \"I2C2\", etc. The numerical\n"
               "                suffix denotes an instance number where no suffix is synonymous with \"I2C1\".\n"
               "    -baud       Communications baud/physical bit rate (UART default 115200, I2C default 400000)\n"
               "    -i2c_addr   Target I2C slave address (default 0x60, ignored for non-I2C)\n"
               "    -timeout    Communications timeout in milliseconds (default 1100ms)\n"
               "    -log        File to write human readable (ascii hex) TX/RX communications data to (default disabled)\n"
               "                If this option is specified without a filename, data is written to stdout\n"
               "\n"
               "Examples:\n"
               "    ezbl_comm.exe -com=COM21 -baud=460800 -timeout=8000 -artifact=\"ex_app_led_blink.production.bl2\"\n"
               "    ezbl_comm.exe -com=I2C1 -i2c_addr=0x60 -baud=400000 -log=i2c_log.txt ex_app_led_blink.production.bl2\n"
               "    ./ezbl_comm -com=/dev/ttyS20 -baud=230400 -timeout=750 -log=log.txt -artifact=app_v2.bl2\n"
               "    ./ezbl_comm -com=/dev/ACM0 \"ex_app_led_blink.production.bl2\"\n"
               "    ./ezbl_comm -com=/dev/I2C-0 -baud=400000 -i2c_addr=0x60 -artifact=\"ex_app_led_blink.production.bl2\" \n");

        return EZBL_ERROR_LOCAL_PARAMETERS; //-1000 (0xFC18) ezbl_comm.exe input parameter error
    }

    if((baud <= 0) || (baud > 0x7FFFFFFF))
        baud = 115200;
    if(!baudSpecified && (strpbrk("i2c", srcPath) || strpbrk("I2C", srcPath)))
        baud = 400000;
    if((slaveI2CAddr < 0) || (slaveI2CAddr > 0x3F))
        slaveI2CAddr = 0x60;
    if((milliTimeout < 0) || (milliTimeout > 0x7FFFFFFF))
        milliTimeout = 1100;
    
    retCode = EZBLBootload(srcPath, comPath, baud, slaveI2CAddr, milliTimeout);
    switch(retCode)
    {
        case EZBL_ERROR_SUCCESS:                // (1)    0x0001 Bootloader: Operation completed successfully (ezbl_comm.exe will not return this value)
            Logf("Operation completed successfully\n");
            break;
            
        case EZBL_ERROR_SUCCESS_VER_GAP:        // (2)    0x0002 Operation completed successfully, but the programmed image did not have an APPID_VER_MAJOR.APPID_VER_MINOR field that was +1 (minor code) from the existing application.
            LogError("Operation completed successfully, but the programmed image did not have an APPID_VER_MAJOR.APPID_VER_MINOR field that was +1 (minor code) from the existing application.\n");
            break;
            
        case EZBL_ERROR_ALREADY_INSTALLED:      // (3)    0x0003 Offered firmware image already matches the existing target firmware
            LogError("Offered firmware image already matches the existing target firmware\n");
            break;

        case EZBL_ERROR_COM_READ_TIMEOUT:       // (-20)  0xFFEC Bootloader signaled communications timeout while waiting for image data
            LogError("Bootloader signaled communications timeout while waiting for image data\n");
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;
            
        case EZBL_ERROR_IMAGE_MALFORMED:        // (-21)  0xFFEB Bootloader rejected firmware as malformed or of unsupported type. Possible communications error.
            LogError("Bootloader rejected firmware as malformed or of unsupported type. Possible communications error.\n");
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_BOOTID_HASH_MISMATCH:   // (-22)  0xFFEA Bootloader rejected firmware as incompatible
            LogError("Bootloader rejected firmware as incompatible\n");
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_APPID_VER_MISMATCH:     // (-23)  0xFFE9 Bootloader rejected firmware as out of the required programming order
            LogError("Bootloader rejected firmware as out of the required programming order\n");
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_HARD_VERIFY_ERROR:      // (-24)  0xFFE8 Bootloader read-back verification failure
            LogError("Bootloader read-back verification failure\n");
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_SOFT_VERIFY_ERROR:      // (-25)  0xFFE7 Bootloader read-back verification mismatch in reserved address range
            LogError("Bootloader read-back verification mismatch in reserved address range\n");
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_IMAGE_CRC:              // (-26)  0xFFE6 Bootloader computed CRC mismatch with CRC contained in firmware image. Probable communications error.
            LogError("Bootloader computed CRC mismatch with CRC contained in firmware image. Probable communications error.\n");
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_IMAGE_REJECTED:         // (-27)  0xFFE5 Bootloader or running application rejected the offered image
            LogError("Bootloader or running application rejected the offered image\n");
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_CUSTOM_MESSAGE:         // (-28)  0xFFE4 Custom bootloader response
            break;

        case EZBL_ERROR_LOCAL_PARAMETERS:       // (-1000)  0xFC18 ezbl_comm.exe input parameter error
            LogError("Input parameter error\n");
            break;

        case EZBL_ERROR_LOCAL_MEMORY:           // (-1001)  0xFC17 ezbl_comm.exe could not allocate memory
            LogError("Could not allocate memory\n");
            break;

        case EZBL_ERROR_LOCAL_FILE_RD:          // (-1002)  0xFC16 ezbl_comm.exe error reading input artifact
            LogError("Error reading from '%s'\n", srcPath);
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_LOCAL_COM:              // (-1003)  0xFC15 ezbl_comm.exe error opening communications port
            // Special error codes from communications driver. LogError() call already printed an error message for 
            // this case, so don't need to print anything generic here.
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_LOCAL_COM_RD:           // (-1004)  0xFC14 ezbl_comm.exe communications port read failure
            // Special error codes from communications driver. LogError() call already printed an error message for 
            // this case, so don't need to print anything generic here.
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_LOCAL_COM_RD_TIMEOUT:   // (-1005)  0xFC13 ezbl_comm.exe communications port data read timeout
            LogError("Timeout reading from '%s'\n", comPath);
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_LOCAL_COM_WR:           // (-1006)  0xFC12 ezbl_comm.exe communications port write failure
            // Special error codes from communications driver. LogError() call already printed an error message for 
            // this case, so don't need to print anything generic here.
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        case EZBL_ERROR_LOCAL_COM_WR_TIMEOUT:   // (-1007)  0xFC11 ezbl_comm.exe communications port data write timeout
            LogError("Timeout writing to '%s'\n", comPath);
            if(logPath)
                printf("Log saved to: %s\n", logPath);
            break;

        default:
            LogError("Unknown error '0x%04X' (unexpected communications data)'\n", (int)(retCode & 0x0000FFFF));
            if(logPath)
                printf("Log saved to: %s\n", logPath);
    }

    LogClose();

    return (retCode == EZBL_ERROR_SUCCESS) ? EZBL_ERROR_LOCAL_SUCCESS : retCode;   // Return 0 for EZBL_ERROR_SUCCESS, otherwise return the termination status code sent from the bootloader
}


// example usage: EZBLBootload("dist/uart/production/ex_app_led_blink.production.bl2", "COM21", 460800, 0, 5000);
// example usage: EZBLBootload("dist/i2c/production/ex_app_led_blink.production.bl2", "I2C1", 400000, 0x60, 1100);
// example usage: EZBLBootload("./ex_app_led_blink.production.bl2", "/dev/ttyS2", 230400, 0, 1100);
// example usage: EZBLBootload("./ex_app_led_blink.production.bl2", "/dev/I2C-0", 400000, 0x60, 1100);
int EZBLBootload(const char *srcFilePath, const char *comPath, long baud, int slaveI2CAddr, long milliTimeout)
{
    unsigned long bl2Len;           // .bl2 total file length
    unsigned long bl2Offset;        // .bl2 file TX byte offset
    FILE *bl2File, *comFile;
    char buf[1024];
    int16_t remoteCode;
    size_t chunkSize;
    int ret;                        // Temporary return codes
    int halfPercentDone = -1;       // Dot transfer status counter
    unsigned long long startTime;   // Timestamp for performance tracking
    int rxByteCount;                // Accumulation of RX bytes
    long originalTimeout = milliTimeout;
    enum
    {
        SM_INIT = 0,
        SM_OFFER_UPDATE,
        SM_GET_REMOTE_STATUS,
        SM_TRANSFERING,
        SM_GET_REMOTE_TERMINATE,
        SM_REMOTE_TERMINATE,
        SM_CLEANUP,
    } sm = SM_INIT, lastState = SM_INIT;
    
    while(1)
    {
        switch(sm)
        {
            case SM_INIT:
                // Set reference time and counters for file transfer throughput statistics
                startTime = NOW_64(); 
                dataRXOffset = 0;
                dataTXOffset = 0;
                rxByteCount = 0;

                // Open source .bl2 file read-only and get its length
                errno = 0;
                bl2File = file_open(srcFilePath, "rb"); // "r" = Open for reading, fail if file does not exist; "b" = Open in binary (untranslated) mode
                if(bl2File == 0)
                {
                    LogError("Error opening '%s': %s (%d)\n", srcFilePath, strerror(errno), errno);
                    return EZBL_ERROR_LOCAL_FILE_RD;    // (-1002)  0xFC16 ezbl_comm.exe error reading input artifact
                }
                
                // Open communications port and initialize baud rate/other settings
                comFile = com_open(comPath, baud, slaveI2CAddr, milliTimeout);
                if(comFile == 0)
                {
                    fclose(bl2File);
                    if(errno == 13)     // Ex: "Error opening 'COM21': Permission denied (13).  Close any applications that may be locking the com interface."
                        LogError("Error opening '%s': %s (%d). Close any applications that may be locking the com interface.\n", comPath, strerror(errno), errno);
                    else if(errno == 2) // Ex: "Error opening 'COM22': No such file or directory (2). Ensure communications hardware is connected and matches the specified com path."
                        LogError("Error opening '%s': %s (%d). Ensure communications hardware is connected and matches the specified com path.\n", comPath, strerror(errno), errno);
                    else if(errno)
                        LogError("Error opening '%s': %s (%d)\n", comPath, strerror(errno), errno);
                    return EZBL_ERROR_LOCAL_COM;    // (-1003)  0xFC15 ezbl_comm.exe error opening communications port
                }

                // Issue dummy read for I2C to empty the remote TX FIFO in case if it currently has something in it (possibly from a prior bootload attempt where the connection was severed)
                if(i2cMode)
                    com_read(0, 1, 32, comFile, 0);

                sm = SM_OFFER_UPDATE;
                break;

            case SM_OFFER_UPDATE:
                // Read first 64 bytes of .bl2 file (or file length, whichever is smaller)
                fseek(bl2File, 0, SEEK_END);
                bl2Len = ftell(bl2File);
                fseek(bl2File, 0, SEEK_SET);
                bl2Offset = 0;
                chunkSize = 64;
                if(chunkSize > bl2Len)
                    chunkSize = (size_t)bl2Len;  // All legal .bl2 files are at least 68 bytes, but to make this utility useful for use with modified bootloaders, permit file transfer anyway.
                
                ret = fread(buf, 1, chunkSize, bl2File);
                if(ret != chunkSize)
                {
                    LogError("%s():%d: Error reading from '%s': %s%s(%d)\n", __FUNCTION__, __LINE__, srcFilePath, (errno ? " " : ""), (errno ? strerror(errno) : ""), errno);
                    sm = SM_CLEANUP;
                    ret = EZBL_ERROR_LOCAL_FILE_RD;     // (-1002)  0xFC16 ezbl_comm.exe error reading input artifact
                    break;
                }

                // Transmit the 64 byte file header. Bootloader will decide if it wants it and erase itself if so.
                ret = com_write(buf, 1, chunkSize, comFile, milliTimeout);
                if(ret != chunkSize)
                {
                    if(ret != EZBL_ERROR_LOCAL_COM_WR_TIMEOUT)
                        LogError("%s():%d: Error writing to '%s': %s (%d)\n", __FUNCTION__, __LINE__, comPath, strerror(errno), errno);
                    sm = SM_CLEANUP;
                    break;
                }
                bl2Offset += chunkSize;

                // Display progress indicator since local file read and com writes seem to be working
                if(halfPercentDone < 0) // Only print this for first try, not and late retries
                {
                    halfPercentDone = 0;
                    printf("Upload progress: |0%%         25%%         50%%         75%%        100%%|\n"
                            "                 |");
                    fflush(stdout);
                }
                sm = SM_GET_REMOTE_STATUS;
                break;

            case SM_GET_REMOTE_STATUS:
                // Read 2 status bytes from the remote node.
                // - 0xFF13 values should be returned initially indicating bootloader wants the update, but is busy erasing.
                //   Bootlooder will periodically send this value to keep us alive/not timed out.
                // - 0xFF11 will arrive when erase completed. Bootloader is ready for the rest of the .bl2 file contents.
                // - 0x0000 indicates remote bootloader abort (something went wrong/offer rejected/bootloader com timout/etc.)
                // - Timeout here indicates passive Bootloader rejection or something went wrong in the Bootloader
                // - Positive values are a flow control size advirtisement (ready for us to send more .bl2 file data)
                remoteCode = 0;
                ret = com_read(&remoteCode, 1, 2, comFile, milliTimeout);
                rxByteCount += ret;
                if(ret == 2) // 2 is the expected number of bytes read
                {
                    milliTimeout = originalTimeout; // Restore original timeout if we truncated it to 50ms for retry
                }
                else // error
                {
                    if((rxByteCount == 0) && (bl2Offset == 64u) && (ret == EZBL_ERROR_LOCAL_COM_RD_TIMEOUT) && (milliTimeout == originalTimeout))
                    {
                        // First timeout: try retransmitting the header again with a very short timeout to see if we can help catch a node that has just been turned on
                        if(milliTimeout > 50)
                        {
                            milliTimeout = 50;
                            sm = SM_OFFER_UPDATE;
                            continue;
                        }

                        LogError("No bootloader response");
                    }
                    else if(ret != EZBL_ERROR_LOCAL_COM_RD_TIMEOUT)
                        LogError("%s():%d: Error reading from '%s': %s%s(%d)\n", __FUNCTION__, __LINE__, comPath, (errno ? " " : ""), (errno ? strerror(errno) : ""), errno);

                    sm = SM_CLEANUP;
                    return ret;
                }
                
                if(remoteCode == 0x0000)
                    sm = SM_REMOTE_TERMINATE;
                else if((remoteCode > 0x0000) || ((remoteCode == (int16_t)0xFF11) && disableFlowControl))
                    sm = SM_TRANSFERING;
                // Negative values stay in this state
                break;

            case SM_TRANSFERING:
                // Read .bl2 file data, saturated to a full buffer, the remote advirtised space available, or the bytes needed to reach EOF
                chunkSize = disableFlowControl ? 0x7FFF : remoteCode;
                if(chunkSize > bl2Len - bl2Offset)
                    chunkSize = bl2Len - bl2Offset;
                if(chunkSize > sizeof(buf))
                    chunkSize = sizeof(buf);
                ret = fread(buf, 1, chunkSize, bl2File);
                if(ret != chunkSize)
                {
                    LogError("%s():%d: Error reading from '%s': %s%s(%d)\n", __FUNCTION__, __LINE__, srcFilePath, (errno ? " " : ""), (errno ? strerror(errno) : ""), errno);
                    sm = SM_CLEANUP;
                    ret = EZBL_ERROR_LOCAL_FILE_RD;     // (-1002)  0xFC16 ezbl_comm.exe error reading input artifact
                    break;
                }

                // Transmit the block of data to the bootloader
                ret = com_write(buf, 1, chunkSize, comFile, milliTimeout);
                if(ret != chunkSize)
                {
                    if(ret != EZBL_ERROR_LOCAL_COM_RD_TIMEOUT)
                        LogError("%s():%d: Error writing to '%s': %s%s(%d)\n", __FUNCTION__, __LINE__, comPath, (errno ? " " : ""), (errno ? strerror(errno) : ""), errno);
                    sm = SM_CLEANUP;
                    break;
                }
                bl2Offset += chunkSize;

                // Update user display with dot marks indicating 2% complete per dot (actually) 
                while(bl2Offset*50ul/bl2Len > (unsigned int)halfPercentDone)
                {
                    halfPercentDone++;
                    printf(".");
                    fflush(stdout);
                }

                remoteCode -= chunkSize;
                if(!disableFlowControl && (remoteCode == 0))
                    sm = SM_GET_REMOTE_STATUS;
                if(bl2Offset == bl2Len)
                    sm = SM_GET_REMOTE_TERMINATE;
                break;

            case SM_GET_REMOTE_TERMINATE:
                printf("|\n");
                fflush(stdout);
                remoteCode = -1;
                
                for(unsigned long long finalStatusStart = NOW_64(); (remoteCode != 0x0000) && (NOW_64()-finalStatusStart < milliTimeout*1000ul); /* no inc */)
                {
                    ret = com_read(&remoteCode, 1, 2, comFile, milliTimeout);
                    rxByteCount += ret;
                    if(ret != 2)
                    {
                        if(ret != EZBL_ERROR_LOCAL_COM_RD_TIMEOUT)
                            LogError("%s():%d: Error reading from '%s': %s%s(%d)\n", __FUNCTION__, __LINE__, comPath, (errno ? " " : ""), (errno ? strerror(errno) : ""), errno);
                        sm = SM_CLEANUP;
                        break;
                    }
                }
                if(remoteCode == 0x0000)
                {
                    sm = SM_REMOTE_TERMINATE;
                }
                else
                {
                    ret = EZBL_ERROR_LOCAL_COM_RD_TIMEOUT;
                    sm = SM_CLEANUP;
                }
                break;

            case SM_REMOTE_TERMINATE:
                // Read final bootloader status return code
                remoteCode = 0;
                ret = com_read(&remoteCode, 1, 2, comFile, milliTimeout);
                rxByteCount += ret;
                if(ret != 2)
                {
                    if(ret != EZBL_ERROR_LOCAL_COM_RD_TIMEOUT)
                        LogError("%s():%d: Error reading from '%s': %s%s(%d)\n", __FUNCTION__, __LINE__, comPath, (errno ? " " : ""), (errno ? strerror(errno) : ""), errno);
                    remoteCode = ret;
                }
                else if(bl2Offset == bl2Len)
                {
                    double timeDif = ((double)(NOW_64() - startTime))/1.0e6;
                    printf("                 %d bytes sent in %1.3fs (%d bytes/second)\n", bl2Len, timeDif, (size_t)(bl2Len/timeDif));
                    Logf("%d bytes sent in %1.3fs (%d bytes/second)\n", bl2Len, timeDif, (size_t)(bl2Len/timeDif));
                }
                
                if(remoteCode == (int16_t)EZBL_ERROR_CUSTOM_MESSAGE)
                {
                    Logf("Remote termination: code 0x%04X", remoteCode);
                    int i = 0;
                    while(com_read(&buf[i], 1, 1, comFile, milliTimeout > 1000 ? 1000 : milliTimeout) == 1)
                    {
                        rxByteCount++;
                        buf[++i] = 0x00;
                        if(i >= sizeof(buf) - 1)
                        {
                            LogError("%s", buf);
                            i = 0;
                        }
                        if(buf[i-1] == 0x00)
                            break;
                    }
                    if(i != 0)
                        LogError("%s", buf);
                }
                fflush(stderr);
                fflush(stdout);

                ret = remoteCode;
                sm = SM_CLEANUP;
                break;

            case SM_CLEANUP:
                if(bl2File)
                    fclose(bl2File);
                if(comFile)
                    com_close(comFile);
                bl2File = 0;
                comFile = 0;
                fflush(stdout);
                fflush(stderr);
                return ret;

            default:    // Impossible
                LogError("default state reached\n");
                sm = SM_CLEANUP;
                break;
        }
    }
}


FILE * file_open(const char *filePath, const char *mode)
{
    if(filePath == 0)
        return 0;
    if(*filePath == '-')
        return stdin;

    #if defined(_WIN32)
    {
        FILE *ret = 0;
        fopen_s(&ret, filePath, mode);
        return ret;
    }
    #else
    {
        return fopen(filePath, mode);
    }
    #endif
}


/**
 * Opens a local communications file and sets the port-specific communications 
 * parameters.
 *
 * NOTE: Currently only supports UART/serial communications interfaces, not I2C 
 *       as the parameters suggest
 *
 * @param *comPath Null terminated file path string indicating the local system's 
 *                 communications port, ex: "/dev/ttyS1"
 * @param baud     Communications baud or physical layer bit rate. Ex: 115200
 * @param slaveI2CAddr  Set to 0 for non-I2C interfaces, otherwise the 7 or 
 *                      10-bit I2C slave target address.
 * @param milliTimeout Communications timeout, in milliseconds. The timeout is 
 *                  measured between read/write requests and work being completed.
 *
 * @return 0 on failure.
 *         On success: file descriptor/handle to the opened communications 
 *                     resource. Use the fclose() function to free the 
 *                     communications hardware resource after use.
 */
FILE * com_open(const char *comPath, long baud, int slaveI2CAddr, long milliTimeout)
{
    FILE *comFile;
    int fileDesc;
    int ret;
    size_t pathLen;
    char *newPath = 0;
    
    i2cMode = strstr(comPath, "I2C") == 0 ? 0 : 1; // 0 == UART, 1 == I2C

#if defined(_WIN32)
    if(i2cMode)
    {
        i2cSlaveAddr = slaveI2CAddr;
        comFile = (FILE*)InitializeI2CPort(vid, pid, (comPath[3] == 0x00 ? 0 : comPath[3]-'0'-1), baud, mcpMilliTimeout, mcpRetries);
        if(comFile == INVALID_HANDLE_VALUE)
            return 0;
        return comFile;
    }
#endif

    // Append \\.\ to the serial port string (needed on Windows in order to access COM10+ since DOS COM port names only exist for COM1-COM9)
    pathLen = strlen(comPath);
    if(pathLen > 3)
    {
        if(((comPath[0] | 0x20) == 'c') && ((comPath[1] | 0x20) == 'o') && ((comPath[2] | 0x20) == 'm'))
        newPath = (char*)calloc(pathLen + 4 + 1, sizeof(comPath[0]));
        if(newPath)
        {
            memcpy(newPath, "\\\\.\\", 4);
            memcpy(&newPath[4], comPath, pathLen + sizeof(comPath[0]));
        }
    }
    
    // Open serial hardware
    if(newPath)
    {
        comFile = file_open(newPath, "r+b");    // r+b = Read + Write Binary mode
        free(newPath);
    }
    else
    {
        comFile = file_open(comPath, "r+b");    // r+b = Read + Write Binary mode
    }
    
    if(comFile == 0)
        return 0;       // Null error return. Calling function prints the error message.

    fileDesc = fileno(comFile);         // Convert C stdio FILE * stream to file descriptor handle for ioctl()/tcsetattr()/tcflush() calls
    if(fileDesc == -1)
    {
        LogError("%s():%d: Error fileno() == -1: %s\n", __FUNCTION__, __LINE__, strerror(errno));
        return comFile;
    }
    
    // I2C mode - non-Windows platforms only
    if(i2cMode)
    {
        ioctl(fileDesc, I2C_SLAVE, slaveI2CAddr);
        return comFile;
    }

    // UART mode
    #if defined(_WIN32)
    {
        HANDLE          hCom;
        DWORD			dwSize;
 	    COMMCONFIG		comConfig;
	    COMMTIMEOUTS	comTimeouts;
        DWORD           statusFlags;
        COMSTAT         comStatus;

        hCom = (HANDLE)_get_osfhandle(fileDesc);
        if(hCom == INVALID_HANDLE_VALUE)
        {
            LogError("%s():%d: _get_osfhandle() error: %s (%d)\n", __FUNCTION__, __LINE__, "Could not obtain handle from file descriptor. Configuration parameters will not be set.", GetLastError());
            return comFile;
        }

        // Disable I/O buffering to make fread()/fwrite() compatible with serial drivers
        if(setvbuf(comFile, 0, _IONBF, 0))  
            LogError("%s():%d: setvbuf() error: %s (%d)\n", __FUNCTION__, __LINE__, strerror(errno), errno);
        
        if(!SetupComm(hCom, 600, 120))      // 600 byte RX buffer, 120 byte TX buffer
            LogError("%s():%d: SetupComm() error: %s (%d)\n", __FUNCTION__, __LINE__, "", GetLastError());
        
        dwSize = sizeof(comConfig);
        memset(&comConfig, 0x00, sizeof(comConfig));
	    if(GetCommConfig(hCom, &comConfig, &dwSize) == 0)
            LogError("%s():%d: GetCommConfig() error: %s (%d)\n", __FUNCTION__, __LINE__, "", GetLastError());

	    comConfig.dcb.DCBlength = sizeof(comConfig.dcb);
	    comConfig.dcb.BaudRate = baud;
	    comConfig.dcb.fBinary = TRUE;
	    comConfig.dcb.fParity = FALSE;		// Do not check and error on parity
	    comConfig.dcb.fOutxCtsFlow = FALSE;	// Do not monitor CTS flow control
	    comConfig.dcb.fOutxDsrFlow = FALSE; // Do not monitor DSR flow control
	    comConfig.dcb.fDtrControl = DTR_CONTROL_ENABLE;	// Turn on DTR line (Data Terminal Ready) and leave it asserted
	    comConfig.dcb.fDsrSensitivity = FALSE;	// Always receive, independent of DSR state
	    comConfig.dcb.fTXContinueOnXoff = TRUE;	//TRUE: transmission continues after the input buffer has come within XoffLim bytes of being full and the driver has transmitted the XoffChar character to stop receiving bytes
	    comConfig.dcb.fOutX = FALSE;		// Do not enable XON/XOFF flow control to avoid remote receive buffer overflow
	    comConfig.dcb.fInX = FALSE;			// Do not enable XON/XOFF flow control to avoid local receive buffer overflow
	    comConfig.dcb.fErrorChar = FALSE;	// No character replacement on parity errors
	    comConfig.dcb.fNull = FALSE;		// No NULL character discarding occurs
	    comConfig.dcb.fRtsControl = RTS_CONTROL_ENABLE;	// Turn on Request-To-Send and leave it asserted
	    comConfig.dcb.fAbortOnError = TRUE; // Abort on read/write error until ClearCommError is called
	    comConfig.dcb.fDummy2 = 0;			// Reserved
	    comConfig.dcb.wReserved = 0;		// Reserved - must be 0
	    comConfig.dcb.XonLim = 32;			// The bytes in uses in the input buffer before flow control is deactivated to allow transmission by the sender
	    comConfig.dcb.XoffLim = 64;			// The free bytes left in the input before before flow control is activated to inhibit the sender
	    comConfig.dcb.ByteSize = 8;
	    comConfig.dcb.Parity = NOPARITY;
	    comConfig.dcb.StopBits = ONESTOPBIT;
	    comConfig.dcb.XonChar = 0x11;       // Not used
	    comConfig.dcb.XoffChar = 0x13;      // Not used
	    comConfig.dcb.ErrorChar = 0x00;
	    comConfig.dcb.EofChar = 0x00;
	    comConfig.dcb.EvtChar = 0x00;
	    comConfig.dcb.wReserved1 = 0;

	    if(!SetCommConfig(hCom, &comConfig, sizeof(comConfig)))
            LogError("%s():%d: SetCommConfig() error: %s (%d)\n", __FUNCTION__, __LINE__, "", GetLastError());

        memset(&comTimeouts, 0x00, sizeof(comTimeouts));
	    comTimeouts.ReadIntervalTimeout = (DWORD)milliTimeout;          // Abort read operations if the data isn't available after milliTimeout milliseconds
	    comTimeouts.ReadTotalTimeoutMultiplier = 1;		                // No extra time given per byte of requested data
	    comTimeouts.ReadTotalTimeoutConstant = 1;                       // No extra time given of fixed duration for reading data
	    comTimeouts.WriteTotalTimeoutMultiplier = 1;	                // Minimal extra dealy time per byte transmitted
	    comTimeouts.WriteTotalTimeoutConstant = (DWORD)milliTimeout;    // Abort write operations if the data can't be placed in the tx buffer after milliTimeout milliseconds
	    if(!SetCommTimeouts(hCom, &comTimeouts))
            LogError("%s():%d: SetCommTimeouts() error: %s (%d)\n", __FUNCTION__, __LINE__, "", GetLastError());

	    // Receive events from COM to wake us from sleeping states
	    if(!SetCommMask(hCom, EV_BREAK | EV_CTS | EV_ERR | EV_RXCHAR | EV_TXEMPTY))
            LogError("%s():%d: SetCommMask() error: %s (%d)\n", __FUNCTION__, __LINE__, "", GetLastError());

        // Clear any RX errors queued by the serial driver
        if(!ClearCommError(hCom, &statusFlags, &comStatus)) 
            LogError("%s():%d: ClearCommError() error: %s (%d)\n", __FUNCTION__, __LINE__, "", GetLastError());
            
        if(comStatus.cbInQue)
        {
            ret = com_read(0, 1, comStatus.cbInQue, comFile, milliTimeout);
            Logf("Throwing %d bytes of already existent RX data away\n\n", ret); 
        }
        
        if(!FlushFileBuffers(hCom))
            LogError("%s():%d: FlushFileBuffers() error: %s (%d)\n", __FUNCTION__, __LINE__, "", GetLastError());
            
        if(!PurgeComm(hCom, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR))
            LogError("%s():%d: PurgeComm() error: %s (%d)\n", __FUNCTION__, __LINE__, "", GetLastError());
    }
    #else   // Linux/Unix case
    {
        struct termios tty;
        memset(&tty, 0x00, sizeof(tty));        // No special input, output, control or local modes. Must set baud rate later.
        tty.c_cflag |= (CLOCAL | CREAD);        // Ignore modem controls and enable RX
        tty.c_cflag |= CS8;                     // 8-bit characters (1 start bit/1 stop bit/no flow control are defaults)

        tty.c_iflag |= BRKINT | IGNPAR;         // RX BREAK causes input and output queue flush

        tty.c_cc[VMIN] = 0;                     // Min 0 bytes for reads or requested length, whichever is less (0 = reads block for no more than VTIME before returning)
        tty.c_cc[VTIME] = 1+milliTimeout/100;   // Block for approximate millisecond timeout for read data to exist before fread() call returns

        switch(baud)    // UART baud rate decoding
        {
            case 0:         baud = B0; break;
            case 50:        baud = B50; break;
            case 75:        baud = B75; break;
            case 110:       baud = B110; break;
            case 134:       baud = B134; break;
            case 150:       baud = B150; break;
            case 200:       baud = B200; break;
            case 300:       baud = B300; break;
            case 600:       baud = B600; break;
            case 1200:      baud = B1200; break;
            case 1800:      baud = B1800; break;
            case 2400:      baud = B2400; break;
            case 4800:      baud = B4800; break;
            case 9600:      baud = B9600; break;
            case 19200:     baud = B19200; break;
            case 38400:     baud = B38400; break;
            case 57600:     baud = B57600; break;   // Baud rates above 38400 are Extended
            case 115200:    baud = B115200; break;
            case 128000:    baud = B128000; break;
            case 230400:    baud = B230400; break;
            case 256000:    baud = B256000; break;
            case 460800:    baud = B460800; break;
            case 500000:    baud = B500000; break;
            case 576000:    baud = B576000; break;
            case 921600:    baud = B921600; break;
            case 1000000:   baud = B1000000; break;
            case 1152000:   baud = B1152000; break;
            case 1500000:   baud = B1500000; break;
            case 2000000:   baud = B2000000; break;
            case 2500000:   baud = B2500000; break;
            case 3000000:   baud = B3000000; break;
            default:
                //// Untested code for setting arbitrary/custom baud rates in Linux. May not work on various platforms.
                //{
                //    struct serial_struct serialParams; 
                //    ioctl(fileDesc, TIOCGSERIAL, &serialParams); 
                //    serialParams.flags = ASYNC_SPD_CUST | ASYNC_LOW_LATENCY; 
                //    serialParams.custom_divisor = (serialParams.baud_base + baud/2) / baud;   // Numerator + half the denominator for rounding after fractional integer truncation
                //    ioctl(fileDesc, TIOCSSERIAL, &serialParams);
                //}
                LogError("Unsupported baud rate: %d\n", baud);
                break;
        }
        
        ret = cfsetospeed(&tty, (speed_t)baud);
        if(ret < 0)
            LogError("%s():%d: Error cfsetospeed() == %d: %s\n", __FUNCTION__, __LINE__, ret, strerror(errno));
        ret = cfsetispeed(&tty, (speed_t)0);    // Zero means use output baud rate
        if(ret < 0)
            LogError("%s():%d: Error cfsetispeed() == %d: %s\n", __FUNCTION__, __LINE__, ret, strerror(errno));
        ret = tcsetattr(fileDesc, TCSANOW, &tty);
        if(ret)
            LogError("%s():%d: Error tcsetattr() == %d: %s\n", __FUNCTION__, __LINE__, ret, strerror(errno));

        tcflush(fileDesc, TCIOFLUSH);         // Throw away any RX/TX data that may already be buffered locally in the serial driver
    }
    #endif

    return comFile;
}


int com_close(FILE *comFile)
{
    if(i2cMode)
        return Mcp2221_Close(comFile);
    return fclose(comFile);
}

// Wrapper for fread() that hides the special protocol header byte during I2C reads (plain fread() for UART)
// Supports a null destPtr, resulting in the read data being thrown away
int com_read(void *destPtr, size_t size, size_t nmemb, FILE *srcFile, long milliTimeout)
{
    static unsigned char buf[257] = {0x00};
    unsigned char tmpBuf[256];
    size_t bytesLeft;
    size_t chunkSize;
    unsigned char *writePtr;
    unsigned long long startTime;
    int ret;
    int retryCount = 1;
    
    startTime = NOW_64();
    
    writePtr = destPtr ? (unsigned char*)destPtr : buf;
    bytesLeft = size * nmemb;
    if(!i2cMode)    // UART
    {
        while(bytesLeft)
        {
            chunkSize = destPtr ? bytesLeft : sizeof(buf);
            if(chunkSize > bytesLeft)
                chunkSize = bytesLeft;
            
            ret = fread(writePtr, 1, chunkSize, srcFile);
            if(ret > 0)
            {
                dataRXOffset += LogData("RX", dataRXOffset, writePtr, ret);
                startTime = NOW_64();   // Progress made, so reset timeout
                if(destPtr)
                    writePtr += ret;
                bytesLeft -= ret;
            }
            else if(ferror(srcFile))
            {
#if defined(_WIN32)
                HANDLE hCom = (HANDLE)_get_osfhandle(fileno(srcFile));
                DWORD errorFlags;
                COMSTAT comStat;
                
                ClearCommError(hCom, &errorFlags, &comStat);
                Logf("COM read error: bytesLeft = %d, chunkSize = %d, ret = %d, *writePtr = 0x%02X, flags = 0x%08X\n"
                     "    comStat = {cbInQue = %d, cbOutQue = %d, fCtsHold = %d, fDsrHold = %d, fEof = %d, fRlsdHold = %d, fTxim = %d, fXoffHold = %d, fXoffSent = %d}\n%s\n", 
                          bytesLeft, chunkSize, ret, (unsigned int)*writePtr, errorFlags, 
                          comStat.cbInQue, comStat.cbOutQue, comStat.fCtsHold, comStat.fDsrHold, comStat.fEof, comStat.fRlsdHold, comStat.fTxim, comStat.fXoffHold, comStat.fXoffSent,
                          retryCount ? "    Retrying read call" : "    Retries exhausted - giving up");

#endif
                clearerr(srcFile);
                if(retryCount--)
                    continue;

                return EZBL_ERROR_LOCAL_COM_RD;                 // -1004 (0xFC14) ezbl_comm.exe communications port read failure
            }
            else
            {
                if((unsigned long)(NOW_64() - startTime) >= ((unsigned long)milliTimeout)*1000ul)
                    return EZBL_ERROR_LOCAL_COM_RD_TIMEOUT;     // -1005 (0xFC13) ezbl_comm.exe communications port data read timeout
                usleep(100);
            }
        }

        return (size * nmemb) - bytesLeft;
    }
    
    // I2C mode - special EZBL header bytes need to be removed from the read data
    writePtr = (unsigned char*)destPtr;
    while(bytesLeft)
    {
        if(buf[0])
        {
            chunkSize = bytesLeft;
            if(chunkSize > (size_t)buf[0])
                chunkSize = (size_t)buf[0];
            if(writePtr)
            {
                memcpy(writePtr, &buf[1], chunkSize);
                writePtr += chunkSize;
            }
            dataRXOffset += LogData("RX", dataRXOffset, &buf[1], chunkSize);
            if(buf[0] - chunkSize)
                memcpy(&buf[1], &buf[chunkSize+1], buf[0] - chunkSize);
            buf[0] -= chunkSize;
            bytesLeft -= chunkSize;
            continue;
        }
        chunkSize = sizeof(tmpBuf);
        if(chunkSize > bytesLeft + 1)
            chunkSize = bytesLeft + 1;
        ret = Mcp2221_I2cRead(srcFile, chunkSize, i2cSlaveAddr, 1, tmpBuf);
        if(ret == 0)    // Success
        {
            //LogData("Raw RX", dataRXOffset, tmpBuf, chunkSize);
            unsigned char *putPtr = &buf[1];
            size_t foundBytes = 0;
            size_t foundTotalBytes = 0;
            for(size_t i = 0; i < chunkSize; i++)
            {
                if(tmpBuf[i] == 0x00u)
                    continue;
                foundBytes = tmpBuf[i];
                if(foundBytes > chunkSize - i - 1u)
                    foundBytes = chunkSize - i - 1u;
                memcpy(putPtr, &tmpBuf[i+1], foundBytes);
                putPtr += foundBytes;
                i += foundBytes;
                foundTotalBytes += foundBytes;
            }
            buf[0] = foundTotalBytes;
            if(buf[0])
                continue;
        }
        else if((ret != E_ERR_I2C_BUSY) && (ret != E_ERR_TIMEOUT))
        {
            LogError("Mcp2221_I2cRead() returned %d (%s)\n", ret, GetMCP2221ErrorString(ret));
            Mcp2221_I2cCancelCurrentTransfer(srcFile);
            return EZBL_ERROR_LOCAL_COM;    // Code to suppress final error message
        }

        if((unsigned long)(NOW_64() - startTime) >= ((unsigned long)milliTimeout)*1000ul)
            return EZBL_ERROR_LOCAL_COM_RD_TIMEOUT; // -1005 (0xFC13) ezbl_comm.exe communications port data read timeout
        usleep(100);        // Sleep for 100us to avoid blocking other threads while waiting for remote node to generate data for us
    }

    return size * nmemb;
}


int com_write(const void *srcData, size_t size, size_t nmemb, FILE *destFile, long milliTimeout)
{
    unsigned int bytesLeft;
    unsigned char *readPtr;
    unsigned long long startTime;
    int ret;
    int retryCount = 1;
    
    startTime = NOW_64();
    readPtr = (unsigned char*)srcData;
    bytesLeft = size * nmemb;

    if(!i2cMode)
    {
        while(bytesLeft)
        {
            ret = fwrite(readPtr, 1, bytesLeft, destFile);
            if(ret > 0)
            {
                dataTXOffset += LogData("TX", dataTXOffset, readPtr, bytesLeft);
                startTime = NOW_64();   // Work successfully done, so reset timeout
                readPtr += (unsigned int)ret;
                bytesLeft -= (unsigned int)ret;
            }
            else if(ferror(destFile))
            {
#if defined(_WIN32)
                HANDLE hCom = (HANDLE)_get_osfhandle(fileno(destFile));
                DWORD errorFlags;
                COMSTAT comStat;
                
                ClearCommError(hCom, &errorFlags, &comStat);
                Logf("COM write error: flags = 0x%08X\n"
                     "    comStat = {cbInQue = %d, cbOutQue = %d, fCtsHold = %d, fDsrHold = %d, fEof = %d, fRlsdHold = %d, fTxim = %d, fXoffHold = %d, fXoffSent = %d}\n", 
                          errorFlags, 
                          comStat.cbInQue, comStat.cbOutQue, comStat.fCtsHold, comStat.fDsrHold, comStat.fEof, comStat.fRlsdHold, comStat.fTxim, comStat.fXoffHold, comStat.fXoffSent);
#endif
                clearerr(destFile);
                if(retryCount--)
                    continue;

                return EZBL_ERROR_LOCAL_COM_WR;                 // -1006 (0xFC12) ezbl_comm.exe communications port write failure
            }
            else
            {
                if(NOW_64() - startTime >= milliTimeout * 1000u)
                    return EZBL_ERROR_LOCAL_COM_WR_TIMEOUT;     // -1007 (0xFC11) ezbl_comm.exe communications port data write timeout
                usleep(100);
            }
        }
    
        return (size * nmemb) - bytesLeft;
    }

    while(1)
    {
	    ret = Mcp2221_I2cWrite(destFile, bytesLeft, i2cSlaveAddr, 1, (unsigned char*)srcData);
	    if(ret == 0)
        {
            dataTXOffset += LogData("TX", dataTXOffset, srcData, bytesLeft);
	        return bytesLeft;
        }
        
        if((ret == E_ERR_TIMEOUT) || (ret == E_ERR_I2C_BUSY))
        {
            Logf("Mcp2221_I2cWrite() returned %d (%s)\n", ret, GetMCP2221ErrorString(ret));
        }
        else
        {
	        LogError("Mcp2221_I2cWrite() returned %d (%s)\n", ret, GetMCP2221ErrorString(ret));
            Mcp2221_I2cCancelCurrentTransfer(destFile);
            errno = 0;
            return EZBL_ERROR_LOCAL_COM;    // Code to suppress final error message
        }

        usleep(100);
        if(NOW_64() - startTime >= milliTimeout * 1000u)
            return EZBL_ERROR_LOCAL_COM_WR_TIMEOUT; // -1007 (0xFC11) ezbl_comm.exe communications port data write timeout
    }
}


/**
 * @return Returns a large monotonic timestamp in microseconds.
 */
unsigned long long NOW_64(void)
{
#if defined(_WIN32)
    static LARGE_INTEGER freq = {0, 0};
    static LARGE_INTEGER timeRef = {0, 0};
	LARGE_INTEGER timeStamp;
    
	QueryPerformanceCounter(&timeStamp);
    if(freq.QuadPart == 0u)
    {
        QueryPerformanceFrequency(&freq);
        timeRef.QuadPart = timeStamp.QuadPart;
    }
    timeStamp.QuadPart -= timeRef.QuadPart;

    return (timeStamp.QuadPart*1000000ull)/freq.QuadPart;
#else
    struct timespec now;
    
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec*1000000ull + now.tv_nsec/1000u;
#endif
}

/*******************************************************************************
  Microchip Easy Bootloader Library (EZBL) Image Transfer Utility (Console)

  Summary:
    EZBL v2.xx console based file upload utility without external dependencies. 
    Intended for execution on Linux or POSIX systems acting as the host for 
    Application firmware update with an EZBL based Bootloader node.

  Copyright (C) 2017 Microchip Technology Inc.

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

const char *GetMCP2221ErrorString(int errorCode);

unsigned long long NOW_64(void);
void Logf(const char *format, ...);
void LogError(const char *format, ...);

        
HANDLE InitializeI2CPort(int vid, int pid, int instanceNum, int SCLFrequency, int mcpMilliTimeout, int mcpRetries)
{
    HANDLE hComm;
    int ret;
    int retries = 12;
    
    Logf("InitializeI2CPort(0x%04X, 0x%04X, %d, %d, %d, %d);\n\n", vid, pid, instanceNum, SCLFrequency, mcpMilliTimeout, mcpRetries);

    errno = 0;
    hComm = Mcp2221_OpenByIndex(vid, pid, instanceNum);	// 0x04D8 is Microchip USB Vender ID (VID), 0x00DD is Microchip MCP2221 Product ID (PID)
	if(hComm == INVALID_HANDLE_VALUE)
	{
		ret = Mcp2221_GetLastError();
		LogError("%s:%d %s(): Mcp2221_OpenByIndex() returned error code %d (%s)\n", __FILE__, __LINE__, __FUNCTION__, ret, GetMCP2221ErrorString(ret));
		return INVALID_HANDLE_VALUE;
	}

    ret = Mcp2221_I2cCancelCurrentTransfer(hComm);
    if(ret)
        LogError("%s:%d %s(): Mcp2221_I2cCancelCurrentTransfer() returned error code %d (%s)\n", __FILE__, __LINE__, __FUNCTION__, ret, GetMCP2221ErrorString(ret));


	ret = Mcp2221_SetSpeed(hComm, SCLFrequency);	// Set the I2C frequency
	if(ret)
	{
		Logf("%s:%d %s(): Mcp2221_SetSpeed() returned error code %d (%s)\n", __FILE__, __LINE__, __FUNCTION__, ret, GetMCP2221ErrorString(ret));

        ret = Mcp2221_SetSpeed(hComm, SCLFrequency);	// Set the I2C frequency
		if(ret)
		{
			Logf("%s:%d %s(): Mcp2221_SetSpeed() returned error code %d (%s)\n", __FILE__, __LINE__, __FUNCTION__, ret, GetMCP2221ErrorString(ret));

			ret = Mcp2221_Reset(hComm);	// Issue a reset in case if the MCP2221 I2C communications ended previously at an inopportune time for the firmware
			if(ret)
				LogError("%s:%d %s(): Mcp2221_Reset() returned error code %d (%s)\n", __FILE__, __LINE__, __FUNCTION__, ret, GetMCP2221ErrorString(ret));
            Mcp2221_CloseAll();

			// Reopen the handle to the MCP2221 since Mcp2221_Reset() closes the handle as well.
            while(retries--)
            {
    			Sleep(250);	// Give a delay for Windows to reenumerate the MCP2221
			    hComm = Mcp2221_OpenByIndex(vid, pid, instanceNum);
			    if(hComm != INVALID_HANDLE_VALUE)
                    break;

                ret = Mcp2221_GetLastError();
                Logf("%s:%d %s(): Mcp2221_OpenByIndex() returned error code %d (%s)\n", __FILE__, __LINE__, __FUNCTION__, ret, GetMCP2221ErrorString(ret));
            }
            if(hComm == INVALID_HANDLE_VALUE)
            {
                LogError("%s:%d %s(): Timeout reopening communications handle after Mcp2221_Reset(): %d (%s)\n", __FILE__, __LINE__, __FUNCTION__, ret, GetMCP2221ErrorString(ret));
                return hComm;
            }

			ret = Mcp2221_SetSpeed(hComm, SCLFrequency);	// Set the I2C frequency
			if(ret)
				LogError("%s:%d %s(): Mcp2221_SetSpeed() returned error code %d (%s)\n", __FILE__, __LINE__, __FUNCTION__, ret, GetMCP2221ErrorString(ret));
		}
	}

    // Cut timeout to 0ms and raise retries (default 3ms, 5 retries). This is needed since a flash page erase takes up to 48ms, so device may be stuck clock stretching for a read.
	ret = Mcp2221_SetAdvancedCommParams(hComm, 0, 100); // Timeout = 0ms, Max Retries = 200
	if(ret)
		LogError("%s:%d %s(): Mcp2221_SetAdvancedCommParams() returned error code %d (%s)\n", __FILE__, __LINE__, __FUNCTION__, ret, GetMCP2221ErrorString(ret));

    return hComm;
}

int GetMCP2221DeviceCount(unsigned int vid, unsigned int pid)
{
    unsigned int ret = 0;
    Mcp2221_GetConnectedDevices(vid, pid, &ret);
    return ret;
}

const char *GetMCP2221ErrorString(int errorCode)
{
	switch(errorCode)
	{
	    case E_NO_ERR:							// 0
		    return "No Error";
	    case E_ERR_UNKOWN_ERROR:				// -1
		    return "UNKOWN_ERROR";
        case E_ERR_CMD_FAILED:					// -2
		    return "CMD_FAILED";
	    case E_ERR_INVALID_HANDLE:				// -3
		    return "INVALID_HANDLE";
	    case E_ERR_INVALID_PARAMETER:			// -4
		    return "INVALID_PARAMETER";
	    case E_ERR_INVALID_PASS:				// -5
		    return "INVALID_PASS";
	    case E_ERR_PASSWORD_LIMIT_REACHED:		// -6
		    return "PASSWORD_LIMIT_REACHED";
	    case E_ERR_FLASH_WRITE_PROTECTED:		// -7
		    return "FLASH_WRITE_PROTECTED";
	    case E_ERR_NULL:						// -10 // null pointer received
		    return "NULL: null pointer received";
	    case E_ERR_DESTINATION_TOO_SMALL:		// -11 // destination string too small
		    return "DESTINATION_TOO_SMALL: destination string too small";
	    case E_ERR_INPUT_TOO_LARGE:				// -12
		    return "INPUT_TOO_LARGE";
	    case E_ERR_FLASH_WRITE_FAILED:			// -13
		    return "FLASH_WRITE_FAILED";
	    case E_ERR_NO_SUCH_INDEX:				// -101	// we tried to connect to a device with a non existent index
		    return "NO_SUCH_INDEX: no MCP2221 at the specified index";
	    case E_ERR_DEVICE_NOT_FOUND:			// -103	// no device matching the provided criteria was found
		    return "DEVICE_NOT_FOUND: no device matching specified VID, PID, instance numbers";
	    case E_ERR_INTERNAL_BUFFER_TOO_SMALL:	// -104	// one of the internal buffers of the function was too small
		    return "INTERNAL_BUFFER_TOO_SMALL";
	    case E_ERR_OPEN_DEVICE_ERROR:			// -105	// an error occurred when trying to get the device handle
		    return "OPEN_DEVICE_ERROR: unable to obtain device handle";
	    case E_ERR_CONNECTION_ALREADY_OPENED:	// -106	// connection already opened
		    return "CONNECTION_ALREADY_OPENED";
	    case E_ERR_CLOSE_FAILED:				// -107
		    return "CLOSE_FAILED";
	    case E_ERR_INVALID_SPEED:				// -401
		    return "INVALID_SPEED";
	    case E_ERR_SPEED_NOT_SET:				// -402
		    return "SPEED_NOT_SET";
	    case E_ERR_INVALID_BYTE_NUMBER:			// -403
		    return "INVALID_BYTE_NUMBER";
	    case E_ERR_INVALID_ADDRESS:				// -404
		    return "INVALID_ADDRESS";
	    case E_ERR_I2C_BUSY:					// -405
		    return "I2C_BUSY";
	    case E_ERR_I2C_READ_ERROR:				// -406		//mcp2221 signaled an error during the i2c read operation
		    return "I2C_READ_ERROR";
	    case E_ERR_ADDRESS_NACK:				// -407
            return "ADDRESS_NACK: no slave acknowledge at I2C bus address";
	    case E_ERR_TIMEOUT:						// -408
		    return "TIMEOUT";
	    case E_ERR_TOO_MANY_RX_BYTES:			// -409
		    return "TOO_MANY_RX_BYTES";
	    case E_ERR_COPY_RX_DATA_FAILED:			// -410	//could not copy the data received from the slave into the provided buffer;
            return "COPY_RX_DATA_FAILED";
	    case E_ERR_COPY_TX_DATA_FAILED:			// -412	// failed to copy the data into the HID buffer
		    return "COPY_TX_DATA_FAILED";
	    case E_ERR_NO_EFFECT:					// -411			// The i2c engine (inside mcp2221) was already idle. The cancellation command had no effect.
		    return "NO_EFFECT: already idle, cancellation had no effect";
	    case E_ERR_INVALID_PEC:					// -413
		    return "INVALID_PEC";
	    case E_ERR_BLOCK_SIZE_MISMATCH:			// -414	// The slave sent a different value for the block size(byte count) than we expected
		    return "BLOCK_SIZE_MISMATCH: unexpected slave byte count";
	    case E_ERR_RAW_TX_TOO_LARGE:			// -301
		    return "RAW_TX_TOO_LARGE";
	    case E_ERR_RAW_TX_COPYFAILED:			// -302
		    return "RAW_TX_COPYFAILED";
	    case E_ERR_RAW_RX_COPYFAILED:			// -303
		    return "RAW_RX_COPYFAILED";
	}
	return "Unrecognized MCP2221 error code";
}



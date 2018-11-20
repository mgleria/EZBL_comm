// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//
#if defined(_WIN32)

#pragma once


#include "targetver.h"

#define _CRT_SECURE_NO_WARNINGS 1

#include <stdio.h>
#include <tchar.h>
#include "Windows.h"


// reference additional headers your program requires here
#define MCP2221_LIB	// We want to statically link to mcp2221_dll_um.lib instead of run-time link with an actual .dll file
#include "mcp2221_dll_um.h"


typedef short int16_t;

// Prototypes for enum_ports.cpp
int EnumCOMPorts(TCHAR *comStrings, int bufTCHARCount);

#else   // Not Windows
    #include <stdint.h>
#endif

#include "main.h"

#define ADDR_SECONDS 0x80	//	WRITE ADDRESS / READ ADDRESS = WRITE ADDRESS + 1
#define ADDR_MIN 0x82
#define ADDR_HOUR 0x84
#define ADDR_DATE 0x86
#define ADDR_MON 0x88
#define ADDR_DAYOFWEEK 0x8a
#define ADDR_YEAR 0x8c
#define ADDR_WRITE_PROTECTED 0x8e

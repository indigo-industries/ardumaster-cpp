#pragma once
/* Windows / MinGW stand-in for POSIX <strings.h> (Linux SITL host has the real one). */
#include <cstring>
#include <string.h>
#ifndef strcasecmp
#define strcasecmp _stricmp
#endif
#ifndef strncasecmp
#define strncasecmp _strnicmp
#endif

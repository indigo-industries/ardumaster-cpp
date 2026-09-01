#pragma once
/* Windows / MinGW stand-ins for POSIX symbols the Linux SITL host provides.
 *
 * Force-included (-include) on WIN32 ONLY, from the if(WIN32) block in the root
 * CMakeLists.txt, so it is the first thing every translation unit sees. A Linux
 * build never sees this file and keeps the POSIX originals unchanged -- same
 * intent as the <strings.h> stand-in beside it.
 *
 * Force-included rather than #included from the sim headers so modules/ap-sim
 * stays free of platform #ifdefs and keeps diffing cleanly against upstream.
 */
#ifdef _WIN32

/* Unlock MinGW's OWN gmtime_r / localtime_r / ctime_r / asctime_r, which it
 * defines as __forceinline wrappers over the _s variants but hides behind this
 * feature macro. Must be set before <time.h> is first pulled in -- hence the
 * force-include. Using the platform's real function beats defining a macro of
 * our own: correct POSIX signature, no risk of rewriting an unrelated call. */
#ifndef _POSIX_THREAD_SAFE_FUNCTIONS
#define _POSIX_THREAD_SAFE_FUNCTIONS 200112L
#endif

#include <fcntl.h>
#include <sys/types.h>
#include <time.h>

/* MinGW's <sys/time.h> declares struct timeval but not suseconds_t, which
 * GPS_Backend::simulation_timeval casts tv_usec through. Re-declaring an
 * identical typedef is legal C++, so this stays safe if MinGW later adds it.
 * long matches MinGW's own tv_usec. */
typedef long suseconds_t;

/* O_CLOEXEC keeps a descriptor from leaking across exec(). Windows has no
 * exec() -- CreateProcess decides inheritance per handle -- so there is nothing
 * to request and 0 is the correct no-op: open() ignores it.
 * This is a genuine behavioural difference, not an exact shim. It is only safe
 * because the sole user (sim_gps_file.hpp) opens a replay file read-only and
 * never spawns a child. */
#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif

#endif /* _WIN32 */

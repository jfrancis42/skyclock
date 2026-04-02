#include "clocksync.h"
#include <cerrno>
#include <cstring>

#ifdef _WIN32
#  include <windows.h>
#else
#  include <sys/time.h>
#endif

bool setSystemClock(time_t utcEpoch, std::string& errOut)
{
#ifdef _WIN32
    // Convert UNIX timestamp to SYSTEMTIME (UTC)
    LONGLONG li = Int32x32To64(utcEpoch, 10000000) + 116444736000000000LL;
    FILETIME ft;
    ft.dwLowDateTime  = (DWORD)(li & 0xFFFFFFFF);
    ft.dwHighDateTime = (DWORD)(li >> 32);
    SYSTEMTIME st;
    FileTimeToSystemTime(&ft, &st);
    if (!SetSystemTime(&st)) {
        errOut = "SetSystemTime failed (need Administrator privileges)";
        return false;
    }
    return true;
#else
    struct timeval tv;
    tv.tv_sec  = utcEpoch;
    tv.tv_usec = 0;
    if (settimeofday(&tv, nullptr) != 0) {
        errOut = std::string("settimeofday failed: ") + strerror(errno)
               + " (run as root or grant CAP_SYS_TIME)";
        return false;
    }
    return true;
#endif
}

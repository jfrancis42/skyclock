#include "clocksync.h"
#include <cerrno>
#include <cstring>

#ifdef _WIN32
#  include <windows.h>
#else
#  include <sys/time.h>
#endif

bool setSystemClock(std::chrono::system_clock::time_point utc, std::string& errOut)
{
    using namespace std::chrono;

    auto since_epoch = utc.time_since_epoch();
    auto secs  = duration_cast<seconds>(since_epoch);
    auto usecs = duration_cast<microseconds>(since_epoch - secs);

#ifdef _WIN32
    // Convert to 100-nanosecond intervals since 1601-01-01 (Windows FILETIME epoch).
    LONGLONG li = static_cast<LONGLONG>(secs.count()) * 10000000LL
                + usecs.count() * 10LL
                + 116444736000000000LL;
    FILETIME ft;
    ft.dwLowDateTime  = static_cast<DWORD>(li & 0xFFFFFFFF);
    ft.dwHighDateTime = static_cast<DWORD>(li >> 32);
    SYSTEMTIME st;
    FileTimeToSystemTime(&ft, &st);
    if (!SetSystemTime(&st)) {
        errOut = "SetSystemTime failed (need Administrator privileges)";
        return false;
    }
    return true;
#else
    struct timeval tv;
    tv.tv_sec  = static_cast<time_t>(secs.count());
    tv.tv_usec = static_cast<suseconds_t>(usecs.count());
    if (settimeofday(&tv, nullptr) != 0) {
        errOut = std::string("settimeofday failed: ") + strerror(errno)
               + " (run as root or grant CAP_SYS_TIME)";
        return false;
    }
    return true;
#endif
}

bool setSystemClock(time_t utcEpoch, std::string& errOut)
{
    return setSystemClock(std::chrono::system_clock::from_time_t(utcEpoch), errOut);
}

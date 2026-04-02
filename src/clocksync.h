#pragma once
#include <chrono>
#include <ctime>
#include <string>

/**
 * clocksync — system clock synchronisation utilities.
 *
 * setSystemClock() requires CAP_SYS_TIME on Linux (run as root or with
 * the appropriate capability) and Administrator rights on Windows.
 * Returns true on success, false with a human-readable message in errOut.
 *
 * The time_point overload preserves sub-second precision via tv_usec /
 * SYSTEMTIME milliseconds.  Prefer it over the time_t overload when an
 * accurate sub-second timestamp is available.
 */
bool setSystemClock(std::chrono::system_clock::time_point utc, std::string& errOut);
bool setSystemClock(time_t utcEpoch, std::string& errOut); // whole-second fallback

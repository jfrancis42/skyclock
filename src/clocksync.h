#pragma once
#include <ctime>
#include <string>

/**
 * clocksync — system clock synchronisation utilities.
 *
 * setSystemClock() requires CAP_SYS_TIME on Linux (run as root or with
 * the appropriate capability) and Administrator rights on Windows.
 * Returns true on success, false with a human-readable message in errOut.
 */
bool setSystemClock(time_t utcEpoch, std::string& errOut);

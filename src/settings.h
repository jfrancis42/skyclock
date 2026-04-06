#pragma once
#include <string>
#include <vector>

/**
 * Settings — persistent configuration stored in ~/.skyclock/settings.json
 *
 * Follows the same pattern as CodeMonkey's Settings class but uses
 * pure C++17 (no Qt) and a hand-written flat JSON parser.
 */
struct Settings {
    // Rig control — direct hamlib (serial/USB)
    int         rigModel    = 1;
    std::string rigPort     = "/dev/ttyUSB0";
    int         rigBaud     = 9600;
    int         rigDataBits = 8;
    int         rigStopBits = 1;
    int         rigParity   = 0;     // 0=None 1=Odd 2=Even
    int         rigHandshake= 0;     // 0=None 1=XON/XOFF 2=Hardware
    int         rigDtrState = 0;     // 0=unset 1=on 2=off
    int         rigRtsState = 0;     // 0=unset 1=on 2=off
    bool        rigEnabled  = false; // false = no radio; user tunes manually

    // Rig control — rigctld (hamlib network daemon)
    bool        rigctldEnabled = false;
    std::string rigctldHost    = "localhost";
    int         rigctldPort    = 4532;

    // WWV reception
    long long   freqKhz     = 10000; // WWV frequency to tune (kHz); 2500/5000/10000/15000/20000
    std::string rigMode     = "AM";  // hamlib mode string (AM recommended for WWV)

    // Audio
    std::string audioDevice;         // empty = system default

    // Clock synchronisation
    bool        setSystemClock = false; // set system clock when confident (requires root/admin)
    int         minConfidence  = 2;     // number of consecutive valid frames required

    static Settings& instance();
    void load();
    void save() const;
    std::string path() const;
};

#pragma once
#include <functional>
#include <string>
#include <vector>

#ifdef HAVE_HAMLIB
#include <hamlib/rig.h>
#include <hamlib/riglist.h>
#endif

/**
 * RigConfig — radio port parameters.
 * Mirrors CodeMonkey's RigConfig struct for cross-project consistency.
 */
struct RigConfig {
    int         model     = 1;
    std::string port      = "/dev/ttyUSB0";
    int         baudRate  = 9600;
    int         dataBits  = 8;
    int         stopBits  = 1;
    int         parity    = 0;     // 0=None 1=Odd 2=Even
    int         handshake = 0;     // 0=None 1=XON/XOFF 2=Hardware
    int         dtrState  = 0;     // 0=unset 1=on 2=off
    int         rtsState  = 0;     // 0=unset 1=on 2=off
};

/**
 * RigControl — non-Qt Hamlib wrapper for skyclock.
 *
 * Adapted from CodeMonkey's HamlibController with Qt removed.
 * Used only for initial setup (tune frequency, set mode); no
 * ongoing polling is required for a receive-only decoder.
 */
class RigControl {
public:
    RigControl();
    ~RigControl();

    // Returns sorted list of "ModelNum — Manufacturer Name" strings.
    static std::vector<std::string> availableRigs();

    // Connect via direct hamlib (serial/USB).
    bool connect(const RigConfig& cfg);

    // Connect via a running rigctld daemon (hamlib NET rigctl backend).
    // host:port defaults to localhost:4532.
    bool connectRigctld(const std::string& host, int port);

    void disconnect();
    bool isConnected() const { return m_connected; }

    // Tune the radio to the specified frequency.
    bool setFreqHz(long long hz);

    // Set the operating mode (e.g. "AM", "USB", "LSB", "CW", "FM").
    bool setMode(const std::string& modeStr);

    // Error callback — called synchronously from connect()/setFreqHz()/setMode().
    void setErrorCallback(std::function<void(const std::string&)> cb) {
        m_errorCb = std::move(cb);
    }

private:
    void emitError(const std::string& msg);

#ifdef HAVE_HAMLIB
    RIG* m_rig = nullptr;
#endif
    bool m_connected = false;
    std::function<void(const std::string&)> m_errorCb;
};

#include "rigcontrol.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

RigControl::RigControl()
{
#ifdef HAVE_HAMLIB
    rig_load_all_backends();
    rig_set_debug(RIG_DEBUG_NONE);
#endif
}

RigControl::~RigControl()
{
    disconnect();
}

std::vector<std::string> RigControl::availableRigs()
{
#ifdef HAVE_HAMLIB
    rig_load_all_backends();
    rig_set_debug(RIG_DEBUG_NONE);
    std::vector<std::string> rigs;
    rig_list_foreach([](const struct rig_caps* caps, void* data) -> int {
        auto* list = static_cast<std::vector<std::string>*>(data);
        char buf[256];
        snprintf(buf, sizeof(buf), "%d \u2014 %s %s",
                 caps->rig_model, caps->mfg_name, caps->model_name);
        list->push_back(buf);
        return 1;
    }, &rigs);
    std::sort(rigs.begin(), rigs.end(), [](const std::string& a, const std::string& b) {
        auto name = [](const std::string& s) -> std::string {
            size_t sp = s.find(' ');
            return (sp != std::string::npos) ? s.substr(sp + 3) : s;
        };
        return name(a) < name(b);
    });
    return rigs;
#else
    return {};
#endif
}

bool RigControl::connect(const RigConfig& cfg)
{
#ifdef HAVE_HAMLIB
    disconnect();

    m_rig = rig_init(cfg.model);
    if (!m_rig) {
        emitError("Failed to initialise rig model " + std::to_string(cfg.model));
        return false;
    }

    strncpy(m_rig->state.rigport.pathname, cfg.port.c_str(), HAMLIB_FILPATHLEN - 1);
    m_rig->state.rigport.timeout = 2000;

    auto& ser = m_rig->state.rigport.parm.serial;
    ser.rate      = cfg.baudRate;
    ser.data_bits = cfg.dataBits;
    ser.stop_bits = cfg.stopBits;
    switch (cfg.parity) {
        case 1:  ser.parity = RIG_PARITY_ODD;   break;
        case 2:  ser.parity = RIG_PARITY_EVEN;  break;
        default: ser.parity = RIG_PARITY_NONE;  break;
    }
    switch (cfg.handshake) {
        case 1:  ser.handshake = RIG_HANDSHAKE_XONXOFF;  break;
        case 2:  ser.handshake = RIG_HANDSHAKE_HARDWARE; break;
        default: ser.handshake = RIG_HANDSHAKE_NONE;     break;
    }
    switch (cfg.dtrState) {
        case 1:  ser.dtr_state = RIG_SIGNAL_ON;    break;
        case 2:  ser.dtr_state = RIG_SIGNAL_OFF;   break;
        default: ser.dtr_state = RIG_SIGNAL_UNSET; break;
    }
    switch (cfg.rtsState) {
        case 1:  ser.rts_state = RIG_SIGNAL_ON;    break;
        case 2:  ser.rts_state = RIG_SIGNAL_OFF;   break;
        default: ser.rts_state = RIG_SIGNAL_UNSET; break;
    }
    // Receive-only: no PTT needed
    m_rig->state.pttport.type.ptt = RIG_PTT_NONE;

    int ret = rig_open(m_rig);
    if (ret != RIG_OK) {
        emitError(std::string("rig_open failed: ") + rigerror(ret));
        rig_cleanup(m_rig);
        m_rig = nullptr;
        return false;
    }

    m_connected = true;
    return true;
#else
    (void)cfg;
    emitError("Hamlib not available in this build");
    return false;
#endif
}

bool RigControl::connectRigctld(const std::string& host, int port)
{
#ifdef HAVE_HAMLIB
    disconnect();

    m_rig = rig_init(RIG_MODEL_NETRIGCTL);
    if (!m_rig) {
        emitError("Failed to initialise NET rigctl backend");
        return false;
    }

    // hamlib NET rigctl expects "host:port" in the pathname field.
    std::string addr = host + ":" + std::to_string(port);
    strncpy(m_rig->state.rigport.pathname, addr.c_str(), HAMLIB_FILPATHLEN - 1);
    m_rig->state.rigport.timeout = 2000;
    m_rig->state.pttport.type.ptt = RIG_PTT_NONE;

    int ret = rig_open(m_rig);
    if (ret != RIG_OK) {
        emitError(std::string("rigctld connect to ") + addr + " failed: " + rigerror(ret));
        rig_cleanup(m_rig);
        m_rig = nullptr;
        return false;
    }

    m_connected = true;
    return true;
#else
    (void)host; (void)port;
    emitError("Hamlib not available in this build");
    return false;
#endif
}

void RigControl::disconnect()
{
#ifdef HAVE_HAMLIB
    if (m_rig) {
        rig_close(m_rig);
        rig_cleanup(m_rig);
        m_rig = nullptr;
    }
#endif
    m_connected = false;
}

bool RigControl::setFreqHz(long long hz)
{
#ifdef HAVE_HAMLIB
    if (!m_rig || !m_connected) return false;
    int ret = rig_set_freq(m_rig, RIG_VFO_CURR, static_cast<freq_t>(hz));
    if (ret != RIG_OK) {
        emitError(std::string("setFreqHz failed: ") + rigerror(ret));
        return false;
    }
    return true;
#else
    (void)hz;
    return false;
#endif
}

bool RigControl::setMode(const std::string& modeStr)
{
#ifdef HAVE_HAMLIB
    if (!m_rig || !m_connected) return false;
    rmode_t mode = rig_parse_mode(modeStr.c_str());
    if (mode == RIG_MODE_NONE) {
        emitError("Unknown mode: " + modeStr);
        return false;
    }
    int ret = rig_set_mode(m_rig, RIG_VFO_CURR, mode, RIG_PASSBAND_NORMAL);
    if (ret != RIG_OK) {
        emitError(std::string("setMode failed: ") + rigerror(ret));
        return false;
    }
    return true;
#else
    (void)modeStr;
    return false;
#endif
}

void RigControl::emitError(const std::string& msg)
{
    if (m_errorCb) m_errorCb(msg);
}

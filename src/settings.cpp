#include "settings.h"
#include <cstdlib>
#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <sys/stat.h>
#ifdef _WIN32
#  include <direct.h>
#endif

/* ── Minimal flat JSON parser ────────────────────────────────────────────────
 * Handles: { "key": "string"|number|true|false }
 * Sufficient for a flat settings object; not a general-purpose parser.
 */
namespace {

struct JVal {
    char        type = 'n'; // 's'=string, 'f'=float, 'b'=bool, 'n'=null
    std::string s;
    double      d = 0;
    bool        b = false;
};

static std::map<std::string, JVal> parseJson(const std::string& text)
{
    std::map<std::string, JVal> out;
    size_t i = 0;
    auto skipWs = [&]() {
        while (i < text.size() && isspace((unsigned char)text[i])) ++i;
    };
    skipWs();
    if (i >= text.size() || text[i] != '{') return out;
    ++i;
    while (i < text.size()) {
        skipWs();
        if (i >= text.size() || text[i] == '}') break;
        if (text[i] == ',') { ++i; continue; }
        if (text[i] != '"') break;
        // Key
        ++i;
        size_t ks = i;
        while (i < text.size() && text[i] != '"') ++i;
        std::string key = text.substr(ks, i - ks);
        if (i < text.size()) ++i;
        skipWs();
        if (i >= text.size() || text[i] != ':') break;
        ++i;
        skipWs();
        JVal v;
        if (i < text.size() && text[i] == '"') {
            ++i;
            size_t vs = i;
            while (i < text.size() && text[i] != '"') {
                if (text[i] == '\\') ++i;
                ++i;
            }
            v.s = text.substr(vs, i - vs);
            v.type = 's';
            if (i < text.size()) ++i;
        } else if (i < text.size() && text[i] == 't') {
            v.b = true; v.type = 'b';
            while (i < text.size() && isalpha((unsigned char)text[i])) ++i;
        } else if (i < text.size() && text[i] == 'f') {
            v.b = false; v.type = 'b';
            while (i < text.size() && isalpha((unsigned char)text[i])) ++i;
        } else if (i < text.size() && (isdigit((unsigned char)text[i]) || text[i] == '-')) {
            size_t ns = i;
            if (text[i] == '-') ++i;
            while (i < text.size() && (isdigit((unsigned char)text[i]) ||
                   text[i] == '.' || text[i] == 'e' || text[i] == 'E' ||
                   text[i] == '+' || text[i] == '-'))
                ++i;
            v.s = text.substr(ns, i - ns);
            v.d = std::stod(v.s);
            v.type = 'f';
        } else {
            while (i < text.size() && isalpha((unsigned char)text[i])) ++i;
        }
        out[key] = v;
    }
    return out;
}

static int    getInt   (const std::map<std::string,JVal>& m, const char* k, int    def) {
    auto it = m.find(k);
    return (it != m.end() && it->second.type == 'f') ? (int)it->second.d : def;
}
static long long getLLong(const std::map<std::string,JVal>& m, const char* k, long long def) {
    auto it = m.find(k);
    return (it != m.end() && it->second.type == 'f') ? (long long)it->second.d : def;
}
static bool   getBool  (const std::map<std::string,JVal>& m, const char* k, bool   def) {
    auto it = m.find(k);
    return (it != m.end() && it->second.type == 'b') ? it->second.b : def;
}
static std::string getStr(const std::map<std::string,JVal>& m, const char* k, const std::string& def) {
    auto it = m.find(k);
    return (it != m.end() && it->second.type == 's') ? it->second.s : def;
}

static std::string escapeStr(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"')       out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else                out += c;
    }
    return out;
}

} // namespace

/* ── Settings singleton ──────────────────────────────────────────────────── */

Settings& Settings::instance() {
    static Settings s;
    s.load();
    return s;
}

std::string Settings::path() const {
    const char* home = getenv("HOME");
#ifdef _WIN32
    if (!home) home = getenv("USERPROFILE");
#endif
    std::string base = home ? std::string(home) : ".";
    return base + "/.skyclock/settings.json";
}

void Settings::load()
{
    std::ifstream f(path());
    if (!f) return;
    std::string text((std::istreambuf_iterator<char>(f)),
                      std::istreambuf_iterator<char>());
    auto m = parseJson(text);

    rigModel     = getInt  (m, "rigModel",     rigModel);
    rigPort      = getStr  (m, "rigPort",       rigPort);
    rigBaud      = getInt  (m, "rigBaud",       rigBaud);
    rigDataBits  = getInt  (m, "rigDataBits",   rigDataBits);
    rigStopBits  = getInt  (m, "rigStopBits",   rigStopBits);
    rigParity    = getInt  (m, "rigParity",     rigParity);
    rigHandshake = getInt  (m, "rigHandshake",  rigHandshake);
    rigDtrState  = getInt  (m, "rigDtrState",   rigDtrState);
    rigRtsState  = getInt  (m, "rigRtsState",   rigRtsState);
    rigEnabled      = getBool (m, "rigEnabled",      rigEnabled);
    rigctldEnabled  = getBool (m, "rigctldEnabled",  rigctldEnabled);
    rigctldHost     = getStr  (m, "rigctldHost",     rigctldHost);
    rigctldPort     = getInt  (m, "rigctldPort",     rigctldPort);
    freqKhz         = getLLong(m, "freqKhz",         freqKhz);
    rigMode      = getStr  (m, "rigMode",       rigMode);
    audioDevice  = getStr  (m, "audioDevice",   audioDevice);
    setSystemClock = getBool(m, "setSystemClock", setSystemClock);
    minConfidence  = getInt  (m, "minConfidence",  minConfidence);
}

void Settings::save() const
{
    // Create directory if needed
    std::string p = path();
    size_t sep = p.rfind('/');
    if (sep != std::string::npos) {
        std::string dir = p.substr(0, sep);
#ifdef _WIN32
        _mkdir(dir.c_str());
#else
        mkdir(dir.c_str(), 0755);
#endif
    }

    std::ofstream f(p);
    if (!f) return;
    auto b = [](bool v) -> const char* { return v ? "true" : "false"; };
    f << "{\n";
    f << "    \"rigEnabled\":       " << b(rigEnabled)       << ",\n";
    f << "    \"rigctldEnabled\":   " << b(rigctldEnabled)   << ",\n";
    f << "    \"rigctldHost\":      \"" << escapeStr(rigctldHost) << "\",\n";
    f << "    \"rigctldPort\":      " << rigctldPort         << ",\n";
    f << "    \"rigModel\":         " << rigModel            << ",\n";
    f << "    \"rigPort\":         \"" << escapeStr(rigPort) << "\",\n";
    f << "    \"rigBaud\":         " << rigBaud            << ",\n";
    f << "    \"rigDataBits\":     " << rigDataBits        << ",\n";
    f << "    \"rigStopBits\":     " << rigStopBits        << ",\n";
    f << "    \"rigParity\":       " << rigParity          << ",\n";
    f << "    \"rigHandshake\":    " << rigHandshake       << ",\n";
    f << "    \"rigDtrState\":     " << rigDtrState        << ",\n";
    f << "    \"rigRtsState\":     " << rigRtsState        << ",\n";
    f << "    \"freqKhz\":         " << freqKhz            << ",\n";
    f << "    \"rigMode\":         \"" << escapeStr(rigMode) << "\",\n";
    f << "    \"audioDevice\":     \"" << escapeStr(audioDevice) << "\",\n";
    f << "    \"setSystemClock\":  " << b(setSystemClock)  << ",\n";
    f << "    \"minConfidence\":   " << minConfidence      << "\n";
    f << "}\n";
}

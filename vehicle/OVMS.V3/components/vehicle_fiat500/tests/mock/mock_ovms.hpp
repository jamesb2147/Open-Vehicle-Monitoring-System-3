// mock_ovms.hpp — Stubs for OVMS framework types enabling native laptop builds
// of vehicle_fiat500e.cpp. Included via -include so it loads before any module header.
//
// Derived from vehicle_vwegolf/tests/mock/mock_ovms.hpp, with additions the Fiat
// module needs: canbus::Write(CAN_frame_t*), a MyMetrics registry, the extra
// standard metrics and commands this module touches, and — most importantly —
// transition counting (see MetricStore below).

#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "[E][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stderr, "[W][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) fprintf(stdout, "[I][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) do {} while(0)
#define ESP_LOGV(tag, fmt, ...) do {} while(0)

// ---------------------------------------------------------------------------
// FreeRTOS stubs
// ---------------------------------------------------------------------------
typedef uint32_t TickType_t;
inline void vTaskDelay(TickType_t) {}
inline TickType_t pdMS_TO_TICKS(uint32_t ms) { return ms; }
extern uint32_t g_tick_count;
inline TickType_t xTaskGetTickCount() { return g_tick_count; }
static constexpr uint32_t portTICK_PERIOD_MS = 1;

// ---------------------------------------------------------------------------
// CAN types
// ---------------------------------------------------------------------------
typedef enum { CAN_frame_std, CAN_frame_ext } CAN_frame_format_t;
typedef enum { CAN_MODE_LISTEN, CAN_MODE_ACTIVE, CAN_MODE_OFF } CAN_mode_t;
typedef enum {
    CAN_SPEED_33KBPS  = 33,
    CAN_SPEED_50KBPS  = 50,
    CAN_SPEED_100KBPS = 100,
    CAN_SPEED_125KBPS = 125,
    CAN_SPEED_250KBPS = 250,
    CAN_SPEED_500KBPS = 500,
} CAN_speed_t;
typedef enum {
    CAN_errorstate_none = 0,
    CAN_errorstate_active,
    CAN_errorstate_warning,
    CAN_errorstate_passive,
    CAN_errorstate_busoff,
} CAN_errorstate_t;

typedef union {
    struct { CAN_frame_format_t FF; uint8_t DLC; } B;
} CAN_FIR_t;

typedef int esp_err_t;
static constexpr esp_err_t ESP_OK   =  0;
static constexpr esp_err_t ESP_FAIL = -1;

struct canbus;

struct CAN_frame_t {
    canbus*    origin   = nullptr;
    void*      callback = nullptr;
    CAN_FIR_t  FIR      = {};
    uint32_t   MsgID    = 0;
    union {
        uint8_t  u8[8];
        uint32_t u32[2];
        uint64_t u64;
    } data = {};
};

// One transmitted frame, recorded so command tests can assert exactly what the
// module put on the bus. The real canbus discards frames after TX; the mock keeps
// an ordered log (std vs ext, ID, DLC, payload).
struct TxRecord {
    bool     extended = false;
    uint32_t id       = 0;
    uint8_t  len      = 0;
    uint8_t  data[8]  = {};
};

struct canbus {
    uint8_t               m_busnumber = 0;
    std::vector<TxRecord> tx_log;
    CAN_errorstate_t      error_state = CAN_errorstate_none;

    // The Fiat module builds a CAN_frame_t and calls Write() directly, rather
    // than the WriteStandard/WriteExtended helpers the e-Golf module uses.
    esp_err_t Write(const CAN_frame_t* f, TickType_t /*maxqueuewait*/ = 0) {
        TxRecord r;
        r.extended = (f->FIR.B.FF == CAN_frame_ext);
        r.id       = f->MsgID;
        r.len      = f->FIR.B.DLC;
        for (uint8_t i = 0; i < r.len && i < 8; i++) r.data[i] = f->data.u8[i];
        tx_log.push_back(r);
        return ESP_OK;
    }
    esp_err_t WriteStandard(uint32_t id, uint8_t len, uint8_t* data, int = 0) {
        return LogTx(false, id, len, data);
    }
    esp_err_t WriteExtended(uint32_t id, uint8_t len, uint8_t* data, int = 0) {
        return LogTx(true, id, len, data);
    }
    CAN_errorstate_t GetErrorState() { return error_state; }
    esp_err_t Reset() { return ESP_OK; }

 private:
    esp_err_t LogTx(bool ext, uint32_t id, uint8_t len, uint8_t* data) {
        TxRecord r;
        r.extended = ext;
        r.id       = id;
        r.len      = len;
        for (uint8_t i = 0; i < len && i < 8; i++) r.data[i] = data[i];
        tx_log.push_back(r);
        return ESP_OK;
    }
};

// ---------------------------------------------------------------------------
// Metric units / staleness tags
// ---------------------------------------------------------------------------
static constexpr int SM_STALE_NONE =     0;
static constexpr int SM_STALE_MIN  =    10;
static constexpr int SM_STALE_MID  =   120;
static constexpr int SM_STALE_HIGH =  3600;
static constexpr int SM_STALE_MAX  = 65535;

typedef enum {
    Native = 0, Watts, kWh, Volts, Amps, Celcius, Percentage,
    Kilometers, Mph, AmpHours, Minutes,
} metric_unit_t;

// ---------------------------------------------------------------------------
// Metrics
//
// Values live in a global store keyed by metric name; each StandardMetrics
// member is a pointer, matching how the real code accesses them (->).
//
// TWO counters are tracked per metric, and the distinction is the entire point
// of this harness:
//
//   writes      — every SetValue() call, whether or not the value changed.
//   transitions — only calls where the stored value actually CHANGED.
//
// The real framework fires events only on change (ovms_metrics.cpp,
// OvmsMetricBool::SetValue: `if (m_value != value) SetModified(true);`), and a
// metric that changes at CAN frame rate floods the event queue — which OVMS
// deliberately turns into abort() (ovms_events.cpp CheckQueueOverflow).
//
// So the storm bugs in this module are TRANSITION bugs, not write bugs. A test
// asserting write counts passes identically on buggy and fixed code. Assert on
// transitions.
// ---------------------------------------------------------------------------
struct MetricStore {
    std::map<std::string, double>      numbers;
    std::map<std::string, std::string> strings;
    std::map<std::string, int>         writes;
    std::map<std::string, int>         transitions;

    void reset() { numbers.clear(); strings.clear(); writes.clear(); transitions.clear(); }

    int write_count(const std::string& n) const {
        auto it = writes.find(n);
        return it == writes.end() ? 0 : it->second;
    }
    int transition_count(const std::string& n) const {
        auto it = transitions.find(n);
        return it == transitions.end() ? 0 : it->second;
    }
};
extern MetricStore g_metrics;

template<typename T>
struct OvmsMetric {
    std::string name;
    explicit OvmsMetric(const char* n) : name(n) {}
    void SetValue(T v) {
        double d  = static_cast<double>(v);
        auto   it = g_metrics.numbers.find(name);
        // Absent counts as a transition: the real metric starts undefined, and
        // the first SetValue fires SetModified(true).
        if (it == g_metrics.numbers.end() || it->second != d)
            g_metrics.transitions[name]++;
        g_metrics.numbers[name] = d;
        g_metrics.writes[name]++;
    }
    void SetValue(T v, metric_unit_t) { SetValue(v); }
    void Clear() { g_metrics.numbers.erase(name); }
    T     AsValue() const { return static_cast<T>(g_metrics.numbers[name]); }
    float AsFloat() const { return static_cast<float>(g_metrics.numbers[name]); }
    int   AsInt()   const { return static_cast<int>(g_metrics.numbers[name]); }
    bool  IsDefined() const { return g_metrics.numbers.count(name) != 0; }
};

template<>
struct OvmsMetric<std::string> {
    std::string name;
    explicit OvmsMetric(const char* n) : name(n) {}
    void SetValue(const std::string& v) {
        auto it = g_metrics.strings.find(name);
        if (it == g_metrics.strings.end() || it->second != v)
            g_metrics.transitions[name]++;
        g_metrics.strings[name] = v;
        g_metrics.writes[name]++;
    }
    void SetValue(const char* v) { SetValue(std::string(v ? v : "")); }
    std::string AsValue() const {
        auto it = g_metrics.strings.find(name);
        return it != g_metrics.strings.end() ? it->second : "";
    }
    std::string AsString() const { return AsValue(); }
    bool IsDefined() const { return g_metrics.strings.count(name) != 0; }
};

template<>
struct OvmsMetric<bool> {
    std::string name;
    explicit OvmsMetric(const char* n) : name(n) {}
    void SetValue(bool v) {
        double d  = v ? 1.0 : 0.0;
        auto   it = g_metrics.numbers.find(name);
        if (it == g_metrics.numbers.end() || it->second != d)
            g_metrics.transitions[name]++;
        g_metrics.numbers[name] = d;
        g_metrics.writes[name]++;
    }
    bool AsValue() const { return g_metrics.numbers[name] != 0.0; }
    bool AsBool()  const { return AsValue(); }
    bool IsDefined() const { return g_metrics.numbers.count(name) != 0; }
};

using OvmsMetricFloat  = OvmsMetric<float>;
using OvmsMetricInt    = OvmsMetric<int>;
using OvmsMetricBool   = OvmsMetric<bool>;
using OvmsMetricString = OvmsMetric<std::string>;

// StandardMetrics — members are POINTERS to match how the real code accesses them.
struct StandardMetricsType {
    // Battery
    OvmsMetricFloat*  ms_v_bat_soc            = new OvmsMetricFloat("ms_v_bat_soc");
    OvmsMetricFloat*  ms_v_bat_voltage        = new OvmsMetricFloat("ms_v_bat_voltage");
    OvmsMetricFloat*  ms_v_bat_12v_voltage    = new OvmsMetricFloat("ms_v_bat_12v_voltage");
    OvmsMetricFloat*  ms_v_bat_current        = new OvmsMetricFloat("ms_v_bat_current");
    OvmsMetricFloat*  ms_v_bat_temp           = new OvmsMetricFloat("ms_v_bat_temp");
    OvmsMetricFloat*  ms_v_bat_range_est      = new OvmsMetricFloat("ms_v_bat_range_est");
    OvmsMetricFloat*  ms_v_bat_energy_used    = new OvmsMetricFloat("ms_v_bat_energy_used");
    // Motor / drivetrain
    OvmsMetricFloat*  ms_v_mot_temp           = new OvmsMetricFloat("ms_v_mot_temp");
    OvmsMetricFloat*  ms_v_gen_temp           = new OvmsMetricFloat("ms_v_gen_temp");
    OvmsMetricFloat*  ms_v_inv_temp           = new OvmsMetricFloat("ms_v_inv_temp");
    // Position
    OvmsMetricFloat*  ms_v_pos_odometer       = new OvmsMetricFloat("ms_v_pos_odometer");
    OvmsMetricFloat*  ms_v_pos_speed          = new OvmsMetricFloat("ms_v_pos_speed");
    // Environment / body
    OvmsMetricFloat*  ms_v_env_temp           = new OvmsMetricFloat("ms_v_env_temp");
    OvmsMetricFloat*  ms_v_env_cabintemp      = new OvmsMetricFloat("ms_v_env_cabintemp");
    OvmsMetricFloat*  ms_v_env_cabinsetpoint  = new OvmsMetricFloat("ms_v_env_cabinsetpoint");
    OvmsMetricBool*   ms_v_env_on             = new OvmsMetricBool("ms_v_env_on");
    OvmsMetricBool*   ms_v_env_awake          = new OvmsMetricBool("ms_v_env_awake");
    OvmsMetricBool*   ms_v_env_locked         = new OvmsMetricBool("ms_v_env_locked");
    OvmsMetricBool*   ms_v_env_valet          = new OvmsMetricBool("ms_v_env_valet");
    OvmsMetricBool*   ms_v_env_hvac           = new OvmsMetricBool("ms_v_env_hvac");
    OvmsMetricBool*   ms_v_env_handbrake      = new OvmsMetricBool("ms_v_env_handbrake");
    // Doors
    OvmsMetricBool*   ms_v_door_fl            = new OvmsMetricBool("ms_v_door_fl");
    OvmsMetricBool*   ms_v_door_fr            = new OvmsMetricBool("ms_v_door_fr");
    OvmsMetricBool*   ms_v_door_rl            = new OvmsMetricBool("ms_v_door_rl");
    OvmsMetricBool*   ms_v_door_rr            = new OvmsMetricBool("ms_v_door_rr");
    OvmsMetricBool*   ms_v_door_trunk         = new OvmsMetricBool("ms_v_door_trunk");
    OvmsMetricBool*   ms_v_door_chargeport    = new OvmsMetricBool("ms_v_door_chargeport");
    // Charge
    OvmsMetricBool*   ms_v_charge_inprogress  = new OvmsMetricBool("ms_v_charge_inprogress");
    OvmsMetricBool*   ms_v_charge_pilot       = new OvmsMetricBool("ms_v_charge_pilot");
    OvmsMetricString* ms_v_charge_state       = new OvmsMetricString("ms_v_charge_state");
    OvmsMetricString* ms_v_charge_substate    = new OvmsMetricString("ms_v_charge_substate");
    OvmsMetricFloat*  ms_v_charge_voltage     = new OvmsMetricFloat("ms_v_charge_voltage");
    OvmsMetricFloat*  ms_v_charge_current     = new OvmsMetricFloat("ms_v_charge_current");
    OvmsMetricFloat*  ms_v_charge_temp        = new OvmsMetricFloat("ms_v_charge_temp");
    // Identity
    OvmsMetricString* ms_v_vin                = new OvmsMetricString("ms_v_vin");
    OvmsMetricString* ms_v_type               = new OvmsMetricString("ms_v_type");
};
extern StandardMetricsType StandardMetrics;
#define StdMetrics StandardMetrics

// ---------------------------------------------------------------------------
// Metric registry (MyMetrics.InitFloat / InitInt / InitBool)
//
// Mirrors the real semantics: look up by name, create if absent, and only seed
// the initial value when the metric is not already defined.
// ---------------------------------------------------------------------------
struct OvmsMetrics {
    std::map<std::string, OvmsMetricFloat*> floats;
    std::map<std::string, OvmsMetricInt*>   ints;
    std::map<std::string, OvmsMetricBool*>  bools;

    OvmsMetricFloat* InitFloat(const char* n, int = 0, float v = 0,
                               metric_unit_t = Native, bool = false) {
        auto& slot = floats[n];
        if (!slot) slot = new OvmsMetricFloat(n);
        if (!slot->IsDefined()) slot->SetValue(v);
        return slot;
    }
    OvmsMetricInt* InitInt(const char* n, int = 0, int v = 0,
                           metric_unit_t = Native, bool = false) {
        auto& slot = ints[n];
        if (!slot) slot = new OvmsMetricInt(n);
        if (!slot->IsDefined()) slot->SetValue(v);
        return slot;
    }
    OvmsMetricBool* InitBool(const char* n, int = 0, bool v = false,
                             metric_unit_t = Native, bool = false) {
        auto& slot = bools[n];
        if (!slot) slot = new OvmsMetricBool(n);
        if (!slot->IsDefined()) slot->SetValue(v);
        return slot;
    }
    void DeregisterMetric(void*) {}
};
extern OvmsMetrics MyMetrics;

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
struct OvmsConfig {
    std::map<std::string, std::string> store;
    void RegisterParam(const char*, const char*, bool = true, bool = true) {}
    std::string GetParamValue(const char* p, const char* i, const char* def = "") const {
        auto it = store.find(std::string(p) + "/" + i);
        return it != store.end() ? it->second : def;
    }
    int GetParamValueInt(const char* p, const char* i, int def = 0) const {
        auto it = store.find(std::string(p) + "/" + i);
        return it != store.end() ? atoi(it->second.c_str()) : def;
    }
    bool GetParamValueBool(const char*, const char*, bool def = false) const { return def; }
    void SetParamValue(const char* p, const char* i, const char* v) {
        store[std::string(p) + "/" + i] = v;
    }
    bool IsDefined(const char*, const char*) const { return false; }
};
extern OvmsConfig MyConfig;

// ---------------------------------------------------------------------------
// Commands / factory
// ---------------------------------------------------------------------------
struct OvmsWriter {
    void printf(const char*, ...) {}
    void puts(const char*) {}
};
struct OvmsCommand {
    OvmsCommand* RegisterCommand(const char*, const char*, ...) { return this; }
};
struct OvmsCommandApp {
    OvmsCommand root;
    OvmsCommand* RegisterCommand(const char*, const char*, ...) { return &root; }
    void UnregisterCommand(const char*) {}
};
extern OvmsCommandApp MyCommandApp;

struct OvmsVehicleFactory {
    template<typename T> void RegisterVehicle(const char*, const char*) {}
};
extern OvmsVehicleFactory MyVehicleFactory;

// ---------------------------------------------------------------------------
// Events — counts signals so tests can assert on event-queue pressure directly,
// not just on the metric transitions that cause it.
// ---------------------------------------------------------------------------
struct OvmsEvents {
    std::map<std::string, int> signalled;
    int total = 0;
    void SignalEvent(const std::string& e, void* = nullptr, size_t = 0) {
        signalled[e]++;
        total++;
    }
    void reset() { signalled.clear(); total = 0; }
    int count(const std::string& e) const {
        auto it = signalled.find(e);
        return it == signalled.end() ? 0 : it->second;
    }
};
extern OvmsEvents MyEvents;

// ---------------------------------------------------------------------------
// OvmsVehicle base class
// ---------------------------------------------------------------------------
enum vehicle_command_t { Success, Fail, NotImplemented };

struct OvmsVehicle {
    using vehicle_command_t = ::vehicle_command_t;

    canbus* m_can1 = nullptr;
    canbus* m_can2 = nullptr;
    canbus* m_can3 = nullptr;
    canbus* m_can4 = nullptr;

    void RegisterCanBus(int bus, CAN_mode_t, CAN_speed_t) {
        canbus** slot = (bus == 1) ? &m_can1 : (bus == 2) ? &m_can2
                      : (bus == 3) ? &m_can3 : &m_can4;
        if (!*slot) {
            *slot = new canbus();
            (*slot)->m_busnumber = static_cast<uint8_t>(bus);
        }
    }

    virtual void IncomingFrameCan1(CAN_frame_t*) {}
    virtual void IncomingFrameCan2(CAN_frame_t*) {}
    virtual void IncomingFrameCan3(CAN_frame_t*) {}
    virtual void IncomingFrameCan4(CAN_frame_t*) {}
    virtual void Ticker1(uint32_t) {}
    virtual void Ticker10(uint32_t) {}

    bool PinCheck(const char*) { return true; }  // always pass in tests

    virtual vehicle_command_t CommandLock(const char*)             { return NotImplemented; }
    virtual vehicle_command_t CommandUnlock(const char*)           { return NotImplemented; }
    virtual vehicle_command_t CommandWakeup()                      { return NotImplemented; }
    virtual vehicle_command_t CommandStartCharge()                 { return NotImplemented; }
    virtual vehicle_command_t CommandStopCharge()                  { return NotImplemented; }
    virtual vehicle_command_t CommandActivateValet(const char*)    { return NotImplemented; }
    virtual vehicle_command_t CommandDeactivateValet(const char*)  { return NotImplemented; }
    virtual vehicle_command_t CommandClimateControl(bool)          { return NotImplemented; }
    virtual vehicle_command_t CommandHomelink(int, int = 1000)     { return NotImplemented; }

    virtual ~OvmsVehicle() {
        delete m_can1; delete m_can2; delete m_can3; delete m_can4;
    }
};

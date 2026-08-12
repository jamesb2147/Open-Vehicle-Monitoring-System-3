// test_commands.cpp — Command TX tests for vehicle_fiat500e.
//
// These assert the exact frames each command puts on the bus.
//
// Preconditioning used to live behind CommandActivateValet/CommandDeactivateValet
// and now lives on CommandClimateControl, with the valet commands retained as
// permanent aliases. The tests below pin the wire behaviour and prove the two
// entry points are byte-identical, so setups bound to valet mode are unaffected.

#include "mock/mock_ovms.hpp"
#include "../src/vehicle_fiat500e.h"

#include <cstdio>
#include <vector>

extern int tests_run;
extern int tests_passed;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else      { printf("  FAIL: %s\n", msg); } \
} while(0)

extern OvmsVehicleFiat500e* make_vehicle();
extern CAN_frame_t make_frame(uint32_t id, std::initializer_list<uint8_t> bytes);

// Every command frame goes out on CAN2 (B-CAN) except the Homelink ODO probe.
static canbus* bcan(OvmsVehicleFiat500e* v) { return v->m_can2; }
static canbus* ccan(OvmsVehicleFiat500e* v) { return v->m_can1; }

static bool tx_is(const TxRecord& r, uint32_t id, uint8_t len,
                  std::initializer_list<uint8_t> bytes) {
    if (r.id != id || r.len != len || !r.extended) return false;
    int i = 0;
    for (uint8_t b : bytes) {
        if (i >= 8) break;
        if (r.data[i++] != b) return false;
    }
    return true;
}

static void dump(const char* label, canbus* b) {
    printf("    %s: %zu frame(s)\n", label, b->tx_log.size());
    for (const auto& r : b->tx_log)
        printf("      %08X [%u] %02X %02X\n", r.id, r.len, r.data[0], r.data[1]);
}

// ---------------------------------------------------------------------------

static void test_wakeup() {
    printf("\ntest_wakeup\n");
    auto* v = make_vehicle();
    v->CommandWakeup();

    auto* b = bcan(v);
    CHECK(b->tx_log.size() == 3, "wakeup sends 3 frames");
    CHECK(b->tx_log.size() == 3 &&
          tx_is(b->tx_log[0], 0xE094000, 6, {0x00, 0x01, 0x00, 0x00, 0x00, 0x00}),
          "wakeup frame is NWM_BCM 0xE094000 {00,01,...}");
    delete v;
}

static void test_start_stop_charge() {
    printf("\ntest_start_stop_charge\n");
    auto* v = make_vehicle();

    v->CommandStartCharge();
    auto* b = bcan(v);
    CHECK(b->tx_log.size() == 3 && tx_is(b->tx_log[0], 0xC41401F, 1, {0x20}),
          "start charge -> TCU_REQ 0x20 x3");

    b->tx_log.clear();
    v->CommandStopCharge();
    CHECK(b->tx_log.size() == 3 && tx_is(b->tx_log[0], 0xC41401F, 1, {0x40}),
          "stop charge -> TCU_REQ 0x40 x3");

    delete v;
}

static void test_lock_unlock() {
    printf("\ntest_lock_unlock\n");
    auto* v = make_vehicle();
    auto* b = bcan(v);

    v->CommandLock("1234");
    CHECK(b->tx_log.size() == 3 && tx_is(b->tx_log[0], 0xC41401F, 1, {0x08}),
          "lock -> TCU_REQ 0x08 x3");

    b->tx_log.clear();
    v->CommandUnlock("1234");
    CHECK(b->tx_log.size() == 3 && tx_is(b->tx_log[0], 0xC41401F, 1, {0x10}),
          "unlock -> TCU_REQ 0x10 x3");

    delete v;
}

// ---------------------------------------------------------------------------
// Cabin preconditioning (server v2 command 26), plus the retained valet aliases.
// ---------------------------------------------------------------------------

// Captures the exact B-CAN traffic for a climate operation, so the valet
// aliases can be proven byte-identical to the primary command.
static std::vector<TxRecord> capture_climate(bool enable, bool via_valet) {
    auto* v = make_vehicle();
    if (via_valet) {
        if (enable) v->CommandActivateValet("anything");
        else        v->CommandDeactivateValet("anything");
    } else {
        v->CommandClimateControl(enable);
    }
    auto log = bcan(v)->tx_log;
    delete v;
    return log;
}

static bool same_traffic(const std::vector<TxRecord>& a,
                         const std::vector<TxRecord>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++) {
        if (a[i].id != b[i].id || a[i].len != b[i].len ||
            a[i].extended != b[i].extended) return false;
        for (int j = 0; j < a[i].len; j++)
            if (a[i].data[j] != b[i].data[j]) return false;
    }
    return true;
}

static void test_climate_control_start() {
    printf("\ntest_climate_control_start\n");
    auto* v = make_vehicle();

    CHECK(v->CommandClimateControl(true) == Success, "climate on returns Success");
    auto* b = bcan(v);
    dump("B-CAN", b);

    // The asymmetry from the original valet implementation is preserved:
    // starting wakes the BCM first, stopping does not.
    CHECK(b->tx_log.size() == 2, "start sends wakeup + precondition");
    CHECK(b->tx_log.size() == 2 &&
          tx_is(b->tx_log[0], 0xE094000, 6, {0x00, 0x01}),
          "first frame is the NWM_BCM wakeup");
    CHECK(b->tx_log.size() == 2 &&
          tx_is(b->tx_log[1], 0xE194031, 2, {0x20, 0x64}),
          "second frame is TCU_PRECOND_NOW start {20,64}");

    delete v;
}

static void test_climate_control_stop() {
    printf("\ntest_climate_control_stop\n");
    auto* v = make_vehicle();

    CHECK(v->CommandClimateControl(false) == Success, "climate off returns Success");
    auto* b = bcan(v);
    dump("B-CAN", b);

    CHECK(b->tx_log.size() == 2, "stop sends precondition stop x2");
    CHECK(b->tx_log.size() == 2 &&
          tx_is(b->tx_log[0], 0xE194031, 2, {0x40, 0x64}),
          "TCU_PRECOND_NOW stop {40,64}");
    CHECK(!StandardMetrics.ms_v_env_hvac->AsBool(),
          "stop clears ms_v_env_hvac optimistically");

    delete v;
}

static void test_valet_aliases_are_byte_identical() {
    printf("\ntest_valet_aliases_are_byte_identical\n");

    // The whole point of retaining the valet commands: existing setups bound to
    // them must behave exactly as before the move.
    CHECK(same_traffic(capture_climate(true,  false), capture_climate(true,  true)),
          "CommandActivateValet produces identical traffic to ClimateControl(true)");
    CHECK(same_traffic(capture_climate(false, false), capture_climate(false, true)),
          "CommandDeactivateValet produces identical traffic to ClimateControl(false)");
}

static void test_valet_metric_untouched_by_climate() {
    printf("\ntest_valet_metric_untouched_by_climate\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    // Preconditioning must no longer masquerade as valet mode: that is what
    // caused "Valet mode enabled" notifications and armed the hood/trunk
    // intrusion alerts while the cabin was being conditioned.
    v->CommandClimateControl(true);
    v->CommandActivateValet("anything");

    auto on = make_frame(0x631400A, {0, 0x40, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan2(&on);
    auto off = make_frame(0x631400A, {0, 0x00, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan2(&off);

    CHECK(g_metrics.transition_count("ms_v_env_valet") == 0,
          "no climate path ever touches ms_v_env_valet");
    CHECK(g_metrics.transition_count("ms_v_env_hvac") > 0,
          "...ms_v_env_hvac carries the state instead");

    delete v;
}

// ---------------------------------------------------------------------------

static void test_homelink_horn_lights() {
    printf("\ntest_homelink_horn_lights\n");
    auto* v = make_vehicle();
    auto* b = bcan(v);

    v->CommandHomelink(0, 1000);
    CHECK(b->tx_log.size() == 3 && tx_is(b->tx_log[0], 0xC41401F, 1, {0x02}),
          "homelink 0 -> horn/lights on");

    b->tx_log.clear();
    v->CommandHomelink(1, 1000);
    CHECK(b->tx_log.size() == 3 && tx_is(b->tx_log[0], 0xC41401F, 1, {0x00}),
          "homelink 1 -> horn/lights off");

    // Button 2 is an ODO diagnostic probe on the C-CAN, not the B-CAN.
    b->tx_log.clear();
    v->CommandHomelink(2, 1000);
    CHECK(ccan(v)->tx_log.size() == 2, "homelink 2 -> 2 diagnostic frames on C-CAN");
    CHECK(b->tx_log.empty(), "homelink 2 sends nothing on B-CAN");

    delete v;
}

void test_commands_all() {
    printf("\n--- commands ---\n");
    test_wakeup();
    test_start_stop_charge();
    test_lock_unlock();
    test_climate_control_start();
    test_climate_control_stop();
    test_valet_aliases_are_byte_identical();
    test_valet_metric_untouched_by_climate();
    test_homelink_horn_lights();
}

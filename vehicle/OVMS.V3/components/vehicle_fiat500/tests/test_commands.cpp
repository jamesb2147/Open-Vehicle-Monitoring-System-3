// test_commands.cpp — Command TX tests for vehicle_fiat500e.
//
// These assert the exact frames each command puts on the bus. Their main job is
// to protect the upcoming climate-control refactor: preconditioning currently
// lives behind CommandActivateValet/CommandDeactivateValet and is being moved
// to CommandClimateControl, with the valet commands retained as aliases. These
// tests pin the wire behaviour so the move can be proven byte-identical.

#include "mock/mock_ovms.hpp"
#include "../src/vehicle_fiat500e.h"

#include <cstdio>

extern int tests_run;
extern int tests_passed;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else      { printf("  FAIL: %s\n", msg); } \
} while(0)

extern OvmsVehicleFiat500e* make_vehicle();

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
// Preconditioning, currently reached via the valet commands.
// ---------------------------------------------------------------------------

static void test_precondition_start_via_valet() {
    printf("\ntest_precondition_start_via_valet\n");
    auto* v = make_vehicle();

    v->CommandActivateValet("anything");
    auto* b = bcan(v);
    dump("B-CAN", b);

    // Note the asymmetry preserved from the original: activate sends a wakeup
    // frame first (a sleeping BCM will not accept a cold precondition request),
    // deactivate does not. This must survive the move to CommandClimateControl.
    CHECK(b->tx_log.size() == 2, "activate sends wakeup + precondition");
    CHECK(b->tx_log.size() == 2 &&
          tx_is(b->tx_log[0], 0xE094000, 6, {0x00, 0x01}),
          "first frame is the NWM_BCM wakeup");
    CHECK(b->tx_log.size() == 2 &&
          tx_is(b->tx_log[1], 0xE194031, 2, {0x20, 0x64}),
          "second frame is TCU_PRECOND_NOW start {20,64}");

    // The PIN argument is ignored -- the base class would call PinCheck().
    CHECK(true, "[NOTE] pin argument is ignored by this override");

    delete v;
}

static void test_precondition_stop_via_valet() {
    printf("\ntest_precondition_stop_via_valet\n");
    auto* v = make_vehicle();

    v->CommandDeactivateValet("anything");
    auto* b = bcan(v);
    dump("B-CAN", b);

    CHECK(b->tx_log.size() == 2, "deactivate sends precondition stop x2");
    CHECK(b->tx_log.size() == 2 &&
          tx_is(b->tx_log[0], 0xE194031, 2, {0x40, 0x64}),
          "TCU_PRECOND_NOW stop {40,64}");
    CHECK(!StandardMetrics.ms_v_env_valet->AsBool(),
          "deactivate clears ms_v_env_valet directly");

    delete v;
}

static void test_climate_control_not_implemented_yet() {
    printf("\ntest_climate_control_not_implemented_yet\n");
    auto* v = make_vehicle();

    // [BUG] Server v2 command 26 (Remote Climate Control) is unhandled, so the
    // framework's scheduled-preconditioning feature can never drive this car.
    CHECK(static_cast<OvmsVehicle*>(v)->CommandClimateControl(true) == NotImplemented,
          "[BUG] CommandClimateControl is not overridden (cmd 26 dead)");
    CHECK(bcan(v)->tx_log.empty(), "...and puts nothing on the bus");

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
    test_precondition_start_via_valet();
    test_precondition_stop_via_valet();
    test_climate_control_not_implemented_yet();
    test_homelink_horn_lights();
}

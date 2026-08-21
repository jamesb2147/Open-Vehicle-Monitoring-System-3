// test_charge_state.cpp — Derived charge-state machine for vehicle_fiat500e.
//
// ChargingSystemSts (0xA194040) is not decoded: the old comparisons against
// (d[3]&0x30) were unreachable, and the field's width is not established well
// enough to repair on inference. See the note at that case in the module.
//
// Instead the state is derived in Ticker1 from ms_v_charge_inprogress (which the
// J1772 S2 switch drives) qualified by SOC -- the same approach taken by
// vehicle_boltev, vehicle_mgev and vehicle_vweup. Direct enum mapping is
// reserved for protocols whose values have actually been observed on a car
// (vehicle_teslaroadster).

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
extern CAN_frame_t make_frame(uint32_t id, std::initializer_list<uint8_t> bytes);

static void tick(OvmsVehicleFiat500e* v, uint32_t n = 1) {
    for (uint32_t i = 0; i < n; i++)
        static_cast<OvmsVehicle*>(v)->Ticker1(i);
}

// SOC rides in d[1]>>1 of 0xC10A040.
static void set_soc(OvmsVehicleFiat500e* v, int soc) {
    auto f = make_frame(0xC10A040, {0, (uint8_t)(soc << 1), 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan1(&f);
}

// J1772 S2 closed/open via 0x820A040.
static void set_charging(OvmsVehicleFiat500e* v, bool on) {
    auto f = make_frame(0x820A040, {0x00, (uint8_t)(on ? 0x10 : 0x00), 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan1(&f);
}

static std::string state()    { return StandardMetrics.ms_v_charge_state->AsString(); }
static std::string substate() { return StandardMetrics.ms_v_charge_substate->AsString(); }

// ---------------------------------------------------------------------------

static void test_enum_frame_no_longer_writes_state() {
    printf("\ntest_enum_frame_no_longer_writes_state\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    // Every possible value of the old 2-bit field must now be inert.
    for (uint8_t b : {0x00, 0x10, 0x20, 0x30}) {
        auto f = make_frame(0xA194040, {0, 0, 0, b, 0, 0, 0, 0});
        v->IncomingFrameCan2(&f);
    }

    CHECK(g_metrics.write_count("ms_v_charge_state") == 0,
          "0xA194040 no longer writes charge_state (was pinned to 'topoff')");
    CHECK(g_metrics.write_count("ms_v_door_chargeport") == 0,
          "0xA194040 no longer writes door_chargeport");

    delete v;
}

static void test_charge_start() {
    printf("\ntest_charge_start\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    set_soc(v, 40);
    set_charging(v, true);
    tick(v);

    CHECK(state() == "charging",       "charge start -> 'charging'");
    CHECK(substate() == "onrequest",   "substate 'onrequest'");
    CHECK(StandardMetrics.ms_v_door_chargeport->AsBool(), "chargeport true while charging");

    delete v;
}

static void test_charge_completes_at_high_soc() {
    printf("\ntest_charge_completes_at_high_soc\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    set_soc(v, 40);
    set_charging(v, true);
    tick(v);

    set_soc(v, 97);            // above FT_CHARGE_DONE_SOC (95)
    set_charging(v, false);
    tick(v);

    CHECK(state() == "done",         "stopping at 97% -> 'done'");
    CHECK(substate() == "onrequest", "substate 'onrequest'");
    CHECK(!StandardMetrics.ms_v_door_chargeport->AsBool(), "chargeport false after charge");

    delete v;
}

static void test_charge_interrupted_at_low_soc() {
    printf("\ntest_charge_interrupted_at_low_soc\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    set_soc(v, 40);
    set_charging(v, true);
    tick(v);

    set_soc(v, 60);            // below FT_CHARGE_DONE_SOC
    set_charging(v, false);
    tick(v);

    CHECK(state() == "stopped",        "stopping at 60% -> 'stopped'");
    CHECK(substate() == "interrupted", "substate 'interrupted'");

    delete v;
}

static void test_topoff_tail() {
    printf("\ntest_topoff_tail\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    set_soc(v, 40);
    set_charging(v, true);
    tick(v, 5);
    CHECK(state() == "charging", "still 'charging' at 40%");

    set_soc(v, 99);            // at FT_CHARGE_TOPOFF_SOC
    tick(v, 5);
    CHECK(state() == "topoff", "crosses to 'topoff' at 99%");

    delete v;
}

static void test_no_churn_across_a_session() {
    printf("\ntest_no_churn_across_a_session\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    // A full session: idle -> charging -> topoff -> done, with plenty of ticks
    // and frames throughout. charge_state changes fire NotifyChargeState(), so
    // the count here is a notification count, not just a metric count.
    set_soc(v, 50);
    tick(v, 10);
    set_charging(v, true);
    for (int i = 0; i < 30; i++) { set_soc(v, 50); tick(v); }
    for (int i = 0; i < 30; i++) { set_soc(v, 99); tick(v); }
    set_charging(v, false);
    tick(v, 30);

    int t = g_metrics.transition_count("ms_v_charge_state");
    printf("    charge_state transitions across the session: %d\n", t);
    CHECK(t == 3, "exactly 3 state transitions: charging -> topoff -> done");
    CHECK(state() == "done", "ends in 'done'");

    delete v;
}

static void test_idle_never_asserts_a_state() {
    printf("\ntest_idle_never_asserts_a_state\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    // Parked, unplugged, bus chattering. Previously this pinned charge_state to
    // "topoff" -- a charging state, on a car that was not charging.
    set_soc(v, 70);
    tick(v, 60);

    CHECK(g_metrics.write_count("ms_v_charge_state") == 0,
          "an idle car never has a charge state asserted");

    delete v;
}

void test_charge_state_all() {
    printf("\n--- derived charge state ---\n");
    test_enum_frame_no_longer_writes_state();
    test_charge_start();
    test_charge_completes_at_high_soc();
    test_charge_interrupted_at_low_soc();
    test_topoff_tail();
    test_no_churn_across_a_session();
    test_idle_never_asserts_a_state();
}

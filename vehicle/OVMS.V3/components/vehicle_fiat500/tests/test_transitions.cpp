// test_transitions.cpp — Metric-transition tests for vehicle_fiat500e.
//
// These are the tests that matter for module stability, and they are the reason
// the mock tracks transitions separately from writes.
//
// In the real firmware a metric that CHANGES fires MetricModified (vehicle.cpp),
// which signals an event; every event heap-allocates its name and queues it
// (ovms_events.cpp SignalEvent). If a non-"ticker.*" event is ever dropped
// because the queue is full, OVMS deliberately calls abort():
//
//     // ovms_events.cpp CheckQueueOverflow
//     // We've dropped a potentially important event, system is instable now.
//     ESP_LOGE(TAG, "%s: lost important event => aborting", from);
//     abort();
//
// So a metric oscillating at CAN frame rate is not a cosmetic problem -- it is a
// reboot. Writes are harmless; transitions are what cost.
//
// Both tests below currently document DEFECTIVE behaviour: they assert that the
// transition count is pathologically high. When the fixes land, these assertions
// inverts to a small bound.

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

// ---------------------------------------------------------------------------
// 1. charge_inprogress driven by two contradictory frames
// ---------------------------------------------------------------------------

static void test_charge_inprogress_storm() {
    printf("\ntest_charge_inprogress_storm\n");
    auto* v = make_vehicle();
    g_metrics.reset();   // ignore construction-time metric init

    // Realistic "plugged in, not yet drawing current" state: the J1772 pilot
    // switch reports closed while the BPCM has not asserted RdyForChrg. Both
    // frames are periodic, so they interleave on the bus indefinitely.
    auto s2_closed   = make_frame(0x820A040, {0x00, 0x10, 0, 0, 0, 0, 0, 0});
    auto not_ready   = make_frame(0x640A046, {0, 0, 0, 0, 0, 0x00, 0, 0});

    const int cycles = 50;
    for (int i = 0; i < cycles; i++) {
        v->IncomingFrameCan1(&s2_closed);
        v->IncomingFrameCan1(&not_ready);
    }

    int transitions = g_metrics.transition_count("ms_v_charge_inprogress");
    printf("    charge_inprogress transitions over %d frame pairs: %d\n",
           cycles, transitions);

    // FIXED: 0x820A040 alone owns the metric, so a disagreeing readiness flag
    // no longer flips it. The state is reached once and then holds.
    // Before the fix this was 100 transitions (2 per frame pair).
    CHECK(transitions == 1,
          "charge_inprogress settles after 1 transition (was 100)");

    delete v;
}

// ---------------------------------------------------------------------------
// 2. valet/precondition flapping around the climate setpoint
// ---------------------------------------------------------------------------

static void test_precondition_setpoint_flap() {
    printf("\ntest_precondition_setpoint_flap\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    // A climate system holding temperature cycles between "actively heating"
    // (0x40) and "setpoint reached" (0x80). Both mean preconditioning is ON.
    auto heating  = make_frame(0x631400A, {0, 0x40, 0, 0, 0, 0, 0, 0});
    auto at_temp  = make_frame(0x631400A, {0, 0x80, 0, 0, 0, 0, 0, 0});

    const int cycles = 25;
    for (int i = 0; i < cycles; i++) {
        v->IncomingFrameCan2(&heating);
        v->IncomingFrameCan2(&at_temp);
    }

    int transitions = g_metrics.transition_count("ms_v_env_valet");
    printf("    valet transitions over %d thermostat cycles: %d\n",
           cycles, transitions);

    // FIXED: "setpoint reached" now maps to active, so holding temperature no
    // longer toggles the metric -- and no longer fires a push notification per
    // thermostat cycle. Before the fix this was 50 transitions.
    CHECK(transitions == 1,
          "valet holds steady across thermostat cycles (was 50)");

    delete v;
}

// ---------------------------------------------------------------------------
// 3. Baseline: a metric driven by a single source should settle
// ---------------------------------------------------------------------------

static void test_stable_metric_settles() {
    printf("\ntest_stable_metric_settles\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    // Control case, proving the transition counter measures what we think it
    // does: a repeated identical frame must produce exactly one transition
    // regardless of how many times it is delivered.
    auto f = make_frame(0x6414000, {0, 0, 0, 0x40, 0, 0, 0, 0});
    for (int i = 0; i < 100; i++)
        v->IncomingFrameCan2(&f);

    CHECK(g_metrics.transition_count("ms_v_env_locked") == 1,
          "repeated identical frame -> exactly 1 transition");
    CHECK(g_metrics.write_count("ms_v_env_locked") == 100,
          "...but 100 writes (this is why writes are the wrong thing to assert)");

    delete v;
}

void test_transitions_all() {
    printf("\n--- metric transitions (event-queue pressure) ---\n");
    test_charge_inprogress_storm();
    test_precondition_setpoint_flap();
    test_stable_metric_settles();
}

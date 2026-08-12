// test_lifecycle.cpp — Sleep/wake detection for vehicle_fiat500e.
//
// The module previously had an empty Ticker1 and never touched ms_v_env_awake,
// so OVMS never learned the car had gone to sleep: vehicle.asleep never fired
// and the framework's auto-poweroff never engaged. On a parked car that means
// the module keeps drawing from the 12V battery indefinitely.

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

// Ticker1 is protected; dispatch through the base pointer.
static void tick(OvmsVehicleFiat500e* v, uint32_t n = 1) {
    for (uint32_t i = 0; i < n; i++)
        static_cast<OvmsVehicle*>(v)->Ticker1(i);
}

// ---------------------------------------------------------------------------

static void test_boot_next_to_sleeping_car_is_quiet() {
    printf("\ntest_boot_next_to_sleeping_car_is_quiet\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    // No traffic at all. The module must not announce a sleep transition it
    // never observed -- that would emit a spurious vehicle.asleep at every boot.
    tick(v, 30);

    CHECK(g_metrics.transition_count("ms_v_env_awake") == 0,
          "no awake transition when no traffic has ever been seen");

    delete v;
}

static void test_bus_activity_marks_awake() {
    printf("\ntest_bus_activity_marks_awake\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    auto f = make_frame(0x6414000, {0, 0, 0, 0x40, 0, 0, 0, 0});
    v->IncomingFrameCan2(&f);

    CHECK(StandardMetrics.ms_v_env_awake->AsBool(), "bus activity -> awake");
    CHECK(g_metrics.transition_count("ms_v_env_awake") == 1,
          "exactly one awake transition");

    delete v;
}

static void test_awake_is_idempotent_per_frame() {
    printf("\ntest_awake_is_idempotent_per_frame\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    // CanActivity() runs on every single frame. It must not itself become a
    // source of metric churn -- the exact failure this module was suffering
    // from elsewhere.
    auto f = make_frame(0x6414000, {0, 0, 0, 0x40, 0, 0, 0, 0});
    for (int i = 0; i < 500; i++)
        v->IncomingFrameCan2(&f);

    CHECK(g_metrics.transition_count("ms_v_env_awake") == 1,
          "500 frames still produce exactly 1 awake transition");

    delete v;
}

static void test_sleep_after_timeout() {
    printf("\ntest_sleep_after_timeout\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    auto f = make_frame(0x6214000, {0x20, 0x00, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan1(&f);
    CHECK(StandardMetrics.ms_v_env_awake->AsBool(), "awake after frame");

    // Timeout is 10s; 9 ticks must not be enough.
    tick(v, 9);
    CHECK(StandardMetrics.ms_v_env_awake->AsBool(),
          "still awake after 9s of silence");

    tick(v, 1);
    CHECK(!StandardMetrics.ms_v_env_awake->AsBool(),
          "asleep after 10s of silence");

    // And it must latch -- not re-fire on every subsequent tick.
    tick(v, 50);
    CHECK(g_metrics.transition_count("ms_v_env_awake") == 2,
          "sleep latches: exactly 2 transitions total (wake + sleep)");

    delete v;
}

static void test_traffic_resets_watchdog() {
    printf("\ntest_traffic_resets_watchdog\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    auto f = make_frame(0x6214000, {0x20, 0x00, 0, 0, 0, 0, 0, 0});

    // Sustained traffic with 9s gaps must never declare sleep.
    for (int i = 0; i < 10; i++) {
        v->IncomingFrameCan1(&f);
        tick(v, 9);
    }
    CHECK(StandardMetrics.ms_v_env_awake->AsBool(),
          "periodic traffic keeps the car awake");
    CHECK(g_metrics.transition_count("ms_v_env_awake") == 1,
          "...with no spurious sleep/wake churn");

    delete v;
}

static void test_wake_after_sleep() {
    printf("\ntest_wake_after_sleep\n");
    auto* v = make_vehicle();
    g_metrics.reset();

    auto f = make_frame(0x6214000, {0x20, 0x00, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan1(&f);
    tick(v, 10);
    CHECK(!StandardMetrics.ms_v_env_awake->AsBool(), "asleep");

    v->IncomingFrameCan1(&f);
    CHECK(StandardMetrics.ms_v_env_awake->AsBool(), "wakes again on new traffic");
    CHECK(g_metrics.transition_count("ms_v_env_awake") == 3,
          "wake -> sleep -> wake = 3 transitions");

    delete v;
}

void test_lifecycle_all() {
    printf("\n--- lifecycle (sleep/wake detection) ---\n");
    test_boot_next_to_sleeping_car_is_quiet();
    test_bus_activity_marks_awake();
    test_awake_is_idempotent_per_frame();
    test_sleep_after_timeout();
    test_traffic_resets_watchdog();
    test_wake_after_sleep();
}

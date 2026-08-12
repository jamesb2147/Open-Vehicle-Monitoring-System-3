// test_can_decode.cpp — Native tests for vehicle_fiat500e CAN frame decoding.
//
// These are CHARACTERIZATION tests: they lock in what the module does TODAY,
// including its bugs. Cases that encode known-defective behaviour are marked
// [BUG] and reference the analysis. When a fix lands, the corresponding
// assertion flips — which makes the behavioural delta explicit in the diff for
// a reviewer who has the vehicle and can sanity-check it against reality.
//
// Run:  make test   (from the tests/ directory)

#include "mock/mock_ovms.hpp"
#include "../src/vehicle_fiat500e.h"

#include <cmath>
#include <cstdio>

int tests_run = 0;
int tests_passed = 0;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else      { printf("  FAIL: %s\n", msg); } \
} while(0)

static bool near(float a, float b, float tol = 0.01f) {
    return std::fabs(a - b) < tol;
}

OvmsVehicleFiat500e* make_vehicle() {
    g_metrics.reset();
    MyEvents.reset();
    return new OvmsVehicleFiat500e();
}

CAN_frame_t make_frame(uint32_t id, std::initializer_list<uint8_t> bytes) {
    CAN_frame_t f{};
    f.MsgID       = id;
    f.FIR.B.FF    = CAN_frame_ext;
    f.FIR.B.DLC   = static_cast<uint8_t>(bytes.size());
    int i = 0;
    for (uint8_t b : bytes) f.data.u8[i++] = b;
    return f;
}

// ---------------------------------------------------------------------------
// CAN1 — C-CAN, 500 kbps
// ---------------------------------------------------------------------------

static void test_bat_soc_range_energy() {
    printf("\ntest_bat_soc_range_energy (0xC10A040)\n");
    auto* v = make_vehicle();

    // d[0] = range, d[1]>>1 = SOC, (d[2]<<5)|(d[3]>>3) = raw energy
    auto f = make_frame(0xC10A040, {0x64, 0x96, 0x0A, 0x40, 0, 0, 0, 0});
    v->IncomingFrameCan1(&f);

    CHECK(near(StandardMetrics.ms_v_bat_range_est->AsFloat(), 100.0f),
          "range_est 100 km from d[0]=0x64");
    CHECK(near(StandardMetrics.ms_v_bat_soc->AsFloat(), 75.0f),
          "SOC 75% from d[1]=0x96>>1");
    // (0x0A<<5)|(0x40>>3) = 320|8 = 328 -> 3.28 - 24
    CHECK(near(StandardMetrics.ms_v_bat_energy_used->AsFloat(), -20.72f),
          "energy_used -20.72 from raw 328");

    delete v;
}

static void test_charge_inprogress_two_sources() {
    printf("\ntest_charge_inprogress_two_sources\n");
    auto* v = make_vehicle();

    // 0x820A040 (J1772_S2_Close) says charging
    auto a = make_frame(0x820A040, {0x00, 0x10, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan1(&a);
    CHECK(StandardMetrics.ms_v_charge_inprogress->AsBool(),
          "charge_inprogress true from 0x820A040 S2_Close");

    // FIXED: 0x640A046 (RdyForChrg) no longer writes this metric, so a
    // disagreeing readiness flag can no longer flip it. 0x820A040 owns it.
    auto b = make_frame(0x640A046, {0, 0, 0, 0, 0, 0x00, 0, 0});
    v->IncomingFrameCan1(&b);
    CHECK(StandardMetrics.ms_v_charge_inprogress->AsBool(),
          "0x640A046 no longer overrides charge_inprogress");

    // ...and S2 opening still clears it, so the metric remains live.
    auto s2_open = make_frame(0x820A040, {0x00, 0x00, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan1(&s2_open);
    CHECK(!StandardMetrics.ms_v_charge_inprogress->AsBool(),
          "S2 opening clears charge_inprogress");

    delete v;
}

static void test_drivetrain_temps() {
    printf("\ntest_drivetrain_temps (0x400A042)\n");
    auto* v = make_vehicle();

    auto f = make_frame(0x400A042, {100, 90, 80, 0, 0, 0, 0, 0});
    v->IncomingFrameCan1(&f);

    CHECK(near(StandardMetrics.ms_v_mot_temp->AsFloat(), 50.0f), "motor temp 50C");
    CHECK(near(StandardMetrics.ms_v_gen_temp->AsFloat(), 40.0f), "rotor temp 40C");
    CHECK(near(StandardMetrics.ms_v_inv_temp->AsFloat(), 30.0f), "inverter temp 30C");

    delete v;
}

static void test_ac_heater_power() {
    printf("\ntest_ac_heater_power (0xC08A040)\n");
    auto* v = make_vehicle();

    auto f = make_frame(0xC08A040, {10, 0, 0, 0, 20, 0, 0, 0});
    v->IncomingFrameCan1(&f);

    CHECK(near((float)g_metrics.numbers["xse.v.b.acelec.pwr"], 40.0f),
          "AC electric power 40W from d[0]=10");
    CHECK(near((float)g_metrics.numbers["xse.v.b.htrelec.pwr"], 80.0f),
          "heater electric power 80W from d[4]=20");

    delete v;
}

static void test_obcm_charge_values() {
    printf("\ntest_obcm_charge_values (0xC50A049)\n");
    auto* v = make_vehicle();

    // charge_temp=d[0]; cvolt=(d[1]<<8)+d[2]; ccurr=(d[3]<<8)|d[4]; bvolt=(d[5]<<8)|d[6]
    auto f = make_frame(0xC50A049, {30, 0x09, 0x60, 0x01, 0x2C, 0x0F, 0xA0, 0});
    v->IncomingFrameCan1(&f);

    CHECK(near(StandardMetrics.ms_v_charge_voltage->AsFloat(), 240.0f), "charge voltage 240V");
    CHECK(near(StandardMetrics.ms_v_charge_current->AsFloat(),  10.0f), "charge current 10A");
    CHECK(near(StandardMetrics.ms_v_bat_voltage->AsFloat(),    400.0f), "HV battery voltage 400V");
    CHECK(near(StandardMetrics.ms_v_charge_temp->AsFloat(),     30.0f), "charger temp 30C");

    delete v;
}

static void test_battery_temp_malformed() {
    printf("\ntest_battery_temp_malformed (0x840A046)\n");
    auto* v = make_vehicle();

    // [BUG] btemp = ((d[2]&0x7f)<<1) | (d[2]&0x80) reads d[2] TWICE, where the
    // adjacent comment diagrams d[2] and d[3]. The two masked ranges also
    // overlap, so the expression is malformed however it is read.
    auto f = make_frame(0x840A046, {0, 0, 0x14, 0x80, 0, 0, 0, 0});
    v->IncomingFrameCan1(&f);

    // (0x14&0x7f)<<1 = 40; (0x14&0x80) = 0  ->  40 - 40 = 0
    CHECK(near(StandardMetrics.ms_v_bat_temp->AsFloat(), 0.0f),
          "[BUG] battery temp uses d[2] twice, ignores d[3]");

    delete v;
}

// ---------------------------------------------------------------------------
// CAN2 — B-CAN, 50 kbps
// ---------------------------------------------------------------------------

static void test_body_status() {
    printf("\ntest_body_status (0x6214000)\n");
    auto* v = make_vehicle();

    auto f = make_frame(0x6214000, {0x20, 0x4C, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan2(&f);

    CHECK(StandardMetrics.ms_v_env_handbrake->AsBool(), "handbrake on");
    CHECK(StandardMetrics.ms_v_door_fl->AsBool(),       "driver door open");
    CHECK(StandardMetrics.ms_v_door_fr->AsBool(),       "passenger door open");
    CHECK(StandardMetrics.ms_v_door_trunk->AsBool(),    "hatch open");

    delete v;
}

static void test_precondition_states() {
    printf("\ntest_precondition_states (0x631400A)\n");
    auto* v = make_vehicle();

    // PreCondCabinSts lives in d[1] bits 6-7: 0x00 off, 0x40 on, 0x80 setpoint reached.
    auto off = make_frame(0x631400A, {0, 0x00, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan2(&off);
    CHECK(!StandardMetrics.ms_v_env_hvac->AsBool(), "precondition off -> hvac false");

    auto on = make_frame(0x631400A, {0, 0x40, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan2(&on);
    CHECK(StandardMetrics.ms_v_env_hvac->AsBool(), "precondition on -> hvac true");

    // FIXED: 0x80 = "setpoint reached" still means the cabin is being
    // conditioned, so it now maps to active instead of falling to default.
    auto setpoint = make_frame(0x631400A, {0, 0x80, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan2(&setpoint);
    CHECK(StandardMetrics.ms_v_env_hvac->AsBool(),
          "setpoint-reached (0x80) is treated as active");

    auto both = make_frame(0x631400A, {0, 0xC0, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan2(&both);
    CHECK(StandardMetrics.ms_v_env_hvac->AsBool(), "0xC0 -> hvac true");

    delete v;
}

static void test_charge_state_dead_branches() {
    printf("\ntest_charge_state_dead_branches (0xA194040)\n");
    auto* v = make_vehicle();

    // Not charging: the only reachable branch.
    auto idle = make_frame(0xA194040, {0, 0, 0, 0x00, 0, 0, 0, 0});
    v->IncomingFrameCan2(&idle);
    CHECK(StandardMetrics.ms_v_charge_state->AsString() == "topoff",
          "[BUG] not-charging sets charge_state to 'topoff'");
    CHECK(!StandardMetrics.ms_v_door_chargeport->AsBool(), "chargeport false when idle");

    // [BUG] (d[3]&0x30) yields only 0x00/0x10/0x20/0x30, so the comparisons
    // against 0x1/0x2/0x3 are unreachable. A charging frame updates NOTHING.
    int before = g_metrics.write_count("ms_v_charge_state");
    auto charging = make_frame(0xA194040, {0, 0, 0, 0x10, 0, 0, 0, 0});
    v->IncomingFrameCan2(&charging);
    CHECK(g_metrics.write_count("ms_v_charge_state") == before,
          "[BUG] charging state 0x10 matches no branch - no metric written");

    delete v;
}

static void test_locked() {
    printf("\ntest_locked (0x6414000)\n");
    auto* v = make_vehicle();

    auto f = make_frame(0x6414000, {0, 0, 0, 0x40, 0, 0, 0, 0});
    v->IncomingFrameCan2(&f);
    CHECK(StandardMetrics.ms_v_env_locked->AsBool(), "doors locked");

    delete v;
}

static void test_env_conditions_early_break() {
    printf("\ntest_env_conditions_early_break (0x63D4000)\n");
    auto* v = make_vehicle();

    // FIXED: both signals now decode from the same frame.
    auto warm = make_frame(0x63D4000, {100, 0x50, 0, 0, 0, 0, 0, 0});
    v->IncomingFrameCan2(&warm);
    CHECK(near(StandardMetrics.ms_v_env_temp->AsFloat(), 10.0f),
          "ambient temp 10C from d[0]=100");
    // (0x50 & 0x7f) * 0.16 = 80 * 0.16 = 12.8
    CHECK(near(StandardMetrics.ms_v_bat_12v_voltage->AsFloat(), 12.8f),
          "12V voltage decodes even when ambient temp is non-zero");

    // ...and it lands on the 12V metric, not the HV pack metric, which is
    // written from the OBCM frame with ~400V values.
    CHECK(g_metrics.write_count("ms_v_bat_voltage") == 0,
          "HV ms_v_bat_voltage untouched by this frame");

    delete v;
}

static void test_cabin_temp_precedence() {
    printf("\ntest_cabin_temp_precedence (0xC414000)\n");
    auto* v = make_vehicle();

    // FIXED: the mask is now parenthesised, so the top bit of d[4] is read.
    // was:  (0x50<<1) | (0x80 & 1)        = 160 | 0 = 160 -> 40.0
    // now:  (0x50<<1) | ((0x80&0x80)>>7)  = 160 | 1 = 161 -> 40.5
    auto f = make_frame(0xC414000, {0, 0, 0, 0x50, 0x80, 0, 0, 0});
    v->IncomingFrameCan2(&f);
    CHECK(near(StandardMetrics.ms_v_env_cabintemp->AsFloat(), 40.5f),
          "cabin temp includes d[4] top bit");

    // The complementary case: top bit clear must give the even half-degree.
    auto g = make_frame(0xC414000, {0, 0, 0, 0x50, 0x00, 0, 0, 0});
    v->IncomingFrameCan2(&g);
    CHECK(near(StandardMetrics.ms_v_env_cabintemp->AsFloat(), 40.0f),
          "cabin temp 40.0 when d[4] top bit is clear");

    delete v;
}

static void test_odometer() {
    printf("\ntest_odometer (0xC014003)\n");
    auto* v = make_vehicle();

    auto f = make_frame(0xC014003, {0, 0x01, 0x02, 0x03, 0, 0, 0, 0});
    v->IncomingFrameCan2(&f);
    // (0x01<<16)|(0x02<<8)|0x03 = 66051
    CHECK(near(StandardMetrics.ms_v_pos_odometer->AsFloat(), 66051.0f),
          "odometer 66051 km");

    delete v;
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

void test_transitions_all();
void test_commands_all();
void test_lifecycle_all();

int main() {
    printf("=== vehicle_fiat500e native tests ===\n");
    printf("\n--- CAN1 decode ---\n");
    test_bat_soc_range_energy();
    test_charge_inprogress_two_sources();
    test_drivetrain_temps();
    test_ac_heater_power();
    test_obcm_charge_values();
    test_battery_temp_malformed();

    printf("\n--- CAN2 decode ---\n");
    test_body_status();
    test_precondition_states();
    test_charge_state_dead_branches();
    test_locked();
    test_env_conditions_early_break();
    test_cabin_temp_precedence();
    test_odometer();

    test_transitions_all();
    test_commands_all();
    test_lifecycle_all();

    printf("\n=== %d/%d passed ===\n", tests_passed, tests_run);
    return tests_passed == tests_run ? 0 : 1;
}

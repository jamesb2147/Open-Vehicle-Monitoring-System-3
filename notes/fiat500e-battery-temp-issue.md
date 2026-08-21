# vehicle_fiat500: battery temperature decode needs a vehicle capture

*(Draft GitHub issue — not yet filed.)*

The expression is **provably wrong as written**, but unlike the other defects
fixed in the accompanying PRs it cannot be *corrected* without knowing what the
bytes actually carry, and I do not have access to a 500e.

Filing rather than guessing: `ms_v_bat_temp` drives user-facing alerts, so a
plausible-but-wrong decode is worse than a visibly wrong one, because it stops
looking wrong.

File: `vehicle/OVMS.V3/components/vehicle_fiat500/src/vehicle_fiat500e.cpp`

---

## `0x840A046` (MSG08_BPCM) — ModTempAvg reads `d[2]` twice

```c
float btemp = (((d[2] & 0x7f) << 1) | (d[2] & 0x80));
StandardMetrics.ms_v_bat_temp->SetValue(btemp-40);
//ModTempAvg (Modultemperatur)
//d[2] 011111111
//d[3]           100000000
```

Two independent problems:

1. **The comment diagrams a value spanning `d[2]` and `d[3]`**, but the
   expression reads `d[2]` for both halves. `d[3]` is never touched.

2. **The expression is malformed regardless of which bytes it reads.**
   `(x & 0x7f) << 1` occupies bits 1-7, and `x & 0x80` occupies bit 7. The two
   operands overlap on bit 7, so the OR is ambiguous even in isolation.

### Observable effect

Battery temperature is wrong by a data-dependent amount. With `d[2] = 0x50` it
reports 120 degrees C. A native test pinning the current behaviour is in
`tests/test_can_decode.cpp` (`test_battery_temp_malformed`).

### Why it is not fixed

The most likely intent is an 8-bit value split 7+1 across two bytes:

```c
float btemp = ((d[2] & 0x7f) << 1) | ((d[3] & 0x80) >> 7);
```

But that is inference from a hand-drawn ASCII bit diagram, and the scaling was
evidently never settled either — the surrounding commented-out alternatives
disagree:

```c
//StandardMetrics.ms_v_bat_temp->SetValue(btemp*0.5-40);
StandardMetrics.ms_v_bat_temp->SetValue(btemp-40);
```

So there are two unknowns (bit layout and scale) and no way to separate them
from the source alone.

### What would settle it

A capture of `0x840A046` alongside a known pack temperature. Even a single
cold-soak reading at a known ambient would disambiguate offset and scale; a
capture spanning a temperature change would confirm the bit layout.

---

## Related, still open: `ChargingSystemSts` remains undecoded

The accompanying PR removes the unreachable charge-state branches at
`0xA194040` and derives charge state from `ms_v_charge_inprogress` plus SOC
instead — the idiom `vehicle_boltev`, `vehicle_mgev` and `vehicle_vweup` all
use. So this is no longer a *defect*, but the field is still not decoded, and
decoding it properly would be an improvement.

Anyone attempting it should know the width is doubtful:

* the in-code comment diagrammed the mask as `01110000` (0x70, three bits)
  while the code used `0x30` (two bits);
* the file header describes `ChargingSystemSts` as a 3-bit field (`28/3`) on a
  **different** message, `STATUS_C_EVCU` (`0xA18A040`), which appears commented
  out elsewhere in the module using `(d[3]&0x70)>>4`.

A capture across a full charge session — plug in, charge, complete, unplug —
showing `d[3]` of both `0xA194040` and `0xA18A040` would resolve it.

---

## Minor, unrelated

`vehicle_fiat500e.h` declares eleven metric pointers that are never assigned or
used anywhere in the module:

```c
OvmsMetricFloat *mt_mb_trip_reset;        // Distance since reset
OvmsMetricFloat *mt_mb_trip_start;
OvmsMetricFloat *mt_mb_consumption_start;
OvmsMetricFloat *mt_mb_eco_accel;
OvmsMetricFloat *mt_mb_eco_const;
OvmsMetricFloat *mt_mb_eco_coast;
OvmsMetricFloat *mt_mb_eco_score;
OvmsMetricFloat *mt_mb_fl_speed;
OvmsMetricFloat *mt_mb_fr_speed;
OvmsMetricFloat *mt_mb_rl_speed;
OvmsMetricFloat *mt_mb_rr_speed;
```

The `mt_mb_` prefix suggests they were copied from a Mercedes module. They are
uninitialised and never dereferenced, so harmless today, but a trap for anyone
who starts using one.

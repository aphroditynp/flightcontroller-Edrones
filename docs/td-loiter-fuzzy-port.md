# TD/V10-Trainer2 Inner Loop + LOITER + Fuzzy L1 Period

## What changed

The previous roll/pitch LQR inner loop is no longer active. `AttitudeController`
now reproduces the active fixed-wing controller used by TD/V10-trainer2:

- Roll: `P=10`, `I=0`, `D=0`
- Pitch: `P=10`, `I=0`, `D=0.20`
- Raw TD output is clamped to `+/-400` and then converted through the same
  `11.378 raw-units/deg` scale used by this refactor.
- Aileron output is reversed exactly like TD (`1500 - u_roll`).
- Rudder correction is neutral (`u_yaw=0`).

The current project's bench-verified IMU convention is retained: `x=roll rate`,
`y=pitch rate`. TD's old wrapper passed `gyro_y` to roll and `gyro_x` to pitch
because its sensor wrapper used a different raw-axis convention; copying those
variable names into this refactor would reintroduce an already-fixed axis bug.

## FBWA

TD mapping is preserved:

- roll stick: `1.3 * map(..., -35,+35 deg)`; internally converted to this
  project's desired-bank sign convention.
- pitch stick: `1.1 * map(..., -25,+35 deg)`.

## LOITER

A dedicated `ModeLoiter` was added. AUTO transitions to LOITER at the final
mission waypoint. MAVLink custom mode `12` can also enter LOITER directly.
Direct LOITER captures the aircraft's current position as the circle center.
Default runtime values match TD's parameter defaults: 50 m radius, CW, 35 deg
bank limit.

The L1 circular controller was already present in this project and is
algorithmically the same ArduPilot-derived loop used by TD.

## Thesis-specific fuzzy adaptation

TD uses a fixed L1 period of 20 s. This project keeps 20 s as the baseline but
runs `FuzzyL1Tuner` online.

`FuzzyL1Tuner` changes the L1 `period` using crosstrack error and its rate. In
LOITER, `L1Controller::updateLoiter()` computes:

```
omega = 2*pi/T
Kx    = omega^2
Kv    = 2*zeta*omega
```

Therefore fuzzy adaptation of `T` directly adapts the circle-tracking gains:
smaller period -> larger Kx/Kv -> more aggressive correction; larger period ->
smaller Kx/Kv -> gentler correction.

For the experimental comparison, `FUZZY_ENABLE=0` freezes the period at the
conventional baseline and `FUZZY_ENABLE=1` enables fuzzy adaptation. A power
cycle is required after changing Params, matching the rest of this firmware.

`g_fuzzyConfig.base_period_s` is synchronized from the EEPROM-loaded
`L1_PERIOD` before the fuzzy tuner is constructed, so the fuzzy scale always
uses the actual baseline being tested.

## EEPROM migration

The parameter schema magic is bumped from `0xFC01` to `0xFC02`. This prevents
old LQR gains saved in EEPROM from silently overriding the new TD defaults.
The familiar GCS names are retained:

- `ROLL_KP = 10`
- `ROLL_KRATE = 0`
- `ROLL_KI = 0`
- `PITCH_KP = 10`
- `PITCH_KRATE = 0.20`
- `PITCH_KI = 0`
- `L1_PERIOD = 20 s`

## Before flight

1. Remove the propeller / make propulsion safe.
2. Verify MANUAL roll direction after the TD aileron reversal.
3. In FBWA, roll the aircraft right by hand; the aileron correction must
   generate a restoring left-roll moment.
4. Raise the nose; elevator correction must generate a restoring nose-down
   moment.
5. Verify center stick pitch behavior: TD intentionally has a +5.5 deg FBWA
   pitch command at nominal 1500 us because its mapping is asymmetric.
6. Test FBWA before AUTO/LOITER.
7. Log desired/actual roll and pitch, servo PWM, crosstrack error, L1 period,
   and fuzzy error-rate during LOITER evaluation.

The code preserves this project's TECS auto-throttle path in AUTO/LOITER.
TD/V10-trainer2 currently comments out its auto-throttle output and uses pilot
throttle there; that unrelated difference was intentionally not copied.

# Refactor Modes (FW-only)

## Automatic LOITER after AUTO

The firmware does not switch to LOITER automatically after the last AUTO
waypoint by default. This behavior is controlled in
[`include/FC_Config.h`](../include/FC_Config.h):

```cpp
#define FC_AUTO_LOITER_ENABLE 0
```

Set it to `1` and reflash to enable the automatic transition. Leave it at `0`
to keep the active mode as AUTO after the mission finishes. LOITER remains
available as an explicit MAVLink mode (`custom_mode=12`); this setting only
controls the automatic transition from AUTO.

## Throttle in AUTO and LOITER

`AUTO` and `LOITER` use the RC throttle channel directly. Navigation and
attitude remain autonomous, but propulsion is written from
`radio.channelThrottle()` in `src/modes/FixedWingModes.cpp`. TECS does not
command the throttle in these two modes (`auto_throttle_mode` is disabled).

Menggabungkan `Mode_setup.h` + `Mode_Manager.h` + `Mode_Fixedwing.h` lama menjadi
struktur FW-only: `ModeId` punya mode fixed-wing (`Manual`, `Fbwa`, `Auto`,
`Guided`, dan `Loiter`; `COPT`/`QHOV`/`TRNS` dihapus), dan **tidak ada lagi cabang runtime
`model_uav`** -- program lama meng-include `Mode_Flywing.h`/`Mode_Copter.h`/
`Mode_Transisi.h` TANPA SYARAT (dikompilasi selalu, dipilih saat runtime);
build baru ini FW-only secara compile-time, kode copter/VTOL tidak pernah ada
dalam biner sama sekali.

## Struktur modul

```text
include/modes/VehicleContext.h    Bag-of-references ke semua subsistem + snapshot per-loop
include/modes/Mode.h              ModeId, ModeBase, ModeManager (port persis dari Mode_setup.h)
include/modes/FixedWingModes.h    4 kelas mode: Manual, Fbwa, Auto, Guided
src/modes/FixedWingModes.cpp
```

## Perubahan arsitektur signifikan: satu attitude controller, bukan dua

Kode lama punya **dua implementasi attitude control paralel yang redundan**
(ditemukan saat audit awal): `FW_CONTROL::updateFBWA_FW()` (P+D sederhana
berbasis stick, dipakai FBWA) dan `Attitude.h::stabilize()` (rate-PID penuh,
dipakai AUTO/GUIDED lewat `FW_roll/pitch/yaw_controller.h`). Refactor ini
**hanya punya satu**: `AttitudeController` (LQR, Fase 4), dipakai oleh
**ketiga** mode (FBWA, AUTO, GUIDED) -- FBWA menghitung setpoint roll/pitch
dari stick (`roll_cmd = 1.3 * map(stick)`, `pitch_cmd = 1.1 * map(stick)`,
formula sama seperti dijelaskan `PROMPT_SIMULASI_FIXEDWING.md`), sementara
AUTO/GUIDED menghitung setpoint dari L1/TECS -- keduanya diteruskan ke
`AttitudeController` LQR yang sama.

**Konsekuensi**: axis yaw di FBWA kini **sepenuhnya otomatis**
(coordinated-turn feedforward dari setpoint roll, sama seperti AUTO), bukan
campuran manual-stick + auto-mix seperti lama. Ini penyederhanaan yang
disengaja begitu yaw dikendalikan LQR, bukan rudder-mix manual.

## Dependency injection: `VehicleContext`

Program lama mengakses semua subsistem lewat singleton global
(`imu`, `ahrs`, `fw_control`, dst). Setiap kelas mode di refactor ini
menyimpan `VehicleContext&` (referensi ke seluruh subsistem + snapshot GNSS/dt
yang di-refresh scheduler tiap loop) -- bukan mengakses global. `VehicleContext`
dibangun dan dimiliki lapisan integrasi (Fase 8, `main.cpp`), bukan oleh mode
itu sendiri.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `ModeId` (COPT/QHOV/GUIDED/MANU/FBWA/AUTO/TRNS) | `fc::ModeId` (Manual/Fbwa/Auto/Guided saja) |
| `ModeBase`/`ModeManager` | `fc::ModeBase`/`fc::ModeManager` (interface identik, port langsung) |
| `ModeManual_FW` | `fc::ModeManual` |
| `ModeFBWA_FW` | `fc::ModeFbwa` (sekarang lewat `AttitudeController`, bukan `FW_CONTROL`) |
| `ModeAuto_FW` | `fc::ModeAuto` |
| `ModeGuided_FW` | `fc::ModeGuided` |
| `init_auto()`/`exit_auto()` | Diganti `ModeAuto::_enter()`/`_exit()`, menyetel `navigation.state().auto_navigation_mode`/`auto_throttle_mode` langsung (bukan global terpisah) |
| `enterauto` | Tidak ada lagi -- state resume/first-entry waypoint ditangani `Navigation::navigate()` sendiri lewat `mission.flag_wp` |

## File lama yang tidak dimigrasikan

```text
include/Mode.h            (legacy scaffolding lama, sudah superseded Mode_setup.h di kode asli)
include/Mode_setup.h
include/Mode_Manager.h
include/Mode_Fixedwing.h
include/Mode_Copter.h, Mode_Flywing.h, Mode_Transisi.h, Transition.h   (copter/VTOL, di luar cakupan)
```

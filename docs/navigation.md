# Refactor Navigation (orkestrasi AUTO/GUIDED)

Menggabungkan `Navigation.h` dan `Auto_setup.h` lama menjadi satu `class Navigation`
yang memegang satu `MissionState` (menggantikan kumpulan variabel global lepas), dan
memanggil `L1Controller`/`Tecs` langsung (bukan `calc_nav_roll()`/`calc_nav_pitch()`
terpisah di `FW_ControlModes.h` yang menulis global `nav_roll_deg`/`nav_pitch_deg`).

## Struktur modul

```text
include/navigation/MissionState.h   Struct state misi (pengganti Auto_setup.h globals)
include/navigation/Navigation.h     Kontrak class Navigation
src/navigation/Navigation.cpp       Implementasi orkestrasi (identik dengan logika lama)
```

## Yang BELUM di-port: takeoff

`auto_takeoff_mode`/`auto_takeoff_state` (logika takeoff otomatis) **sengaja belum
dimasukkan** ke `MissionState`/`Navigation` — `takeoff.h` terikat erat dengan
`Actuator.h` dan `Mode_setup.h` yang belum di-refactor (Fase 5/6). Akan ditambahkan
bersamaan saat kedua modul itu di-port, bukan diasumsikan sekarang.

## Dependency injection, bukan global

- `ModeId::GUIDED` check lama → parameter `bool is_guided_mode` (`Mode` belum
  di-port; Fase 6 akan meneruskan hasil pengecekan mode aktifnya sebagai boolean ini).
- `notifyWaypointReached(seq)` (fungsi global, dulu diimplementasikan di `Mavlink.h`)
  → `WaypointReachedCallback` yang didaftarkan lewat `setWaypointReachedCallback()`,
  supaya `Navigation` tidak perlu tahu apa pun tentang MAVLink. Fase 7 akan
  mendaftarkan callback yang mengirim `MISSION_ITEM_REACHED`.
- `mode_fbwa = true` (global `Radio.h`, dipanggil saat misi selesai) → **tidak
  ditulis langsung**. Pemanggil (lapisan Mode, Fase 6) bertanggung jawab memeriksa
  `mission.auto_navigation_mode` beralih ke `false` dan memutuskan pindah mode
  sendiri — `Navigation` tidak boleh menulis state RC/mode yang bukan miliknya.
- `arming`/`ch_throttle`/`scaleToPercent()` (Radio.h/Actuator.h, Fase 5) →
  parameter `bool armed`, `int16_t throttle_stick_percent` eksplisit.

## Pemetaan API lama

| Program lama | Program baru |
| --- | --- |
| `nav_gps()` | `navigation.updateHomeAndPosition(ahrs, gnss_data, baro_data, armed, now_ms)` |
| `update_wp_nav()` | `navigation.updateWaypointNav(l1, ahrs_data, imu_data)` (dipanggil internal oleh `navigate()` juga) |
| `update_alt()` | `navigation.updateAltitude(tecs, ahrs_data, imu_data, baro_data, airspeed_data, throttle_stick_percent, enable_nudge)` |
| `update_speed_height()` | `navigation.updateSpeedHeight(tecs, ahrs_data, baro_data, imu_data, airspeed_data)` |
| `get_adaptive_wp_airspeed(...)` | `navigation.getAdaptiveWaypointAirspeed(...)` |
| `updateAuto_FW()` / `calc_nav_roll()` / `calc_nav_pitch()` | `navigation.updateAutoAttitudeTargets(l1, tecs, imu_data)`, hasil di `navigation.state().nav_roll_deg`/`nav_pitch_deg` |

`navRollCd()` dari L1 memakai tanda lateral-acceleration pada navigation
frame, sedangkan attitude controller fixed-wing memakai konvensi bank
sebaliknya. Karena itu `updateAutoAttitudeTargets()` membalik tanda roll tepat
di batas navigasi-ke-attitude. Koreksi ini berlaku konsisten untuk AUTO,
GUIDED, dan LOITER; jangan membaliknya lagi di `L1Controller.cpp` atau
`AttitudeController.cpp`.
| `navigate()` | `navigation.navigate(is_guided_mode, l1, ahrs_data, imu_data, eas2tas)` |
| `set_next_WP(loc)` | `navigation.setNextWaypoint(loc)` |
| `get_next_ground_course(default)` | `navigation.getNextGroundCourse(default)` |
| `waypoint[]` / `wp_sum` / `flag_wp` / `auto_state` / `target_altitude` | `navigation.state().waypoint[]` / `.wp_sum` / `.flag_wp` / `.auto_state` / `.target_altitude` |
| `rpi_external_setpoint_*` | `navigation.state().rpi_external_setpoint_*` (diisi Mavlink.h saat Fase 7) |
| `calc_groundspeed_undershoot()` | `navigation.groundspeedUndershoot()` (dihitung internal tiap `updateHomeAndPosition()`) |

## File lama yang tidak dimigrasikan

```text
include/Navigation.h
include/Auto_setup.h
include/AutoMode.h        (kemungkinan dead/duplikat Auto_setup.h, tidak pernah dipakai jalur aktif)
include/WP_Loiter.h       (dead code — dikonfirmasi audit awal, tidak ada pemakai jalur aktif)
```

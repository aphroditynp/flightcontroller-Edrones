# Altitude Pitch Controller (AUTO/LOITER)

`fc::AltitudePitchController` (`include/control/AltitudePitchController.h`,
`src/control/AltitudePitchController.cpp`) mengatur **pitch** untuk menahan
ketinggian di mode AUTO dan LOITER. Controller ini **terpisah dari TECS**: TECS
tetap ada dan hanya mengurus auto-throttle.

## Kenapa dibutuhkan

Sebelumnya `nav_pitch_deg` di AUTO/LOITER berasal dari
`Tecs::pitchDemandDeg()`. Tetapi `Tecs::update()` hanya dipanggil oleh
`Navigation::updateAutoThrottle()` saat `auto_throttle_mode == true`, dan
AUTO/LOITER sengaja mematikan auto-throttle (throttle dari stik RC). Akibatnya
`pitchDemandDeg()` tidak pernah dihitung (tetap 0), sehingga **tidak ada
altitude hold sama sekali**: pesawat terbang dengan pitch target 0° dan
ketinggiannya bergantung pada throttle pilot.

## Hukum kendali (cascade)

```
climb_dem = clamp(ALT_KP * (h_target - h), -ALT_SINK_MAX, +ALT_CLMB_MAX)   [m/s]
e_rate    = climb_dem - LPF(climb_rate)                                     [m/s]
I        += ALT_RATE_KI * e_rate * dt          (clamp ±5°, anti-windup kondisional)
pitch     = ALT_PTCH_TRIM
          + ALT_PTCH_FF * asin(climb_dem / V)   (feedforward sudut jalur terbang)
          + ALT_RATE_KP * e_rate + I
          + ALT_ROLL_CMP * (1/cos(roll) - 1)    (kompensasi lift saat bank)
pitch     = slew_10deg/s( clamp(pitch, PITCH_MIN, pitch_max_eff) )
```

- `h` = `AhrsData::position.alt` (baro, positif ke atas). `h_target` =
  `MissionState::target_altitude` (waypoint `item.z` di AUTO, ketinggian saat
  masuk di LOITER). Keduanya memakai frame yang sama.
- `climb_rate` = `BarometerData::climb_rate_mps`, ditambah LPF 2 Hz.
- **Proteksi underspeed** (hanya jika sensor airspeed valid): batas pitch atas
  diturunkan linear dari `PITCH_MAX` pada `MIN_AIRSPEED + 2 m/s` sampai 0° pada
  `MIN_AIRSPEED`. Ini penting karena throttle manual, jadi loop ketinggian tidak
  boleh menaikkan hidung sampai stall.
- Transfer bumpless: `reset(-imu.pitch_deg)` dipanggil di `_enter()` AUTO/LOITER.
- dt tidak valid (≤0 atau >0.1 s) atau NaN diabaikan, dan output terakhir dipertahankan.

## Parameter (EEPROM, ditambahkan di akhir tabel)

| Param | Default | Arti |
| --- | --- | --- |
| `ALT_CTRL_EN` | 1 | 1 = controller ini aktif; 0 = perilaku lama (pitch dari TECS) |
| `ALT_KP` | 0.3 | (m/s) per m error ketinggian |
| `ALT_RATE_KP` | 3.0 | derajat per (m/s) error climb rate |
| `ALT_RATE_KI` | 0.5 | integrator (menyerap trim yang belum diketahui) |
| `ALT_CLMB_MAX` / `ALT_SINK_MAX` | 3.0 / 2.5 | batas laju naik/turun (m/s) |
| `ALT_PTCH_TRIM` | 0 | pitch level-flight pada cruise (derajat) |
| `ALT_PTCH_FF` | 1.0 | gain feedforward sudut jalur terbang |
| `ALT_ROLL_CMP` | 10 | kompensasi bank (derajat), penting untuk LOITER 35° |

Batas pitch (`PITCH_MIN/MAX` dari `NavigationConfig`) dan airspeed
(`MIN_AIRSPEED`, `AIRSPEED_CRUISE` dari `TecsConfig`) disalin di `setup()`
`main.cpp`, jadi tidak didaftarkan dua kali. `kMaxParams` dinaikkan 32 → 48
(EEPROM berakhir di 1196, masih jauh di bawah blok IMU-cal di 2000). EEPROM lama
akan ter-load sebagai `SchemaUpgraded`: nilai lama dipertahankan, param ALT_*
diisi default. Seperti param lain, perubahan berlaku setelah **power-cycle**.

## Telemetri

`NAV_CONTROLLER_OUTPUT.alt_error` sekarang berisi error ketinggian (m), dan
`nav_pitch` berisi demand dari controller ini. Keduanya bisa dilihat di Mission
Planner untuk tuning.

## Tuning lapangan (urutan disarankan)

1. Terbang level di AUTO dengan throttle cruise, lalu catat pitch rata-rata dan
   isi ke `ALT_PTCH_TRIM`, supaya integrator tidak menanggung trim.
2. Jika ketinggian berosilasi lambat, turunkan `ALT_KP`. Jika osilasi cepat
   (pitch bergetar), turunkan `ALT_RATE_KP`.
3. Jika LOITER kehilangan ketinggian, naikkan `ALT_ROLL_CMP`.
4. Pilot tetap memegang throttle: controller ini **tidak bisa** menambah
   energi. Naik ketinggian besar tetap butuh throttle yang cukup.

Hasil simulasi host (model point-mass, trim tak diketahui +2°): hold, naik 30 m,
turun 30 m, loiter bank 35°, dan tanpa sensor airspeed semuanya konvergen ke
error < 3 cm dengan overshoot < 0.7 m.

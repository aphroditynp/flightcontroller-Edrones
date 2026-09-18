> **HISTORICAL / INACTIVE:** The active fixed-wing inner loop was replaced by
> the TD/V10-trainer2 P+D controller for the TD+LOITER+fuzzy thesis branch.
> See `docs/td-loiter-fuzzy-port.md`. The LQR source/tooling is retained only
> for traceability and comparison; `AttitudeController` no longer calls it.

# LQR Attitude Controller (roll + pitch + yaw)

**Modul baru sepenuhnya** — tidak ada LQR di program KHAGESWARA lama.
`include/LQR_PF.h`, meski namanya menjanjikan LQR, isinya salinan lama algoritma
L1 guidance yang di-comment total (bukan LQR sama sekali) — dikonfirmasi lewat
audit kode di awal refactor ini. Attitude controller ini menggantikan PID
per-axis lama (`FW_roll/pitch/yaw_controller.h`, `FW_controller.h`,
`Control_data.h`) sepenuhnya, sesuai keputusan Anda: **roll + pitch + yaw**,
gain `K` **tetap, dihitung offline**.

## Struktur modul

```text
include/control/LqrAxisController.h   Regulator satu-axis, dipakai ulang 3x
src/control/LqrAxisController.cpp
include/control/AttitudeController.h  Orkestrasi 3-axis + speed scaler + yaw feedforward
src/control/AttitudeController.cpp
tools/lqr_gain_design/lqr_gain_design.py   Script Python offline penghitung K
tools/lqr_gain_design/.venv/               Virtual environment lokal (numpy, scipy, control)
```

## Model state-space per axis

**Roll** (state `x = [φ, p, ∫(φ_cmd−φ)dt]`, input `u = δa`):
```
φ̇ = p
ṗ = L_p·p + L_δa·δa
```
**Pitch** (state `x = [θ, q, ∫(θ_cmd−θ)dt]`, input `u = δe`):
```
θ̇ = q
q̇ = M_q·q + M_δe·δe
```
**Yaw** (state `x = [r]`, input `u = δr`, setpoint = *kecepatan-belok
terkoordinasi* `r_cmd`, bukan sudut yaw):
```
ṙ = N_r·r + N_δr·δr
```

Roll dan pitch memakai **integral augmentation** (servomechanism/type-1
tracking) — ini **mekanisme utama penolakan gangguan** untuk skenario "gangguan
roll eksternal" di judul skripsi: gangguan momen roll konstan/lambat (hembusan
angin, trim asimetris) menghasilkan error tunak di bawah state feedback murni;
hanya aksi integral yang membuatnya konvergen ke nol. Yaw **tidak** memakai
integral — settingnya adalah *rate*, dikomando ke `r_cmd` yang secara sah
nonzero berkepanjangan saat menikung; mengintegralkan error rate saat command
nonzero berkepanjangan berisiko windup/melawan hukum koordinasi-belok
(alasan yang sama kenapa loop rudder ArduPilot juga tidak mengintegralkan).

**Turunan stabilitas/kendali** (`L_p`, `M_q`, `N_r`, `L_δa`, `M_δe`, `N_δr` di
`lqr_gain_design.py`) adalah **estimasi generik** untuk rangka 2.8 kg/1.8 m/
0.45 m² pada 18 m/s — **bukan hasil pengukuran wahana ini**. Ini adalah
kelemahan asumsi terlemah dalam desain ini; **perlu disempurnakan lewat system
identification uji terbang** (doublet aileron/elevator/rudder) sebelum
dipercaya sepenuhnya. Inersia (Ixx/Iyy/Izz, tersirat dalam nilai damping) juga
sebaiknya diverifikasi lewat pengukuran bifilar-pendulum.

## Umpan balik keadaan: langsung dari IMU, tanpa estimator baru

`φ, θ` dan `p, q, r` diambil langsung dari `fc::ImuData` (fusi NDOF BNO055
sudah menghasilkan sudut + rate yang bersih) — **tidak ada EKF/filter
tambahan**. Ini cukup sebagai full-state feedback karena BNO055 sudah
melakukan fusi sensor onboard.

## Alur perhitungan gain `K` (offline, `lqr_gain_design.py`)

1. Bangun `(A, B)` per axis pada trim jelajah (18 m/s), basis **radian**
   (konvensi teori kontrol).
2. Augmentasi integral (roll/pitch): `A_aug = [[A,0],[-C,0]]`,
   `B_aug = [[B],[0]]` — bentuk servomechanism standar untuk melacak referensi
   konstan sekaligus menolak gangguan yang masuk di titik yang sama dengan `B`
   (gangguan momen roll eksternal cocok dengan asumsi ini — "matched
   disturbance").
3. Diskritisasi zero-order-hold pada 200 Hz (`control.c2d`) — sesuai laju
   loop kontrol sebenarnya.
4. `Q`/`R` lewat **kaidah Bryson**: `Q_ii = 1/(deviasi maksimum x_i)²`,
   `R_jj = 1/(deviasi maksimum u_j)²`, dari target: error roll maks 7.5°, error
   pitch maks 5°, defleksi permukaan maks 25°.
5. Selesaikan discrete Riccati (`control.dlqr`) → `K`.
6. **Validasi di dua ujung amplop kecepatan (14 & 22 m/s), bukan cuma
   jelajah**: `B` dihitung ulang pada kedua kecepatan (efektivitas kendali
   berskala kuadrat terhadap tekanan dinamis), `K` yang SAMA (dari trim
   jelajah) diuji — semua pole loop-tertutup harus tetap stabil (`|eig|<1` di
   domain diskrit). Hasil terkini (lihat output skrip): **stabil di seluruh
   amplop 14-22 m/s** untuk ketiga axis, tanpa perlu gain-scheduling `K`.

Jalankan:
```bash
cd tools/lqr_gain_design
./.venv/Scripts/python.exe lqr_gain_design.py
```

## Catatan tanda `k_integral` — PENTING, sumber kesalahan yang mudah terjadi

State integral di desain Python didefinisikan `ξ̇ = -x₁` (x₁ = state sudut),
yang secara aljabar **sama** dengan `integral_state_` yang diakumulasi firmware
(`∫(setpoint − measured)dt`, dengan asumsi setpoint quasi-statis). Hukum
kontrol LQR murni adalah `u = -K·x_aug` untuk **ketiga** komponen. Mensubstitusi
`x₁ = measured - setpoint` menunjukkan `k_primary = K₁` dan `k_rate = K₂`
langsung terpakai di bentuk firmware `+k_primary·error − k_rate·rate`, **tapi
suku integral perlu dibalik tandanya**: firmware menghitung
`+k_integral·integral_state`, sedangkan hukum kontrolnya adalah `-K₃·ξ`, jadi
`k_integral = -K₃`. Script `lqr_gain_design.py` **sudah membalik tanda ini**
pada bagian cetak "Firmware LqrAxisConfig values" — nilai yang dicetak di
sana siap tempel langsung ke `AttitudeControllerConfig`, jangan ambil nilai
`K` mentah dari baris "K = [...]" di atasnya untuk `k_integral`.

**Unit `K` tidak perlu dikonversi** antara basis radian (desain) dan basis
derajat (firmware) — karena `u=-Kx` linear dan kedua sisi (state *dan* output)
diskalakan oleh faktor derajat↔radian yang sama, faktor itu saling
menghilangkan. Nilai numerik `K` identik di kedua basis (lihat komentar di
`lqr_gain_design.py`).

## Speed scaler — bukan gain-scheduling

Efektivitas kendali (`L_δa`, `M_δe`, `N_δr`) berubah ~2.5× sepanjang 14-22 m/s
(berskala tekanan dinamis, ~V²), sementara redaman (`L_p`, `M_q`, `N_r`) hanya
berubah ~1.6× — margin redaman dari kaidah Bryson sudah menoleransi ini
(dikonfirmasi lewat pengecekan pole dual-kecepatan di atas). Jadi:
**`K` tunggal pada trim jelajah, dengan speed scaler sebagai pengali PASCA
output** — bukan menjadwalkan beberapa matriks `K`:

```
scaler = clamp((V_trim / V_measured)², scaler_min, scaler_max)
output_final = clamp(K_output_raw * scaler, ±output_limit_deg)
```

Ini mereplikasi praktik `speed_scaler` PID lama, hanya diterapkan ke output
LQR alih-alih ke gain PID.

## Yaw: coordinated-turn feedforward, bukan rudder-mixing lama

Setpoint axis yaw bukan sudut yaw, tapi **kecepatan belok terkoordinasi**:
```
r_cmd (rad/s) = g·tan(φ_cmd) / V,   lalu dikonversi ke deg/s
```
dihitung dari `nav_roll_deg` (setpoint L1, bukan roll terukur) dan airspeed
saat ini. Ini menggantikan peran *rudder-mixing* PID lama
(`Attitude.h::calc_nav_yaw_coordinated()`), **bukan** logika *load-factor*
L1/TECS (`1/cos(pitch)` pada `nav_roll_cd()`) — itu tetap di `L1Controller`,
tidak disentuh.

## Antarmuka firmware

```cpp
#include "control/AttitudeController.h"

fc::AttitudeController attitude;

fc::AttitudeController::Output cmd =
    attitude.update(nav_roll_deg, nav_pitch_deg, imu_data, airspeed_mps, dt);
// cmd.aileron_deg, cmd.elevator_deg, cmd.rudder_deg -> mixing servo (Fase 5, Actuator)
```

`AttitudeController` memegang 3 instance `LqrAxisController` (bukan 3 kelas
terpisah) — hanya konfigurasinya yang berbeda per axis (roll/pitch:
`AngleAndRate` + integral; yaw: `RateOnly`, tanpa integral).

## Yang perlu diverifikasi sebelum terbang (jangan lewati)

- Seluruh turunan stabilitas/kendali dan inersia — estimasi generik, sempurnakan
  dari system-ID uji terbang.
- Pemetaan sumbu/tanda gyro BNO055 (`invert_gyro_y` di `Imu.h`) — verifikasi
  bench, jangan asumsikan dari komentar kode lama.
- Bandwidth/rate limit servo sesungguhnya — validasi bandwidth loop-tertutup
  yang dipilih terhadapnya.
- Validasi `Q`/`R` dan `K` yang dihasilkan lewat simulasi (step response,
  respons penolakan gangguan) di seluruh amplop 14-22 m/s — sudah dicek
  pole/redaman lewat skrip, **belum** disimulasikan closed-loop penuh.
- Validasi tanda/skala formula `r_cmd` coordinated-turn terhadap arah defleksi
  rudder sesungguhnya di pesawat.
- Batas anti-windup integrator — uji skenario saturasi realistis (mis. recovery
  pasca-stall) di SITL sebelum terbang.
- Konsistensi konversi derajat↔radian dan konvensi tanda output dengan kode
  mixing servo (Fase 5) — pastikan end-to-end sama dengan konvensi ±25° lama.

## File lama yang tidak dimigrasikan

```text
include/FW_controller.h
include/FW_roll_controller.h
include/FW_pitch_controller.h
include/FW_yaw_controller.h
include/Control_data.h
include/FW_control.h        (FW_CONTROL::updateAUTO_FW — sudah dead-commented di Mode_Fixedwing.h lama)
include/LQR_PF.h            (nama menyesatkan, isinya L1 lama yang di-comment total)
```

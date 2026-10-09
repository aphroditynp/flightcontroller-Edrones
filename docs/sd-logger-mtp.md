# SD card logging + akses MTP (fc::SdLogger)

Ditambahkan 2026-08-19: log CSV flight-data (roll/pitch/yaw/altitude/mode/
armed/channel 1-5) ke microSD bawaan Teensy 4.1 (`BUILTIN_SDCARD`), dan SD
card itu bisa langsung dibuka di File Explorer/file manager PC begitu kabel
USB Teensy dicolok -- tanpa cabut kartu -- lewat protokol MTP.

## Kenapa menggantikan `fc::DataLogger`/`SerialUSB1`, bukan tambahan

Teensy 4.1 tidak punya kombinasi USB "Dual Serial + MTP" (dicek langsung ke
`boards.txt` core-nya, menu `teensy41.menu.usb.*`) -- pilihannya cuma salah
satu:

- `USB_DUAL_SERIAL` (kondisi sebelumnya): `Serial` (MAVLink) + `SerialUSB1`
  (live CSV barometer, `fc::DataLogger`), TANPA MTP.
- `USB_MTPDISK_SERIAL` (kondisi sekarang): `Serial` (MAVLink) + MTP disk,
  TANPA `SerialUSB1`.

Karena permintaannya eksplisit "colok kabel, SD kebaca di file manager", yang
dipilih adalah `USB_MTPDISK_SERIAL`. Data yang dulu di-live-stream lewat
`SerialUSB1` (perbandingan altitude Kalman vs complementary filter) sekarang
sudah tidak di-log sama sekali oleh build aktif -- kalau butuh lagi, itu bisa
ditambahkan sebagai kolom baru di `SdLogger`, bukan mengaktifkan
`SerialUSB1` lagi (karena tidak mungkin coexist dengan MTP di board ini).
`fc::DataLogger` sendiri TIDAK dihapus dari source tree, lihat
`docs/data-logger-usb.md`'s status note.

## Struktur modul

```text
include/communication/SdLogger.h   Kontrak class SdLogger
src/communication/SdLogger.cpp     Implementasi (SD.begin, penamaan file, CSV)
```

`main.cpp` memanggilnya lewat task `taskSdLog` (prioritas 2, 20 Hz,
`kSdLogPeriodMs = 50`) -- terpisah dari task MTP (`taskMtp`, juga prioritas
2, 100 Hz, `kMtpPeriodMs = 10`, isinya cuma `MTP.loop()` untuk melayani
transaksi file dari host).

## Format CSV

```text
timestamp_ms,roll_deg,pitch_deg,yaw_deg,altitude_m,mode,armed,ch1_roll,ch2_pitch,ch3_throttle,ch4_yaw,ch5_arm_raw
1234,2.10,-0.35,178.40,12.30,FBWA,1,1502,1498,1650,1500,1780
```

Firmware terbaru menambahkan tiga kolom yang dipakai untuk analisis LOITER:

```text
...,mahony_roll_deg,mahony_pitch_deg,mahony_yaw_deg,radial_error_m,target_radius_m,l1_period_s
```

- `radial_error_m`: error radius LOITER dari `L1Controller` (radius aktual
  dikurangi radius target); hanya terisi pada mode `LOIT`, selain itu `nan`.
- `target_radius_m`: radius efektif yang dipakai L1 setelah kompensasi
  `eas2tas`; hanya terisi pada mode `LOIT`.
- `l1_period_s`: period L1 aktual, termasuk perubahan dari fuzzy tuner.

Setelah menyalin `LOGnnn.CSV` dari SD, hitung metrik pasca-terbang dengan:

```bash
python3 tools/flight_analysis/analyze_loiter_log.py LOGnnn.CSV \
  --json loiter_metrics.json
```

Analyzer mengambil baris `LOIT`, membuang 25% awal sebagai fase capture, lalu
menghasilkan `radial_error_rms_m`, `radial_error_mean_abs_m`,
`radial_error_max_abs_m`, `final_radius_m`, `target_radius_m`, dan
`period_mean_s`. Angka tersebut berasal dari sampel penerbangan, bukan dari
model simulasi.

- `roll_deg`/`pitch_deg`/`yaw_deg`: `fc::ImuData` (BNO055), sama seperti yang
  dipakai `AttitudeController` -- lihat `docs/attitude-lqr.md`.
- `altitude_m`: `fc::BarometerData::altitude_m` (Opsi 1, Kalman onboard, yang
  dipakai TECS/Navigation).
- `mode`: `ModeManager::code4()` (`"MANU"/"FBWA"/"AUTO"/"GUID"`).
- `armed`: `Radio::armed()`, 0/1.
- `ch1_roll`..`ch4_yaw`: nilai PWM-equivalent yang sudah dikalibrasi/dibatasi
  dead-band (`Radio::channelRoll()` dst, domain 988-2012), BUKAN raw SBUS
  count -- ini nilai yang sama yang benar-benar dipakai Manual/FBWA.
- `ch5_arm_raw`: `Radio::channelArmRaw()`, raw SBUS count `ch[4]` TANPA
  kalibrasi (tidak ada scale/offset terdaftar untuk channel ini) -- untuk
  diagnostik ambang arming, bukan buat dipakai ulang sebagai kontrol.

## Penamaan file: satu file baru per boot

`SdLogger::begin()` mencari `LOG001.CSV`, `LOG002.CSV`, ... sampai ketemu
nama yang belum ada di kartu, supaya data penerbangan sebelumnya tidak
tertimpa. Setiap `logRow()` langsung `flush()` ke kartu (bukan buffer di
RAM) -- pada rate 20 Hz ini overhead-nya diabaikan, dan menjamin data selamat
sampai baris terakhir kalau baterai putus mendadak.

## Status output: selector Serial (PuTTY) / MAVLink, bukan buzzer (fix 2026-08-20)

`Serial` cuma satu port, dipakai bareng MAVLink (lihat bagian "Kenapa
menggantikan..." di atas) -- kalau `taskMavlink` jalan bareng, pesan teks
biasa (`Serial.println(...)`) tenggelam/rusak diselang-seling stream biner
MAVLink. Percobaan sebelumnya (2026-08-19) mengonfirmasi status SD lewat
`fc::Buzzer::playPattern()` (1 beep panjang = OK, 6 beep cepat = gagal),
tapi itu cuma bisa membawa 1 bit info (OK/gagal) untuk SATU hal (SD card) --
tidak untuk semua status boot lainnya (IMU/Baro/Airspeed/IMU-calibration),
dan tidak pernah terkonfirmasi bunyinya kedengaran di hardware ini.

Diganti dengan selector `kStatusOutput` di `main.cpp` (konstanta compile-time
-- ubah nilainya lalu re-flash untuk pindah mode, BUKAN prompt runtime):

```cpp
enum class StatusOutput : uint8_t { SerialText = 0, Mavlink = 1 };
constexpr StatusOutput kStatusOutput = StatusOutput::SerialText;
```

- **`SerialText` (0)** -- semua diagnostik boot (SD card, IMU/Baro/Airspeed
  init, IMU calibration restore/save) dicetak sebagai teks biasa ke
  `Serial`. Buka terminal serial (PuTTY, dll) di `kUsbBaud` (115200) dan
  baca langsung. `fc::Mavlink::setUsbEnabled(false)` dipanggil di awal
  `setup()` untuk mode ini, supaya MAVLink TIDAK menulis/membaca apa pun di
  port USB (Telemetry/`Serial2` dan Companion/`Serial7` tetap jalan seperti
  biasa) -- jadi teksnya tidak ketiban traffic biner.
- **`Mavlink` (1)** -- pesan yang sama dikirim sebagai MAVLink `STATUSTEXT`
  (`fc::Mavlink::sendStatusText()`, sekarang public), muncul di tab
  Messages GCS (Mission Planner/QGC). Konfigurasi normal untuk terbang.
  `setUsbEnabled(true)` (default), MAVLink jalan penuh di USB seperti
  sebelumnya.

Semua titik yang dulu manggil `Serial.println`/`Serial.printf` langsung
untuk status boot sekarang lewat satu fungsi `reportStatus(severity, text)`
di `main.cpp` -- satu jalur, bukan tersebar, dan otomatis ikut selector di
atas tanpa perlu diubah satu-satu lagi kalau mode-nya diganti nanti.

## Membaca log di PC

Colok kabel USB Teensy ke laptop seperti biasa. Selain COM port MAVLink
(`Serial`), akan muncul satu drive baru ("SD Card") di File Explorer/file
manager -- buka, cari `LOGnnn.CSV` terbaru, salin/buka langsung seperti
flashdisk biasa. Tidak perlu cabut microSD dari slot Teensy 4.1.

## Library eksternal

`MTP_Teensy` (KurtE/MTP_Teensy, MIT), bukan bagian Teensyduino core --
di-pin ke satu commit di `platformio.ini` `lib_deps`, sama seperti pola
`FlexCAN_T4` di file yang sama. `SD`/`SdFat` sudah bagian dari
`framework-arduinoteensy`, tidak perlu `lib_deps` tambahan.

## Urutan init: SD/MTP sebelum interlock radio (fix 2026-08-20)

`Radio::begin()` (dipanggil di `setup()`) punya `while(armed_)`/
`while(signal_lost_)` TANPA timeout -- sengaja blocking sebagai safety
interlock pre-flight (lihat `docs/radio.md`), tapi ini berarti kalau belum
ada transmitter yang di-bind/sinyalnya belum bagus (mis. saat bench-test
cuma nyolok USB ke laptop buat cek SD card), `setup()` nyangkut selamanya
di situ SEBELUM sempat sampai ke `MTP.begin()`/`g_sdLogger.begin()`.

Akibatnya: Teensy tetap muncul sebagai device "Teensy" di File
Explorer/file manager (itu terjadi di level deskriptor USB, dari
`USB_MTPDISK_SERIAL` di `platformio.ini`, sebelum `setup()` jalan sama
sekali) TAPI tidak pernah benar-benar menjawab request MTP (list storage,
kapasitas, file) -- makanya kelihatan seperti drive kosong tanpa info
GB kosong, dan `SdLogger` tidak pernah sempat `begin()` jadi tidak ada yang
ke-log ke SD card sama sekali.

Fix: blok SD card + MTP (`MTP.begin()`, `g_sdLogger.begin()`,
`MTP.addFilesystem()`) dipindah ke SEBELUM `g_radio.begin()` di
`main.cpp`. Sekarang SD card mount dan MTP jadi responsif duluan,
independen dari lama/tidaknya interlock radio menunggu transmitter.

## Yang belum divalidasi

Fitur ini baru ditambahkan lewat code review, belum diuji di hardware
sesungguhnya (belum ada microSD terpasang saat penambahan ini). Sebelum
terbang:

- Pastikan microSD terpasang di slot Teensy 4.1 sebelum power-on --
  `SdLogger::begin()` gagal diam-diam (cuma print sekali ke `Serial`, yang
  praktisnya TIDAK kebaca -- lihat bagian "Status via buzzer" di bawah)
  kalau kartu tidak terbaca, task `taskSdLog` lalu jadi no-op (`logRow()`
  cek `open_` dan langsung return).
- Konfirmasi drive MTP benar-benar muncul di Windows setelah upload firmware
  baru -- MTP di Teensyduino core masih berlabel "(Experimental)".
- Kalau drive tidak muncul/hang, coba cabut-colok ulang kabel USB dulu
  sebelum curiga ke firmware -- device MTP butuh proses enumerasi ulang oleh
  host yang kadang tidak otomatis re-trigger.

# Simulasi loiter L1 + fuzzy

Simulasi Python ini membandingkan kendali loiter **LQR navigasi radial**, **L1 dengan
period tetap**, dan **L1 dengan fuzzy tuner** pada kondisi awal dan gangguan
roll yang sama. Persamaan
L1 circular guidance, nilai awal fuzzy, dan rule base mengikuti
`src/navigation/L1Controller.cpp`, `src/navigation/FuzzyL1Tuner.cpp`, serta
`include/navigation/FuzzyL1Tuner.h`.

## Model dan batasan

- Gerak pesawat adalah model titik 2D dengan airspeed konstan dan koordinat North/East.
- Perintah percepatan lateral controller diubah ke sudut bank, dibatasi maksimum 35°,
  lalu melewati respons roll orde satu. Ini **bukan** model aerodinamika atau
  replika keseluruhan firmware/TECS/inner-loop.
- Gangguan roll dimodelkan sebagai tambahan roll-rate selama interval tertentu.
- Input fuzzy memakai besar `abs(error radius)` dan perubahan error bertanda
  `Δe` seperti proposal. Nilai negatif berarti error mengecil, nol berarti
  relatif tetap, dan positif berarti error membesar. Defuzzifikasi Mamdani
  centroid didiskretisasi; hasil numeriknya bisa sedikit berbeda dari
  implementasi eFLL di Teensy.
- Membership function dan rule base masih nilai awal proyek. Hasil simulasi
  bukan klaim performa terbang; kalibrasikan model dan validasi dengan log/data.

Tersedia tiga controller simulasi:

- `lqr`: LQR navigasi radial menghasilkan target bank. Dalam firmware, target
  ini diteruskan ke `AttitudeController`; LQR attitude tetap menjadi inner loop.
  Dalam simulasi, pelacakan target bank direpresentasikan oleh model respons
  roll orde satu, bukan oleh LQR attitude.
- `l1`: L1 dengan period tetap.
- `fuzzy`: L1 dengan period yang diubah fuzzy.

Tersedia dua profil output fuzzy:

- `firmware`: nilai awal persis dari `FuzzyL1TunerConfig` proyek. Pada simulasi
  awal, profil ini menghasilkan period rata-rata sekitar 26 s dan error lebih
  besar karena terlalu sering memilih output `gentle`.
- `balanced`: output `Naik` dan `Tetap` dipersempit agar period tetap dekat
  baseline 20 s. Ini titik awal tuning simulasi, bukan perubahan otomatis pada
  firmware.

## Struktur fuzzy yang dipakai

Fuzzy tuner menggunakan metode Mamdani dengan:

### Input 1 — besar error radius `|e|`

| Himpunan | Breakpoint segitiga (m) |
| --- | --- |
| Kecil | `(0, 5, 5, 15)` |
| Sedang | `(5, 15, 15, 30)` |
| Besar | `(15, 30, 30, 60)` |

### Input 2 — perubahan error bertanda `Δe`

| Himpunan | Breakpoint segitiga (m/s) |
| --- | --- |
| Negatif | `(-12, -6, -6, 0)` |
| Nol | `(-6, 0, 0, 6)` |
| Positif | `(0, 6, 6, 12)` |

### Output — skala L1 period

Output fuzzy dikalikan dengan `base_period_s`, kemudian dibatasi antara
`min_period_s` dan `max_period_s` (default 10–30 s).

Profil `firmware` menggunakan:

| Output | Breakpoint skala |
| --- | --- |
| Turun | `(0.60, 0.70, 0.70, 0.85)` |
| Tetap | `(0.75, 1.00, 1.00, 1.25)` |
| Naik | `(1.15, 1.30, 1.30, 1.50)` |

Profil `balanced` yang dipakai default simulasi menggunakan:

| Output | Breakpoint skala |
| --- | --- |
| Turun | `(0.50, 0.60, 0.60, 0.75)` |
| Tetap | `(0.65, 0.80, 0.80, 1.00)` |
| Naik | `(0.90, 1.05, 1.05, 1.20)` |

Rule base yang sama dengan firmware:

| `|e| \ Δe` | Negatif | Nol | Positif |
| --- | --- | --- | --- |
| Kecil | Naik | Tetap | Turun |
| Sedang | Tetap | Tetap | Turun |
| Besar | Tetap | Turun | Turun |

Grafik `loiter_comparison.png` sekarang terdiri dari 8 panel: lintasan,
radial error, L1 period, bank aktual, tiga grafik membership function dengan
breakpoint numerik di legend, dan tabel rule base. Dengan begitu perubahan
membership function dapat dilacak langsung dari gambar setiap run.

## Instalasi dan menjalankan

Dari root repository:

```bash
python3 -m pip install -r tools/flight_analysis/requirements.txt
python3 tools/loiter_simulation/loiter_sim.py
```

Output default dibuat di subfolder timestamp baru di
`tools/loiter_simulation/results/`. Run lama tidak ditimpa:

- `loiter_lqr.csv` — data run LQR radial.
- `l1_fixed.csv` — data run L1 period tetap.
- `l1_fuzzy.csv` — data run L1 + fuzzy.
- `l1_fixed.csv` — data run L1 period tetap.
- `l1_fuzzy.csv` — data run L1 + fuzzy.
- `run_config.json` — profil dan parameter run untuk tracking.

Nama folder run menggunakan timestamp sampai mikrodetik, sehingga grafik dari
run baru tidak menggantikan `loiter_comparison.png` dari run sebelumnya.

Contoh mengubah kondisi simulasi:

```bash
python3 tools/loiter_simulation/loiter_sim.py \
  --duration 300 --dt 0.05 --radius 50 --airspeed 18 \
  --disturbance-start 100 --disturbance-duration 15 --disturbance-rate 8
```

Jalankan hanya satu controller bila diperlukan:

```bash
python3 tools/loiter_simulation/loiter_sim.py --controller lqr
```

Untuk mereproduksi nilai fuzzy awal firmware:

```bash
python3 tools/loiter_simulation/loiter_sim.py --tuning-profile firmware
```

Jalankan tes unit:

```bash
python3 -m unittest discover -s tools/loiter_simulation -p 'test_*.py' -v
```

Metric RMS error yang dicetak mengabaikan 25% awal run untuk mengurangi pengaruh
fase capture awal. Gangguan menggunakan roll-rate bertanda positif selama interval
yang ditentukan; coba juga `--disturbance-rate -7` untuk menguji arah sebaliknya.
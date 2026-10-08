#!/usr/bin/env python3
"""Simplified fixed-wing L1 loiter simulation and fixed-period/fuzzy comparison.

The circular L1 guidance equations and fuzzy rule table follow this project's
src/navigation/L1Controller.cpp and src/navigation/FuzzyL1Tuner.cpp. The airframe
is intentionally a transparent 2-D point-mass model, not a flight-dynamics model.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


GRAVITY_MPS2 = 9.80665


@dataclass(frozen=True)
class SimulationConfig:
    duration_s: float = 240.0
    dt_s: float = 0.05
    airspeed_mps: float = 18.0
    loiter_radius_m: float = 50.0
    base_period_s: float = 20.0
    damping: float = 0.73
    max_bank_deg: float = 35.0
    roll_time_constant_s: float = 0.7
    initial_radius_error_m: float = 35.0
    disturbance_start_s: float = 90.0
    disturbance_duration_s: float = 12.0
    disturbance_roll_rate_deg_s: float = 7.0
    min_period_s: float = 10.0
    max_period_s: float = 30.0


@dataclass
class SimulationResult:
    name: str
    time_s: np.ndarray
    north_m: np.ndarray
    east_m: np.ndarray
    heading_rad: np.ndarray
    bank_rad: np.ndarray
    bank_demand_rad: np.ndarray
    radial_error_m: np.ndarray
    period_s: np.ndarray
    error_rate_mps: np.ndarray

    def metrics(self, radius_m: float) -> dict[str, float]:
        # Exclude initial capture transient from the steady-loiter RMS metric.
        steady_start = int(0.25 * len(self.time_s))
        error = self.radial_error_m[steady_start:]
        return {
            "radial_error_rms_m": float(np.sqrt(np.mean(error**2))),
            "radial_error_mean_abs_m": float(np.mean(np.abs(error))),
            "radial_error_max_abs_m": float(np.max(np.abs(error))),
            "final_radius_m": float(math.hypot(self.north_m[-1], self.east_m[-1])),
            "target_radius_m": radius_m,
            "period_mean_s": float(np.mean(self.period_s[steady_start:])),
        }


def trapezoid_membership(value: float, points: tuple[float, float, float, float]) -> float:
    """Membership in a trapezoid, including triangular/shoulder edge cases."""
    a, b, c, d = points
    if not a <= b <= c <= d:
        raise ValueError(f"Invalid trapezoid breakpoints: {points}")
    if value < a or value > d:
        return 0.0
    if b <= value <= c:
        return 1.0
    if value < b:
        return 1.0 if a == b else (value - a) / (b - a)
    return 1.0 if c == d else (d - value) / (d - c)


class FuzzyL1Tuner:
    """Mamdani 2-input/3-set tuner using the firmware's starting rule base."""

    ERROR_SETS = (
        (0.0, 5.0, 5.0, 15.0),
        (5.0, 15.0, 15.0, 30.0),
        (15.0, 30.0, 30.0, 60.0),
    )
    RATE_SETS = (
        (0.0, 1.0, 1.0, 3.0),
        (1.0, 3.0, 3.0, 6.0),
        (3.0, 6.0, 6.0, 12.0),
    )
    OUTPUT_SETS = (
        (0.6, 0.7, 0.7, 0.85),  # aggressive: shorter L1 period
        (0.75, 1.0, 1.0, 1.25),  # normal
        (1.15, 1.3, 1.3, 1.5),  # gentle: longer L1 period
    )
    # Rows: error small/medium/large; columns: rate small/medium/large.
    RULES = (
        (2, 1, 1),
        (1, 1, 0),
        (0, 0, 0),
    )

    def __init__(self, base_period_s: float, min_period_s: float, max_period_s: float,
                 output_sets: tuple[tuple[float, float, float, float], ...] | None = None):
        self.base_period_s = base_period_s
        self.min_period_s = min_period_s
        self.max_period_s = max_period_s
        self.output_sets = output_sets or self.OUTPUT_SETS
        self.previous_abs_error_m: float | None = None

    def update(self, signed_error_m: float, dt_s: float) -> tuple[float, float]:
        abs_error = abs(signed_error_m)
        if self.previous_abs_error_m is None or dt_s <= 1.0e-3:
            error_rate = 0.0
        else:
            error_rate = (abs_error - self.previous_abs_error_m) / dt_s
        self.previous_abs_error_m = abs_error

        # Firmware fuzzifies absolute e and absolute d|e|/dt.
        e_value = min(abs_error, self.ERROR_SETS[-1][-1])
        de_value = min(abs(error_rate), self.RATE_SETS[-1][-1])
        e_memberships = [trapezoid_membership(e_value, s) for s in self.ERROR_SETS]
        de_memberships = [trapezoid_membership(de_value, s) for s in self.RATE_SETS]

        # Mamdani min-AND / max aggregation and centroid defuzzification.
        universe = np.linspace(0.6, 1.5, 901)
        aggregated = np.zeros_like(universe)
        for e_index in range(3):
            for de_index in range(3):
                strength = min(e_memberships[e_index], de_memberships[de_index])
                if strength <= 0.0:
                    continue
                output_index = self.RULES[e_index][de_index]
                output_membership = np.array(
                    [trapezoid_membership(float(x), self.output_sets[output_index]) for x in universe]
                )
                aggregated = np.maximum(aggregated, np.minimum(strength, output_membership))

        area = float(np.trapezoid(aggregated, universe))
        scale = (
            float(np.trapezoid(universe * aggregated, universe) / area)
            if area > 1.0e-12
            else 1.0
        )
        period = float(np.clip(self.base_period_s * scale, self.min_period_s, self.max_period_s))
        return period, error_rate


def _wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _loiter_lateral_acceleration(
    north_m: float,
    east_m: float,
    velocity_north_mps: float,
    velocity_east_mps: float,
    radius_m: float,
    period_s: float,
    damping: float,
) -> tuple[float, float]:
    """Circular L1 guidance core ported from L1Controller::updateLoiter()."""
    distance = max(math.hypot(north_m, east_m), 1.0e-6)
    ground_speed = max(math.hypot(velocity_north_mps, velocity_east_mps), 1.0)
    outward_north, outward_east = north_m / distance, east_m / distance

    # Vector2f cross and dot products, with N/E axes as in the firmware.
    xtrack_velocity_cap = outward_north * velocity_east_mps - outward_east * velocity_north_mps
    ltrack_velocity_cap = -(velocity_north_mps * outward_north + velocity_east_mps * outward_east)
    nu = float(np.clip(math.atan2(xtrack_velocity_cap, ltrack_velocity_cap), -math.pi / 2, math.pi / 2))

    l1_distance = 0.3183099 * damping * period_s * ground_speed
    k_l1 = 4.0 * damping * damping
    accel_capture = k_l1 * ground_speed**2 / max(l1_distance, 1.0e-6) * math.sin(nu)

    omega = 2.0 * math.pi / period_s
    radial_error = distance - radius_m
    radial_velocity = velocity_north_mps * outward_north + velocity_east_mps * outward_east
    accel_pd = radial_error * omega**2 + radial_velocity * (2.0 * damping * omega)
    tangent_velocity = xtrack_velocity_cap
    accel_centripetal = tangent_velocity**2 / max(0.5 * radius_m, radius_m + radial_error)
    accel_circle = accel_pd + accel_centripetal

    if radial_error > 0.0 and accel_capture < accel_circle:
        lateral_acceleration = accel_capture
    else:
        lateral_acceleration = accel_circle
    return lateral_acceleration, radial_error


TUNING_PROFILES = {
    # Exact starting values from include/navigation/FuzzyL1Tuner.h.
    "firmware": FuzzyL1Tuner.OUTPUT_SETS,
    # Keeps fuzzy adaptation active, but prevents small errors from immediately
    # increasing T far above the 20 s conventional baseline.
    "balanced": (
        (0.50, 0.60, 0.60, 0.75),
        (0.65, 0.80, 0.80, 1.00),
        (0.90, 1.05, 1.05, 1.20),
    ),
}


def simulate(config: SimulationConfig, fuzzy_enabled: bool,
             tuning_profile: str = "balanced") -> SimulationResult:
    if config.dt_s <= 0 or config.duration_s <= 0:
        raise ValueError("duration_s and dt_s must be positive")
    if config.airspeed_mps <= 0 or config.loiter_radius_m <= 0:
        raise ValueError("airspeed and loiter radius must be positive")

    count = int(math.floor(config.duration_s / config.dt_s)) + 1
    time_s = np.arange(count, dtype=float) * config.dt_s
    north = np.empty(count)
    east = np.empty(count)
    heading = np.empty(count)
    bank = np.empty(count)
    bank_demand = np.empty(count)
    radial_error = np.empty(count)
    period = np.empty(count)
    error_rate = np.zeros(count)

    north[0] = config.loiter_radius_m + config.initial_radius_error_m
    east[0] = 0.0
    heading[0] = math.pi / 2  # eastbound tangent at the north side of the circle
    bank[0] = 0.0
    if tuning_profile not in TUNING_PROFILES:
        raise ValueError(f"Unknown tuning profile: {tuning_profile}")
    tuner = FuzzyL1Tuner(config.base_period_s, config.min_period_s, config.max_period_s,
                         TUNING_PROFILES[tuning_profile])
    name = "L1 + fuzzy" if fuzzy_enabled else "L1 period tetap"

    for i in range(count):
        t = time_s[i]
        velocity_north = config.airspeed_mps * math.cos(heading[i])
        velocity_east = config.airspeed_mps * math.sin(heading[i])

        current_period = config.base_period_s
        if i > 0 and fuzzy_enabled:
            # The firmware calls the tuner after updateLoiter() using fresh error.
            current_period, error_rate[i] = tuner.update(radial_error[i - 1], config.dt_s)
        period[i] = current_period

        accel, error = _loiter_lateral_acceleration(
            north[i], east[i], velocity_north, velocity_east,
            config.loiter_radius_m, current_period, config.damping,
        )
        radial_error[i] = error
        if i == count - 1:
            break

        desired_bank = math.atan2(accel, GRAVITY_MPS2)
        bank_limit = math.radians(config.max_bank_deg)
        desired_bank = float(np.clip(desired_bank, -bank_limit, bank_limit))
        bank_demand[i] = desired_bank

        disturbance = 0.0
        if config.disturbance_start_s <= t < config.disturbance_start_s + config.disturbance_duration_s:
            disturbance = math.radians(config.disturbance_roll_rate_deg_s)

        roll_tau = max(config.roll_time_constant_s, config.dt_s)
        bank_rate = (desired_bank - bank[i]) / roll_tau + disturbance
        bank[i + 1] = float(np.clip(bank[i] + bank_rate * config.dt_s, -bank_limit, bank_limit))

        turn_rate = GRAVITY_MPS2 * math.tan(bank[i]) / config.airspeed_mps
        heading[i + 1] = _wrap_pi(heading[i] + turn_rate * config.dt_s)
        north[i + 1] = north[i] + config.airspeed_mps * math.cos(heading[i]) * config.dt_s
        east[i + 1] = east[i] + config.airspeed_mps * math.sin(heading[i]) * config.dt_s

    bank_demand[-1] = bank_demand[-2] if count > 1 else 0.0
    return SimulationResult(name, time_s, north, east, heading, bank, bank_demand,
                            radial_error, period, error_rate)


def save_csv(result: SimulationResult, path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(("time_s", "north_m", "east_m", "heading_deg", "bank_deg",
                         "bank_demand_deg", "radial_error_m", "l1_period_s", "abs_error_rate_mps"))
        writer.writerows(zip(
            result.time_s, result.north_m, result.east_m,
            np.degrees(result.heading_rad), np.degrees(result.bank_rad),
            np.degrees(result.bank_demand_rad), result.radial_error_m,
            result.period_s, np.abs(result.error_rate_mps),
        ))


def _plot_membership_sets(ax, sets, names, x_max, xlabel, title):
    colors = ("tab:green", "tab:blue", "tab:red")
    x_values = np.linspace(0.0, x_max, 1001)
    for points, name, color in zip(sets, names, colors):
        values = [trapezoid_membership(float(x), points) for x in x_values]
        ax.plot(x_values, values, color=color, linewidth=2,
                label=f"{name} {points}")
        for breakpoint in sorted(set(points)):
            ax.axvline(breakpoint, color=color, linewidth=0.6, alpha=0.18)
    ax.set(title=title, xlabel=xlabel, ylabel="Derajat keanggotaan μ", ylim=(-0.04, 1.08))
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")


def save_plot(results: list[SimulationResult], config: SimulationConfig, path: Path,
              tuning_profile: str) -> None:
    colors = ("tab:blue", "tab:orange")
    figure, axes = plt.subplots(4, 2, figsize=(16, 20), constrained_layout=True)

    circle = np.linspace(0.0, 2.0 * math.pi, 500)
    axes[0, 0].plot(config.loiter_radius_m * np.cos(circle),
                    config.loiter_radius_m * np.sin(circle), "k--", label="lingkaran target")
    for result, color in zip(results, colors):
        axes[0, 0].plot(result.north_m, result.east_m, color=color, label=result.name, linewidth=1.2)
    axes[0, 0].set(title="Lintasan loiter", xlabel="North (m)", ylabel="East (m)", aspect="equal")
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    for result, color in zip(results, colors):
        axes[0, 1].plot(result.time_s, result.radial_error_m, color=color, label=result.name)
    axes[0, 1].axhline(0.0, color="black", linewidth=0.8)
    axes[0, 1].set(title="Error radius (jarak dari pusat − radius target)", xlabel="Waktu (s)", ylabel="Error (m)")
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    for result, color in zip(results, colors):
        axes[1, 0].plot(result.time_s, result.period_s, color=color, label=result.name)
    axes[1, 0].set(title="L1 period", xlabel="Waktu (s)", ylabel="Period (s)")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    for result, color in zip(results, colors):
        axes[1, 1].plot(result.time_s, np.degrees(result.bank_rad), color=color, label=f"{result.name}: aktual")
    axes[1, 1].axvspan(config.disturbance_start_s,
                       config.disturbance_start_s + config.disturbance_duration_s,
                       color="red", alpha=0.12, label="gangguan roll")
    axes[1, 1].set(title="Sudut bank aktual", xlabel="Waktu (s)", ylabel="Bank (deg)")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    profile_sets = TUNING_PROFILES[tuning_profile]
    _plot_membership_sets(
        axes[2, 0], FuzzyL1Tuner.ERROR_SETS, ("Kecil", "Sedang", "Besar"), 60.0,
        "|e| (m)", "Fuzzy input 1: error radius",
    )
    _plot_membership_sets(
        axes[2, 1], FuzzyL1Tuner.RATE_SETS, ("Kecil", "Sedang", "Besar"), 12.0,
        "|d|e|/dt| (m/s)", "Fuzzy input 2: laju perubahan error",
    )
    _plot_membership_sets(
        axes[3, 0], profile_sets, ("Agresif", "Normal", "Gentle"), 1.5,
        "Skala period L1", f"Output fuzzy ({tuning_profile}; T = base period × skala)",
    )

    rule_ax = axes[3, 1]
    rule_ax.axis("off")
    rule_ax.set_title("Rule base fuzzy (Mamdani AND)", pad=12)
    rule_values = np.array([["Gentle", "Normal", "Normal"],
                            ["Normal", "Normal", "Agresif"],
                            ["Agresif", "Agresif", "Agresif"]])
    table = rule_ax.table(
        cellText=rule_values,
        rowLabels=("Kecil", "Sedang", "Besar"),
        colLabels=("Δe kecil", "Δe sedang", "Δe besar"),
        cellLoc="center", rowLoc="center", loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.1, 2.0)
    rule_ax.text(0.5, 0.12,
                 "Baris: |e|     Kolom: |d|e|/dt|\n"
                 "Gentle → period lebih panjang; agresif → lebih pendek\n"
                 f"Base={config.base_period_s:g} s, batas=[{config.min_period_s:g}, {config.max_period_s:g}] s",
                 ha="center", va="center", transform=rule_ax.transAxes, fontsize=10)

    figure.suptitle("Simulasi loiter UAV fixed-wing: L1 konvensional vs fuzzy", fontsize=16)
    figure.savefig(path, dpi=160)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=240.0, help="durasi simulasi dalam detik")
    parser.add_argument("--dt", type=float, default=0.05, help="langkah waktu dalam detik")
    parser.add_argument("--radius", type=float, default=50.0, help="radius loiter dalam meter")
    parser.add_argument("--airspeed", type=float, default=18.0, help="airspeed dalam m/s")
    parser.add_argument("--disturbance-start", type=float, default=90.0, help="awal gangguan roll (s)")
    parser.add_argument("--disturbance-duration", type=float, default=12.0, help="durasi gangguan roll (s)")
    parser.add_argument("--disturbance-rate", type=float, default=7.0,
                        help="gangguan roll sebagai tambahan roll-rate (deg/s)")
    parser.add_argument("--tuning-profile", choices=tuple(TUNING_PROFILES), default="balanced",
                        help="firmware = nilai awal proyek; balanced = period fuzzy lebih dekat baseline")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent / "results",
                        help="root direktori output; setiap run dibuat dalam subfolder timestamp unik")
    parser.add_argument("--no-plot", action="store_true", help="hanya simpan CSV, jangan buat grafik")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = SimulationConfig(
        duration_s=args.duration,
        dt_s=args.dt,
        loiter_radius_m=args.radius,
        airspeed_mps=args.airspeed,
        disturbance_start_s=args.disturbance_start,
        disturbance_duration_s=args.disturbance_duration,
        disturbance_roll_rate_deg_s=args.disturbance_rate,
    )
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run_dir = args.output_dir / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    results = [
        simulate(config, fuzzy_enabled=False, tuning_profile=args.tuning_profile),
        simulate(config, fuzzy_enabled=True, tuning_profile=args.tuning_profile),
    ]
    for result in results:
        filename = "l1_fuzzy.csv" if result.name == "L1 + fuzzy" else "l1_fixed.csv"
        save_csv(result, run_dir / filename)
        metrics = result.metrics(config.loiter_radius_m)
        print(f"\n{result.name}")
        for key, value in metrics.items():
            print(f"  {key}: {value:.3f}")
    if not args.no_plot:
        plot_path = run_dir / "loiter_comparison.png"
        save_plot(results, config, plot_path, args.tuning_profile)
        print(f"\nGrafik: {plot_path}")
    metadata = {
        "run_id": run_id,
        "tuning_profile": args.tuning_profile,
        "config": config.__dict__,
        "files": ["l1_fixed.csv", "l1_fuzzy.csv"] + ([] if args.no_plot else ["loiter_comparison.png"]),
    }
    (run_dir / "run_config.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"CSV dan konfigurasi: {run_dir}")


if __name__ == "__main__":
    main()
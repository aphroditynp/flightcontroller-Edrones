#pragma once

#include <Fuzzy.h>
#include <stdint.h>

#include "navigation/L1Controller.h"

namespace fc {

/**
 * Trapezoidal membership function breakpoints (a <= b <= c <= d), matching
 * eFLL's `FuzzySet(a, b, c, d)` convention.
 */
struct FuzzyTrapezoid {
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;
};

struct FuzzyL1TunerConfig {
    // Runtime A/B selection persisted through Params (0=conventional L1, 1=fuzzy).
    float enabled = 1.0f;

    // Baseline L1 period this tuner scales (should match L1Controller's
    // configured period at startup).
    float base_period_s = 20.0f;
    float min_period_s = 10.0f;
    float max_period_s = 30.0f;

    // Crosstrack error `e` membership functions, meters. Starting points —
    // refine against simulation/flight-log crosstrack error magnitudes
    // before trusting in flight; see docs/fuzzy-l1-tuner.md.
    FuzzyTrapezoid e_small{0.0f, 5.0f, 5.0f, 15.0f};
    FuzzyTrapezoid e_medium{5.0f, 15.0f, 15.0f, 30.0f};
    FuzzyTrapezoid e_large{15.0f, 30.0f, 30.0f, 60.0f};

    // Crosstrack error rate `de/dt` membership functions, meters/second.
    FuzzyTrapezoid de_small{0.0f, 1.0f, 1.0f, 3.0f};
    FuzzyTrapezoid de_medium{1.0f, 3.0f, 3.0f, 6.0f};
    FuzzyTrapezoid de_large{3.0f, 6.0f, 6.0f, 12.0f};

    // Output: multiplier applied to base_period_s. Smaller period = more
    // aggressive L1 tracking (shorter L1 distance, tighter turns); larger
    // period = gentler tracking.
    FuzzyTrapezoid scale_aggressive{0.6f, 0.7f, 0.7f, 0.85f};
    FuzzyTrapezoid scale_normal{0.75f, 1.0f, 1.0f, 1.25f};
    FuzzyTrapezoid scale_gentle{1.15f, 1.3f, 1.3f, 1.5f};
};

/**
 * Fuzzy self-tuner for L1Controller's period, the thesis's core contribution:
 * a 2-input (crosstrack error `e`, its rate `de/dt`) x 3-set eFLL fuzzy
 * system that scales L1Controller::period() at runtime, versus the fixed-
 * period "conventional L1" baseline.
 *
 * Repurposes the same eFLL pattern as the legacy (dormant) fuzzy roll/pitch
 * gain scheduler in Fuzzy_FW_Roll.h/Fuzzy_FW_Pitch.h — same library API, new
 * inputs/output targeting L1 guidance instead of attitude PID gain.
 *
 * setEnabled(false) restores the configured base L1 period, making
 * "L1 conventional" vs. "L1 + fuzzy" a runtime A/B toggle
 * rather than two separate firmware builds — this is the thesis's
 * comparison axis.
 */
class FuzzyL1Tuner final {
public:
    explicit FuzzyL1Tuner(const FuzzyL1TunerConfig& config = FuzzyL1TunerConfig{});
    ~FuzzyL1Tuner();

    FuzzyL1Tuner(const FuzzyL1Tuner&) = delete;
    FuzzyL1Tuner& operator=(const FuzzyL1Tuner&) = delete;

    /** Builds the fuzzy rule base. Call once during initialization. */
    void begin();

    void setEnabled(bool enabled);
    bool enabled() const;

    /**
     * Computes crosstrack error rate internally from consecutive samples,
     * fuzzifies (e, de/dt) -> period scale, and applies the result to `l1`
     * via setPeriod(). When disabled, l1 is forced to base_period_s.
     */
    void update(L1Controller& l1, float crosstrack_error_m, float dt_s);

    float lastAppliedPeriod() const;
    float basePeriod() const;
    float lastCrosstrackErrorRateMps() const;

private:
    void buildRuleBase();

    FuzzyL1TunerConfig config_{};
    Fuzzy* fuzzy_ = nullptr;
    bool enabled_ = true;

    float last_crosstrack_error_m_ = 0.0f;
    float crosstrack_error_rate_mps_ = 0.0f;
    float last_applied_period_s_ = 20.0f;
    bool has_previous_sample_ = false;
};

}  // namespace fc

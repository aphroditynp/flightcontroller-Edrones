#pragma once

#include "modes/Mode.h"
#include "modes/VehicleContext.h"

namespace fc {

/**
 * Fixed-wing modes using the TD/V10-trainer2 inner attitude loop.
 *
 * MANUAL reproduces TD's reversed aileron output. FBWA reproduces TD's
 * stick-to-attitude mapping. AUTO/GUIDED use L1/TECS targets but track them
 * with the same TD P+D inner loop. LOITER uses the existing ArduPilot-derived
 * circular L1 guidance; the thesis-specific FuzzyL1Tuner adapts L1 period
 * online, which changes Kx/Kv inside the circular guidance law.
 */

class ModeManual final : public ModeBase {
public:
    explicit ModeManual(VehicleContext& ctx) : ctx_(ctx) {}
    ModeId id() const override { return ModeId::Manual; }

protected:
    void _update() override;

private:
    VehicleContext& ctx_;
};

class ModeFbwa final : public ModeBase {
public:
    explicit ModeFbwa(VehicleContext& ctx) : ctx_(ctx) {}
    ModeId id() const override { return ModeId::Fbwa; }

protected:
    bool _enter() override;
    void _update() override;

private:
    static float mapStickToDeg(uint16_t channel_pwm, float max_deg);
    static float mapStickToRange(uint16_t channel_pwm, float min_deg, float max_deg);

    VehicleContext& ctx_;
};

class ModeAuto final : public ModeBase {
public:
    explicit ModeAuto(VehicleContext& ctx) : ctx_(ctx) {}
    ModeId id() const override { return ModeId::Auto; }

protected:
    bool _enter() override;
    void _update() override;
    void _exit() override;

private:
    VehicleContext& ctx_;
};

class ModeGuided final : public ModeBase {
public:
    explicit ModeGuided(VehicleContext& ctx) : ctx_(ctx) {}
    ModeId id() const override { return ModeId::Guided; }

protected:
    bool _enter() override;
    void _update() override;
    void _exit() override;

private:
    VehicleContext& ctx_;
};


class ModeLoiter final : public ModeBase {
public:
    explicit ModeLoiter(VehicleContext& ctx) : ctx_(ctx) {}
    ModeId id() const override { return ModeId::Loiter; }

protected:
    bool _enter() override;
    void _update() override;
    void _exit() override;

private:
    VehicleContext& ctx_;
};

}  // namespace fc

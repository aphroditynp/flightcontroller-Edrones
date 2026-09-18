#pragma once

#include <stdint.h>

namespace fc {

/**
 * FW-only mode set (legacy Mode_setup.h's ModeId had COPT/QHOV/TRNS too).
 * There's only one vehicle type now, so the model_uav runtime branch that
 * selected which modes to register is gone  --  this is a compile-time FW-only
 * build, not a runtime dispatch.
 */
enum class ModeId : uint8_t { Manual, Fbwa, Auto, Guided, Loiter, Count };

inline const char* modeCode4(ModeId id)
{
    switch (id) {
        case ModeId::Manual: return "MANU";
        case ModeId::Fbwa: return "FBWA";
        case ModeId::Auto: return "AUTO";
        case ModeId::Guided: return "GUID";
        case ModeId::Loiter: return "LOIT";
        default: return "UNKN";
    }
}

class ModeBase {
public:
    virtual ~ModeBase() = default;
    virtual ModeId id() const = 0;
    virtual const char* code4() const { return modeCode4(id()); }

    bool enter() { return _enter(); }
    void update() { _update(); }
    void exit() { _exit(); }

protected:
    virtual bool _enter() { return true; }
    virtual void _update() {}
    virtual void _exit() {}
};

class ModeManager final {
public:
    ModeManager()
    {
        for (auto& p : table_) {
            p = nullptr;
        }
    }

    void registerMode(ModeBase* mode)
    {
        if (mode == nullptr) {
            return;
        }
        table_[static_cast<uint8_t>(mode->id())] = mode;
    }

    bool setMode(ModeId id)
    {
        ModeBase* next = table_[static_cast<uint8_t>(id)];
        if (next == nullptr) {
            return false;
        }
        if (current_ != nullptr) {
            current_->exit();
        }
        current_ = next;
        if (!current_->enter()) {
            current_ = nullptr;
            return false;
        }
        code4_ = current_->code4();
        return true;
    }

    void update()
    {
        if (current_ != nullptr) {
            current_->update();
        }
    }

    ModeBase* current() const { return current_; }
    const char* code4() const { return code4_; }

private:
    ModeBase* table_[static_cast<uint8_t>(ModeId::Count)]{};
    ModeBase* current_ = nullptr;
    const char* code4_ = "UNKN";
};

}  // namespace fc

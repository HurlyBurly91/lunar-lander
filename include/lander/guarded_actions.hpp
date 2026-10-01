#pragma once

#include <algorithm>
#include <cstdint>
#include <string>

namespace lander {

enum class GuardedKey {
    kNewSeed,
    kCircularizeCW,
    kCircularizeCCW,
    kRetry,
    kSyncOrbit,
    kTransfer,
};

enum class GuardedAction {
    kNone,
    kNewSeed,
    kCircularizeCW,
    kCircularizeCCW,
    kRetry,
    kSyncOrbit,
    kTransfer,
};

// Reusable triple-tap guard for dangerous and debug controls.
//
// The GUI maps discrete, non-autorepeat key-down events onto one of the
// guarded keys and forwards them with a real input/presentation timestamp
// (milliseconds). The helper counts up to three taps per key, expires
// partial sequences after `kTripleTapWindowMs`, fires exactly once on the
// third tap, and clears itself so an immediate fourth tap starts a fresh
// sequence. A mismatched guarded key cancels the previous sequence and
// starts a new one, so two-handed chords cannot trigger an action.
//
// The guarded keys cover the seed/retry controls (N, R) and the developer
// controls (O, Shift+O, B, T). The GUI decides what a fired action does;
// this helper only tracks tap sequences and compact progress feedback.
class TripleTapGuard {
public:
    static constexpr std::uint64_t kTripleTapWindowMs = 700;
    static constexpr int kKeyCount = 6;

    TripleTapGuard() = default;

    // Records one discrete tap of `key` at `now_ms`. `repeat` taps (SDL
    // autorepeat) are ignored. Returns the fired action on the third tap
    // within the window, otherwise kNone.
    GuardedAction press(GuardedKey key, std::uint64_t now_ms,
                        bool repeat = false) {
        if (repeat) {
            return GuardedAction::kNone;
        }

        const int index = static_cast<int>(key);
        for (int i = 0; i < kKeyCount; ++i) {
            if (i != index) {
                sequences_[i] = Sequence{};
            }
        }

        Sequence& seq = sequences_[index];
        if (!seq.active || now_ms < seq.last_ms ||
            now_ms - seq.start_ms > kTripleTapWindowMs) {
            seq.active = true;
            seq.taps = 1;
            seq.start_ms = now_ms;
            seq.last_ms = now_ms;
            return GuardedAction::kNone;
        }

        ++seq.taps;
        seq.last_ms = now_ms;
        if (seq.taps < 3) {
            return GuardedAction::kNone;
        }

        const GuardedAction action = action_for_key(key);
        reset();
        return action;
    }

    // Compact progress label for the most recent unexpired partial sequence
    // (e.g. "N 2/3", "RETRY 1/3", "SYNC ORBIT 2/3"). Returns an empty
    // string when no feedback is due.
    std::string progress_label(std::uint64_t now_ms) const {
        const Sequence* latest = nullptr;
        for (const Sequence& seq : sequences_) {
            if (!seq.active || seq.taps >= 3 || now_ms < seq.last_ms ||
                now_ms - seq.start_ms > kTripleTapWindowMs) {
                continue;
            }
            if (latest == nullptr || seq.last_ms > latest->last_ms) {
                latest = &seq;
            }
        }
        if (latest == nullptr) {
            return std::string();
        }
        const int index = static_cast<int>(latest - &sequences_[0]);
        return std::string(key_name(GuardedKey(index))) + " " +
               std::to_string(latest->taps) + "/3";
    }

    void reset() {
        for (auto& seq : sequences_) {
            seq = Sequence{};
        }
    }

    bool empty() const {
        for (const Sequence& seq : sequences_) {
            if (seq.active) {
                return false;
            }
        }
        return true;
    }

private:
    struct Sequence {
        bool active = false;
        int taps = 0;
        std::uint64_t start_ms = 0;
        std::uint64_t last_ms = 0;
    };

    static const char* key_name(GuardedKey key) {
        switch (key) {
            case GuardedKey::kNewSeed:
                return "N";
            case GuardedKey::kCircularizeCW:
                return "O";
            case GuardedKey::kCircularizeCCW:
                return "SHIFT+O";
            case GuardedKey::kRetry:
                return "RETRY";
            case GuardedKey::kSyncOrbit:
                return "SYNC ORBIT";
            case GuardedKey::kTransfer:
                return "TRANSFER";
        }
        return "";
    }

    static GuardedAction action_for_key(GuardedKey key) {
        switch (key) {
            case GuardedKey::kNewSeed:
                return GuardedAction::kNewSeed;
            case GuardedKey::kCircularizeCW:
                return GuardedAction::kCircularizeCW;
            case GuardedKey::kCircularizeCCW:
                return GuardedAction::kCircularizeCCW;
            case GuardedKey::kRetry:
                return GuardedAction::kRetry;
            case GuardedKey::kSyncOrbit:
                return GuardedAction::kSyncOrbit;
            case GuardedKey::kTransfer:
                return GuardedAction::kTransfer;
        }
        return GuardedAction::kNone;
    }

    Sequence sequences_[kKeyCount]{};
};

// M05-R4-03 / M05-R5: reaction-wheel control state for the GUI/control
// layer. The stored toggle is changed only by an unmodified, non-autorepeat
// `E` press; a `Shift+E` press is reported but does not toggle it. The GUI
// composes the stored toggle with the transient `Shift+E` hold, the
// manual-rotation priority rule, and the active/crash context into the
// per-frame `Input.reaction_wheels` bool, and resets the stored toggle
// whenever a new mission/seed begins.
class ReactionWheelToggle {
public:
    // Records one `E` key-down. A normal press toggles the stored state;
    // autorepeat and Shift-qualified presses do not. Returns the stored
    // state after the event.
    bool press(bool repeat = false, bool shift = false) {
        if (!repeat && !shift) {
            enabled_ = !enabled_;
        }
        return enabled_;
    }

    void reset() { enabled_ = false; }

    bool enabled() const { return enabled_; }

    // Composes the per-frame simulation input. `hold` is the transient
    // `Shift+E` key state; it arms damping for this call only. Manual
    // rotation takes priority, and `active == false` (for example after a
    // crash) suppresses the input without changing stored state.
    bool input(bool manual_rotation, bool active = true,
               bool hold = false) const {
        return active && (enabled_ || hold) && !manual_rotation;
    }

private:
    bool enabled_{false};
};

}  // namespace lander

#pragma once

#include <algorithm>
#include <cstdint>
#include <string>

namespace lander {

enum class GuardedKey {
    kNewSeed,
    kCircularizeCW,
    kCircularizeCCW,
};

enum class GuardedAction {
    kNone,
    kNewSeed,
    kCircularizeCW,
    kCircularizeCCW,
};

// Reusable triple-tap guard for dangerous/debug controls.
//
// The GUI maps discrete, non-autorepeat key-down events onto one of the
// guarded keys and forwards them with a real input/presentation timestamp
// (milliseconds). The helper counts up to three taps per key, expires
// partial sequences after `kTripleTapWindowMs`, fires exactly once on the
// third tap, and clears itself so an immediate fourth tap starts a fresh
// sequence. A mismatched guarded key cancels the previous sequence and
// starts a new one.
class TripleTapGuard {
public:
    static constexpr std::uint64_t kTripleTapWindowMs = 700;

    TripleTapGuard() = default;

    GuardedAction press(GuardedKey key, std::uint64_t now_ms,
                        bool repeat = false) {
        if (repeat) {
            return GuardedAction::kNone;
        }

        const int index = static_cast<int>(key);
        for (int i = 0; i < 3; ++i) {
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
    // (e.g. "N 2/3"). Returns an empty string when no feedback is due.
    std::string progress_label(std::uint64_t now_ms) const {
        const Sequence* latest = nullptr;
        for (const Sequence& seq : sequences_) {
            if (!seq.active || seq.taps >= 3 ||
                now_ms < seq.last_ms ||
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
        const int index =
            static_cast<int>(latest - &sequences_[0]);
        const char* name =
            index == 0 ? "N" : (index == 1 ? "O" : "SHIFT+O");
        return std::string(name) + " " + std::to_string(latest->taps) +
               "/3";
    }

    void reset() {
        sequences_[0] = Sequence{};
        sequences_[1] = Sequence{};
        sequences_[2] = Sequence{};
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

    static GuardedAction action_for_key(GuardedKey key) {
        switch (key) {
            case GuardedKey::kNewSeed:
                return GuardedAction::kNewSeed;
            case GuardedKey::kCircularizeCW:
                return GuardedAction::kCircularizeCW;
            case GuardedKey::kCircularizeCCW:
                return GuardedAction::kCircularizeCCW;
        }
        return GuardedAction::kNone;
    }

    Sequence sequences_[3]{};
};

}  // namespace lander

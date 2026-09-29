#include "lander/guarded_actions.hpp"

#include <cstdio>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

using lander::GuardedAction;
using lander::GuardedKey;
using lander::TripleTapGuard;

void test_single_and_double_press_do_not_fire() {
    TripleTapGuard guard;

    check(guard.press(GuardedKey::kNewSeed, 100) == GuardedAction::kNone,
          "a single N press does not fire");
    check(guard.press(GuardedKey::kNewSeed, 200) == GuardedAction::kNone,
          "a double N press does not fire");

    TripleTapGuard circularize;
    check(circularize.press(GuardedKey::kCircularizeCW, 100) ==
              GuardedAction::kNone,
          "a single O press does not fire");
    check(circularize.press(GuardedKey::kCircularizeCW, 200) ==
              GuardedAction::kNone,
          "a double O press does not fire");
}

void test_triple_press_fires_exactly_once() {
    TripleTapGuard guard;
    guard.press(GuardedKey::kNewSeed, 100);
    guard.press(GuardedKey::kNewSeed, 200);
    check(guard.press(GuardedKey::kNewSeed, 300) == GuardedAction::kNewSeed,
          "the third N press fires NEW SEED");
    check(guard.empty(), "the sequence clears after firing");

    // An immediate fourth press starts a new sequence instead of refiring.
    check(guard.press(GuardedKey::kNewSeed, 350) == GuardedAction::kNone,
          "an immediate fourth press starts a new sequence");
    check(guard.press(GuardedKey::kNewSeed, 400) == GuardedAction::kNone,
          "the fourth press is only tap one of the new sequence");
    check(guard.press(GuardedKey::kNewSeed, 450) == GuardedAction::kNewSeed,
          "the next triple tap fires again");

    TripleTapGuard cw;
    cw.press(GuardedKey::kCircularizeCW, 100);
    cw.press(GuardedKey::kCircularizeCW, 200);
    check(cw.press(GuardedKey::kCircularizeCW, 300) ==
              GuardedAction::kCircularizeCW,
          "the third O press fires circularize CW");

    TripleTapGuard ccw;
    ccw.press(GuardedKey::kCircularizeCCW, 100);
    ccw.press(GuardedKey::kCircularizeCCW, 200);
    check(ccw.press(GuardedKey::kCircularizeCCW, 300) ==
              GuardedAction::kCircularizeCCW,
          "the third Shift+O press fires circularize CCW");
}

void test_timeout_resets_partial_sequence() {
    TripleTapGuard guard;
    guard.press(GuardedKey::kNewSeed, 100);
    guard.press(GuardedKey::kNewSeed, 200);
    check(guard.press(GuardedKey::kNewSeed,
                      100 + TripleTapGuard::kTripleTapWindowMs + 1) ==
              GuardedAction::kNone,
          "a timeout between the second and third tap resets the sequence");
    check(guard.press(GuardedKey::kNewSeed, 900) == GuardedAction::kNone,
          "after a timeout a second tap still does not fire");
    check(guard.press(GuardedKey::kNewSeed, 1000) ==
              GuardedAction::kNewSeed,
          "a fresh triple tap after a timeout fires");
}

void test_autorepeat_is_ignored() {
    TripleTapGuard guard;
    check(guard.press(GuardedKey::kCircularizeCW, 100, true) ==
              GuardedAction::kNone,
          "a repeating O key-down is ignored");
    check(guard.press(GuardedKey::kCircularizeCW, 150, true) ==
              GuardedAction::kNone,
          "repeated O key-downs do not accumulate");
    check(guard.empty(), "holding O does not start a partial sequence");

    guard.press(GuardedKey::kCircularizeCW, 100);
    check(guard.press(GuardedKey::kCircularizeCW, 150, true) ==
              GuardedAction::kNone,
          "an autorepeat between taps does not count");
    check(guard.press(GuardedKey::kCircularizeCW, 200) ==
              GuardedAction::kNone,
          "only one discrete tap has counted after the ignored repeat");
}

void test_mixed_o_and_shift_o_sequences() {
    TripleTapGuard guard;
    guard.press(GuardedKey::kCircularizeCW, 100);
    guard.press(GuardedKey::kCircularizeCCW, 200);
    check(guard.press(GuardedKey::kCircularizeCW, 300) ==
              GuardedAction::kNone,
          "O, Shift+O, O does not fire CW");
    check(guard.press(GuardedKey::kCircularizeCCW, 400) ==
              GuardedAction::kNone,
          "the mismatched chord restarts the Shift+O sequence");

    guard.press(GuardedKey::kCircularizeCCW, 500);
    check(guard.press(GuardedKey::kCircularizeCCW, 600) ==
              GuardedAction::kCircularizeCCW,
          "three consecutive Shift+O taps after a mismatch fire CCW");
}

void test_progress_label_and_expiry() {
    TripleTapGuard guard;
    check(guard.progress_label(0).empty(), "no label before any tap");

    guard.press(GuardedKey::kNewSeed, 100);
    check(guard.progress_label(150) == "N 1/3",
          "one tap shows compact 1/3 feedback");
    guard.press(GuardedKey::kNewSeed, 200);
    check(guard.progress_label(250) == "N 2/3",
          "two taps show compact 2/3 feedback");
    guard.press(GuardedKey::kNewSeed, 300);
    check(guard.progress_label(350).empty(),
          "the label clears once the triple tap has fired");

    TripleTapGuard ccw;
    ccw.press(GuardedKey::kCircularizeCCW, 1000);
    check(ccw.progress_label(1050) == "SHIFT+O 1/3",
          "Shift+O progress uses its own label");
    check(ccw.progress_label(
              1000 + TripleTapGuard::kTripleTapWindowMs + 1)
              .empty(),
          "progress feedback expires with the partial sequence");
}

}  // namespace

int main() {
    test_single_and_double_press_do_not_fire();
    test_triple_press_fires_exactly_once();
    test_timeout_resets_partial_sequence();
    test_autorepeat_is_ignored();
    test_mixed_o_and_shift_o_sequences();
    test_progress_label_and_expiry();

    if (failures == 0) {
        std::puts("All lander_guarded_actions_tests passed");
        return 0;
    }
    std::printf("%d lander_guarded_actions_tests failed\n", failures);
    return 1;
}

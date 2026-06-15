// Self-test for inc/trace_segmenter.h against the segmentation-algorithm deck.
// Each scenario feeds the committed-uop sequence from the slides and asserts
// the collected traces match the slides' expected outcome.
//
// Build & run (standalone, not part of the ChampSim build):
//   g++ -std=c++17 -Iinc -Ivcpkg_installed/x64-linux/include \
//       test/segmenter_selftest.cc -o /tmp/segmenter_selftest && /tmp/segmenter_selftest

#include <cstdio>
#include <vector>

#include "trace_segmenter.h"

namespace
{
int failures = 0;

struct step {
  uint64_t ip;
  uint64_t target = 0;
  uint8_t type = BRANCH_OTHER;
  bool is_branch = false;
  bool taken = false;
};

struct harness {
  trace_segmenter seg{};
  cpu_stats stats{};
  void feed(const std::vector<step>& steps)
  {
    for (const auto& s : steps) {
      seg.push_raw(s.ip, s.target, s.type, s.is_branch, s.taken, stats);
    }
  }
};

void check(bool cond, const char* what)
{
  if (cond) {
    std::printf("  PASS  %s\n", what);
  } else {
    std::printf("  FAIL  %s\n", what);
    ++failures;
  }
}

bool trace_is(const trace_segmenter::trace& t, const std::vector<uint64_t>& ips) { return t.ips == ips; }

// convenience branch step makers
step nb(uint64_t ip) { return {ip}; }                                                                  // non-branch uop
step br(uint64_t ip, uint64_t tgt, uint8_t type, bool taken) { return {ip, tgt, type, true, taken}; } // branch uop
} // namespace

int main()
{
  constexpr uint64_t A = 0x1000, B = 0x1004, C = 0x1008, D = 0x100c, E = 0x1010, F = 0x1014;

  // ---- deck p.2-5: basic loop trace -------------------------------------
  {
    std::printf("[basic loop]  A B C D(->B) B C  => trace B,C,D\n");
    harness h;
    h.feed({nb(A), nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true), nb(B), nb(C)});
    check(h.stats.seg_traces_loop == 1, "one loop trace");
    check(h.seg.get_traces().size() == 1 && trace_is(h.seg.get_traces()[0], {B, C, D}), "trace == {B,C,D}");
    check(h.stats.seg_invariant_violations == 0, "no invariant violations");
  }

  // ---- deck p.6-9: function trace ----------------------------------------
  {
    std::printf("[function]  A B(call->X) X Y(ret->C) C  => trace X,Y\n");
    constexpr uint64_t X = 0x2000, Y = 0x2004;
    harness h;
    h.feed({nb(A), br(B, X, BRANCH_DIRECT_CALL, true), nb(X), br(Y, C, BRANCH_RETURN, true), nb(C)});
    check(h.stats.seg_traces_function == 1, "one function trace");
    check(h.seg.get_traces().size() == 1 && trace_is(h.seg.get_traces()[0], {X, Y}), "trace == {X,Y}");
    check(h.stats.seg_invariant_violations == 0, "no invariant violations");
  }

  // ---- deck p.30: dedup on second occurrence ------------------------------
  {
    std::printf("[dedup]  loop body committed twice  => 1 trace + 1 dedup hit\n");
    harness h;
    h.feed({nb(A), nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true), nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true)});
    check(h.stats.seg_traces_loop == 1, "one stored loop trace");
    check(h.stats.seg_traces_dedup == 1, "one dedup hit");
    check(h.seg.get_traces().size() == 1 && h.seg.get_traces()[0].occurrences == 2, "occurrences == 2");
  }

  // ---- deck p.26-33: nested traces (outer captured as-is) -----------------
  {
    std::printf("[nested]  A B C(->B) B C D(->A)  => traces {B,C} and {A,B,C,B,C,D}\n");
    harness h;
    h.feed({nb(A), nb(B), br(C, B, BRANCH_CONDITIONAL, true), nb(B), br(C, B, BRANCH_CONDITIONAL, false), br(D, A, BRANCH_CONDITIONAL, true)});
    check(h.stats.seg_traces_loop == 2, "two loop traces");
    check(h.seg.get_traces().size() == 2 && trace_is(h.seg.get_traces()[0], {B, C}), "inner trace == {B,C}");
    check(trace_is(h.seg.get_traces()[1], {A, B, C, B, C, D}), "outer trace == {A,B,C,B,C,D} (no truncation)");
    check(h.stats.seg_traces_entangled == 0, "nested is not entangled");
  }

  // ---- deck p.10-18: entangled traces -------------------------------------
  {
    std::printf("[entangled]  A B C D(->B) B(->E) E F(->C)  => traces {B,C,D} and {B,E,F}\n");
    harness h;
    h.feed({nb(A), nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true), br(B, E, BRANCH_CONDITIONAL, true), nb(E), br(F, C, BRANCH_CONDITIONAL, true)});
    check(h.stats.seg_traces_loop == 2, "two loop traces");
    check(h.stats.seg_traces_entangled == 1, "one entangled truncation");
    check(h.seg.get_traces().size() == 2 && trace_is(h.seg.get_traces()[1], {B, E, F}), "entangled trace == {B,E,F}");
  }

  // ---- deck p.35-47: entangled with bad layout -> dropped ------------------
  {
    std::printf("[bad layout]  uncond F->C entangled by uncond D->A with inner fwd B->E  => dropped\n");
    harness h;
    h.feed({nb(A), br(B, E, BRANCH_CONDITIONAL, false), nb(C), br(D, A, BRANCH_DIRECT_JUMP, true), // first iteration: trace {A,B,C,D}
            nb(A), br(B, E, BRANCH_CONDITIONAL, true), nb(E), br(F, C, BRANCH_DIRECT_JUMP, true),  // F->C: entangled + bad layout -> drop
            nb(C), br(D, A, BRANCH_DIRECT_JUMP, true)});                                           // deck p.48-50: D->A collects {A,B,E,F,C,D}
    check(h.stats.seg_traces_dropped_bad_layout == 1, "one bad-layout drop");
    check(h.stats.seg_traces_loop == 2, "two loop traces stored");
    check(h.seg.get_traces().size() == 2 && trace_is(h.seg.get_traces()[0], {A, B, C, D}), "first trace == {A,B,C,D}");
    check(trace_is(h.seg.get_traces()[1], {A, B, E, F, C, D}), "second trace == {A,B,E,F,C,D} (deck p.50)");
  }

  // ---- deck p.19-25: unmatched call truncates a loop trace ----------------
  {
    std::printf("[call inside]  A B(call->X) X Y(->A backward)  => loop trace kept only until the call: {A,B}\n");
    constexpr uint64_t X = 0x2000, Y = 0x2004;
    harness h;
    h.feed({nb(A), br(B, X, BRANCH_DIRECT_CALL, true), nb(X), br(Y, A, BRANCH_CONDITIONAL, true)});
    check(h.stats.seg_traces_loop == 1, "one loop trace");
    check(h.seg.get_traces().size() == 1 && trace_is(h.seg.get_traces()[0], {A, B}), "trace == {A,B} (truncated at call)");
  }

  // ---- coverage accounting: no double counting -----------------------------
  {
    std::printf("[coverage]  overlapping traces never double-count an ip\n");
    harness h;
    h.feed({nb(A), nb(B), br(C, B, BRANCH_CONDITIONAL, true), nb(B), br(C, B, BRANCH_CONDITIONAL, false), br(D, A, BRANCH_CONDITIONAL, true)});
    // unique ips seen: A,B,C,D = 4; covered: A,B,C,D = 4 (B,C in both traces, counted once)
    check(h.stats.seg_unique_ips_seen == 4, "4 unique ips seen");
    check(h.stats.seg_unique_ips_covered == 4, "4 unique ips covered (B,C counted once)");
  }

  std::printf("\n%s (%d failure%s)\n", failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED", failures, failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}

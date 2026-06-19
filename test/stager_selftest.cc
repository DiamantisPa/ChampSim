// Self-test for inc/trace_stager.h (the staging-buffer builder). It mirrors the
// backward segmenter's capture semantics via a short ring + pos-lookup + call
// stack, so it must reproduce the segmenter self-test's outcomes exactly when
// the window M is large enough that nothing overflows.
//
// Build & run (standalone, not part of the ChampSim build):
//   g++ -std=c++17 -Iinc -Ivcpkg_installed/x64-linux/include \
//       test/stager_selftest.cc -o /tmp/stager_selftest && /tmp/stager_selftest

#include <cstdio>
#include <vector>

#include "trace_stager.h"

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
  trace_stager stg{512, 1024, 512}; // large window/pos/branch-fifo so the deck scenarios never overflow
  cpu_stats stats{};
  void feed(const std::vector<step>& steps)
  {
    for (const auto& s : steps) {
      stg.push_raw(s.ip, s.target, s.type, s.is_branch, s.taken, stats);
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

bool trace_is(const trace_stager::trace& t, const std::vector<uint64_t>& ips) { return t.ips == ips; }

step nb(uint64_t ip) { return {ip}; }
step br(uint64_t ip, uint64_t tgt, uint8_t type, bool taken) { return {ip, tgt, type, true, taken}; }
} // namespace

int main()
{
  constexpr uint64_t A = 0x1000, B = 0x1004, C = 0x1008, D = 0x100c, E = 0x1010, F = 0x1014;

  // ---- basic loop trace ---------------------------------------------------
  {
    std::printf("[basic loop]  A B C D(->B) B C  => trace B,C,D\n");
    harness h;
    h.feed({nb(A), nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true), nb(B), nb(C)});
    check(h.stats.stg_traces_loop == 1, "one loop trace");
    check(h.stg.get_traces().size() == 1 && trace_is(h.stg.get_traces()[0], {B, C, D}), "trace == {B,C,D}");
    check(h.stats.stg_invariant_violations == 0, "no invariant violations");
  }

  // ---- function trace -----------------------------------------------------
  {
    std::printf("[function]  A B(call->X) X Y(ret->C) C  => trace X,Y\n");
    constexpr uint64_t X = 0x2000, Y = 0x2004;
    harness h;
    h.feed({nb(A), br(B, X, BRANCH_DIRECT_CALL, true), nb(X), br(Y, C, BRANCH_RETURN, true), nb(C)});
    check(h.stats.stg_traces_function == 1, "one function trace");
    check(h.stg.get_traces().size() == 1 && trace_is(h.stg.get_traces()[0], {X, Y}), "trace == {X,Y}");
    check(h.stats.stg_invariant_violations == 0, "no invariant violations");
  }

  // ---- dedup --------------------------------------------------------------
  {
    std::printf("[dedup]  loop body committed twice  => 1 trace + 1 dedup hit\n");
    harness h;
    h.feed({nb(A), nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true), nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true)});
    check(h.stats.stg_traces_loop == 1, "one stored loop trace");
    check(h.stats.stg_traces_dedup == 1, "one dedup hit");
    check(h.stg.get_traces().size() == 1 && h.stg.get_traces()[0].occurrences == 2, "occurrences == 2");
  }

  // ---- nested traces (outer captured as-is) -------------------------------
  {
    std::printf("[nested]  A B C(->B) B C D(->A)  => traces {B,C} and {A,B,C,B,C,D}\n");
    harness h;
    h.feed({nb(A), nb(B), br(C, B, BRANCH_CONDITIONAL, true), nb(B), br(C, B, BRANCH_CONDITIONAL, false), br(D, A, BRANCH_CONDITIONAL, true)});
    check(h.stats.stg_traces_loop == 2, "two loop traces");
    check(h.stg.get_traces().size() == 2 && trace_is(h.stg.get_traces()[0], {B, C}), "inner trace == {B,C}");
    check(trace_is(h.stg.get_traces()[1], {A, B, C, B, C, D}), "outer trace == {A,B,C,B,C,D} (no truncation)");
    check(h.stats.stg_traces_entangled == 0, "nested is not entangled");
  }

  // ---- entangled traces ---------------------------------------------------
  {
    std::printf("[entangled]  A B C D(->B) B(->E) E F(->C)  => traces {B,C,D} and {B,E,F}\n");
    harness h;
    h.feed({nb(A), nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true), br(B, E, BRANCH_CONDITIONAL, true), nb(E), br(F, C, BRANCH_CONDITIONAL, true)});
    check(h.stats.stg_traces_loop == 2, "two loop traces");
    check(h.stats.stg_traces_entangled == 1, "one entangled truncation");
    check(h.stg.get_traces().size() == 2 && trace_is(h.stg.get_traces()[1], {B, E, F}), "entangled trace == {B,E,F}");
  }

  // ---- entangled with bad layout -> dropped -------------------------------
  {
    std::printf("[bad layout]  uncond F->C entangled by uncond D->A with inner fwd B->E  => dropped\n");
    harness h;
    h.feed({nb(A), br(B, E, BRANCH_CONDITIONAL, false), nb(C), br(D, A, BRANCH_DIRECT_JUMP, true),
            nb(A), br(B, E, BRANCH_CONDITIONAL, true), nb(E), br(F, C, BRANCH_DIRECT_JUMP, true),
            nb(C), br(D, A, BRANCH_DIRECT_JUMP, true)});
    check(h.stats.stg_traces_dropped_bad_layout == 1, "one bad-layout drop");
    check(h.stats.stg_traces_loop == 2, "two loop traces stored");
    check(h.stg.get_traces().size() == 2 && trace_is(h.stg.get_traces()[0], {A, B, C, D}), "first trace == {A,B,C,D}");
    check(trace_is(h.stg.get_traces()[1], {A, B, E, F, C, D}), "second trace == {A,B,E,F,C,D}");
  }

  // ---- unmatched call truncates a loop trace ------------------------------
  {
    std::printf("[call inside]  A B(call->X) X Y(->A backward)  => loop trace {A,B}\n");
    constexpr uint64_t X = 0x2000, Y = 0x2004;
    harness h;
    h.feed({nb(A), br(B, X, BRANCH_DIRECT_CALL, true), nb(X), br(Y, A, BRANCH_CONDITIONAL, true)});
    check(h.stats.stg_traces_loop == 1, "one loop trace");
    check(h.stg.get_traces().size() == 1 && trace_is(h.stg.get_traces()[0], {A, B}), "trace == {A,B} (truncated at call)");
  }

  // ---- coverage: no double counting ---------------------------------------
  {
    std::printf("[coverage]  overlapping traces never double-count an ip\n");
    harness h;
    h.feed({nb(A), nb(B), br(C, B, BRANCH_CONDITIONAL, true), nb(B), br(C, B, BRANCH_CONDITIONAL, false), br(D, A, BRANCH_CONDITIONAL, true)});
    check(h.stats.stg_unique_ips_seen == 4, "4 unique ips seen");
    check(h.stats.stg_unique_ips_covered == 4, "4 unique ips covered (B,C counted once)");
  }

  // ---- window overflow: loop longer than M is dropped ---------------------
  {
    std::printf("[overflow]  loop body longer than the window M is dropped\n");
    trace_stager small{4, 1024, 512}; // tiny window, ample pos/branch-fifo (only the window bounds)
    cpu_stats st{};
    // body A,B,C,D,E (5 ips) with back-edge E->A, window only 4 -> target A falls out
    small.push_raw(A, 0, BRANCH_OTHER, false, false, st);
    small.push_raw(B, 0, BRANCH_OTHER, false, false, st);
    small.push_raw(C, 0, BRANCH_OTHER, false, false, st);
    small.push_raw(D, 0, BRANCH_OTHER, false, false, st);
    small.push_raw(E, A, BRANCH_CONDITIONAL, true, true, st); // A already evicted from 4-deep ring
    check(st.stg_traces_dropped_overflow >= 1, "overflow drop when trace exceeds window");
    check(small.get_traces().empty(), "no trace stored on overflow");
  }

  std::printf("\n%s (%d failure%s)\n", failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED", failures, failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}

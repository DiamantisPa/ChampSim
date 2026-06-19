// Self-test for inc/trace_recorder.h -- the FORWARD armed-recorder trace
// builder.  Forward capture sees the NEXT iteration after the one that armed a
// loop, so every loop scenario below feeds enough iterations for the arm to
// fire and the recorder to complete.
//
// Build & run (standalone, not part of the ChampSim build):
//   g++ -std=c++17 -Iinc -Ivcpkg_installed/x64-linux/include \
//       test/recorder_selftest.cc -o /tmp/recorder_selftest && /tmp/recorder_selftest

#include <cstdio>
#include <vector>

#include "trace_recorder.h"

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
  trace_recorder rec{8}; // fixed pool so the test is independent of PROMETHEUS_RECORDERS
  cpu_stats stats{};
  void feed(const std::vector<step>& steps)
  {
    for (const auto& s : steps) {
      rec.push_raw(s.ip, s.target, s.type, s.is_branch, s.taken, stats);
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

bool has_trace(const trace_recorder& r, const std::vector<uint64_t>& ips)
{
  for (const auto& t : r.get_traces()) {
    if (t.ips == ips) {
      return true;
    }
  }
  return false;
}

step nb(uint64_t ip) { return {ip}; }
step br(uint64_t ip, uint64_t tgt, uint8_t type, bool taken) { return {ip, tgt, type, true, taken}; }
} // namespace

int main()
{
  constexpr uint64_t A = 0x100, B = 0x110, C = 0x120, D = 0x130;

  // ---- basic loop: arm on iter1 back-edge, fire on a later head -----------
  {
    std::printf("[basic loop]  body B,C,D back-edge D->B, several iterations\n");
    harness h;
    h.feed({nb(A),
            nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true),  // iter1 (arms next)
            nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true),  // iter2 (fires next)
            nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, true),  // iter3 (recorded)
            nb(B), nb(C), br(D, B, BRANCH_CONDITIONAL, false), // iter4 exit
            nb(A)});
    check(has_trace(h.rec, {B, C, D}), "captured loop trace {B,C,D}");
    check(h.stats.rec_traces_loop == 1, "one unique loop trace");
    check(h.stats.rec_invariant_violations == 0, "no invariant violations");
  }

  // ---- dedup: same body captured on multiple iterations -------------------
  {
    std::printf("[dedup]  long-running loop -> one stored trace, occurrences > 1\n");
    harness h;
    std::vector<step> s{nb(A)};
    for (int i = 0; i < 6; ++i) {
      s.push_back(nb(B));
      s.push_back(nb(C));
      s.push_back(br(D, B, BRANCH_CONDITIONAL, i < 5));
    }
    s.push_back(nb(A));
    h.feed(s);
    check(h.stats.rec_traces_loop == 1, "one stored loop trace");
    check(h.stats.rec_traces_dedup >= 1, "at least one dedup hit");
  }

  // ---- function: call pushes a recorder, return finalizes it --------------
  {
    std::printf("[function]  A CALL->X, X, Y(ret->C), C  => trace {X,Y}\n");
    constexpr uint64_t X = 0x200, Y = 0x210;
    harness h;
    h.feed({nb(A), br(B, X, BRANCH_DIRECT_CALL, true), nb(X), br(Y, C, BRANCH_RETURN, true), nb(C)});
    check(h.stats.rec_traces_function == 1, "one function trace");
    check(has_trace(h.rec, {X, Y}), "function trace == {X,Y}");
  }

  // ---- nested function: outer captures inner flat -------------------------
  {
    std::printf("[nested fn]  f calls g; outer fn trace includes g flat\n");
    constexpr uint64_t F0 = 0x200, F1 = 0x204, G0 = 0x300, G1 = 0x304, RF = 0x208, RG = 0x308;
    harness h;
    h.feed({br(A, F0, BRANCH_DIRECT_CALL, true), // call f
            nb(F0), br(F1, G0, BRANCH_DIRECT_CALL, true), // f body, call g
            nb(G0), br(RG, F1 + 4, BRANCH_RETURN, true),  // g body, return to f
            br(RF, B, BRANCH_RETURN, true),               // f returns
            nb(B)});
    check(has_trace(h.rec, {G0, RG}), "inner fn trace {G0,RG}");
    check(has_trace(h.rec, {F0, F1, G0, RG, RF}), "outer fn trace flat incl. g");
  }

  // ---- nested loops: inner back-edge (target >= start) does NOT truncate ---
  {
    std::printf("[nested loop]  outer {A..D}, inner {B,C}; outer flat, not truncated\n");
    harness h;
    auto outer_iter = [&](std::vector<step>& s, bool last) {
      s.push_back(nb(A));
      s.push_back(nb(B));
      s.push_back(br(C, B, BRANCH_CONDITIONAL, true));  // inner back-edge taken
      s.push_back(nb(B));
      s.push_back(br(C, B, BRANCH_CONDITIONAL, false)); // inner exit
      s.push_back(br(D, A, BRANCH_CONDITIONAL, !last)); // outer back-edge
    };
    std::vector<step> s;
    outer_iter(s, false); // iter1: arms
    outer_iter(s, false); // iter2: fires inner / arms outer
    outer_iter(s, false); // iter3: fires outer
    outer_iter(s, true);  // iter4: outer exit
    h.feed(s);
    check(has_trace(h.rec, {B, C}), "inner loop trace {B,C}");
    check(has_trace(h.rec, {A, B, C, B, C, D}), "outer loop trace flat {A,B,C,B,C,D}");
    check(h.stats.rec_traces_entangled == 0, "nested is not entangled");
  }

  // ---- entangled: foreign backward branch (target < start) truncates -------
  {
    std::printf("[entangled]  active loop {P..Q}, foreign R->S with S<P truncates\n");
    constexpr uint64_t S = 0x80, P = 0x200, R = 0x300, Q = 0x400;
    harness h;
    h.feed({nb(P), br(R, S, BRANCH_CONDITIONAL, false), br(Q, P, BRANCH_CONDITIONAL, true), // iter1 arms
            nb(P), br(R, S, BRANCH_CONDITIONAL, false), br(Q, P, BRANCH_CONDITIONAL, true), // iter2
            nb(P), br(R, S, BRANCH_CONDITIONAL, true)});                                    // iter3 fires, R->S entangles
    check(h.stats.rec_traces_entangled == 1, "one entangled truncation");
    check(has_trace(h.rec, {P, R}), "truncated trace {P,R}");
  }

  // ---- bad layout: uncond end + taken fwd cond inside -> drop --------------
  {
    std::printf("[bad layout]  end uncond Q, taken fwd-cond F inside, foreign R->S -> drop\n");
    constexpr uint64_t S = 0x80, P = 0x200, F = 0x210, W = 0x260, R = 0x300, Q = 0x400;
    harness h;
    h.feed({nb(P), br(Q, P, BRANCH_DIRECT_JUMP, true), // iter1 arms (uncond end)
            nb(P), br(Q, P, BRANCH_DIRECT_JUMP, true), // iter2
            nb(P), br(F, W, BRANCH_CONDITIONAL, true), // iter3 fires; taken fwd cond
            br(R, S, BRANCH_CONDITIONAL, true)});      // foreign backward -> bad layout drop
    check(h.stats.rec_traces_dropped_bad_layout == 1, "one bad-layout drop");
    check(!has_trace(h.rec, {P, F, R}), "bad-layout trace not stored");
  }

  // ---- call inside loop: unmatched call truncates to [start..call] ---------
  {
    std::printf("[call inside]  loop body with unmatched call truncates trace\n");
    constexpr uint64_t X = 0x500;
    harness h;
    // body: A, B(call->X, no return before back-edge), back-edge D->A
    auto iter = [&](std::vector<step>& s, bool last) {
      s.push_back(nb(A));
      s.push_back(br(B, X, BRANCH_DIRECT_CALL, true)); // call, never returns in-body
      s.push_back(nb(X));
      s.push_back(br(D, A, BRANCH_CONDITIONAL, !last));
    };
    std::vector<step> s;
    iter(s, false);
    iter(s, false);
    iter(s, false);
    iter(s, true);
    h.feed(s);
    check(has_trace(h.rec, {A, B}), "loop trace truncated at call -> {A,B}");
  }

  // ---- coverage: overlapping traces never double-count an ip ---------------
  {
    std::printf("[coverage]  unique-ip coverage counts each ip once\n");
    harness h;
    std::vector<step> s{nb(A)};
    for (int i = 0; i < 4; ++i) {
      s.push_back(nb(B));
      s.push_back(nb(C));
      s.push_back(br(D, B, BRANCH_CONDITIONAL, i < 3));
    }
    h.feed(s);
    check(h.stats.rec_unique_ips_covered <= h.stats.rec_unique_ips_seen, "covered <= seen");
    check(h.stats.rec_unique_ips_covered == 3, "covered {B,C,D} once each");
  }

  std::printf("\n%s (%d failure%s)\n", failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED", failures, failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}

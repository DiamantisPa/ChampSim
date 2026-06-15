#ifndef TRACE_SEGMENTER_H
#define TRACE_SEGMENTER_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core_stats.h"
#include "instruction.h"

// Prometheus-style trace segmentation, observing the post-merge u-op stream
// (the program-ordered stream entering DISPATCH_BUFFER).
//
// Mechanism (per the segmentation-algorithm deck):
//  * A circular buffer holds the last N u-ops.
//  * A taken BACKWARD branch (not a call/return) triggers a loop trace:
//      collect [branch target ... branch].
//  * A taken RETURN triggers a function trace:
//      collect [instruction after the matching call ... return]
//      (call/return matching by nesting depth).
//  * Entangled traces: if the loop window contains an inner taken backward
//    branch whose target lies BEFORE the window start, truncate the trace to
//    begin right after that branch (deck pages 10-18).
//  * Bad layout: if the trigger is unconditional, the trace is entangled by an
//    unconditional inner backward branch, and the truncated trace still holds
//    a taken forward branch (no fall-through path to its target), the trace is
//    dropped entirely (deck pages 35-47).
//  * Function call inside a loop trace: an unmatched call truncates the trace,
//    keeping only [start ... call] (deck pages 19-25).
//  * Nested traces: the outer trace is captured as-is; its inner traces were
//    already collected by their own (earlier) triggers (deck pages 26-33).
//  * Dedup: a trace whose content already exists only bumps an occurrence
//    counter (deck page 30).
//
// Coverage statistics are set-based, so an instruction address is never
// counted twice no matter how many traces contain it.
//
// The trace table is unbounded (idealized storage) -- this models the
// trace-CREATION mechanism and measures achievable code coverage; replacement
// and the u-op-cache fill path are intentionally out of scope here.
class trace_segmenter
{
public:
  static constexpr std::size_t RING_CAPACITY = 512; // last-N u-ops window
  static constexpr std::size_t MIN_TRACE_UOPS = 2;  // shorter traces are noise

  struct trace {
    uint32_t id = 0;
    uint64_t hash = 0;
    uint64_t entry = 0;
    bool is_function = false;
    std::vector<uint64_t> ips; // flat u-op (ip) sequence of the trace
    uint64_t occurrences = 1;
  };

  void push(const ooo_model_instr& in, cpu_stats& stats)
  {
    push_raw(in.ip.to<uint64_t>(), in.branch_target.to<uint64_t>(), in.branch, in.is_branch, static_cast<bool>(in.branch_taken), stats);
  }

  // raw-field variant (also used by the self-test)
  void push_raw(uint64_t ip, uint64_t target, uint8_t type, bool is_branch, bool taken, cpu_stats& stats)
  {
    // dynamic-stream coverage: is this u-op already covered by a trace?
    ++stats.seg_dynamic_uops;
    if (covered_ips.count(ip) > 0) {
      ++stats.seg_dynamic_uops_covered;
    }
    if (seen_ips.insert(ip).second) {
      ++stats.seg_unique_ips_seen;
    }

    ring.push_back({ip, target, type, is_branch, taken});
    if (ring.size() > RING_CAPACITY) {
      ring.pop_front();
    }

    const rec& r = ring.back();
    if (r.is_branch && r.taken) {
      if (r.type == BRANCH_RETURN) {
        build_function_trace(stats);
      } else if (!is_call(r.type) && r.target < r.ip) {
        build_loop_trace(stats);
      }
    }
  }

  [[nodiscard]] const std::vector<trace>& get_traces() const { return traces; }

private:
  struct rec {
    uint64_t ip = 0;
    uint64_t target = 0;
    uint8_t type = BRANCH_OTHER;
    bool is_branch = false;
    bool taken = false;
  };

  static bool is_call(uint8_t t) { return t == BRANCH_DIRECT_CALL || t == BRANCH_INDIRECT_CALL; }
  static bool is_unconditional(uint8_t t) { return t == BRANCH_DIRECT_JUMP || t == BRANCH_INDIRECT; }
  static bool is_backward(const rec& r) { return r.is_branch && r.taken && !is_call(r.type) && r.type != BRANCH_RETURN && r.target < r.ip; }
  static bool is_taken_forward(const rec& r) { return r.is_branch && r.taken && !is_call(r.type) && r.type != BRANCH_RETURN && r.target > r.ip; }

  // loop trace: trigger is the taken backward branch at the back of the ring
  void build_loop_trace(cpu_stats& stats)
  {
    const std::size_t j = ring.size() - 1;
    const rec trig = ring[j];

    // youngest occurrence of the branch target in the buffer = window start
    std::size_t t_idx = SIZE_MAX;
    for (std::size_t k = j; k-- > 0;) {
      if (ring[k].ip == trig.target) {
        t_idx = k;
        break;
      }
    }
    if (t_idx == SIZE_MAX) { // loop body longer than the buffer
      ++stats.seg_traces_dropped_overflow;
      return;
    }

    // entangled: youngest inner taken backward branch targeting BEFORE the
    // window start -> trace begins right after it (deck pages 10-18)
    std::size_t start = t_idx;
    bool entangled = false;
    bool entangling_uncond = false;
    for (std::size_t k = j; k-- > t_idx;) {
      if (is_backward(ring[k]) && ring[k].target < trig.target) {
        entangled = true;
        entangling_uncond = is_unconditional(ring[k].type);
        start = k + 1;
        break;
      }
    }
    if (entangled) {
      ++stats.seg_traces_entangled;
    }

    // bad layout: unconditional trigger + entangled by an unconditional branch
    // (no fall-through past it) + truncated trace still holds a taken forward
    // branch -> not a valid unique path, drop (deck pages 35-47)
    if (entangled && is_unconditional(trig.type) && entangling_uncond) {
      for (std::size_t k = start; k < j; ++k) {
        if (is_taken_forward(ring[k])) {
          ++stats.seg_traces_dropped_bad_layout;
          return;
        }
      }
    }

    // function call inside the trace: an unmatched call truncates the trace,
    // keeping only [start ... call] (deck pages 19-25)
    std::size_t end = j;
    {
      std::vector<std::size_t> open_calls;
      for (std::size_t k = start; k <= j; ++k) {
        const rec& c = ring[k];
        if (c.is_branch && c.taken && is_call(c.type)) {
          open_calls.push_back(k);
        } else if (c.is_branch && c.taken && c.type == BRANCH_RETURN && !open_calls.empty()) {
          open_calls.pop_back();
        }
      }
      if (!open_calls.empty()) {
        end = open_calls.front(); // oldest unmatched call
      }
    }

    if (end < start || (end - start + 1) < MIN_TRACE_UOPS) {
      ++stats.seg_traces_dropped_short;
      return;
    }
    finalize(start, end, false, stats);
  }

  // function trace: trigger is the taken return at the back of the ring
  void build_function_trace(cpu_stats& stats)
  {
    const std::size_t j = ring.size() - 1;

    // walk back to the matching call (skip nested call/return pairs)
    int depth = 0;
    std::size_t c_idx = SIZE_MAX;
    for (std::size_t k = j; k-- > 0;) {
      const rec& c = ring[k];
      if (!(c.is_branch && c.taken)) {
        continue;
      }
      if (c.type == BRANCH_RETURN) {
        ++depth;
      } else if (is_call(c.type)) {
        if (depth == 0) {
          c_idx = k;
          break;
        }
        --depth;
      }
    }
    if (c_idx == SIZE_MAX) { // call not in the buffer anymore
      ++stats.seg_traces_dropped_overflow;
      return;
    }

    const std::size_t start = c_idx + 1; // function entry (call excluded)
    if (start > j || (j - start + 1) < MIN_TRACE_UOPS) {
      ++stats.seg_traces_dropped_short;
      return;
    }
    // NOTE: no entangled truncation for function traces -- the body keeps its
    // inner loops (deck page 25 collects C->D->A->B->E including D->A).
    finalize(start, j, true, stats);
  }

  void finalize(std::size_t start, std::size_t end, bool is_function, cpu_stats& stats)
  {
    // invariant: a stored trace must be a valid committed path -- after a
    // taken branch the next ip is its target; otherwise the next ip is a
    // small forward (fall-through) step. Violations indicate a capture bug.
    for (std::size_t k = start; k < end; ++k) {
      const rec& a = ring[k];
      const uint64_t next = ring[k + 1].ip;
      const bool ok = (a.is_branch && a.taken) ? (next == a.target) : (next > a.ip && next - a.ip <= 16);
      if (!ok) {
        ++stats.seg_invariant_violations;
        break;
      }
    }

    std::vector<uint64_t> w;
    w.reserve(end - start + 1);
    for (std::size_t k = start; k <= end; ++k) {
      w.push_back(ring[k].ip);
    }

    // content hash (FNV-1a over the ip sequence)
    uint64_t h = 1469598103934665603ULL;
    for (auto v : w) {
      h ^= v;
      h *= 1099511628211ULL;
    }

    // dedup: same content -> bump occurrences only (deck page 30)
    if (auto it = hash_to_id.find(h); it != hash_to_id.end()) {
      ++stats.seg_traces_dedup;
      ++traces[it->second].occurrences;
      return;
    }

    trace tr;
    tr.id = static_cast<uint32_t>(traces.size());
    tr.hash = h;
    tr.entry = w.front();
    tr.is_function = is_function;
    tr.ips = std::move(w);

    stats.seg_stored_uops += tr.ips.size();
    if (is_function) {
      ++stats.seg_traces_function;
    } else {
      ++stats.seg_traces_loop;
    }

    // code coverage: set-based, so no instruction is ever counted twice
    for (auto v : tr.ips) {
      if (covered_ips.insert(v).second) {
        ++stats.seg_unique_ips_covered;
      }
    }

    hash_to_id.emplace(h, tr.id);
    traces.push_back(std::move(tr));
  }

  std::deque<rec> ring;
  std::vector<trace> traces;
  std::unordered_map<uint64_t, uint32_t> hash_to_id;
  std::unordered_set<uint64_t> seen_ips;
  std::unordered_set<uint64_t> covered_ips;
};

#endif

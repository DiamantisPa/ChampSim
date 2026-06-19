#ifndef TRACE_RECORDER_H
#define TRACE_RECORDER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core_stats.h"
#include "instruction.h"
#include "trace_coverage.h"

// Prometheus forward "armed-recorder" trace builder -- a hardware-faithful
// alternative to trace_segmenter (inc/trace_segmenter.h).
//
// It holds NO history buffer.  A "previous PC" register detects a backward
// transfer (PC(N) < PC(N-1)); a small CAM (the arm table) is armed with
// {start = PC(N), end = PC(N-1)}; when a later u-op's PC hits an armed `start`
// a free recorder activates and captures FORWARD until it reaches `end`.
//
// Loops and functions share ONE unified pool of N recorders (the knob below):
//   * Loops are armed via the CAM, fire on a start-PC match, and finalize on an
//     end-PC match (taken or not) -- the start u-op is included.
//   * Functions allocate from the same pool eagerly: a taken CALL grabs a
//     recorder for the callee (started at the next u-op, so the call itself is
//     excluded); the matching RETURN finalizes it (LIFO via call_stack).  An
//     outer function captures its inner ones flat because every active recorder
//     receives every u-op (the K-wide broadcast).
// Because both kinds draw from one pool, total working storage is bounded to
// N * MAX_TRACE_UOPS regardless of call-nesting depth.  When the pool is full a
// new arm/call is dropped (counted as overflow).
//
// Number of recorders (N) is a runtime knob: the PROMETHEUS_RECORDERS env var
// (default DEFAULT_NUM_RECORDERS), or the explicit constructor.
//
// Forward entanglement / bad-layout rules (loop recorders):
//   * Entangled: a taken backward branch whose ip != the recorder's `end` AND
//     whose target lies BEFORE `start` truncates the recorder.  A foreign
//     backward branch whose target is >= start is a NESTED inner loop -- the
//     outer recorder keeps recording it flat.
//   * Bad layout: if such a truncating recorder's `end` is UNCONDITIONAL and it
//     saw a taken forward CONDITIONAL branch, the trace is dropped.
//
// Storage of completed traces is unbounded (idealized) -- this models trace
// CREATION and measures achievable coverage; replacement and the u-op-cache
// fill path are out of scope, as in trace_segmenter.
class trace_recorder
{
public:
  static constexpr std::size_t DEFAULT_NUM_RECORDERS = 1; // pool size when unset
  static constexpr std::size_t ARM_TABLE_SIZE = 8;        // CAM entries
  static constexpr std::size_t MAX_TRACE_UOPS = 512;      // per-recorder buffer cap
  static constexpr std::size_t MIN_TRACE_UOPS = 2;        // shorter traces are noise

  struct trace {
    uint32_t id = 0;
    uint64_t hash = 0;
    uint64_t entry = 0;
    bool is_function = false;
    std::vector<uint64_t> ips;
    uint64_t occurrences = 1;
  };

  trace_recorder() : trace_recorder(env_num_recorders()) {}
  explicit trace_recorder(std::size_t num_recorders) : pool(num_recorders < 1 ? 1 : num_recorders) {}

  [[nodiscard]] std::size_t num_recorders() const { return pool.size(); }

  void push(const ooo_model_instr& in, cpu_stats& stats)
  {
    push_raw(in.ip.to<uint64_t>(), in.branch_target.to<uint64_t>(), in.branch, in.is_branch, static_cast<bool>(in.branch_taken), stats);
  }

  // raw-field variant (also used by the self-test)
  void push_raw(uint64_t ip, uint64_t target, uint8_t type, bool is_branch, bool taken, cpu_stats& stats)
  {
    // dynamic-stream coverage + per-IP attribution (see inc/trace_coverage.h)
    cov.observe(ip, stats.rec_dynamic_uops, stats.rec_dynamic_uops_covered, stats.rec_unique_ips_seen);

    const rec cur{ip, target, type, is_branch, taken};

    // 1. fire loop recorders BEFORE append, so a fired recorder captures the
    //    start u-op but a target does not fire on the u-op that armed it.
    fire_loop_recorders(cur, stats);

    // 2. append the u-op to every active recorder (K-wide broadcast).
    append(cur, stats);

    // 3. end-match: a loop recorder whose `end` PC is this u-op finalizes here
    //    (taken or not -- reaching the end PC means one full body was recorded).
    check_end_match(cur, stats);

    // 4. control handling on the current u-op.
    if (is_branch && taken) {
      if (is_call(type)) {
        // function: grab a recorder for the callee (created AFTER append, so the
        // call u-op is excluded); track the call for unmatched-call truncation.
        const std::size_t slot = allocate_slot();
        if (slot != SIZE_MAX) {
          pool[slot] = recorder{};
          pool[slot].active = true;
          pool[slot].is_function = true;
          call_stack.push_back({true, slot});
        } else {
          ++stats.rec_traces_dropped_overflow; // pool full
          call_stack.push_back({false, 0});
        }
        for (auto& r : pool) {
          if (r.active && !r.dropped && !r.is_function) {
            r.open_calls.push_back(r.buf.size() - 1); // index of this call in the loop buffer
          }
        }
      } else if (type == BRANCH_RETURN) {
        // finalize the innermost function recorder ...
        if (!call_stack.empty()) {
          const call_frame cf = call_stack.back();
          call_stack.pop_back();
          if (cf.has_rec) {
            if (!pool[cf.slot].dropped) {
              finalize_function(pool[cf.slot], stats);
            }
            pool[cf.slot] = recorder{}; // free the slot
          }
        }
        // ... and close the innermost matched call for loop recorders.
        for (auto& r : pool) {
          if (r.active && !r.dropped && !r.is_function && !r.open_calls.empty()) {
            r.open_calls.pop_back();
          }
        }
      } else if (target < ip) {
        entangle(cur, stats);
      }
    }

    // 5. arm: PC(N) < PC(N-1) means the PREVIOUS u-op was a taken backward
    //    branch; arm {start = this ip, end = previous ip}.
    if (have_prev && ip < prev.ip && prev.is_branch && prev.taken && !is_call(prev.type) && prev.type != BRANCH_RETURN) {
      arm(ip, prev.ip, is_unconditional(prev.type));
    }

    prev = cur;
    have_prev = true;
  }

  [[nodiscard]] const std::vector<trace>& get_traces() const { return traces; }

  // coverage-loss attribution (see inc/trace_coverage.h); call once at end of run
  void finalize_coverage(cpu_stats& stats) const
  {
    cov.finalize(stats.rec_dyn_covered_final, stats.rec_dyn_lost_no_trigger, stats.rec_dyn_lost_overflow, stats.rec_dyn_lost_bad_layout, stats.rec_dyn_lost_short);
  }
  [[nodiscard]] bool covers(uint64_t ip) const { return cov.covers(ip); }
  [[nodiscard]] const std::unordered_map<uint64_t, uint64_t>& dyn_counts() const { return cov.dyn_count; }

private:
  struct rec {
    uint64_t ip = 0;
    uint64_t target = 0;
    uint8_t type = BRANCH_OTHER;
    bool is_branch = false;
    bool taken = false;
  };

  struct arm_entry {
    bool valid = false;
    uint64_t start = 0;
    uint64_t end = 0;
    bool end_uncond = false;
  };

  // one unified recorder, used as a loop OR a function recorder
  struct recorder {
    bool active = false;
    bool is_function = false;
    bool dropped = false; // overflowed past MAX_TRACE_UOPS (slot held until return)
    uint64_t start = 0;   // loop only
    uint64_t end = 0;     // loop only
    bool end_uncond = false;
    bool saw_taken_fwd_cond = false;
    std::vector<rec> buf;
    std::vector<std::size_t> open_calls; // loop only: unmatched-call truncation
  };

  struct call_frame {
    bool has_rec = false; // whether a recorder slot was allocated for this call
    std::size_t slot = 0;
  };

  static std::size_t env_num_recorders()
  {
    if (const char* e = std::getenv("PROMETHEUS_RECORDERS"); e != nullptr) {
      if (const long v = std::atol(e); v > 0) {
        return static_cast<std::size_t>(v);
      }
    }
    return DEFAULT_NUM_RECORDERS;
  }

  static bool is_call(uint8_t t) { return t == BRANCH_DIRECT_CALL || t == BRANCH_INDIRECT_CALL; }
  static bool is_unconditional(uint8_t t) { return t == BRANCH_DIRECT_JUMP || t == BRANCH_INDIRECT; }
  static bool is_taken_fwd_cond(const rec& r) { return r.is_branch && r.taken && r.type == BRANCH_CONDITIONAL && r.target > r.ip; }

  void mark_buf(const std::vector<rec>& buf, std::size_t begin, std::size_t end, coverage_meter::reason reason)
  {
    for (std::size_t k = begin; k < end && k < buf.size(); ++k) {
      cov.mark(buf[k].ip, reason);
    }
  }

  std::size_t allocate_slot()
  {
    for (std::size_t i = 0; i < pool.size(); ++i) {
      if (!pool[i].active) {
        return i;
      }
    }
    return SIZE_MAX;
  }

  void arm(uint64_t start, uint64_t end, bool end_uncond)
  {
    for (auto& a : arm_table) { // dedup
      if (a.valid && a.start == start && a.end == end) {
        return;
      }
    }
    for (auto& a : arm_table) { // free slot
      if (!a.valid) {
        a = {true, start, end, end_uncond};
        return;
      }
    }
    // CAM full: drop the arm (replacement intentionally out of scope).
  }

  void fire_loop_recorders(const rec& cur, cpu_stats& stats)
  {
    for (auto& a : arm_table) {
      if (!a.valid || a.start != cur.ip) {
        continue;
      }
      bool already = false; // already recording this loop?
      for (auto& r : pool) {
        if (r.active && !r.is_function && r.start == a.start && r.end == a.end) {
          already = true;
          break;
        }
      }
      if (already) {
        continue;
      }
      const std::size_t slot = allocate_slot();
      if (slot == SIZE_MAX) {
        ++stats.rec_traces_dropped_overflow; // pool full
        continue;
      }
      pool[slot] = recorder{};
      pool[slot].active = true;
      pool[slot].start = a.start;
      pool[slot].end = a.end;
      pool[slot].end_uncond = a.end_uncond;
    }
  }

  void append(const rec& cur, cpu_stats& stats)
  {
    for (auto& r : pool) {
      if (!r.active || r.dropped) {
        continue;
      }
      r.buf.push_back(cur);
      if (!r.is_function && is_taken_fwd_cond(cur)) {
        r.saw_taken_fwd_cond = true;
      }
      if (r.buf.size() > MAX_TRACE_UOPS) {
        ++stats.rec_traces_dropped_overflow;
        mark_buf(r.buf, 0, r.buf.size(), coverage_meter::OVERFLOW);
        if (r.is_function) {
          r.dropped = true; // hold the slot until the matching return pops it
          r.buf.clear();
        } else {
          r = recorder{}; // free the loop slot immediately
        }
      }
    }
  }

  // a loop recorder whose `end` PC is this u-op finalizes
  void check_end_match(const rec& cur, cpu_stats& stats)
  {
    for (auto& r : pool) {
      if (r.active && !r.dropped && !r.is_function && cur.ip == r.end) {
        finalize_loop(r, stats);
      }
    }
  }

  // a taken backward branch that is not a recorder's end; if it leaves a loop
  // recorder's region backward (target < start) it truncates that recorder
  void entangle(const rec& cur, cpu_stats& stats)
  {
    for (auto& r : pool) {
      if (!r.active || r.dropped || r.is_function || cur.target >= r.start) {
        continue; // target >= start is a NESTED inner loop -> keep recording
      }
      ++stats.rec_traces_entangled;
      if (r.end_uncond && r.saw_taken_fwd_cond) {
        ++stats.rec_traces_dropped_bad_layout; // no valid unique fall-through path -> drop
        mark_buf(r.buf, 0, r.buf.size(), coverage_meter::BAD_LAYOUT);
        r = recorder{};
      } else {
        finalize_loop(r, stats);
      }
    }
  }

  void finalize_loop(recorder& r, cpu_stats& stats)
  {
    std::size_t end = r.buf.size();
    if (!r.open_calls.empty()) {
      end = r.open_calls.front() + 1; // keep [start ... call] inclusive, drop the rest
    }
    finalize(r.buf, 0, end, false, stats, stats.rec_traces_loop);
    r = recorder{};
  }

  void finalize_function(recorder& r, cpu_stats& stats) { finalize(r.buf, 0, r.buf.size(), true, stats, stats.rec_traces_function); }

  // store buf[begin, end) as a trace (shared loop/function path)
  void finalize(const std::vector<rec>& buf, std::size_t begin, std::size_t end, bool is_function, cpu_stats& stats, uint64_t& kind_counter)
  {
    if (end <= begin || (end - begin) < MIN_TRACE_UOPS) {
      ++stats.rec_traces_dropped_short;
      mark_buf(buf, begin, end, coverage_meter::SHORT);
      return;
    }
    if ((end - begin) > MAX_TRACE_UOPS) {
      ++stats.rec_traces_dropped_overflow;
      mark_buf(buf, begin, end, coverage_meter::OVERFLOW);
      return;
    }

    // invariant: a stored trace must be a valid committed path -- after a taken
    // branch the next ip is its target; otherwise the next ip is a small forward
    // step, OR the SAME ip again (x86 REP-string ops repeat one ip).
    for (std::size_t k = begin; k + 1 < end; ++k) {
      const rec& a = buf[k];
      const uint64_t next = buf[k + 1].ip;
      const bool ok = (a.is_branch && a.taken) ? (next == a.target) : (next >= a.ip && next - a.ip <= 16);
      if (!ok) {
        ++stats.rec_invariant_violations;
        break;
      }
    }

    std::vector<uint64_t> w;
    w.reserve(end - begin);
    for (std::size_t k = begin; k < end; ++k) {
      w.push_back(buf[k].ip);
    }

    uint64_t h = 1469598103934665603ULL; // FNV-1a
    for (auto v : w) {
      h ^= v;
      h *= 1099511628211ULL;
    }

    if (auto it = hash_to_id.find(h); it != hash_to_id.end()) {
      ++stats.rec_traces_dedup;
      ++traces[it->second].occurrences;
      return;
    }

    trace tr;
    tr.id = static_cast<uint32_t>(traces.size());
    tr.hash = h;
    tr.entry = w.front();
    tr.is_function = is_function;
    tr.ips = std::move(w);

    stats.rec_stored_uops += tr.ips.size();
    ++kind_counter;

    for (auto v : tr.ips) {
      cov.cover(v, stats.rec_unique_ips_covered);
    }

    hash_to_id.emplace(h, tr.id);
    traces.push_back(std::move(tr));
  }

  std::vector<recorder> pool;          // unified loop + function recorders (size N)
  std::array<arm_entry, ARM_TABLE_SIZE> arm_table{};
  std::vector<call_frame> call_stack;  // open calls, for LIFO return matching

  rec prev{};
  bool have_prev = false;

  std::vector<trace> traces;
  std::unordered_map<uint64_t, uint32_t> hash_to_id;
  coverage_meter cov;
};

#endif

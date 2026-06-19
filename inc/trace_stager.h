#ifndef TRACE_STAGER_H
#define TRACE_STAGER_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core_stats.h"
#include "instruction.h"
#include "trace_coverage.h"

// Prometheus "staging-buffer" trace builder -- a hardware-realizable form of the
// backward segmenter (inc/trace_segmenter.h): a SHORT ring (M = u-op-queue depth)
// plus an O(1) target lookup, so every taken backward branch / return emits a
// trace from the recent window immediately (one per trigger, like the oracle).
//
// HARDWARE-FAITHFUL METADATA SPLIT.  The ring models the u-op queue, whose
// entries carry only u-op-intrinsic decode info -- {ip, type, is_branch, taken}
// plus a seq tag -- NOT a resolved branch target (that is execute-resolved).  The
// branch targets needed for the entanglement / bad-layout / invariant analysis
// live in a separate, bounded BRANCH FIFO (the fetch-side branch history) keyed
// by seq.  The PC->seq position table is likewise a bounded set-associative CAM,
// not an unbounded map.  Both are knob-sized:
//   * PROMETHEUS_STAGING_DEPTH (M)        -- ring / window depth (default 144)
//   * PROMETHEUS_POS_ENTRIES              -- pos CAM entries     (default 256)
//   * PROMETHEUS_BRANCH_FIFO              -- branch FIFO entries (default 64)
// When pos or the branch FIFO are too small, a target / entangling branch can
// fall out -> a loop is dropped (overflow) or not truncated -- the realistic
// finite-structure cost, measurable by sweeping the knobs.
//
// The capture semantics (entanglement / bad-layout / unmatched-call / nested)
// are the segmenter's, just sourced from the branch FIFO instead of the ring.
class trace_stager
{
public:
  static constexpr std::size_t DEFAULT_DEPTH = 144;       // u-op-queue-sized window
  static constexpr std::size_t DEFAULT_POS_ENTRIES = 256; // PC->seq CAM
  static constexpr std::size_t DEFAULT_BRANCH_FIFO = 64;  // recent-branch history
  static constexpr std::size_t MIN_TRACE_UOPS = 2;

  struct trace {
    uint32_t id = 0;
    uint64_t hash = 0;
    uint64_t entry = 0;
    bool is_function = false;
    std::vector<uint64_t> ips;
    uint64_t occurrences = 1;
  };

  trace_stager()
      : trace_stager(env_or("PROMETHEUS_STAGING_DEPTH", DEFAULT_DEPTH), env_or("PROMETHEUS_POS_ENTRIES", DEFAULT_POS_ENTRIES),
                     env_or("PROMETHEUS_BRANCH_FIFO", DEFAULT_BRANCH_FIFO))
  {
  }
  trace_stager(std::size_t depth, std::size_t pos_entries, std::size_t branch_fifo)
      : M(std::max(MIN_TRACE_UOPS, depth)), BRANCH_FIFO(std::max<std::size_t>(1, branch_fifo)), pos(std::max<std::size_t>(1, pos_entries))
  {
  }

  [[nodiscard]] std::size_t window_depth() const { return M; }
  [[nodiscard]] std::size_t pos_entries() const { return pos.capacity(); }
  [[nodiscard]] std::size_t branch_fifo_entries() const { return BRANCH_FIFO; }

  void push(const ooo_model_instr& in, cpu_stats& stats)
  {
    push_raw(in.ip.to<uint64_t>(), in.branch_target.to<uint64_t>(), in.branch, in.is_branch, static_cast<bool>(in.branch_taken), stats);
  }

  // raw-field variant (also used by the self-test)
  void push_raw(uint64_t ip, uint64_t target, uint8_t type, bool is_branch, bool taken, cpu_stats& stats)
  {
    cov.observe(ip, stats.stg_dynamic_uops, stats.stg_dynamic_uops_covered, stats.stg_unique_ips_seen);

    const uint64_t s = seq_counter++;
    ring.push_back({s, ip, type, is_branch, taken}); // u-op-queue metadata only (no target)
    pos.put(ip, s);
    if (ring.size() > M) {
      ring.pop_front();
    }
    // branch FIFO: targets/types of recent branches (fetch-side), keyed by seq
    if (is_branch) {
      btab.push_back({s, ip, target, type, taken});
      btgt[s] = target;
      if (btab.size() > BRANCH_FIFO) {
        btgt.erase(btab.front().seq);
        btab.pop_front();
      }
    }
    // prune calls that have fallen out of the window
    while (!call_stack.empty() && call_stack.front() < ring.front().seq) {
      call_stack.pop_front();
    }

    if (is_branch && taken) {
      if (is_call(type)) {
        call_stack.push_back(s);
      } else if (type == BRANCH_RETURN) {
        build_function_trace(stats);
      } else if (target < ip) {
        build_loop_trace(ip, target, type, s, stats);
      }
    }
  }

  [[nodiscard]] const std::vector<trace>& get_traces() const { return traces; }

  // coverage-loss attribution (see inc/trace_coverage.h); call once at end of run
  void finalize_coverage(cpu_stats& stats) const
  {
    cov.finalize(stats.stg_dyn_covered_final, stats.stg_dyn_lost_no_trigger, stats.stg_dyn_lost_overflow, stats.stg_dyn_lost_bad_layout, stats.stg_dyn_lost_short);
  }
  [[nodiscard]] bool covers(uint64_t ip) const { return cov.covers(ip); }
  [[nodiscard]] const std::unordered_map<uint64_t, uint64_t>& dyn_counts() const { return cov.dyn_count; }

private:
  struct rec {
    uint64_t seq = 0;
    uint64_t ip = 0;
    uint8_t type = BRANCH_OTHER;
    bool is_branch = false;
    bool taken = false;
  };

  struct bent {
    uint64_t seq = 0;
    uint64_t ip = 0;
    uint64_t target = 0;
    uint8_t type = BRANCH_OTHER;
    bool taken = false;
  };

  // bounded set-associative PC -> latest-seq table (the pos CAM)
  class pos_cam
  {
  public:
    explicit pos_cam(std::size_t entries) : ways(4), sets(std::max<std::size_t>(1, entries / 4)), t(sets * ways) {}
    [[nodiscard]] std::size_t capacity() const { return sets * ways; }
    void put(uint64_t ip, uint64_t seq)
    {
      const std::size_t base = set_of(ip) * ways;
      for (std::size_t w = 0; w < ways; ++w) {
        if (t[base + w].valid && t[base + w].tag == ip) {
          t[base + w].seq = seq;
          t[base + w].lru = ++tick;
          return;
        }
      }
      std::size_t victim = base;
      for (std::size_t w = 0; w < ways; ++w) {
        if (!t[base + w].valid) {
          victim = base + w;
          break;
        }
        if (t[base + w].lru < t[victim].lru) {
          victim = base + w;
        }
      }
      t[victim] = {true, ip, seq, ++tick};
    }
    bool get(uint64_t ip, uint64_t& out)
    {
      const std::size_t base = set_of(ip) * ways;
      for (std::size_t w = 0; w < ways; ++w) {
        if (t[base + w].valid && t[base + w].tag == ip) {
          t[base + w].lru = ++tick;
          out = t[base + w].seq;
          return true;
        }
      }
      return false;
    }

  private:
    struct ent {
      bool valid = false;
      uint64_t tag = 0;
      uint64_t seq = 0;
      uint64_t lru = 0;
    };
    std::size_t set_of(uint64_t ip) const { return (ip >> 2) % sets; }
    std::size_t ways, sets;
    std::vector<ent> t;
    uint64_t tick = 0;
  };

  static std::size_t env_or(const char* name, std::size_t def)
  {
    if (const char* e = std::getenv(name); e != nullptr) {
      if (const long v = std::atol(e); v > 0) {
        return static_cast<std::size_t>(v);
      }
    }
    return def;
  }

  static bool is_call(uint8_t t) { return t == BRANCH_DIRECT_CALL || t == BRANCH_INDIRECT_CALL; }
  static bool is_unconditional(uint8_t t) { return t == BRANCH_DIRECT_JUMP || t == BRANCH_INDIRECT; }
  static bool bent_backward(const bent& b) { return b.taken && !is_call(b.type) && b.type != BRANCH_RETURN && b.target < b.ip; }
  static bool bent_taken_forward(const bent& b) { return b.taken && !is_call(b.type) && b.type != BRANCH_RETURN && b.target > b.ip; }

  std::size_t index_of(uint64_t s) const
  {
    if (ring.empty() || s < ring.front().seq || s > ring.back().seq) {
      return SIZE_MAX;
    }
    return static_cast<std::size_t>(s - ring.front().seq);
  }

  void mark_range(std::size_t lo, std::size_t hi, coverage_meter::reason reason)
  {
    for (std::size_t k = lo; k <= hi && k < ring.size(); ++k) {
      cov.mark(ring[k].ip, reason);
    }
  }

  void build_loop_trace([[maybe_unused]] uint64_t trig_ip, uint64_t trig_target, uint8_t trig_type, uint64_t trig_seq, cpu_stats& stats)
  {
    const std::size_t j = ring.size() - 1;

    // window start = youngest occurrence of the target -- O(1) via the pos CAM
    uint64_t target_seq = 0;
    if (!pos.get(trig_target, target_seq)) { // target not in the CAM
      ++stats.stg_traces_dropped_overflow;
      mark_range(0, j, coverage_meter::OVERFLOW);
      return;
    }
    const std::size_t t_idx = index_of(target_seq);
    if (t_idx == SIZE_MAX || t_idx >= j) { // target fell out of the ring window
      ++stats.stg_traces_dropped_overflow;
      mark_range(0, j, coverage_meter::OVERFLOW);
      return;
    }

    // entangled: youngest inner backward branch (in the branch FIFO) targeting
    // BEFORE the window start -> trace begins right after it (deck 10-18)
    std::size_t start = t_idx;
    bool entangled = false;
    bool entangling_uncond = false;
    for (auto it = btab.rbegin(); it != btab.rend(); ++it) {
      if (it->seq >= trig_seq) {
        continue;
      }
      if (it->seq <= target_seq) {
        break;
      }
      if (bent_backward(*it) && it->target < trig_target) {
        entangled = true;
        entangling_uncond = is_unconditional(it->type);
        start = index_of(it->seq + 1);
        break;
      }
    }
    if (entangled) {
      ++stats.stg_traces_entangled;
    }

    const uint64_t start_seq = ring[start].seq;

    // bad layout: uncond trigger + uncond entangling branch + a taken forward
    // branch in the truncated window -> drop (deck 35-47)
    if (entangled && is_unconditional(trig_type) && entangling_uncond) {
      for (const auto& b : btab) {
        if (b.seq < start_seq || b.seq >= trig_seq) {
          continue;
        }
        if (bent_taken_forward(b)) {
          ++stats.stg_traces_dropped_bad_layout;
          mark_range(start, j, coverage_meter::BAD_LAYOUT);
          return;
        }
      }
    }

    // unmatched call inside the trace truncates it to [start .. call] (deck 19-25)
    std::size_t end = j;
    {
      std::vector<uint64_t> open_calls;
      for (const auto& b : btab) {
        if (b.seq < start_seq || b.seq > trig_seq) {
          continue;
        }
        if (b.taken && is_call(b.type)) {
          open_calls.push_back(b.seq);
        } else if (b.taken && b.type == BRANCH_RETURN && !open_calls.empty()) {
          open_calls.pop_back();
        }
      }
      if (!open_calls.empty()) {
        end = index_of(open_calls.front());
      }
    }

    if (end == SIZE_MAX || end < start || (end - start + 1) < MIN_TRACE_UOPS) {
      ++stats.stg_traces_dropped_short;
      mark_range(start, end, coverage_meter::SHORT);
      return;
    }
    finalize(start, end, false, stats);
  }

  void build_function_trace(cpu_stats& stats)
  {
    const std::size_t j = ring.size() - 1;
    if (call_stack.empty()) {
      ++stats.stg_traces_dropped_overflow;
      mark_range(0, j, coverage_meter::OVERFLOW);
      return;
    }
    const uint64_t call_seq = call_stack.back();
    call_stack.pop_back();
    const std::size_t c_idx = index_of(call_seq);
    if (c_idx == SIZE_MAX) {
      ++stats.stg_traces_dropped_overflow;
      mark_range(0, j, coverage_meter::OVERFLOW);
      return;
    }
    const std::size_t start = c_idx + 1;
    if (start > j || (j - start + 1) < MIN_TRACE_UOPS) {
      ++stats.stg_traces_dropped_short;
      mark_range(start, j, coverage_meter::SHORT);
      return;
    }
    finalize(start, j, true, stats);
  }

  void finalize(std::size_t start, std::size_t end, bool is_function, cpu_stats& stats)
  {
    // invariant: taken branch -> next ip is its target (looked up by seq in the
    // branch FIFO; skip if it has aged out); else fall-through / same ip (REP).
    for (std::size_t k = start; k < end; ++k) {
      const rec& a = ring[k];
      const uint64_t next = ring[k + 1].ip;
      bool ok = true;
      if (a.is_branch && a.taken) {
        if (auto it = btgt.find(a.seq); it != btgt.end()) {
          ok = (next == it->second);
        }
      } else {
        ok = (next >= a.ip && next - a.ip <= 16);
      }
      if (!ok) {
        ++stats.stg_invariant_violations;
        break;
      }
    }

    std::vector<uint64_t> w;
    w.reserve(end - start + 1);
    for (std::size_t k = start; k <= end; ++k) {
      w.push_back(ring[k].ip);
    }

    uint64_t h = 1469598103934665603ULL; // FNV-1a
    for (auto v : w) {
      h ^= v;
      h *= 1099511628211ULL;
    }

    if (auto hit = hash_to_id.find(h); hit != hash_to_id.end()) {
      ++stats.stg_traces_dedup;
      ++traces[hit->second].occurrences;
      return;
    }

    trace tr;
    tr.id = static_cast<uint32_t>(traces.size());
    tr.hash = h;
    tr.entry = w.front();
    tr.is_function = is_function;
    tr.ips = std::move(w);

    stats.stg_stored_uops += tr.ips.size();
    if (is_function) {
      ++stats.stg_traces_function;
    } else {
      ++stats.stg_traces_loop;
    }

    for (auto v : tr.ips) {
      cov.cover(v, stats.stg_unique_ips_covered);
    }

    hash_to_id.emplace(h, tr.id);
    traces.push_back(std::move(tr));
  }

  const std::size_t M;            // window depth (u-op queue size)
  const std::size_t BRANCH_FIFO;  // recent-branch history depth
  uint64_t seq_counter = 0;
  std::deque<rec> ring;           // last M u-op-queue entries (no target)
  pos_cam pos;                    // bounded PC -> latest seq
  std::deque<bent> btab;          // bounded recent-branch FIFO (targets/types)
  std::unordered_map<uint64_t, uint64_t> btgt; // seq -> target (O(1), pruned with btab)
  std::deque<uint64_t> call_stack;             // seqs of open calls (back = innermost)

  std::vector<trace> traces;
  std::unordered_map<uint64_t, uint32_t> hash_to_id;
  coverage_meter cov;
};

#endif

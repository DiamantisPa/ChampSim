#ifndef CORE_STATS_H
#define CORE_STATS_H

#include <cstdint>
#include <string>

#include "event_counter.h"
#include "instruction.h"

struct cpu_stats {
  std::string name;
  long long begin_instrs = 0;
  long long begin_cycles = 0;
  long long end_instrs = 0;
  long long end_cycles = 0;
  uint64_t total_rob_occupancy_at_branch_mispredict = 0;
  uint64_t switch_stalls = 0;    // u-op-cache stream<->build mode-switch stalls
  uint64_t uop_cache_reads = 0;  // u-op-cache lookups (one per instruction checked)
  uint64_t uop_cache_hits = 0;   // u-op-cache lookups that hit
  uint64_t uop_trace_fill_hits = 0;    // u-op-cache misses served by trace-fill (option a)
  uint64_t uop_trace_fill_windows = 0; // windows installed by trace-fill

  // frontend IPC-loss decomposition (real u-op cache vs ideal). Misses split by
  // whether they occur post-misprediction (recovery refill) or on the correct
  // path (steady state); fe_stall_* are dispatch-starvation cycles in BUILD mode
  // (u-op-miss-attributable, branch penalty excluded), same split.
  uint64_t uop_miss_steady = 0;    // u-op-cache misses on the correct path
  uint64_t uop_miss_recovery = 0;  // u-op-cache misses while recovering from a misprediction
  // ... of which the missing IP is covered by a stored trace (the stager): the
  // ceiling for what trace-fill could serve. Subset of the two counters above.
  uint64_t uop_miss_steady_traced = 0;
  uint64_t uop_miss_recovery_traced = 0;
  uint64_t fe_stall_steady = 0;    // build-mode dispatch-starvation cycles, correct path (upper bound)
  uint64_t fe_stall_recovery = 0;  // build-mode dispatch-starvation cycles, post-misprediction (upper bound)
  uint64_t rob_idle_steady = 0;    // build-mode cycles with ROB fully empty, correct path (tight lower bound)
  uint64_t rob_idle_recovery = 0;  // build-mode cycles with ROB fully empty, post-misprediction (tight lower bound)

  // trace segmentation (see inc/trace_segmenter.h)
  uint64_t seg_traces_loop = 0;              // stored traces triggered by backward branches
  uint64_t seg_traces_function = 0;          // stored traces triggered by returns
  uint64_t seg_traces_dedup = 0;             // triggers whose trace already existed
  uint64_t seg_traces_entangled = 0;         // loop traces truncated by an entangling backward branch
  uint64_t seg_traces_dropped_bad_layout = 0;
  uint64_t seg_traces_dropped_overflow = 0;  // target/call fell out of the capture buffer
  uint64_t seg_traces_dropped_short = 0;
  uint64_t seg_stored_uops = 0;              // total u-ops stored across unique traces
  uint64_t seg_unique_ips_seen = 0;          // unique instruction addresses observed
  uint64_t seg_unique_ips_covered = 0;       // unique instruction addresses inside stored traces
  uint64_t seg_dynamic_uops = 0;             // u-ops observed at the post-merge stream
  uint64_t seg_dynamic_uops_covered = 0;     // ... whose ip was covered by a trace at that time
  uint64_t seg_invariant_violations = 0;     // stored windows that were not a valid committed path (should be 0)

  // forward armed-recorder trace builder (see inc/trace_recorder.h) -- the
  // hardware-faithful alternative to the backward segmenter above, run in
  // parallel so the two can be diffed.
  uint64_t rec_traces_loop = 0;
  uint64_t rec_traces_function = 0;
  uint64_t rec_traces_dedup = 0;
  uint64_t rec_traces_entangled = 0;
  uint64_t rec_traces_dropped_bad_layout = 0;
  uint64_t rec_traces_dropped_overflow = 0;
  uint64_t rec_traces_dropped_short = 0;
  uint64_t rec_stored_uops = 0;
  uint64_t rec_unique_ips_seen = 0;
  uint64_t rec_unique_ips_covered = 0;
  uint64_t rec_dynamic_uops = 0;
  uint64_t rec_dynamic_uops_covered = 0;
  uint64_t rec_invariant_violations = 0;

  // staging-buffer trace builder (see inc/trace_stager.h) -- the synthesizable
  // form of the backward segmenter (short ring + pos-lookup), run in parallel.
  uint64_t stg_traces_loop = 0;
  uint64_t stg_traces_function = 0;
  uint64_t stg_traces_dedup = 0;
  uint64_t stg_traces_entangled = 0;
  uint64_t stg_traces_dropped_bad_layout = 0;
  uint64_t stg_traces_dropped_overflow = 0;
  uint64_t stg_traces_dropped_short = 0;
  uint64_t stg_stored_uops = 0;
  uint64_t stg_unique_ips_seen = 0;
  uint64_t stg_unique_ips_covered = 0;
  uint64_t stg_dynamic_uops = 0;
  uint64_t stg_dynamic_uops_covered = 0;
  uint64_t stg_invariant_violations = 0;

  // coverage-loss attribution (see inc/trace_coverage.h), per builder.  For each:
  // covered_final (time-agnostic) + lost_{no_trigger,overflow,bad_layout,short}
  // == total dynamic u-ops; (covered_final - dynamic_uops_covered) is build latency.
  uint64_t seg_dyn_covered_final = 0;
  uint64_t seg_dyn_lost_no_trigger = 0;
  uint64_t seg_dyn_lost_overflow = 0;
  uint64_t seg_dyn_lost_bad_layout = 0;
  uint64_t seg_dyn_lost_short = 0;
  uint64_t rec_dyn_covered_final = 0;
  uint64_t rec_dyn_lost_no_trigger = 0;
  uint64_t rec_dyn_lost_overflow = 0;
  uint64_t rec_dyn_lost_bad_layout = 0;
  uint64_t rec_dyn_lost_short = 0;
  uint64_t stg_dyn_covered_final = 0;
  uint64_t stg_dyn_lost_no_trigger = 0;
  uint64_t stg_dyn_lost_overflow = 0;
  uint64_t stg_dyn_lost_bad_layout = 0;
  uint64_t stg_dyn_lost_short = 0;

  // cross-builder dynamic-weighted coverage diffs (oracle vs the realizable ones)
  uint64_t xcov_seg_not_stg = 0; // dyn u-ops covered by segmenter but not stager
  uint64_t xcov_seg_not_rec = 0; // ... by segmenter but not recorder
  uint64_t xcov_stg_not_rec = 0; // ... by stager but not recorder

  champsim::stats::event_counter<branch_type> total_branch_types = {};
  champsim::stats::event_counter<branch_type> branch_type_misses = {};

  [[nodiscard]] auto instrs() const { return end_instrs - begin_instrs; }
  [[nodiscard]] auto cycles() const { return end_cycles - begin_cycles; }
};

cpu_stats operator-(cpu_stats lhs, cpu_stats rhs);

#endif

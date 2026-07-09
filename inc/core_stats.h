#ifndef CORE_STATS_H
#define CORE_STATS_H

#include <array>
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

  // stall-triggered segmenter (see inc/trace_stall.h): traces recorded across a
  // build-mode stretch that drained the ROB (a costly backend-idle stall).
  uint64_t stall_traces = 0;              // distinct traces committed (passed the occurrence filter)
  uint64_t stall_traces_candidates = 0;   // distinct stretch-start IPs seen (total, before filtering)
  uint64_t stall_dedup = 0;              // costly stretches whose start IP was already captured
  uint64_t stall_gated_l1i = 0;          // costly stretches rejected by the L1I admission gate (no L1I miss observed)
  uint64_t stall_stored_uops = 0;
  uint64_t stall_dynamic_uops = 0;
  uint64_t stall_dynamic_uops_covered = 0;
  uint64_t stall_unique_ips_seen = 0;
  uint64_t stall_unique_ips_covered = 0;
  // Pareto view of stall traces: dynamic weight (occurrences x length) captured by
  // the top-N traces, ranked by occurrence and by dynamic weight.  Cumulative; as a
  // fraction of stall_total_dynweight these say "keep N traces -> capture X%".
  uint64_t stall_total_dynweight = 0;
  uint64_t stall_occ_top16 = 0, stall_occ_top32 = 0, stall_occ_top64 = 0, stall_occ_top128 = 0;
  uint64_t stall_occ_top256 = 0, stall_occ_top512 = 0, stall_occ_top1024 = 0;
  uint64_t stall_cov_top16 = 0, stall_cov_top32 = 0, stall_cov_top64 = 0, stall_cov_top128 = 0;
  uint64_t stall_cov_top256 = 0, stall_cov_top512 = 0, stall_cov_top1024 = 0;
  // per bucket {16,32,64,128,256,512,1024}: avg occurrence and avg length of the
  // top-N traces, ranked by occurrence and by coverage (dyn-weight).
  std::array<double, 7> stall_occ_avgocc{}, stall_occ_avglen{}; // by-occurrence ranking
  std::array<double, 7> stall_cov_avgocc{}, stall_cov_avglen{}; // by-coverage ranking
  // cost-ranked Pareto: traces ranked by the ROB-stall cycles their stretches cost
  // (note_stall() cycles attributed to each stored trace, summed over re-captures).
  // "keep the N costliest traces -> capture X% of the stall cycles / Y% of dyn-weight".
  uint64_t stall_total_cost = 0;              // total ROB-stall cycles attributed to stored traces
  std::array<uint64_t, 7> stall_cost_cum{};   // cum stall-cycles of the top-N by cost
  std::array<uint64_t, 7> stall_cost_cumw{};  // cum dyn-weight of the top-N by cost
  std::array<double, 7> stall_cost_avgocc{}, stall_cost_avglen{}; // by-cost ranking
  // cross-metric: cum stall-cycles captured by the top-N under the OTHER rankings
  // (how well occurrence / coverage retention proxies for cost).
  std::array<uint64_t, 7> stall_occ_cumcost{}, stall_cov_cumcost{};
  // ALT fill mode (metadata trace cache + timed pre-decode walk at branch decode):
  uint64_t alt_triggers = 0;          // walks launched (alternate-path PC hit the trace store)
  uint64_t alt_drops = 0;             // triggers dropped because alt_walk_max walks were in flight
  uint64_t alt_installed_windows = 0; // u-op-cache windows installed by walks
  uint64_t alt_late_misses = 0;       // demand misses whose window was in a walk still in flight (too slow)
  uint64_t alt_useful_hits = 0;       // demand hits on walk-installed windows (walk accuracy numerator)
  uint64_t alt_wait_cycles = 0;       // fetch-stall cycles waiting for a walk-pending window (hit-under-fill)
  uint64_t alt_lines_issued = 0;      // real-L1I mode: cache-line reads the walks issued through the L1I
  uint64_t alt_line_stalls = 0;       // real-L1I mode: walk-cycles stalled waiting for a line to arrive
  uint64_t alt_walk_aborts = 0;       // watchdog: walks aborted after making no progress (lost line response)
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

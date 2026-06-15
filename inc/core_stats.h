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

  champsim::stats::event_counter<branch_type> total_branch_types = {};
  champsim::stats::event_counter<branch_type> branch_type_misses = {};

  [[nodiscard]] auto instrs() const { return end_instrs - begin_instrs; }
  [[nodiscard]] auto cycles() const { return end_cycles - begin_cycles; }
};

cpu_stats operator-(cpu_stats lhs, cpu_stats rhs);

#endif

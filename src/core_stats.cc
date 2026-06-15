#include "core_stats.h"

cpu_stats operator-(cpu_stats lhs, cpu_stats rhs)
{
  lhs.begin_instrs -= rhs.begin_instrs;
  lhs.begin_cycles -= rhs.begin_cycles;
  lhs.end_instrs -= rhs.end_instrs;
  lhs.end_cycles -= rhs.end_cycles;
  lhs.total_rob_occupancy_at_branch_mispredict -= rhs.total_rob_occupancy_at_branch_mispredict;
  lhs.switch_stalls -= rhs.switch_stalls;
  lhs.uop_cache_reads -= rhs.uop_cache_reads;
  lhs.uop_cache_hits -= rhs.uop_cache_hits;

  lhs.seg_traces_loop -= rhs.seg_traces_loop;
  lhs.seg_traces_function -= rhs.seg_traces_function;
  lhs.seg_traces_dedup -= rhs.seg_traces_dedup;
  lhs.seg_traces_entangled -= rhs.seg_traces_entangled;
  lhs.seg_traces_dropped_bad_layout -= rhs.seg_traces_dropped_bad_layout;
  lhs.seg_traces_dropped_overflow -= rhs.seg_traces_dropped_overflow;
  lhs.seg_traces_dropped_short -= rhs.seg_traces_dropped_short;
  lhs.seg_stored_uops -= rhs.seg_stored_uops;
  lhs.seg_unique_ips_seen -= rhs.seg_unique_ips_seen;
  lhs.seg_unique_ips_covered -= rhs.seg_unique_ips_covered;
  lhs.seg_dynamic_uops -= rhs.seg_dynamic_uops;
  lhs.seg_dynamic_uops_covered -= rhs.seg_dynamic_uops_covered;
  lhs.seg_invariant_violations -= rhs.seg_invariant_violations;

  lhs.total_branch_types -= rhs.total_branch_types;
  lhs.branch_type_misses -= rhs.branch_type_misses;

  return lhs;
}

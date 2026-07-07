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
  lhs.uop_trace_fill_hits -= rhs.uop_trace_fill_hits;
  lhs.uop_trace_fill_windows -= rhs.uop_trace_fill_windows;
  lhs.uop_miss_steady -= rhs.uop_miss_steady;
  lhs.uop_miss_recovery -= rhs.uop_miss_recovery;
  lhs.uop_miss_steady_traced -= rhs.uop_miss_steady_traced;
  lhs.uop_miss_recovery_traced -= rhs.uop_miss_recovery_traced;
  lhs.stall_traces -= rhs.stall_traces;
  lhs.stall_traces_candidates -= rhs.stall_traces_candidates;
  lhs.stall_dedup -= rhs.stall_dedup;
  lhs.stall_stored_uops -= rhs.stall_stored_uops;
  lhs.stall_dynamic_uops -= rhs.stall_dynamic_uops;
  lhs.stall_dynamic_uops_covered -= rhs.stall_dynamic_uops_covered;
  lhs.stall_unique_ips_seen -= rhs.stall_unique_ips_seen;
  lhs.stall_unique_ips_covered -= rhs.stall_unique_ips_covered;
  lhs.stall_total_dynweight -= rhs.stall_total_dynweight;
  lhs.stall_occ_top16 -= rhs.stall_occ_top16;
  lhs.stall_occ_top32 -= rhs.stall_occ_top32;
  lhs.stall_occ_top64 -= rhs.stall_occ_top64;
  lhs.stall_occ_top128 -= rhs.stall_occ_top128;
  lhs.stall_occ_top256 -= rhs.stall_occ_top256;
  lhs.stall_occ_top512 -= rhs.stall_occ_top512;
  lhs.stall_occ_top1024 -= rhs.stall_occ_top1024;
  lhs.stall_cov_top16 -= rhs.stall_cov_top16;
  lhs.stall_cov_top32 -= rhs.stall_cov_top32;
  lhs.stall_cov_top64 -= rhs.stall_cov_top64;
  lhs.stall_cov_top128 -= rhs.stall_cov_top128;
  lhs.stall_cov_top256 -= rhs.stall_cov_top256;
  lhs.stall_cov_top512 -= rhs.stall_cov_top512;
  lhs.stall_cov_top1024 -= rhs.stall_cov_top1024;
  for (std::size_t i = 0; i < lhs.stall_occ_avgocc.size(); ++i) {
    lhs.stall_occ_avgocc[i] -= rhs.stall_occ_avgocc[i];
    lhs.stall_occ_avglen[i] -= rhs.stall_occ_avglen[i];
    lhs.stall_cov_avgocc[i] -= rhs.stall_cov_avgocc[i];
    lhs.stall_cov_avglen[i] -= rhs.stall_cov_avglen[i];
    lhs.stall_cost_cum[i] -= rhs.stall_cost_cum[i];
    lhs.stall_cost_cumw[i] -= rhs.stall_cost_cumw[i];
    lhs.stall_cost_avgocc[i] -= rhs.stall_cost_avgocc[i];
    lhs.stall_cost_avglen[i] -= rhs.stall_cost_avglen[i];
    lhs.stall_occ_cumcost[i] -= rhs.stall_occ_cumcost[i];
    lhs.stall_cov_cumcost[i] -= rhs.stall_cov_cumcost[i];
  }
  lhs.stall_total_cost -= rhs.stall_total_cost;
  lhs.alt_triggers -= rhs.alt_triggers;
  lhs.alt_drops -= rhs.alt_drops;
  lhs.alt_installed_windows -= rhs.alt_installed_windows;
  lhs.alt_late_misses -= rhs.alt_late_misses;
  lhs.alt_useful_hits -= rhs.alt_useful_hits;
  lhs.alt_wait_cycles -= rhs.alt_wait_cycles;
  lhs.fe_stall_steady -= rhs.fe_stall_steady;
  lhs.fe_stall_recovery -= rhs.fe_stall_recovery;
  lhs.rob_idle_steady -= rhs.rob_idle_steady;
  lhs.rob_idle_recovery -= rhs.rob_idle_recovery;

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

  lhs.rec_traces_loop -= rhs.rec_traces_loop;
  lhs.rec_traces_function -= rhs.rec_traces_function;
  lhs.rec_traces_dedup -= rhs.rec_traces_dedup;
  lhs.rec_traces_entangled -= rhs.rec_traces_entangled;
  lhs.rec_traces_dropped_bad_layout -= rhs.rec_traces_dropped_bad_layout;
  lhs.rec_traces_dropped_overflow -= rhs.rec_traces_dropped_overflow;
  lhs.rec_traces_dropped_short -= rhs.rec_traces_dropped_short;
  lhs.rec_stored_uops -= rhs.rec_stored_uops;
  lhs.rec_unique_ips_seen -= rhs.rec_unique_ips_seen;
  lhs.rec_unique_ips_covered -= rhs.rec_unique_ips_covered;
  lhs.rec_dynamic_uops -= rhs.rec_dynamic_uops;
  lhs.rec_dynamic_uops_covered -= rhs.rec_dynamic_uops_covered;
  lhs.rec_invariant_violations -= rhs.rec_invariant_violations;

  lhs.stg_traces_loop -= rhs.stg_traces_loop;
  lhs.stg_traces_function -= rhs.stg_traces_function;
  lhs.stg_traces_dedup -= rhs.stg_traces_dedup;
  lhs.stg_traces_entangled -= rhs.stg_traces_entangled;
  lhs.stg_traces_dropped_bad_layout -= rhs.stg_traces_dropped_bad_layout;
  lhs.stg_traces_dropped_overflow -= rhs.stg_traces_dropped_overflow;
  lhs.stg_traces_dropped_short -= rhs.stg_traces_dropped_short;
  lhs.stg_stored_uops -= rhs.stg_stored_uops;
  lhs.stg_unique_ips_seen -= rhs.stg_unique_ips_seen;
  lhs.stg_unique_ips_covered -= rhs.stg_unique_ips_covered;
  lhs.stg_dynamic_uops -= rhs.stg_dynamic_uops;
  lhs.stg_dynamic_uops_covered -= rhs.stg_dynamic_uops_covered;
  lhs.stg_invariant_violations -= rhs.stg_invariant_violations;

  lhs.seg_dyn_covered_final -= rhs.seg_dyn_covered_final;
  lhs.seg_dyn_lost_no_trigger -= rhs.seg_dyn_lost_no_trigger;
  lhs.seg_dyn_lost_overflow -= rhs.seg_dyn_lost_overflow;
  lhs.seg_dyn_lost_bad_layout -= rhs.seg_dyn_lost_bad_layout;
  lhs.seg_dyn_lost_short -= rhs.seg_dyn_lost_short;
  lhs.rec_dyn_covered_final -= rhs.rec_dyn_covered_final;
  lhs.rec_dyn_lost_no_trigger -= rhs.rec_dyn_lost_no_trigger;
  lhs.rec_dyn_lost_overflow -= rhs.rec_dyn_lost_overflow;
  lhs.rec_dyn_lost_bad_layout -= rhs.rec_dyn_lost_bad_layout;
  lhs.rec_dyn_lost_short -= rhs.rec_dyn_lost_short;
  lhs.stg_dyn_covered_final -= rhs.stg_dyn_covered_final;
  lhs.stg_dyn_lost_no_trigger -= rhs.stg_dyn_lost_no_trigger;
  lhs.stg_dyn_lost_overflow -= rhs.stg_dyn_lost_overflow;
  lhs.stg_dyn_lost_bad_layout -= rhs.stg_dyn_lost_bad_layout;
  lhs.stg_dyn_lost_short -= rhs.stg_dyn_lost_short;

  lhs.xcov_seg_not_stg -= rhs.xcov_seg_not_stg;
  lhs.xcov_seg_not_rec -= rhs.xcov_seg_not_rec;
  lhs.xcov_stg_not_rec -= rhs.xcov_stg_not_rec;

  lhs.total_branch_types -= rhs.total_branch_types;
  lhs.branch_type_misses -= rhs.branch_type_misses;

  return lhs;
}

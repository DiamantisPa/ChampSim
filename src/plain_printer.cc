/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include <numeric>
#include <ratio>
#include <string_view> // for string_view
#include <utility>
#include <vector>
#include <fmt/chrono.h>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include "stats_printer.h"

namespace
{
template <typename N, typename D>
auto print_ratio(N num, D denom)
{
  if (denom > 0) {
    return fmt::format("{:.4g}", std::ceil(num) / std::ceil(denom));
  }
  return std::string{"-"};
}
} // namespace

std::vector<std::string> champsim::plain_printer::format(O3_CPU::stats_type stats)
{
  constexpr std::array types{branch_type::BRANCH_DIRECT_JUMP, branch_type::BRANCH_INDIRECT,      branch_type::BRANCH_CONDITIONAL,
                             branch_type::BRANCH_DIRECT_CALL, branch_type::BRANCH_INDIRECT_CALL, branch_type::BRANCH_RETURN};
  auto total_branch = std::ceil(
      std::accumulate(std::begin(types), std::end(types), 0LL, [tbt = stats.total_branch_types](auto acc, auto next) { return acc + tbt.value_or(next, 0); }));
  auto total_mispredictions = std::ceil(
      std::accumulate(std::begin(types), std::end(types), 0LL, [btm = stats.branch_type_misses](auto acc, auto next) { return acc + btm.value_or(next, 0); }));

  std::vector<std::string> lines{};
  lines.push_back(fmt::format("{} cumulative IPC: {} instructions: {} cycles: {}", stats.name, ::print_ratio(stats.instrs(), stats.cycles()), stats.instrs(),
                              stats.cycles()));

  lines.push_back(fmt::format("{} Branch Prediction Accuracy: {}% MPKI: {} Average ROB Occupancy at Mispredict: {}", stats.name,
                              ::print_ratio(100 * (total_branch - total_mispredictions), total_branch),
                              ::print_ratio(std::kilo::num * total_mispredictions, stats.instrs()),
                              ::print_ratio(stats.total_rob_occupancy_at_branch_mispredict, total_mispredictions)));

  lines.push_back(fmt::format("{} uop-cache mode-switch stalls: {} MPKI: {}", stats.name, stats.switch_stalls,
                              ::print_ratio(std::kilo::num * static_cast<long long>(stats.switch_stalls), stats.instrs())));

  lines.push_back(fmt::format("{} uop-cache hit rate: {}% hits: {} reads: {} MPKI: {}", stats.name,
                              ::print_ratio(100 * static_cast<long long>(stats.uop_cache_hits), static_cast<long long>(stats.uop_cache_reads)),
                              stats.uop_cache_hits, stats.uop_cache_reads,
                              ::print_ratio(std::kilo::num * static_cast<long long>(stats.uop_cache_reads - stats.uop_cache_hits), stats.instrs())));

  lines.push_back(fmt::format("{} trace-fill(option-a): hits {} windows-installed {} (of {} uop-cache reads, {} hits total)", stats.name,
                              stats.uop_trace_fill_hits, stats.uop_trace_fill_windows, stats.uop_cache_reads, stats.uop_cache_hits));

  // frontend IPC-loss decomposition: u-op misses and build-mode dispatch-starvation
  // cycles, split by correct-path (steady) vs post-misprediction (recovery).
  const auto fe_misses = stats.uop_cache_reads - stats.uop_cache_hits;
  const auto fe_stall_total = stats.fe_stall_steady + stats.fe_stall_recovery + stats.switch_stalls;
  lines.push_back(fmt::format("{} frontend-loss misses: steady {} (traced {}) recovery {} (traced {}) (of {} total misses)", stats.name,
                              stats.uop_miss_steady, stats.uop_miss_steady_traced, stats.uop_miss_recovery, stats.uop_miss_recovery_traced, fe_misses));
  lines.push_back(fmt::format("{} frontend-loss stall-cycles (upper): steady {} recovery {} switch {} | total {} ({}% of {} cycles)", stats.name,
                              stats.fe_stall_steady, stats.fe_stall_recovery, stats.switch_stalls, fe_stall_total,
                              ::print_ratio(100 * static_cast<long long>(fe_stall_total), stats.cycles()), stats.cycles()));
  lines.push_back(fmt::format("{} frontend-loss l1i-missed-fetches: steady {} recovery {} wait-cycles steady {} recovery {}", stats.name,
                              stats.fetch_l1i_miss_steady, stats.fetch_l1i_miss_recovery, stats.fetch_l1i_wait_steady, stats.fetch_l1i_wait_recovery));
  const auto rob_idle_total = stats.rob_idle_steady + stats.rob_idle_recovery;
  lines.push_back(fmt::format("{} frontend-loss backend-idle (tight): steady {} recovery {} | total {} ({}% of {} cycles)", stats.name,
                              stats.rob_idle_steady, stats.rob_idle_recovery, rob_idle_total,
                              ::print_ratio(100 * static_cast<long long>(rob_idle_total), stats.cycles()), stats.cycles()));
  // top-down-style middle bound: dispatch slots left unfilled in BUILD mode
  // while the ROB had room; cycle-equivalents = slots / dispatch width
  const auto lost_slots_total = stats.fe_lost_slots_steady + stats.fe_lost_slots_recovery;
  lines.push_back(fmt::format("{} frontend-loss lost-slots (mid): steady {} recovery {} | total {} slots", stats.name,
                              stats.fe_lost_slots_steady, stats.fe_lost_slots_recovery, lost_slots_total));

  lines.push_back(fmt::format("{} trace-seg: loop {} function {} dedup-hits {} entangled {} dropped: bad-layout {} overflow {} short {} stored-uops {} invariant-violations {}",
                              stats.name, stats.seg_traces_loop, stats.seg_traces_function, stats.seg_traces_dedup, stats.seg_traces_entangled,
                              stats.seg_traces_dropped_bad_layout, stats.seg_traces_dropped_overflow, stats.seg_traces_dropped_short,
                              stats.seg_stored_uops, stats.seg_invariant_violations));

  lines.push_back(fmt::format("{} trace-seg coverage: unique IPs {}/{} ({}%) dynamic uops {}/{} ({}%)", stats.name, stats.seg_unique_ips_covered,
                              stats.seg_unique_ips_seen,
                              ::print_ratio(100 * static_cast<long long>(stats.seg_unique_ips_covered), static_cast<long long>(stats.seg_unique_ips_seen)),
                              stats.seg_dynamic_uops_covered, stats.seg_dynamic_uops,
                              ::print_ratio(100 * static_cast<long long>(stats.seg_dynamic_uops_covered), static_cast<long long>(stats.seg_dynamic_uops))));

  lines.push_back(fmt::format("{} trace-rec: loop {} function {} dedup-hits {} entangled {} dropped: bad-layout {} overflow {} short {} stored-uops {} invariant-violations {}",
                              stats.name, stats.rec_traces_loop, stats.rec_traces_function, stats.rec_traces_dedup, stats.rec_traces_entangled,
                              stats.rec_traces_dropped_bad_layout, stats.rec_traces_dropped_overflow, stats.rec_traces_dropped_short,
                              stats.rec_stored_uops, stats.rec_invariant_violations));

  lines.push_back(fmt::format("{} trace-rec coverage: unique IPs {}/{} ({}%) dynamic uops {}/{} ({}%)", stats.name, stats.rec_unique_ips_covered,
                              stats.rec_unique_ips_seen,
                              ::print_ratio(100 * static_cast<long long>(stats.rec_unique_ips_covered), static_cast<long long>(stats.rec_unique_ips_seen)),
                              stats.rec_dynamic_uops_covered, stats.rec_dynamic_uops,
                              ::print_ratio(100 * static_cast<long long>(stats.rec_dynamic_uops_covered), static_cast<long long>(stats.rec_dynamic_uops))));

  lines.push_back(fmt::format("{} trace-stg: loop {} function {} dedup-hits {} entangled {} dropped: bad-layout {} overflow {} short {} stored-uops {} invariant-violations {}",
                              stats.name, stats.stg_traces_loop, stats.stg_traces_function, stats.stg_traces_dedup, stats.stg_traces_entangled,
                              stats.stg_traces_dropped_bad_layout, stats.stg_traces_dropped_overflow, stats.stg_traces_dropped_short,
                              stats.stg_stored_uops, stats.stg_invariant_violations));

  lines.push_back(fmt::format("{} trace-stg coverage: unique IPs {}/{} ({}%) dynamic uops {}/{} ({}%)", stats.name, stats.stg_unique_ips_covered,
                              stats.stg_unique_ips_seen,
                              ::print_ratio(100 * static_cast<long long>(stats.stg_unique_ips_covered), static_cast<long long>(stats.stg_unique_ips_seen)),
                              stats.stg_dynamic_uops_covered, stats.stg_dynamic_uops,
                              ::print_ratio(100 * static_cast<long long>(stats.stg_dynamic_uops_covered), static_cast<long long>(stats.stg_dynamic_uops))));

  lines.push_back(fmt::format("{} trace-stall: traces {} (of {} candidates) dedup-hits {} stored-uops {} l1i-gated {} marks-dropped {}", stats.name,
                              stats.stall_traces, stats.stall_traces_candidates, stats.stall_dedup, stats.stall_stored_uops, stats.stall_gated_l1i,
                              stats.stall_marks_dropped));
  lines.push_back(fmt::format("{} trace-stall-readmit: evicted-recaptures {} readmitted {} filter-evictions {}", stats.name, stats.stall_lost_recaptures,
                              stats.stall_readmits, stats.stall_filter_evictions));
  lines.push_back(fmt::format("{} trace-stall-mpopen: opened {} traces {}", stats.name, stats.stall_mp_opened, stats.stall_mp_traces));
  if (stats.mp_windows > 0) {
    const auto c = static_cast<long long>(stats.mp_cost);
    auto pc = [c](uint64_t v) { return ::print_ratio(100 * static_cast<long long>(v), c); };
    lines.push_back(fmt::format("{} uop-miss-profile: windows {} misses {} stall-cycles {} other-stall-cycles {}", stats.name, stats.mp_windows,
                                stats.mp_misses, stats.mp_cost, stats.mp_other));
    lines.push_back(fmt::format("{} uop-miss-profile top-by-cost(1K/2K/4K/8K) cum% {} {} {} {}  top-by-misses(1K/2K/4K/8K) cum% {} {} {} {}", stats.name,
                                pc(stats.mp_cost_top[0]), pc(stats.mp_cost_top[1]), pc(stats.mp_cost_top[2]), pc(stats.mp_cost_top[3]),
                                pc(stats.mp_cost_freq[0]), pc(stats.mp_cost_freq[1]), pc(stats.mp_cost_freq[2]), pc(stats.mp_cost_freq[3])));
    auto pt = [&stats](std::size_t i) {
      return ::print_ratio(100 * static_cast<long long>(stats.mp_traced_top[i]), static_cast<long long>(stats.mp_cost_top[i]));
    };
    lines.push_back(fmt::format("{} uop-miss-profile traced-by-segmenter: all {}%  top1K {}%  top2K {}%  top4K {}%  top8K {}%", stats.name,
                                pc(stats.mp_traced_all), pt(0), pt(1), pt(2), pt(3)));
  }
  lines.push_back(fmt::format("{} trace-starts: traces {} windows {} lines {} multi-start-windows {} max-per-window {}", stats.name, stats.starts_traces,
                              stats.starts_windows, stats.starts_lines, stats.starts_multi_windows, stats.starts_max_per_window));
  lines.push_back(fmt::format("{} trace-stall-split: continuations {} inherited-costly {} stored {}", stats.name, stats.stall_split_segments,
                              stats.stall_split_inherited, stats.stall_split_traces));
  lines.push_back(fmt::format("{} trace-run: cut-at-jump {} cut-at-limit {} inherited-costly {} continuation-traces-stored {}", stats.name,
                              stats.run_cut_jump, stats.run_cut_full, stats.stall_split_inherited, stats.stall_split_traces));
  lines.push_back(fmt::format("{} trace-stall-lead: keyed {} self {} traces {} conflicts {} dist(4K/64K/1M/far) {}", stats.name, stats.stall_lead_keyed,
                              stats.stall_lead_self, stats.stall_lead_traces, stats.stall_lead_conflicts, fmt::join(stats.stall_lead_dist, " ")));
  lines.push_back(fmt::format("{} trace-stall-starve: offset-windows(0/1/2/3/4-7/8+/past-depth/before/after): {}", stats.name,
                              fmt::join(stats.stall_starve_offset, " ")));
  lines.push_back(fmt::format("{} trace-store: evictions {} conflict-evictions {} occupancy {} windows {} shared {}", stats.name, stats.store_evictions,
                              stats.store_conflict_evictions, stats.store_occupancy, stats.store_win_added, stats.store_win_shared));
  lines.push_back(fmt::format("{} trace-store-tags: lookups {} alias-hits {} alias-overwrites {}", stats.name, stats.store_lookups,
                              stats.store_alias_hits, stats.store_alias_overwrites));
  if (stats.fup_walks > 0) {
    lines.push_back(fmt::format("{} ftq-uop-prefetch: walks {}", stats.name, stats.fup_walks));
  }
  if (stats.fdip_issued > 0 || stats.fdip_dropped > 0) {
    lines.push_back(fmt::format("{} ftq-fdip: issued {} dropped {} wrong-path {} filtered {}", stats.name, stats.fdip_issued, stats.fdip_dropped, stats.fdip_wp_issued,
                                stats.fdip_filtered));
  }
  if (stats.td_arrivals > 0) {
    // trace_trigger_diag; class order: unclassified uncovered not-stored side-door not-probed dropped resident-at-probe late staged-evicted walked-gone
    //                                  cold past-depth never-costly costly-once costly-repeat
    lines.push_back(fmt::format("{} trace-trigger-diag probes(resident/conf/cap/dedup/launched): {} {} {} {} {}", stats.name, stats.td_probes[1],
                                stats.td_probes[2], stats.td_probes[3], stats.td_probes[4], stats.td_probes[5]));
    lines.push_back(fmt::format("{} trace-trigger-diag arrivals {} resident {} staged {} missed-by-class: {}", stats.name, stats.td_arrivals,
                                stats.td_arr_resident, stats.td_arr_staged, fmt::join(stats.td_arr_miss, " ")));
    lines.push_back(fmt::format("{} trace-trigger-diag lead(8/16/32/64/128/inf) served: {} missed: {}", stats.name, fmt::join(stats.td_lead_served, " "),
                                fmt::join(stats.td_lead_missed, " ")));
    lines.push_back(fmt::format("{} trace-trigger-diag misses-by-class: {}", stats.name, fmt::join(stats.td_miss, " ")));
    lines.push_back(fmt::format("{} trace-trigger-diag staged {} late {} evicted-unused: late {} on-time {}", stats.name, stats.td_staged,
                                stats.td_staged_late, stats.td_unused_late, stats.td_unused_ontime));
    lines.push_back(fmt::format("{} trace-trigger-diag staged-position(0/1/2/3/4+) used: {} unused: {}", stats.name, fmt::join(stats.td_pos_used, " "),
                                fmt::join(stats.td_pos_unused, " ")));
    lines.push_back(fmt::format("{} trace-trigger-diag recovery-misses-by-class: {}", stats.name, fmt::join(stats.td_miss_rec, " ")));
    lines.push_back(fmt::format("{} trace-trigger-diag empty-rob-cycles-by-run-head: {} other {}", stats.name, fmt::join(stats.td_head_tight, " "),
                                stats.td_cost_tight_other));
    lines.push_back(fmt::format("{} trace-trigger-diag starve-cycles-by-run-head: {} other {}", stats.name, fmt::join(stats.td_head_upper, " "),
                                stats.td_cost_upper_other));
    lines.push_back(fmt::format("{} trace-trigger-diag empty-rob-cycles-by-class: {} other {}", stats.name, fmt::join(stats.td_cost_tight, " "),
                                stats.td_cost_tight_other));
    lines.push_back(fmt::format("{} trace-trigger-diag starve-cycles-by-class: {} other {}", stats.name, fmt::join(stats.td_cost_upper, " "),
                                stats.td_cost_upper_other));
  }
  if (stats.chain_miss_inflight > 0 || stats.chain_miss_entry > 0 || stats.chain_miss_midtrace > 0) {
    lines.push_back(fmt::format("{} chain-miss-classes: in-flight {} entry {} mid-trace {}", stats.name, stats.chain_miss_inflight,
                                stats.chain_miss_entry, stats.chain_miss_midtrace));
  }
  if (stats.event_probes > 0 || stats.stall_event_traces > 0) {
    lines.push_back(fmt::format("{} trace-event: probes {} probe-hits {} traces {} (of {} candidates) dedup-hits {}", stats.name, stats.event_probes,
                                stats.event_probe_hits, stats.stall_event_traces, stats.stall_event_candidates, stats.stall_event_dedup));
  }
  lines.push_back(fmt::format("{} trace-stall coverage: unique IPs {}/{} ({}%) dynamic uops {}/{} ({}%)", stats.name, stats.stall_unique_ips_covered,
                              stats.stall_unique_ips_seen,
                              ::print_ratio(100 * static_cast<long long>(stats.stall_unique_ips_covered), static_cast<long long>(stats.stall_unique_ips_seen)),
                              stats.stall_dynamic_uops_covered, stats.stall_dynamic_uops,
                              ::print_ratio(100 * static_cast<long long>(stats.stall_dynamic_uops_covered), static_cast<long long>(stats.stall_dynamic_uops))));
  {
    const auto tw = static_cast<long long>(stats.stall_total_dynweight);
    auto p = [tw](uint64_t v) { return ::print_ratio(100 * static_cast<long long>(v), tw); };
    const auto tc = static_cast<long long>(stats.stall_total_cost);
    auto pc = [tc](uint64_t v) { return ::print_ratio(100 * static_cast<long long>(v), tc); };
    auto a = [](const std::array<double, 7>& v) {
      return fmt::format("16={:.1f} 32={:.1f} 64={:.1f} 128={:.1f} 256={:.1f} 512={:.1f} 1024={:.1f}", v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
    };
    auto pcs = [&pc](const std::array<uint64_t, 7>& v) {
      return fmt::format("16={}% 32={}% 64={}% 128={}% 256={}% 512={}% 1024={}%", pc(v[0]), pc(v[1]), pc(v[2]), pc(v[3]), pc(v[4]), pc(v[5]), pc(v[6]));
    };
    lines.push_back(fmt::format("{} trace-stall top-by-occurrence (cum% dyn-weight): 16={}% 32={}% 64={}% 128={}% 256={}% 512={}% 1024={}%", stats.name,
                                p(stats.stall_occ_top16), p(stats.stall_occ_top32), p(stats.stall_occ_top64), p(stats.stall_occ_top128),
                                p(stats.stall_occ_top256), p(stats.stall_occ_top512), p(stats.stall_occ_top1024)));
    lines.push_back(fmt::format("{} trace-stall top-by-occurrence (cum% stall-cycles): {}", stats.name, pcs(stats.stall_occ_cumcost)));
    lines.push_back(fmt::format("{} trace-stall top-by-occurrence avg-occ: {}", stats.name, a(stats.stall_occ_avgocc)));
    lines.push_back(fmt::format("{} trace-stall top-by-occurrence avg-len: {}", stats.name, a(stats.stall_occ_avglen)));
    lines.push_back(fmt::format("{} trace-stall top-by-coverage (cum% dyn-weight): 16={}% 32={}% 64={}% 128={}% 256={}% 512={}% 1024={}%", stats.name,
                                p(stats.stall_cov_top16), p(stats.stall_cov_top32), p(stats.stall_cov_top64), p(stats.stall_cov_top128),
                                p(stats.stall_cov_top256), p(stats.stall_cov_top512), p(stats.stall_cov_top1024)));
    lines.push_back(fmt::format("{} trace-stall top-by-coverage (cum% stall-cycles): {}", stats.name, pcs(stats.stall_cov_cumcost)));
    lines.push_back(fmt::format("{} trace-stall top-by-coverage avg-occ: {}", stats.name, a(stats.stall_cov_avgocc)));
    lines.push_back(fmt::format("{} trace-stall top-by-coverage avg-len: {}", stats.name, a(stats.stall_cov_avglen)));
    lines.push_back(fmt::format("{} trace-stall top-by-cost (cum% stall-cycles): 16={}% 32={}% 64={}% 128={}% 256={}% 512={}% 1024={}% (total {})",
                                stats.name, pc(stats.stall_cost_cum[0]), pc(stats.stall_cost_cum[1]), pc(stats.stall_cost_cum[2]),
                                pc(stats.stall_cost_cum[3]), pc(stats.stall_cost_cum[4]), pc(stats.stall_cost_cum[5]), pc(stats.stall_cost_cum[6]),
                                stats.stall_total_cost));
    lines.push_back(fmt::format("{} trace-stall top-by-cost (cum% dyn-weight): 16={}% 32={}% 64={}% 128={}% 256={}% 512={}% 1024={}%", stats.name,
                                p(stats.stall_cost_cumw[0]), p(stats.stall_cost_cumw[1]), p(stats.stall_cost_cumw[2]), p(stats.stall_cost_cumw[3]),
                                p(stats.stall_cost_cumw[4]), p(stats.stall_cost_cumw[5]), p(stats.stall_cost_cumw[6])));
    lines.push_back(fmt::format("{} trace-stall top-by-cost-big (cum% stall-cycles): 2048={}% 4096={}% 8192={}% (of {} traces)", stats.name,
                                pc(stats.stall_cost_cum_big[0]), pc(stats.stall_cost_cum_big[1]), pc(stats.stall_cost_cum_big[2]), stats.stall_traces));
    lines.push_back(fmt::format("{} trace-stall top-by-cost avg-occ: {}", stats.name, a(stats.stall_cost_avgocc)));
    lines.push_back(fmt::format("{} trace-stall top-by-cost avg-len: {}", stats.name, a(stats.stall_cost_avglen)));
  }

  lines.push_back(fmt::format("{} trace-alt: triggers {} dropped {} installed-windows {} late-misses {} useful-hits {} unused-evicted {} wait-cycles {} lines-issued {} line-stalls {}",
                              stats.name, stats.alt_triggers, stats.alt_drops, stats.alt_installed_windows, stats.alt_late_misses, stats.alt_useful_hits,
                              stats.alt_pf_evicted_unused, stats.alt_wait_cycles, stats.alt_lines_issued, stats.alt_line_stalls));
  if (std::accumulate(std::begin(stats.tax_upper), std::end(stats.tax_upper), uint64_t{0}) > 0) {
    for (const auto* arr : {&stats.tax_tight, &stats.tax_upper}) {
      lines.push_back(fmt::format("{} stall-taxonomy {} (steady seen L1I/L2/LLC/DRAM/none | steady first | recovery seen | recovery first): {} | other: not-looked-up {} uop-hit {}",
                                  stats.name, (arr == &stats.tax_tight) ? "tight" : "upper", fmt::join(*arr, " "),
                                  (arr == &stats.tax_tight) ? stats.tax_other[0] : stats.tax_other[1], (arr == &stats.tax_tight) ? stats.tax_other[2] : stats.tax_other[3]));
    }
  }
  if (stats.l2_perfect_hits > 0 || stats.l2_emissary_marks > 0 || stats.emissary_mark_events > 0) {
    lines.push_back(fmt::format("{} l2-instr: perfect-hits {} emissary-mark-events {} l2-marks {} diverted-evictions {} high-priority-evicted {}", stats.name,
                                stats.l2_perfect_hits, stats.emissary_mark_events, stats.l2_emissary_marks, stats.l2_emissary_diverted,
                                stats.l2_emissary_prot_evicted));
  }
  {
    const auto& m = stats.rec_mispredicts;
    const auto& c = stats.rec_idle_cycles;
    if (std::accumulate(std::begin(m), std::end(m), uint64_t{0}) > 0) {
      lines.push_back(fmt::format("{} recovery-by-branch (mispredictions / empty-ROB recovery cycles): cond-H2P {} / {}  cond-confident {} / {}  indirect {} / {}  "
                                  "return {} / {}  direct(BTB-miss) {} / {}  other {} / {}",
                                  stats.name, m[0], c[0], m[1], c[1], m[2], c[2], m[3], c[3], m[4], c[4], m[5], c[5]));
    }
  }
  if (stats.rtt_captures > 0 || stats.rtt_lookups > 0) {
    if (stats.rtt_dir_full + stats.rtt_dir_bits_out + stats.rtt_dir_btb_miss + stats.rtt_dir_target > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: direction-bit traces end: whole {} bits used up {} taken branch the BTB misses {} BTB target differs {}  avg bits {:.1f}",
                                  stats.name, stats.rtt_dir_full, stats.rtt_dir_bits_out, stats.rtt_dir_btb_miss, stats.rtt_dir_target,
                                  stats.rtt_captures > 0 ? static_cast<double>(stats.rtt_dir_bits_used) / static_cast<double>(stats.rtt_captures) : 0.0));
    }
    if (stats.rtt_truncated > 0 || stats.rtt_unencodable > 0 || stats.rtt_stored_instrs > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: manifest-encoded: truncated {} rejected {} avg stored length {:.1f} instrs", stats.name,
                                  stats.rtt_truncated, stats.rtt_unencodable,
                                  stats.rtt_captures > 0 ? static_cast<double>(stats.rtt_stored_instrs) / static_cast<double>(stats.rtt_captures) : 0.0));
    }
    if (stats.walk_pq_issued > 0) {
      lines.push_back(fmt::format("{} walk-pq: walk lines issued as L1I prefetches {}  waited (prefetch queue full or in-flight cap) {}", stats.name,
                                  stats.walk_pq_issued, stats.walk_pq_full));
    }
    if (stats.walker_mshr_peak > 0) {
      lines.push_back(fmt::format("{} walker-mshr: peak occupancy {}  issue cycles blocked (all walker MSHRs busy) {}", stats.name, stats.walker_mshr_peak,
                                  stats.walker_mshr_full));
    }
    if (stats.walk_res_lines + stats.walk_miss_lines > 0) {
      lines.push_back(fmt::format("{} walk-lines: L1I-resident at launch {} (read via idle demand port {}, evicted before read {})  not resident {}", stats.name,
                                  stats.walk_res_lines, stats.walk_res_read, stats.walk_res_evicted, stats.walk_miss_lines));
    }
    if (stats.late_hol + stats.late_decode + stats.late_unissued + stats.late_memory > 0) {
      lines.push_back(fmt::format("{} walk-late: u-op misses on a window still being walked: line arrived, behind an earlier window {}  line arrived, waiting to pre-decode {}  line not yet issued {}  line issued, in the memory hierarchy {}",
                                  stats.name, stats.late_hol, stats.late_decode, stats.late_unissued, stats.late_memory));
    }
    if (stats.rtt_fb_walks + stats.rtt_fb_wp_walks > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: generated alternate paths walked (no trace) {} (wrong path {})", stats.name, stats.rtt_fb_walks,
                                  stats.rtt_fb_wp_walks));
    }
    if (stats.rtt_win_total > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: stored windows {} distinct {} ({:.1f}% duplicates)  windows in >= 2 traces {}  recordings ended at reconvergence {}",
                                  stats.name, stats.rtt_win_total, stats.rtt_win_distinct,
                                  100.0 * static_cast<double>(stats.rtt_win_total - stats.rtt_win_distinct) / static_cast<double>(stats.rtt_win_total),
                                  stats.rtt_win_shared, stats.rtt_reconv_cut));
    }
    if (stats.rtt_cost_sum + stats.rtt_cost_zero > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: trace cost: recordings with an empty-ROB recovery {} without {}  avg cost bucket {:.2f}  low-confidence walks allowed by cost {}",
                                  stats.name, stats.rtt_captures - stats.rtt_cost_zero, stats.rtt_cost_zero,
                                  stats.rtt_captures > 0 ? static_cast<double>(stats.rtt_cost_sum) / static_cast<double>(stats.rtt_captures) : 0.0,
                                  stats.rtt_cost_rescued));
    }
    if (stats.rtt_seg_rerecords > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: stable-prefix re-recordings {}  cut at divergence {}  avg kept {:.1f} instrs", stats.name,
                                  stats.rtt_seg_rerecords, stats.rtt_seg_trimmed,
                                  static_cast<double>(stats.rtt_seg_kept_instrs) / static_cast<double>(stats.rtt_seg_rerecords)));
    }
    if (stats.rtt_cold_triggers > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: no-trace taken H2P fall-through prefetch: triggers {} (wrong path {}) lines {} already in L1I {}", stats.name,
                                  stats.rtt_cold_triggers, stats.rtt_cold_wp_triggers, stats.rtt_cold_lines, stats.rtt_cold_resident));
    }
    if (stats.rtt_boost_hits > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: confident conditionals with a trace {}  walked (entry counter high) {}", stats.name,
                                  stats.rtt_boost_hits, stats.rtt_boost_walks));
    }
    if (stats.rtt_ret_hits > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: returns with a trace {}  walked (entry counter high) {}", stats.name, stats.rtt_ret_hits,
                                  stats.rtt_ret_walks));
    }
    if (stats.rtt_repeat_held > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: new traces held back until their second recording {}", stats.name, stats.rtt_repeat_held));
    }
    if (stats.rtt_admit_skipped > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: recordings not stored (no u-op-cache miss) {}", stats.name, stats.rtt_admit_skipped));
    }
    if (stats.rtt_conf_up + stats.rtt_conf_down > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: confidence: walks suppressed {}  trained up (mispredicted) {} down (correct) {}", stats.name,
                                  stats.rtt_conf_suppressed, stats.rtt_conf_up, stats.rtt_conf_down));
    }
    if (stats.rtt_cancelled > 0 || stats.walk_mshr_yield > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: walks cancelled at correct resolution {} (lines not issued {}, staged windows dropped {})  walk-issue cycles yielded to FDIP/demand {}",
                                  stats.name, stats.rtt_cancelled, stats.rtt_cancel_lines, stats.rtt_cancel_windows, stats.walk_mshr_yield));
    }
    if (stats.rtt_wp_lookups > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: wrong-path predictions: lookups {} hits {} walks {}", stats.name, stats.rtt_wp_lookups,
                                  stats.rtt_wp_hits, stats.rtt_wp_walks));
    }
    if (stats.rtt_alt > 0 || stats.rtt_alt_nobtb > 0) {
      lines.push_back(fmt::format("{} recovery-trace-table: alternate paths generated {} (not-taken branch with no BTB target, none: {})", stats.name, stats.rtt_alt, stats.rtt_alt_nobtb));
    }
    lines.push_back(fmt::format("{} recovery-trace-table: opened {} (at mispredictions {}) stored {} avg-len {:.1f} not stored (ROB never empty) {} table {} traces  lookups {} hits {} walks {}",
                                stats.name, stats.rtt_opens, stats.rtt_opens_wrong, stats.rtt_captures,
                                stats.rtt_captures > 0 ? static_cast<double>(stats.rtt_cap_instrs) / static_cast<double>(stats.rtt_captures) : 0.0,
                                stats.rtt_filtered, stats.rtt_table_size, stats.rtt_lookups, stats.rtt_hits, stats.rtt_walks));
  } else if (stats.walk_mshr_yield > 0) {
    lines.push_back(fmt::format("{} walk-issue: cycles yielded to FDIP/demand {}", stats.name, stats.walk_mshr_yield)); // e.g. UCP walks under walk_mshr_limit
  }
  if (stats.rtt_dg_lookups > 0) {
    auto pct = [](uint64_t a, uint64_t b) { return b > 0 ? 100.0 * static_cast<double>(a) / static_cast<double>(b) : 0.0; };
    lines.push_back(fmt::format("{} rtt-diag: mispredictions {} trace-found {:.1f}% avg-prefix-match {:.1f} full-match {:.1f}% recall {:.1f}% ({} missing windows on the real path) precision {:.1f}% ({} stored windows to walk)",
                                stats.name, stats.rtt_dg_lookups, pct(stats.rtt_dg_found, stats.rtt_dg_lookups),
                                stats.rtt_dg_found > 0 ? static_cast<double>(stats.rtt_dg_prefix) / static_cast<double>(stats.rtt_dg_found) : 0.0,
                                pct(stats.rtt_dg_full, stats.rtt_dg_found), pct(stats.rtt_dg_real_cov, stats.rtt_dg_real_win), stats.rtt_dg_real_win,
                                pct(stats.rtt_dg_stored_use, stats.rtt_dg_stored_win), stats.rtt_dg_stored_win));
    lines.push_back(fmt::format("{} rtt-diag missing real-path windows: line not in L1I {} (first-touch {:.1f}%, stored trace covers {:.1f}%, {:.1f}% of the seen-before)  line in L1I {} (first-touch {:.1f}%, covered {:.1f}%, {:.1f}% of the seen-before)",
                                stats.name, stats.rtt_dg_far, pct(stats.rtt_dg_far_first, stats.rtt_dg_far), pct(stats.rtt_dg_far_cov, stats.rtt_dg_far),
                                pct(stats.rtt_dg_far_cov, stats.rtt_dg_far - stats.rtt_dg_far_first), stats.rtt_dg_near, pct(stats.rtt_dg_near_first, stats.rtt_dg_near),
                                pct(stats.rtt_dg_near_cov, stats.rtt_dg_near), pct(stats.rtt_dg_near_cov, stats.rtt_dg_near - stats.rtt_dg_near_first)));
  }
  if (stats.oracle_recovery_triggers > 0) {
    lines.push_back(fmt::format("{} oracle-recovery: mispredictions {} real-path walks {}", stats.name, stats.oracle_recovery_triggers, stats.oracle_recovery_walks));
  }
  if (stats.fdip_pd_windows > 0) {
    lines.push_back(fmt::format("{} fdip-predecode: windows {} costly {} launched {} installed {} expired {} dropped {} cwt-marks {}", stats.name,
                                stats.fdip_pd_windows, stats.fdip_pd_costly, stats.fdip_pd_launched, stats.fdip_pd_installed, stats.fdip_pd_expired,
                                stats.fdip_pd_dropped, stats.cwt_marks));
  }
  if (stats.rp_lookups > 0) {
    lines.push_back(fmt::format("{} resteer-prefetch: lookups {} hits {} prefetches {} dropped {} records {}", stats.name, stats.rp_lookups, stats.rp_hits,
                                stats.rp_prefetches, stats.rp_dropped, stats.rp_records));
  }
  if (stats.uop_incl_invalidations > 0) {
    lines.push_back(fmt::format("{} uop-inclusive: invalidations {}", stats.name, stats.uop_incl_invalidations));
  }
  if (stats.retain_marks > 0) {
    lines.push_back(fmt::format("{} uop-retain: marks {} diverted-evictions {} costly-evicted {}", stats.name, stats.retain_marks, stats.retain_diverted,
                                stats.retain_costly_evicted));
  }
  if (stats.alt_walk_aborts > 0) {
    lines.push_back(fmt::format("{} trace-alt WARNING: walk-aborts {}", stats.name, stats.alt_walk_aborts));
  }
  lines.push_back(fmt::format("{} trace-chain: unencodable {} truncated {} buffer-hits {} buffer-evicted-unused {} filtered {} conf-suppressed {} evicted-then-missed {} age(256/1K/4K/16K/inf): {} {} {} {} {} slack(4/8/16/32/64/inf): {} {} {} {} {} {}",
                              stats.name, stats.chain_unencodable, stats.chain_truncated, stats.chain_buf_hits, stats.chain_buf_evict_unused,
                              stats.chain_filtered_windows, stats.chain_conf_suppressed, stats.chain_evicted_then_missed,
                              stats.chain_evicted_age[0], stats.chain_evicted_age[1], stats.chain_evicted_age[2], stats.chain_evicted_age[3],
                              stats.chain_evicted_age[4], stats.chain_slack[0], stats.chain_slack[1], stats.chain_slack[2], stats.chain_slack[3],
                              stats.chain_slack[4], stats.chain_slack[5]));
  lines.push_back(fmt::format("{} trace-chain-promote: buffer-rehits {}", stats.name, stats.chain_buf_rehits));
  lines.push_back(fmt::format("{} trace-chain-shadow: mispredicts {} past-mispredict walks {} lines {} staged-windows {} buffer-hits {}", stats.name,
                              stats.chain_shadow_mispredicts, stats.chain_shadow_walks, stats.chain_shadow_lines, stats.chain_shadow_windows,
                              stats.chain_shadow_hits));
  lines.push_back(fmt::format("{} trace-wrong-path: cursor-stops {} shadows {} addresses {} fetched {} uop-misses {} fetch-cycles {} decoder-busy-cycles {}"
                              " store-hits {} ucp-triggers {} walks {} flushed {} lines {} staged {} buffer-hits {} l1i-lines {} line-stall-cycles {}"
                              " backend-full-cycles {} stops {} uop-fills {}",
                              stats.name, stats.wp_cursor_stops, stats.wp_shadows, stats.wp_addresses, stats.wp_fetched, stats.wp_uop_misses,
                              stats.wp_fetch_cycles, stats.wp_decoder_busy_cycles, stats.wp_store_hits, stats.wp_ucp_triggers, stats.wp_walks,
                              stats.wp_walks_flushed, stats.wp_lines, stats.wp_staged, stats.wp_buffer_hits, stats.wp_l1i_lines,
                              stats.wp_line_stall_cycles, stats.wp_full_cycles, stats.wp_stops, stats.wp_uop_fills));
  lines.push_back(fmt::format("{} trace-peek: probes {} store-hits {}", stats.name, stats.peek_probes, stats.peek_store_hits));
  lines.push_back(fmt::format("{} trace-succession steady: addr(next/near/4K/jump) {} gap(0/1-8/9-64/65-256/more) {}", stats.name,
                              fmt::join(stats.succ_addr_steady, " "), fmt::join(stats.succ_gap_steady, " ")));
  lines.push_back(fmt::format("{} trace-succession after-mispredict: addr(next/near/4K/jump) {} gap(0/1-8/9-64/65-256/more) {}", stats.name,
                              fmt::join(stats.succ_addr_mp, " "), fmt::join(stats.succ_gap_mp, " ")));
  lines.push_back(fmt::format("{} trace-chain-follow: links {} hops {} found {} walks(hop1/2/3/4+) {} buffer-hits(hop1/2/3/4+) {}", stats.name,
                              stats.chain_links, stats.chain_hops, stats.chain_found, fmt::join(stats.chain_walks_hop, " "),
                              fmt::join(stats.chain_bhits_hop, " ")));
  lines.push_back(fmt::format("{} trace-vpred: lookups {} matches {} fires {} store-hits {} walks {} walks-at-mispredict {} buffer-hits {} buffer-hits-at-mispredict {}",
                              stats.name, stats.vp_lookups, stats.vp_matches, stats.vp_fires, stats.vp_store_hits, stats.vp_walks, stats.vp_walks_mp,
                              stats.vp_buffer_hits, stats.vp_buffer_hits_mp));
  lines.push_back(fmt::format("{} trace-vpred-train: events {} after-mispredict {} correct {} correct-after-mispredict {} wrong {} no-provider {} none {} allocs {} alloc-fail {}",
                              stats.name, stats.vp_train, stats.vp_train_mp, stats.vp_correct, stats.vp_correct_mp, stats.vp_wrong, stats.vp_noprov,
                              stats.vp_none, stats.vp_allocs, stats.vp_alloc_fail));
  lines.push_back(fmt::format("{} trace-recovery-key: mispredicts {} filed {} probes {} hits {} walks {} buffer-hits {}", stats.name,
                              stats.rk_mispredicts, stats.rk_filed, stats.rk_probes, stats.rk_hits, stats.rk_walks, stats.rk_buffer_hits));
  lines.push_back(fmt::format("{} trace-peek-diag: mispredicts {} with-trace {}", stats.name, stats.pdiag_mispredicts, stats.pdiag_with_trace));
  {
    static constexpr std::array<const char*, 8> PEEK_KEY_NAMES{"pc", "pc+ghr8", "pc+ghr16", "pc+ghr32", "pc+ghr64", "pc+th1", "pc+th2", "pc+th4"};
    for (std::size_t i = 0; i < PEEK_KEY_NAMES.size(); ++i) {
      lines.push_back(fmt::format("{} trace-peek-diag key {}: fires {} useful {} wrong {}", stats.name, PEEK_KEY_NAMES[i], stats.pdiag_fires[i],
                                  stats.pdiag_useful[i], stats.pdiag_wrong[i]));
    }
  }
  lines.push_back(fmt::format("{} trace-chain-deltas: seq {} short {} long {} far {} traces: single {} seq-only {} with-short {} with-long {} far-truncated {}",
                              stats.name, stats.chain_delta_seq, stats.chain_delta_short, stats.chain_delta_long, stats.chain_delta_far,
                              stats.chain_trace_single, stats.chain_trace_seq_only, stats.chain_trace_with_short, stats.chain_trace_with_long,
                              stats.chain_far_truncated));
  lines.push_back(fmt::format("{} trace-chain-need: bits(16/24/32/48/64/inf): {} {} {} {} {} {} windows(2/3/4/6/inf): {} {} {} {} {} trunc-windows {}",
                              stats.name, stats.chain_need_bits[0], stats.chain_need_bits[1], stats.chain_need_bits[2], stats.chain_need_bits[3],
                              stats.chain_need_bits[4], stats.chain_need_bits[5], stats.chain_need_windows[0], stats.chain_need_windows[1],
                              stats.chain_need_windows[2], stats.chain_need_windows[3], stats.chain_need_windows[4], stats.chain_trunc_windows));
  lines.push_back(fmt::format("{} trace-chain-deltabits(<=3..16,>16): {}", stats.name,
                              fmt::join(std::next(std::begin(stats.chain_delta_bits), 3), std::end(stats.chain_delta_bits), " ")));
  lines.push_back(fmt::format("{} trace-chain-reach: transfers(4K/128K/256K/512K/1M/inf): {} {} {} {} {} {} traces(none/4K/128K/256K/512K/1M/inf): {} {} {} {} {} {} {}",
                              stats.name, stats.chain_reach_xfer[0], stats.chain_reach_xfer[1], stats.chain_reach_xfer[2], stats.chain_reach_xfer[3],
                              stats.chain_reach_xfer[4], stats.chain_reach_xfer[5], stats.chain_reach_none, stats.chain_reach_trace[0],
                              stats.chain_reach_trace[1], stats.chain_reach_trace[2], stats.chain_reach_trace[3], stats.chain_reach_trace[4],
                              stats.chain_reach_trace[5]));
  for (std::size_t s = 0; s < 2; ++s) {
    const auto& d = stats.cold_dist[s];
    const auto& c = stats.cold_class[s];
    if (d[0] + d[1] + d[2] + d[3] + d[4] + d[5] > 0) {
      lines.push_back(fmt::format("{} cold-diag {}: post-redirect L1I-missing u-op misses by line distance from the target: 0 {} 1 {} 2-3 {} 4-7 {} 8+ {} behind {}"
                                  " | by mispredict: cond-H2P {} cond-confident {} indirect {} return {} direct(BTB-miss) {} other {}",
                                  stats.name, s == 0 ? "seen-before" : "never-seen", d[0], d[1], d[2], d[3], d[4], d[5], c[0], c[1], c[2], c[3], c[4], c[5]));
    }
  }
  if (stats.uop_split_hits[0] + stats.uop_split_hits[1] + stats.uop_split_hits[2] + stats.uop_split_hits[3] > 0) {
    lines.push_back(fmt::format("{} uop-ideal-split: misses served as hits: steady/L1I-resident {} steady/L1I-missing {} recovery/resident {} recovery/missing {}", stats.name,
                                stats.uop_split_hits[0], stats.uop_split_hits[1], stats.uop_split_hits[2], stats.uop_split_hits[3]));
  }
  if (stats.ucp_cond_seen > 0) {
    lines.push_back(fmt::format("{} trace-ucp: cond {} cond-misses {} h2p-marked {} h2p-marked-misses {} paths {} stops(sat/ind/btbmiss/maxip): {} {} {} {} ind-walked {} bank-conflicts {}",
                                stats.name, stats.ucp_cond_seen, stats.ucp_cond_misses, stats.ucp_h2p_marked, stats.ucp_h2p_marked_misses, stats.ucp_paths,
                                stats.ucp_stop_sat, stats.ucp_stop_ind, stats.ucp_stop_btbmiss, stats.ucp_stop_maxip, stats.ucp_ind_walked,
                                stats.ucp_bank_conflicts));
    if (stats.ucp_art_dropped + stats.ucp_art_merged + stats.ucp_art_pf_issued > 0) {
      lines.push_back(fmt::format("{} trace-ucp-artifact: windows dropped by a newer path {} merged into an in-flight path {}  L1I prefetches {}  waited (PQ full / 32 in flight) {}",
                                  stats.name, stats.ucp_art_dropped, stats.ucp_art_merged, stats.ucp_art_pf_issued, stats.ucp_art_pf_full));
    }
  }

  lines.push_back(fmt::format("{} trace-seg loss(dyn): covered-final {} latency {} | no-trigger {} overflow {} bad-layout {} short {}", stats.name,
                              stats.seg_dyn_covered_final, stats.seg_dyn_covered_final - stats.seg_dynamic_uops_covered, stats.seg_dyn_lost_no_trigger,
                              stats.seg_dyn_lost_overflow, stats.seg_dyn_lost_bad_layout, stats.seg_dyn_lost_short));
  lines.push_back(fmt::format("{} trace-rec loss(dyn): covered-final {} latency {} | no-trigger {} overflow {} bad-layout {} short {}", stats.name,
                              stats.rec_dyn_covered_final, stats.rec_dyn_covered_final - stats.rec_dynamic_uops_covered, stats.rec_dyn_lost_no_trigger,
                              stats.rec_dyn_lost_overflow, stats.rec_dyn_lost_bad_layout, stats.rec_dyn_lost_short));
  lines.push_back(fmt::format("{} trace-stg loss(dyn): covered-final {} latency {} | no-trigger {} overflow {} bad-layout {} short {}", stats.name,
                              stats.stg_dyn_covered_final, stats.stg_dyn_covered_final - stats.stg_dynamic_uops_covered, stats.stg_dyn_lost_no_trigger,
                              stats.stg_dyn_lost_overflow, stats.stg_dyn_lost_bad_layout, stats.stg_dyn_lost_short));
  lines.push_back(fmt::format("{} trace xcov(dyn): seg-not-stg {} seg-not-rec {} stg-not-rec {}", stats.name, stats.xcov_seg_not_stg, stats.xcov_seg_not_rec,
                              stats.xcov_stg_not_rec));

  lines.emplace_back("Branch type MPKI");
  for (auto idx : types) {
    lines.push_back(fmt::format("{}: {}", branch_type_names.at(champsim::to_underlying(idx)),
                                ::print_ratio(std::kilo::num * stats.branch_type_misses.value_or(idx, 0), stats.instrs())));
  }

  return lines;
}

std::vector<std::string> champsim::plain_printer::format(CACHE::stats_type stats)
{
  using hits_value_type = typename decltype(stats.hits)::value_type;
  using misses_value_type = typename decltype(stats.misses)::value_type;
  using miss_merge_value_type = typename decltype(stats.miss_merge)::value_type;
  using fill_value_type = typename decltype(stats.fill)::value_type;

  std::vector<std::size_t> cpus;

  // build a vector of all existing cpus
  auto stat_keys = {stats.hits.get_keys(), stats.misses.get_keys(), stats.miss_merge.get_keys(), stats.fill.get_keys()};
  for (auto keys : stat_keys) {
    std::transform(std::begin(keys), std::end(keys), std::back_inserter(cpus), [](auto val) { return val.second; });
  }
  std::sort(std::begin(cpus), std::end(cpus));
  auto uniq_end = std::unique(std::begin(cpus), std::end(cpus));
  cpus.erase(uniq_end, std::end(cpus));

  for (const auto type : {access_type::LOAD, access_type::RFO, access_type::PREFETCH, access_type::WRITE, access_type::TRANSLATION}) {
    for (auto cpu : cpus) {
      stats.hits.allocate(std::pair{type, cpu});
      stats.misses.allocate(std::pair{type, cpu});
      stats.miss_merge.allocate(std::pair{type, cpu});
      stats.fill.allocate(std::pair{type, cpu});
    }
  }

  std::vector<std::string> lines{};
  for (auto cpu : cpus) {
    hits_value_type total_hits = 0;
    misses_value_type total_misses = 0;
    miss_merge_value_type total_miss_merge = 0;
    fill_value_type total_fill = 0;
    for (const auto type : {access_type::LOAD, access_type::RFO, access_type::PREFETCH, access_type::WRITE, access_type::TRANSLATION}) {
      total_hits += stats.hits.value_or(std::pair{type, cpu}, hits_value_type{});
      total_misses += stats.misses.value_or(std::pair{type, cpu}, misses_value_type{});
      total_miss_merge += stats.miss_merge.value_or(std::pair{type, cpu}, miss_merge_value_type{});
      total_fill += stats.fill.value_or(std::pair{type, cpu}, miss_merge_value_type{});
    }

    fmt::format_string<std::string_view, std::string_view, int, int, int> hitmiss_fmtstr{
        "cpu{}->{} {:<12s} ACCESS: {:10d} HIT: {:10d} MISS: {:10d} MISS_MERGE: {:10d}"};
    lines.push_back(fmt::format(hitmiss_fmtstr, cpu, stats.name, "TOTAL", total_hits + total_misses, total_hits, total_misses, total_miss_merge));
    for (const auto type : {access_type::LOAD, access_type::RFO, access_type::PREFETCH, access_type::WRITE, access_type::TRANSLATION}) {
      lines.push_back(
          fmt::format(hitmiss_fmtstr, cpu, stats.name, access_type_names.at(champsim::to_underlying(type)),
                      stats.hits.value_or(std::pair{type, cpu}, hits_value_type{}) + stats.misses.value_or(std::pair{type, cpu}, misses_value_type{}),
                      stats.hits.value_or(std::pair{type, cpu}, hits_value_type{}), stats.misses.value_or(std::pair{type, cpu}, misses_value_type{}),
                      stats.miss_merge.value_or(std::pair{type, cpu}, miss_merge_value_type{})));
    }

    lines.push_back(fmt::format("cpu{}->{} PREFETCH REQUESTED: {:10} ISSUED: {:10} USEFUL: {:10} USELESS: {:10}", cpu, stats.name, stats.pf_requested,
                                stats.pf_issued, stats.pf_useful, stats.pf_useless));

    uint64_t total_downstream_demands = total_fill - stats.fill.value_or(std::pair{access_type::PREFETCH, cpu}, fill_value_type{});
    lines.push_back(
        fmt::format("cpu{}->{} AVERAGE MISS LATENCY: {} cycles", cpu, stats.name, ::print_ratio(stats.total_miss_latency_cycles, total_downstream_demands)));
  }

  return lines;
}

std::vector<std::string> champsim::plain_printer::format(DRAM_CHANNEL::stats_type stats)
{
  std::vector<std::string> lines{};
  lines.push_back(fmt::format("{} RQ ROW_BUFFER_HIT: {:10}", stats.name, stats.RQ_ROW_BUFFER_HIT));
  lines.push_back(fmt::format("  ROW_BUFFER_MISS: {:10}", stats.RQ_ROW_BUFFER_MISS));
  lines.push_back(fmt::format("  AVG DBUS CONGESTED CYCLE: {}", ::print_ratio(stats.dbus_cycle_congested, stats.dbus_count_congested)));
  lines.push_back(fmt::format("{} WQ ROW_BUFFER_HIT: {:10}", stats.name, stats.WQ_ROW_BUFFER_HIT));
  lines.push_back(fmt::format("  ROW_BUFFER_MISS: {:10}", stats.WQ_ROW_BUFFER_MISS));
  lines.push_back(fmt::format("  FULL: {:10}", stats.WQ_FULL));

  if (stats.refresh_cycles > 0)
    lines.push_back(fmt::format("{} REFRESHES ISSUED: {:10}", stats.name, stats.refresh_cycles));
  else
    lines.push_back(fmt::format("{} REFRESHES ISSUED: -", stats.name));

  return lines;
}

void champsim::plain_printer::print(champsim::phase_stats& stats)
{
  auto lines = format(stats);
  std::copy(std::begin(lines), std::end(lines), std::ostream_iterator<std::string>(stream, "\n"));
}

std::vector<std::string> champsim::plain_printer::format(champsim::phase_stats& stats)
{
  std::vector<std::string> lines{};
  lines.push_back(fmt::format("=== {} ===", stats.name));

  int i = 0;
  for (auto tn : stats.trace_names) {
    lines.push_back(fmt::format("CPU {} runs {}", i++, tn));
  }

  if (NUM_CPUS > 1) {
    lines.emplace_back("");
    lines.emplace_back("Total Simulation Statistics (not including warmup)");

    for (const auto& stat : stats.sim_cpu_stats) {
      auto sublines = format(stat);
      lines.emplace_back("");
      std::move(std::begin(sublines), std::end(sublines), std::back_inserter(lines));
      lines.emplace_back("");
    }

    for (const auto& stat : stats.sim_cache_stats) {
      auto sublines = format(stat);
      std::move(std::begin(sublines), std::end(sublines), std::back_inserter(lines));
    }
  }

  lines.emplace_back("");
  lines.emplace_back("Region of Interest Statistics");

  for (const auto& stat : stats.roi_cpu_stats) {
    auto sublines = format(stat);
    lines.emplace_back("");
    std::move(std::begin(sublines), std::end(sublines), std::back_inserter(lines));
    lines.emplace_back("");
  }

  for (const auto& stat : stats.roi_cache_stats) {
    auto sublines = format(stat);
    std::move(std::begin(sublines), std::end(sublines), std::back_inserter(lines));
  }

  lines.emplace_back("");
  lines.emplace_back("DRAM Statistics");
  for (const auto& stat : stats.roi_dram_stats) {
    auto sublines = format(stat);
    lines.emplace_back("");
    std::move(std::begin(sublines), std::end(sublines), std::back_inserter(lines));
  }

  return lines;
}

void champsim::plain_printer::print(std::vector<phase_stats>& stats)
{
  for (auto p : stats) {
    print(p);
  }
}

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
  uint64_t chain_miss_inflight = 0;       // chain misses with the window's walk already in flight (late)
  uint64_t chain_miss_entry = 0;          // chain misses whose IP is a stored trace ENTRY (front door existed)
  uint64_t chain_miss_midtrace = 0;       // chain misses falling mid-trace in a stored window (trigger gap)
  uint64_t store_evictions = 0;           // trace-store evictions (any geometry)
  uint64_t store_conflict_evictions = 0;  // set-assoc evictions while the store was globally underfull (set skew)
  uint64_t store_occupancy = 0;           // trace-store valid entries at end of phase (level, not subtracted)
  uint64_t store_win_added = 0;           // distinct windows inserted with new traces
  uint64_t store_lookups = 0;            // trace-store entry probes
  uint64_t store_alias_hits = 0;         // probes matching a DIFFERENT trace's partial tag (trace_store_tag_bits)
  uint64_t store_alias_overwrites = 0;   // inserts replacing a different trace with the same partial tag
  uint64_t store_win_shared = 0;          // of those, already covered by another resident trace (fragmentation)
  uint64_t stall_traces = 0;              // distinct traces committed (passed the occurrence filter)
  std::array<uint64_t, 9> stall_starve_offset{}; // costly stretches by where the first starving instruction sits: window 0,1,2,3,4-7,8+, past recorded depth, before the stretch, at/after its closing hit
  uint64_t stall_lost_recaptures = 0;    // costly re-captures of a stretch the trace store had evicted
  uint64_t stall_filter_evictions = 0;   // trace_stall_filter_entries: candidates forgotten on table overflow
  uint64_t stall_readmits = 0;           // of those, re-inserted (trace_stall_readmit)
  uint64_t stall_mp_opened = 0;          // stretches opened by a mispredicted branch (trace_stall_mispredict_open)
  uint64_t stall_mp_traces = 0;          // of those, stored as traces
  uint64_t stall_split_segments = 0;     // trace_stall_split: continuation stretches opened (miss run past the trace depth)
  uint64_t stall_split_inherited = 0;    // ...that took the costly verdict of the segment before them
  uint64_t stall_split_traces = 0;       // ...stored as traces (also trace_run_format continuations)
  uint64_t run_cut_jump = 0;             // trace_run_format: stretches cut at a non-sequential window
  uint64_t run_cut_full = 0;             // ...cut because the run reached its window limit
  // u-op-cache miss-cost profile (end of run): windows ranked by the empty-ROB
  // cycles charged to their misses
  uint64_t mp_windows = 0;               // distinct windows that missed
  uint64_t mp_misses = 0;                // u-op-cache misses (ROI)
  uint64_t mp_cost = 0;                  // empty-ROB build-mode cycles charged to a missing window
  uint64_t mp_other = 0;                 // ...charged to nothing (oldest instruction hit / not looked up yet)
  std::array<uint64_t, 4> mp_cost_top{};   // cost held by the costliest 1K / 2K / 4K / 8K windows
  std::array<uint64_t, 4> mp_cost_freq{};  // cost held by the 1K / 2K / 4K / 8K most-missing windows
  std::array<uint64_t, 4> mp_traced_top{}; // of the costliest 1K..8K, cost in windows the segmenter's traces contain
  uint64_t mp_traced_all = 0;            // all cost in windows the segmenter's traces contain
  uint64_t starts_traces = 0;            // end of run: distinct start pcs of captured traces
  uint64_t starts_windows = 0;           // ...distinct 32B windows holding a start
  uint64_t starts_lines = 0;             // ...distinct 64B lines holding a start
  uint64_t starts_multi_windows = 0;     // ...windows holding two or more starts
  uint64_t starts_max_per_window = 0;    // ...most starts in one window
  uint64_t stall_lead_keyed = 0;         // stretches keyed by an earlier pc (trace_stall_trigger_lead)
  uint64_t stall_lead_self = 0;          // stretches with no pc that old: keyed by their own pc
  uint64_t stall_lead_traces = 0;        // stored traces keyed by an earlier pc
  uint64_t stall_lead_conflicts = 0;     // re-captures under a key whose stored stretch starts at a different pc
  std::array<uint64_t, 4> stall_lead_dist{}; // stored lead-keyed traces by key-to-first-pc distance: <=4KB, <=64KB, <=1MB, beyond
  uint64_t stall_marks_dropped = 0;       // tight-stall cycles whose costly-mark found no open stretch (close-to-drain race)
  uint64_t stall_event_traces = 0;        // mispredict-keyed traces committed (trace_stall_mispredict)
  uint64_t stall_event_candidates = 0;    // distinct mispredict-event keys seen (unfiltered)
  uint64_t stall_event_dedup = 0;         // re-captures of stored mispredict-keyed traces
  uint64_t event_probes = 0;              // resolution-time store probes (mispredict flushes)
  uint64_t event_probe_hits = 0;          // probes that found an event-keyed trace (walk launched)
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
  std::array<uint64_t, 3> stall_cost_cum_big{}; // cum stall cycles of the top 2K / 4K / 8K traces by cost
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
  uint64_t alt_pf_evicted_unused = 0; // walk-installed windows evicted never demand-hit (accuracy denominator with useful)
  uint64_t retain_marks = 0;          // uop_retain: u-op-cache lines marked costly
  uint64_t retain_diverted = 0;       // uop_retain: evictions moved off a costly LRU line
  uint64_t retain_costly_evicted = 0; // uop_retain: costly lines evicted anyway (set over quota)
  // stall taxonomy (PROMETHEUS_STALL_TAXONOMY): build-mode dispatch-starve cycles charged to the u-op-cache
  // miss dispatch waits for, index = recovery*10 + first_touch*5 + level (0 L1I, 1 L2, 2 LLC, 3 DRAM, 4 no fetch)
  std::array<uint64_t, 20> tax_tight{}; // ROB empty
  std::array<uint64_t, 20> tax_upper{}; // ROB has room
  uint64_t l2_perfect_hits = 0;          // l2_perfect_instr: instruction requests served by the L2 ceiling
  uint64_t l2_emissary_marks = 0;        // EMISSARY: L2 lines marked high-priority
  uint64_t l2_emissary_diverted = 0;     // ...evictions moved off the base policy's choice
  uint64_t l2_emissary_prot_evicted = 0; // ...high-priority lines evicted anyway
  uint64_t emissary_mark_events = 0;     // ...starving L1I misses selected for marking
  uint64_t fdip_pd_windows = 0;
  uint64_t fdip_pd_costly = 0;
  uint64_t fdip_pd_launched = 0;
  uint64_t fdip_pd_installed = 0;
  uint64_t fdip_pd_expired = 0;
  uint64_t fdip_pd_dropped = 0;
  uint64_t cwt_marks = 0;
  uint64_t rtt_captures = 0;
  uint64_t rtt_table_size = 0; // traces in the table at the end (not a delta)
  uint64_t rtt_opens = 0;
  std::array<uint64_t, 6> rec_mispredicts{};  // resolved mispredictions by class (O3_CPU::mispredict_class)
  std::array<uint64_t, 6> rec_idle_cycles{};  // empty-ROB u-op-miss recovery cycles by the class of the misprediction
  uint64_t rtt_fb_walks = 0;
  uint64_t rtt_fb_wp_walks = 0;
  uint64_t rtt_boost_hits = 0;
  uint64_t rtt_boost_walks = 0;
  uint64_t rtt_win_total = 0;          // RTT store census at the end (not a delta): windows held per trace, summed
  uint64_t rtt_win_distinct = 0;       // ...distinct windows
  uint64_t rtt_win_shared = 0;         // ...windows held by >= 2 traces
  uint64_t rtt_reconv_cut = 0;         // rtt_seg_reconv: recordings ended at reconvergence with the wrong path
  uint64_t rtt_repeat_held = 0;        // rtt_admit_repeat: first sightings not stored
  uint64_t rtt_cost_rescued = 0;       // rtt_cost_walk: walks allowed by a costly trace despite low confidence
  uint64_t rtt_cost_sum = 0;           // rtt_cost: sum of inserted cost buckets
  uint64_t rtt_cost_zero = 0;          // ...recordings whose recovery never emptied the ROB
  uint64_t rtt_seg_rerecords = 0;      // rtt_seg_stable: recordings of a key already stored
  uint64_t rtt_seg_trimmed = 0;        // ...cut where they left the stored path
  uint64_t rtt_seg_kept_instrs = 0;    // ...instructions kept by those re-recordings
  uint64_t rtt_cold_triggers = 0;      // rtt_cold_seq: no-trace taken H2P predictions that prefetched (wrong path included)
  uint64_t rtt_cold_wp_triggers = 0;
  uint64_t rtt_cold_lines = 0;         // ...L1I prefetches issued
  uint64_t rtt_cold_resident = 0;      // ...lines skipped (already in the L1I)
  uint64_t rtt_ret_hits = 0;
  uint64_t rtt_ret_walks = 0;
  uint64_t walk_pq_issued = 0;         // walk_pq: walk lines issued as L1I prefetches
  uint64_t walk_pq_full = 0;           // walk_pq: issue attempts that found the prefetch queue full or walk_pq lines in flight
  uint64_t walker_mshr_full = 0;
  uint64_t walker_mshr_peak = 0; // not a delta
  uint64_t walk_res_lines = 0;
  uint64_t walk_res_read = 0;
  uint64_t walk_res_evicted = 0;
  uint64_t walk_miss_lines = 0;
  uint64_t late_hol = 0;
  uint64_t late_decode = 0;
  uint64_t late_unissued = 0;
  uint64_t late_memory = 0;
  uint64_t rtt_admit_skipped = 0;
  uint64_t rtt_conf_suppressed = 0;
  uint64_t rtt_conf_up = 0;
  uint64_t rtt_conf_down = 0;
  uint64_t rtt_cancelled = 0;
  uint64_t rtt_cancel_lines = 0;
  uint64_t rtt_cancel_windows = 0;
  uint64_t walk_mshr_yield = 0;
  uint64_t rtt_dir_bits_out = 0;
  uint64_t rtt_dir_btb_miss = 0;
  uint64_t rtt_dir_target = 0;
  uint64_t rtt_dir_full = 0;
  uint64_t rtt_dir_bits_used = 0;
  uint64_t rtt_wp_lookups = 0;
  uint64_t rtt_wp_hits = 0;
  uint64_t rtt_wp_walks = 0;
  uint64_t rtt_truncated = 0;
  uint64_t rtt_unencodable = 0;
  uint64_t rtt_stored_instrs = 0;
  uint64_t rtt_alt = 0;
  uint64_t rtt_alt_nobtb = 0;
  uint64_t rtt_opens_wrong = 0;
  uint64_t rtt_filtered = 0;
  uint64_t rtt_cap_instrs = 0;
  uint64_t rtt_lookups = 0;
  uint64_t rtt_hits = 0;
  uint64_t rtt_walks = 0;
  uint64_t rtt_dg_lookups = 0;
  uint64_t rtt_dg_found = 0;
  uint64_t rtt_dg_prefix = 0;
  uint64_t rtt_dg_full = 0;
  uint64_t rtt_dg_real_win = 0;
  uint64_t rtt_dg_real_cov = 0;
  uint64_t rtt_dg_stored_win = 0;
  uint64_t rtt_dg_stored_use = 0;
  uint64_t rtt_dg_far = 0;
  uint64_t rtt_dg_far_cov = 0;
  uint64_t rtt_dg_far_first = 0;
  uint64_t rtt_dg_near = 0;
  uint64_t rtt_dg_near_cov = 0;
  uint64_t rtt_dg_near_first = 0;
  uint64_t oracle_recovery_triggers = 0; // PROMETHEUS_ORACLE_RECOVERY: wrong predictions seen at the cursor
  uint64_t oracle_recovery_walks = 0;    // ...real-path walks launched
  uint64_t fdip_filtered = 0;           // FDIP lines skipped because already in the L1I (ftq_fdip_filter)
  uint64_t rp_lookups = 0;               // resteer prefetcher: new lines the predictor reached
  uint64_t rp_hits = 0;                  // ...that were filed triggers
  uint64_t rp_prefetches = 0;            // ...L1I prefetches issued
  uint64_t rp_dropped = 0;               // ...dropped (L1I prefetch queue full)
  uint64_t rp_records = 0;               // ...targets / paths filed
  uint64_t uop_incl_invalidations = 0; // PROMETHEUS_UOP_INCLUSIVE: windows dropped because the L1I evicted their line
  std::array<uint64_t, 4> tax_other{};  // [0/1] tight/upper: oldest not looked up yet (redirect bubble); [2/3]: oldest is a u-op hit
  uint64_t alt_wait_cycles = 0;       // fetch-stall cycles waiting for a walk-pending window (hit-under-fill)
  uint64_t alt_lines_issued = 0;      // real-L1I mode: cache-line reads the walks issued through the L1I
  uint64_t alt_line_stalls = 0;       // real-L1I mode: walk-cycles stalled waiting for a line to arrive
  uint64_t alt_walk_aborts = 0;       // watchdog: walks aborted after making no progress (lost line response)
  // CHAIN fill mode:
  uint64_t chain_unencodable = 0;       // committed traces rejected: a window delta overflows 16 bits (>±1MB)
  uint64_t chain_truncated = 0;         // committed traces cut to meta_windows window slots
  uint64_t chain_buf_hits = 0;          // demand hits served from the trace-uop buffer (then promoted)
  uint64_t chain_buf_evict_unused = 0;  // buffer windows evicted without ever being demanded (walk overshoot)
  uint64_t chain_filtered_windows = 0;  // windows skipped at walk launch (already u-op-cache-resident or staged)
  uint64_t chain_buf_rehits = 0;        // staging-buffer hits on an entry that already served a hit (trace_buffer_promote = 0)
  // deep probe past a mispredicted branch (correct-path PCs a real FTQ would not hold)
  uint64_t chain_shadow_mispredicts = 0; // mispredictions at prediction time
  uint64_t chain_shadow_walks = 0;       // walks probed past one (flagged, or squashed with trace_probe_squash 1)
  uint64_t chain_shadow_lines = 0;       // line requests those walks issued (L1I side, never undone)
  uint64_t chain_shadow_windows = 0;     // windows they staged (flagged, or squashed)
  uint64_t chain_shadow_hits = 0;        // buffer hits served by flagged windows = oracle benefit (0 when squashing)
  // trace_wrong_path: the frontend past a mispredicted branch
  uint64_t wp_cursor_stops = 0;          // level >= 1: branches the predictor got wrong at the cursor (cursor waits for resolution)
  uint64_t wp_shadows = 0;               // level >= 2: mispredictions whose wrong path was generated
  uint64_t wp_addresses = 0;             // wrong-path addresses the predictor generated
  uint64_t wp_fetched = 0;               // ...fetched before resolution
  uint64_t wp_uop_misses = 0;            // ...that missed the u-op cache (and staging buffer)
  uint64_t wp_fetch_cycles = 0;          // cycles fetch spent on a wrong path
  uint64_t wp_decoder_busy_cycles = 0;   // ...in which the wrong path held the decoders (no walk pre-decode)
  uint64_t wp_store_hits = 0;            // level 3: trace-store hits on wrong-path PCs
  uint64_t wp_ucp_triggers = 0;          // level 3: UCP alternate paths generated from wrong-path H2P branches
  uint64_t wp_walks = 0;                 // walks launched from the wrong path
  uint64_t wp_walks_flushed = 0;         // ...still in flight at resolution (flushed)
  uint64_t wp_lines = 0;                 // line requests wrong-path walks issued
  uint64_t wp_staged = 0;                // windows wrong-path walks staged (chain buffer)
  uint64_t wp_buffer_hits = 0;           // demand buffer hits served by those windows
  uint64_t wp_l1i_lines = 0;             // level 3: L1I line requests made by wrong-path fetch
  uint64_t wp_line_stall_cycles = 0;     // level 3: wrong-path fetch cycles waiting on a line in flight
  uint64_t wp_full_cycles = 0;           // level 3: wrong-path fetch cycles stopped by a full backend
  uint64_t wp_stops = 0;                 // ucp2: wrong paths that ended early (code never executed / unknown target)
  uint64_t wp_uop_fills = 0;             // ucp2: wrong-path instructions decoded into the u-op cache
  uint64_t peek_probes = 0;              // trace_oracle_peek: real-path probes past a stopping misprediction
  uint64_t peek_store_hits = 0;          // ...that hit a stored trace (walk launched unless resident/in flight)
  uint64_t pdiag_mispredicts = 0;        // trace_peek_diag: mispredictions seen at prediction
  uint64_t pdiag_with_trace = 0;         // ...followed by a stored trace within the truth window
  std::array<uint64_t, 8> pdiag_fires{};  // per key: predictions where the table named a trace
  std::array<uint64_t, 8> pdiag_useful{}; // ...at a misprediction, and it was the trace that followed
  std::array<uint64_t, 8> pdiag_wrong{};  // ...at a misprediction, and another trace (or none) followed
  // trace_trigger_diag (class indices: O3_CPU::TD_*, probe outcomes: O3_CPU::TRIG_*)
  std::array<uint64_t, 6> td_probes{};       // cursor probes that hit a stored trace entry, by outcome
  uint64_t td_arrivals = 0;                  // demand reached a stored trace's entry pc
  uint64_t td_arr_resident = 0;              // ...its window already in the u-op cache
  uint64_t td_arr_staged = 0;                // ...served from the staging buffer (prefetched in time)
  std::array<uint64_t, 15> td_arr_miss{};    // ...missed, by class
  std::array<uint64_t, 6> td_lead_served{};  // walked entries served: probe->arrival cycles <=8, <=16, <=32, <=64, <=128, >128
  std::array<uint64_t, 6> td_lead_missed{};  // walked entries that still missed, same buckets
  std::array<uint64_t, 15> td_miss{};        // u-op-cache misses by class
  std::array<uint64_t, 15> td_miss_rec{};    // ...of them in recovery (after a misprediction, before the first hit)
  std::array<uint64_t, 15> td_head_tight{};  // empty-ROB cycles charged to the class of the waited-for miss run's FIRST miss
  std::array<uint64_t, 15> td_head_upper{};  // dispatch-starve cycles (ROB not full), likewise
  std::array<uint64_t, 15> td_cost_tight{};  // empty-ROB build-mode cycles charged to the miss dispatch waits for, by class
  std::array<uint64_t, 15> td_cost_upper{};  // dispatch-starve cycles with the ROB not full, charged likewise
  uint64_t td_cost_tight_other = 0;          // ...cycles whose waited-for instruction hit (or none in the front end)
  uint64_t fdip_issued = 0;                  // ftq_fdip: L1I line prefetches the cursor issued
  uint64_t fdip_dropped = 0;                 // ...refused (L1I prefetch queue full)
  uint64_t fdip_wp_issued = 0;               // ...issued for wrong-path blocks (trace_wrong_path ucp2)
  uint64_t fup_walks = 0;                    // ftq_uop_prefetch: single-window walks launched at the cursor
  uint64_t td_staged = 0;                    // windows staged into the buffer
  uint64_t td_staged_late = 0;               // ...after demand had already missed them (since the walk launched)
  uint64_t td_unused_late = 0;               // evicted unused: staged late (demand had built them)
  uint64_t td_unused_ontime = 0;             // evicted unused: staged before demand, which never came while staged
  std::array<uint64_t, 5> td_pos_used{};     // staged windows demanded, by position in their walk (0,1,2,3,4+)
  std::array<uint64_t, 5> td_pos_unused{};   // staged windows evicted unused, same positions
  uint64_t td_cost_upper_other = 0;
  uint64_t rk_probes = 0;        // trace_recovery_key: predictions that probed a recovery key
  uint64_t rk_hits = 0;          // ...and found a recovery trace
  uint64_t rk_walks = 0;         // ...whose walk launched (not resident / in flight / dropped)
  uint64_t rk_mispredicts = 0;   // mispredictions seen (each arms one filing)
  uint64_t rk_filed = 0;         // ...followed by a stored trace within 192 instructions: filed under the recovery key
  uint64_t rk_buffer_hits = 0;   // demand hits on windows first staged by a recovery-key walk
  std::array<uint64_t, 4> succ_addr_steady{}; // consecutive costly stretches A->B: B's start vs A's last pc: next instr (<=16B) / near (<=256B) / same 4KB / jump
  std::array<uint64_t, 4> succ_addr_mp{};     // ...when a misprediction came in between
  std::array<uint64_t, 5> succ_gap_steady{};  // ...instructions between them: 0 / 1-8 / 9-64 / 65-256 / more
  std::array<uint64_t, 5> succ_gap_mp{};
  uint64_t chain_links = 0;      // trace_chain_depth: successor pointers written (previous costly trace was stored)
  uint64_t chain_hops = 0;       // successor hops attempted after a trace hit
  uint64_t chain_found = 0;      // ...whose successor was known and stored
  std::array<uint64_t, 4> chain_walks_hop{};  // successor walks launched, by hop 1 / 2 / 3 / 4+
  std::array<uint64_t, 4> chain_bhits_hop{};  // demand hits on windows they staged, by hop
  uint64_t vp_lookups = 0;       // trace_vpred: predictions looked up
  uint64_t vp_matches = 0;       // ...a table matched
  uint64_t vp_fires = 0;         // ...with enough confidence
  uint64_t vp_store_hits = 0;    // ...and the predicted trace was in the store
  uint64_t vp_walks = 0;         // ...whose walk launched
  uint64_t vp_walks_mp = 0;      // ...of which fired at a misprediction
  uint64_t vp_buffer_hits = 0;   // demand hits on windows first staged by a value-predictor walk
  uint64_t vp_buffer_hits_mp = 0; // ...from walks fired at a misprediction
  uint64_t vp_train = 0;         // costly stretches that trained a prediction
  uint64_t vp_train_mp = 0;      // ...whose prediction was a misprediction (recovery)
  uint64_t vp_correct = 0;       // ...and the provider had named that trace
  uint64_t vp_correct_mp = 0;
  uint64_t vp_wrong = 0;         // ...the provider named another trace
  uint64_t vp_noprov = 0;        // ...no table matched
  uint64_t vp_none = 0;          // matched predictions no costly stretch followed
  uint64_t vp_allocs = 0;
  uint64_t vp_alloc_fail = 0;
  uint64_t chain_conf_suppressed = 0;   // walk launches withheld by the per-trace launch confidence (trace_launch_conf)
  uint64_t chain_delta_seq = 0;   // manifest transfers by kind: fall-through (+1 window)
  uint64_t chain_delta_short = 0; // ... taken, |delta| <= 128 windows (8-bit)
  uint64_t chain_delta_long = 0;  // ... taken, needs the 16-bit delta
  uint64_t chain_delta_far = 0;   // ... beyond the widest encodable delta: trace rejected (or truncated, below)
  uint64_t chain_far_truncated = 0; // traces cut at an unencodable jump, prefix kept (trace_meta_far_truncate)
  std::array<uint64_t, 6> chain_need_bits{};    // captured stretches by packed bits needed to store WHOLE: <=16, <=24, <=32, <=48, <=64, >64
  std::array<uint64_t, 5> chain_need_windows{}; // ...by windows spanned: <=2, <=3, <=4, <=6, >6 (depth-capped at capture)
  uint64_t chain_trunc_windows = 0;             // windows lost to manifest truncation (sum over truncated traces)
  std::array<uint64_t, 18> chain_delta_bits{};  // taken transfers by signed delta bits needed; [3] = <=3, [4..16] exact, [17] = >16
  std::array<uint64_t, 6> chain_reach_xfer{};   // taken transfers by |delta|: <=4KB(8b), <=128KB(13b), <=256KB(14b), <=512KB(15b), <=1MB(16b), beyond
  std::array<uint64_t, 6> chain_reach_trace{};  // captured stretches by their FARTHEST transfer (same buckets): the delta width each needs
  uint64_t chain_reach_none = 0;                // captured stretches with no taken transfer at all (sequential only / single window)
  uint64_t chain_trace_single = 0;     // stored manifests by hardest transfer: entry window only
  uint64_t chain_trace_seq_only = 0;   // ... fall-through windows only
  uint64_t chain_trace_with_short = 0; // ... at least one short (8-bit) transfer, no long
  uint64_t chain_trace_with_long = 0;  // ... at least one long (16-bit) transfer
  uint64_t chain_evicted_then_missed = 0;     // demand misses on windows the staging buffer had evicted unused
  std::array<uint64_t, 5> chain_evicted_age{}; // ...by eviction->miss age: <=256, <=1K, <=4K, <=16K, >16K cycles (short = FIFO-capacity opportunity, long = next recurrence)
  std::array<uint64_t, 6> chain_slack{}; // probe->first-demand-touch cycles: <=4, <=8, <=16, <=32, <=64, >64
  // UCP port (alternate-path u-op prefetching on H2P conditionals):
  uint64_t ucp_cond_seen = 0;          // conditional branches observed (H2P detector denominator base)
  uint64_t ucp_cond_misses = 0;        // conditional mispredictions (coverage denominator)
  uint64_t ucp_h2p_marked = 0;         // conditionals classified hard-to-predict (accuracy denominator)
  uint64_t ucp_h2p_marked_misses = 0;  // H2P-marked conditionals that actually mispredicted
  uint64_t ucp_paths = 0;              // alternate paths generated (post-dedup/residency, pre-capacity)
  uint64_t ucp_stop_sat = 0;           // walks ended by the weighted stop counter (>= threshold)
  uint64_t ucp_stop_ind = 0;           // walks ended at an indirect branch (no Alt-Ind flavor)
  uint64_t ucp_stop_btbmiss = 0;       // walks ended by a predicted-taken branch with no BTB target
  uint64_t ucp_stop_maxip = 0;         // walks ended by the straight-line run limit
  uint64_t ucp_ind_walked = 0;         // indirect branches walked through on a predicted target (Alt-Ind flavor)
  uint64_t cold_dist[2][6] = {};       // cold_diag: [seen, unseen] x line distance from the redirect target (0, 1, 2-3, 4-7, 8+, behind)
  uint64_t cold_class[2][6] = {};      // cold_diag: [seen, unseen] x mispredict class (mispredict_class)
  uint64_t uop_split_hits[4] = {};     // uop_ideal_split: misses served as hits (steady/resident, steady/missing, recovery/resident, recovery/missing)
  uint64_t ucp_art_dropped = 0;        // trace_ucp_artifact 1: windows dropped unissued by a newer trigger
  uint64_t ucp_art_merged = 0;         // trace_ucp_artifact 1: windows already held by an in-flight path (uop-MSHR merge)
  uint64_t ucp_art_pf_issued = 0;      // trace_ucp_artifact 2: L1I prefetches issued
  uint64_t ucp_art_pf_full = 0;        // trace_ucp_artifact 2: cycles a line waited on a full L1I PQ or 32 lines in flight
  uint64_t ucp_bank_conflicts = 0;     // walk installs stalled by BTB bank conflicts (trace_ucp_btb_banks)
  uint64_t fe_stall_steady = 0;    // build-mode dispatch-starvation cycles, correct path (upper bound)
  uint64_t fe_stall_recovery = 0;  // build-mode dispatch-starvation cycles, post-misprediction (upper bound)
  uint64_t fetch_l1i_miss_steady = 0;   // demand fetch lines that returned slower than an L1I hit, correct path
  uint64_t fetch_l1i_miss_recovery = 0; // same, while recovering from a misprediction
  uint64_t fetch_l1i_wait_steady = 0;   // their summed issue-to-return cycles (not overlap-aware)
  uint64_t fetch_l1i_wait_recovery = 0;
  uint64_t rob_idle_steady = 0;    // build-mode cycles with ROB fully empty, correct path (tight lower bound)
  uint64_t rob_idle_recovery = 0;  // build-mode cycles with ROB fully empty, post-misprediction (tight lower bound)
  uint64_t fe_lost_slots_steady = 0;   // build-mode dispatch SLOTS lost while ROB had room, correct path (top-down middle bound)
  uint64_t fe_lost_slots_recovery = 0; // build-mode dispatch SLOTS lost while ROB had room, post-misprediction

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

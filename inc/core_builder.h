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

#ifndef CORE_BUILDER_H
#define CORE_BUILDER_H

#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include "chrono.h"

class CACHE;
class O3_CPU;
namespace champsim
{
class channel;
template <typename...>
class core_builder_module_type_holder
{
};
namespace detail
{
struct core_builder_base {
  uint32_t m_cpu{};
  champsim::chrono::picoseconds m_clock_period{250};
  std::size_t m_dib_set{1};
  std::size_t m_dib_way{1};
  std::size_t m_dib_window{1};
  int m_dib_ideal{0}; // ideal u-op cache mode: 0=off, 1=oracle (always hit), 2=cold-miss (infinite capacity)
  std::string m_trace_builder{"both"}; // Prometheus trace builder: both/backward/forward/off
  std::string m_trace_fill{"off"};     // Prometheus trace-fill mode: off/miss/every/window
  int m_trace_stall_min_occ{1};        // stall segmenter: min occurrences to capture a trace (1 = no filter)
  int m_trace_stall_rob{0};            // stall segmenter: ROB-occupancy trigger threshold (0 = fully empty)
  int m_trace_stall_readmit{0};        // stall segmenter: re-insert evicted traces on re-capture, 0 = never (legacy), 1 = strict, 2 = immediate
  int m_trace_stall_filter_entries{0}; // stall segmenter: candidate table entries (stretch starts below min_occ + re-qualifying evicted traces); 0 = unbounded
  int m_trace_stall_mispredict_open{0}; // stall segmenter: 1 = a mispredicted branch opens a stretch (like a u-op-cache miss)
  int m_trace_stall_split{0};            // stall segmenter: split miss runs longer than the trace depth into consecutive traces
  int m_trace_run_format{0};             // traces are runs of sequential windows, cut at jumps and chained
  int m_trace_run_max_windows{8};        // ...windows per run (the count field: 8 = 3 bits)
  long m_trace_stall_trigger_lead{0};   // stall segmenter: key stretches by the pc checked this many cycles earlier (0 = own pc)
  int m_trace_stall_attrib{0};         // stall segmenter: costly-mark attribution, 0 = open stretch at the lookup, 1 = dispatch-side
  int m_trace_stall_depth{64};         // stall segmenter: max trace length in u-ops (truncation cap)
  int m_trace_store{256};              // trace-cache capacity in traces (bounded fill modes)
  int m_trace_store_sets{0};           // trace store: sets (ways>0: geometry = sets x ways; ways==0: capacity override, 0 = use trace_store)
  int m_trace_store_ways{0};           // trace store: associativity (0 = fully associative, the legacy design)
  int m_trace_store_tag_bits{0};       // trace store: partial-tag width in bits (0 = full entry-PC match)
  int m_trace_store_hash{0};           // trace store set index: 0 = plain modulo, 1 = xor-fold of higher PC bits
  int m_trace_store_cost_decay{0};     // cost_evict aging: halve costs every N insertions (0 = off)
  int m_trace_store_cost_bits{0};      // cost counter width in bits, saturating (0 = unbounded idealized)
  int m_trace_walk_delay{5};           // alt fill: trigger-to-first-install latency in cycles
  int m_trace_walk_max{2};             // alt fill: max concurrent walks
  int m_trace_walk_width{1};           // alt fill: windows installed per walk per cycle
  int m_trace_walk_wait{0};            // alt fill: 1 = miss on a walk-pending window stalls fetch instead of switching to build
  int m_trace_walk_l1i{0};             // alt fill: 1 = walk fetches bytes through the real L1I (installs gated on line arrival)
  int m_trace_walk_wait_cap{16};       // alt fill: max cycles an instruction waits on a pending window (0 = unbounded)
  int m_trace_stall_l1i_gate{0};       // stall segmenter: 1 = only commit stretches that also missed L1I
  int m_trace_stall_mispredict{0};     // stall segmenter: 1 = mispredict-keyed capture + resolution-time probe
  int m_ftq_depth{0};                  // instruction-supply lookahead depth (FTQ model); 0 = legacy 2 x fetch_width
  int m_ftq_lookup_window{0};          // decoupled front end: u-op-cache lookup only within the first N FTQ entries (0 = legacy: at FTQ insertion)
  int m_bpu_blocks{1};                 // decoupled front end: fetch blocks the branch predictor inserts into the FTQ per cycle
  int m_fetch_buffer_blocks{0};        // fetch: at most N fetch blocks in flight in the fetch (IFETCH) buffer (0 = instruction limit only)
  int m_ftq_uop_prefetch{0};           // CHAIN: walk every window the predictor puts in the FTQ that the u-op cache lacks (u-op FDIP)
  int m_trace_chain_parallel{0};       // CHAIN ceiling: a miss in a stored trace's window is served instantly (pair with trace_walk_max 0)
  int m_ftq_fdip_filter{0};            // FDIP skips lines already resident in the L1I
  int m_trace_walk_nofill{0};          // SUCP walk line reads allocate in no cache level
  int m_fdip_predecode{0};             // FDIP-triggered pre-decode of costly windows (1 SUCP-store windows, 2 costly-window table, 3 = 1 + SUCP walks)
  int m_fdip_predecode_target{0};      // 0 = u-op cache, 1 = staging buffer
  int m_cwt_entries{4096};             // costly-window table entries (mode 2)
  int m_uop_cost_mode{0};              // costly-window oracle: 1 profile pass, 2 oracle pass
  long m_uop_cost_windows{75};         // oracle: top-N windows (75 ~ 5 KB)
  int m_uop_ideal_split{0};            // oracle: serve u-op misses of the selected kinds as hits (bit mask, see ooo_cpu.h)
  long m_uop_ideal_split_window{64};   // ...recovery = this many instructions after a misprediction
  int m_rtt_mode{0};                   // recovery-trace table trigger (0 off, 1 oracle, 2 every prediction, 3 low-confidence conditionals)
  int m_rtt_capture{0};                // 0 = first u-op miss to next u-op hit, 1 = fixed length
  int m_rtt_len{128};                  // max instructions per recovery trace
  int m_rtt_open{0};                   // recording opens at: 0 every misprediction, 1 low-confidence conditionals + mispredicted others
  int m_rtt_wrong_path{0};             // modes 2/3: also look up at wrong-path predictions
  int m_rtt_max_windows{0};            // >0: stored traces keep their first N windows
  int m_rtt_format{0};                 // rtt_table 1: 0 window manifest, 1 direction bits
  int m_rtt_cancel{0};                 // cancel recovery walks of correctly resolved branches
  int m_walk_mshr_limit{0};            // walks issue only below this L1I MSHR occupancy (0 = off)
  int m_rtt_conf_min{0};               // walk only if the trace entry's confidence >= this (0 = off)
  int m_rtt_admit_miss{0};             // store only recordings that missed the u-op cache
  int m_rtt_conf_evict{0};             // usefulness-trained counters, evict conf-0 first
  int m_walk_prio_head{0};             // first N lines of a walk ignore walk_mshr_limit
  int m_walk_ooo{0};                   // walks install windows out of order as lines arrive
  int m_walk_resident_port{0};         // walks read L1I-resident lines via the idle demand port (2 = instant)
  int m_walk_l2_mshrs{0};              // walker MSHRs for walk line misses (0 = off)
  int m_walk_pq{0};                    // recovery-walk lines as L1I prefetches, at most this many in flight (0 = off)
  int m_rtt_boost{0};                  // confident conditionals walk if the entry counter >= this (0 = off)
  int m_rtt_cold_seq{0};               // no-trace taken H2P conditional: prefetch N fall-through lines
  int m_rtt_cold_mode{0};
  int m_rtt_cold_target{0};
  int m_rtt_cold_l2{0};                // cold prefetch fills the L2 only            // also no-trace H2P predicted not taken: prefetch N lines from the BTB target              // 0 no trace / 1 + never fell through / 2 + window never looked up (oracle)
  int m_rtt_l2_keep{0};                // useful recovery-window lines get the EMISSARY L2 priority bit
  int m_rtt_conf_table{0};
  int m_rtt_admit_repeat{0};           // second-sighting admission filter entries (0 = off)             // shared walk-confidence table entries (0 = a counter per trace entry)
  int m_rtt_seg_reconv{0};             // recordings end where the recovery path rejoins the wrong path
  int m_rtt_cost{0};                   // traces carry their recovery's empty-ROB cost (store cost field)
  int m_rtt_cost_walk{0};              // cost bucket >= this walks despite rtt_conf_min (0 = off)
  int m_rtt_key_target{0};             // conditional traces keyed by the recovery target only (shared across branches)
  int m_rtt_seg_stable{0};             // re-recording keeps only the prefix that repeats
  int m_rtt_ret{0};                    // returns record/walk recovery traces; walk if the entry counter >= this (0 = off)
  int m_rtt_fallback{0};               // walk a generated alternate path when no trace exists
  int m_rtt_gen_cond{0};               // conditionals always walk the generated path; table for indirects
  int m_rtt_table{0};                  // 0 unbounded, 1 the SUCP trace store
  int m_rtt_max_open{4};               // recordings open at once
  int m_rtt_store_filter{0};           // 0 store all, 1 only if the ROB went empty on the recording, 2 ...on a u-op miss in it
  int m_ftq_fdip_uop_check{1};         // FDIP skips windows the u-op cache holds (0 = L1I-only FDIP)
  int m_ftq_shared_wp{0};              // wrong path shares the FTQ with the queued correct path (implied by fetch_buffer_blocks > 0)
  int m_trace_stall_taxonomy{0};       // diagnostic: charge u-op-miss starve cycles by recovery / first touch / serving level
  int m_l2_perfect_instr{0};           // ceiling: 1 = re-requested instruction lines always hit L2, 2 = all instruction lines
  int m_l2_emissary_ways{0};           // EMISSARY P(N) at L2 (0 = off)
  int m_l2_emissary_rand{32};          // EMISSARY marking probability 1/r
  int m_rp_mode{0};                    // resteer prefetcher: 1 = PDIP-style, 2 = SUCP path lines
  int m_rp_sets{512};
  int m_rp_ways{8};
  int m_rp_targets{2};
  int m_rp_window{64};
  int m_ftq_fdip{0};                   // decoupled front end: L1I prefetch of each block the probe cursor predicts (FDIP from the FTQ)
  int m_trace_store_cost_evict{0};     // trace store: 1 = evict minimum-stall-cost trace instead of LRU
  int m_trace_head_uops{8};            // head fill: uops stored per trace head (served instantly)
  int m_trace_tail_targets{3};         // head fill: taken targets in the tail manifest (<0 = unlimited)
  int m_trace_uop_buffer{16};          // chain fill: trace-uop staging buffer capacity in windows (0 = install direct)
  int m_trace_meta_windows{4};         // chain fill: manifest window slots per trace (entry + 16-bit deltas)
  int m_trace_meta_bits{0};            // chain fill: packed manifest slot-field width in bits (0 = legacy fixed 16-bit slots, meta_windows cap)
  int m_trace_meta_short_bits{8};      // chain fill (packed): signed delta width of a short-jump record (record = 2 + this)
  int m_trace_meta_long_bits{16};      // chain fill (packed): signed delta width of a long-jump record (0 = no long class)
  int m_trace_meta_kind_bits{2};       // chain fill (packed): kind-code width, 2 = seq/short/long/end, 1 = seq/jump (no long class)
  int m_trace_meta_far_truncate{0};    // chain fill: 1 = an unencodable jump ends the trace (prefix kept) instead of rejecting it
  int m_trace_probe_ahead{0};          // chain fill: deep-probe lead in instructions ahead of enqueue (0 = enqueue only)
  int m_trace_probe_blocks{1};         // chain fill: fetch blocks the deep-probe cursor (the predictor) covers per cycle
  int m_trace_probe_squash{0};         // chain fill: 1 = squash walks probed past a mispredicted branch, cursor frozen until resolution (0 = legacy, flag only)
  std::string m_trace_wrong_path{"0"}; // frontend past a mispredict: 0 legacy, 1 cursor stops, ucp = 1 + UCP_ISCA24 fetch-block rule, 2 + wrong-path contention, 3 + wrong-path prefetch
  int m_trace_oracle_peek{0};          // diagnostic ceiling: walks from the real path up to N instrs past a cursor-stopping mispredict (0 = off)
  int m_trace_peek_diag{0};            // diagnostic: next-trace predictability at mispredicts from PC / GHR / trace-history keys (0 = off)
  int m_trace_trigger_diag{0};         // diagnostic: why stored traces did not serve their misses, cost-weighted (0 = off)
  int m_trace_recovery_key{0};         // recovery key: the trace that followed a misprediction, filed under (branch, predicted direction, GHR)
  int m_trace_recovery_ghr{0};         // ...GHR bits in that key (0-64)
  int m_trace_recovery_h2p{0};         // ...probe only predictions flagged hard-to-predict
  int m_trace_vpred{0};                // trace value predictor (TAGE-organized; value = next costly trace)
  int m_trace_vpred_tables{4};         // ...tagged tables above the base table
  int m_trace_vpred_entries{512};      // ...entries per table
  int m_trace_vpred_max_hist{64};      // ...longest global history (bits, <= 64)
  int m_trace_vpred_tag_bits{12};      // ...tag width
  int m_trace_vpred_conf{0};           // ...confidence needed to walk (0 = any tag match)
  int m_trace_chain_depth{0};          // trace chaining: successor hops walked after a trace hit (0 = off)
  int m_trace_chain_window{0};         // ...max instructions between two costly traces to link them (0 = any)
  int m_trace_walk_filter{0};          // chain fill: 1 = stage only windows absent from the u-op cache and staging buffer
  int m_trace_buffer_promote{1};       // chain fill: staging-buffer hit fills the u-op cache at MRU (1), not at all (0), or at LRU (2)
  int m_trace_launch_conf{0};          // chain fill: 1 = 2-bit per-trace launch confidence gates walks; 2 = also evicts conf-0 traces first
  int m_trace_ucp{0};                  // UCP port: 1 = alternate-path u-op prefetching on H2P conditionals (ISCA'24)
  int m_trace_ucp_threshold{500};      // UCP: Table-I weighted stop-counter threshold (paper H2P_T)
  int m_trace_ucp_max_ip{64};          // UCP: straight-line steps without a branch before the walk stops
  int m_trace_ucp_step{4};             // UCP: alt-path instruction stride in bytes (ARM traces: 4)
  int m_trace_ucp_altind{0};           // UCP: 1 = 4KB Alt-ITTAGE, walks continue through indirects (12.95KB flavor)
  int m_trace_ucp_artifact{0};         // UCP: UCP_ISCA24 semantics bit mask: 1 single path, 2 L1I-prefetch lines, 4 dedicated decoders (7 = all)
  int m_trace_ucp_btb_banks{0};        // UCP: >0 = model N BTB banks (walk installs stall on demand bank conflicts); 0 = ideal banking
  int m_trace_warmup_train{0};         // 1 = chain/UCP train (and prefetch) during warmup, like the UCP_ISCA24 artifact; 0 = cold-start at ROI
  int m_trace_walk_issue_cap{0};       // walks: global line-issues per cycle across all walks (0 = unlimited legacy schema)
  int m_trace_walk_decode_cap{0};      // walks: global window-installs (pre-decodes) per cycle (0 = unlimited legacy schema)
  int m_trace_walk_decode_shared{0};   // walks: 1 = installs only when demand is in stream mode (shared decoders, no dedicated hw); 2 = shared by age: walks use the decode width demand left this cycle
  std::size_t m_ifetch_buffer_size{1};
  std::size_t m_decode_buffer_size{1};
  std::size_t m_dispatch_buffer_size{1};

  std::size_t m_dib_hit_buffer_size{1};

  std::size_t m_register_file_size{1};
  std::size_t m_rob_size{1};
  std::size_t m_lq_size{1};
  std::size_t m_sq_size{1};

  champsim::bandwidth::maximum_type m_fetch_width{1};
  champsim::bandwidth::maximum_type m_decode_width{1};
  champsim::bandwidth::maximum_type m_dispatch_width{1};
  champsim::bandwidth::maximum_type m_schedule_width{1};
  champsim::bandwidth::maximum_type m_execute_width{1};
  champsim::bandwidth::maximum_type m_lq_width{1};
  champsim::bandwidth::maximum_type m_sq_width{1};
  champsim::bandwidth::maximum_type m_retire_width{1};
  champsim::bandwidth::maximum_type m_dib_inorder_width{1};

  unsigned m_dib_hit_latency{};

  unsigned m_mispredict_penalty{};
  unsigned m_decode_latency{};
  unsigned m_dispatch_latency{};
  unsigned m_schedule_latency{};
  unsigned m_execute_latency{};

  CACHE* m_l1i{};
  champsim::bandwidth::maximum_type m_l1i_bw{1};
  champsim::bandwidth::maximum_type m_l1d_bw{1};
  champsim::channel* m_fetch_queues{};
  champsim::channel* m_data_queues{};
};
} // namespace detail

template <typename B = core_builder_module_type_holder<>, typename T = core_builder_module_type_holder<>>
class core_builder : public detail::core_builder_base
{
  using self_type = core_builder<B, T>;

  friend class ::O3_CPU;

  template <typename OTHER_B, typename OTHER_T>
  friend class core_builder;

  explicit core_builder(const detail::core_builder_base& other) : detail::core_builder_base(other) {}

public:
  core_builder() = default;

  self_type& index(uint32_t cpu_);

  /**
   * Specify the core's clock period.
   */
  self_type& clock_period(champsim::chrono::picoseconds clock_period_);

  /**
   * Specify the number of sets in the Decoded Instruction Buffer.
   */
  self_type& dib_set(std::size_t dib_set_);

  /**
   * Specify the number of ways in the Decoded Instruction Buffer.
   */
  self_type& dib_way(std::size_t dib_way_);

  /**
   * Specify the size of the window within which Decoded Instruction Buffer entries are equivalent.
   */
  self_type& dib_window(std::size_t dib_window_);

  /**
   * Decoded Instruction Buffer (u-op cache) ideal mode: 0=off, 1=oracle (every
   * lookup hits), 2=cold-miss (infinite capacity; first access to a window misses,
   * then always hits).
   */
  self_type& dib_ideal(int dib_ideal_);

  /**
   * Select which Prometheus trace builder(s) observe the post-merge u-op
   * stream: "both" (default), "backward"/"seg", "forward"/"rec", or "off".
   */
  self_type& trace_builder(std::string trace_builder_);

  /**
   * Trace-fill mode: install stored traces into the u-op cache.  "off" disables;
   * "miss" fills on a miss at a trace entry; "every" fills on hit-or-miss at a
   * trace entry; "window" fills on a miss anywhere in a trace's footprint.  Any
   * non-off mode forces the staging builder on to feed the trace cache.
   */
  self_type& trace_fill(std::string trace_fill_);

  /**
   * Stall segmenter occurrence filter: only capture a stall trace once its start
   * IP has recurred at least this many times (1 = no filtering).
   */
  self_type& trace_stall_min_occ(int trace_stall_min_occ_);

  /**
   * Stall segmenter trigger threshold: a build-mode dispatch-starve cycle marks the
   * stretch costly when ROB occupancy <= this value (0 = fully empty; higher captures
   * partial/near-empty stalls).
   */
  self_type& trace_stall_rob(int trace_stall_rob_);
  /**
   * Stall segmenter: which stretch a ROB-empty dispatch stall charges.  0 (default)
   * = the stretch open at the u-op-cache lookup; 1 = the stretch containing the
   * instruction dispatch is waiting for (recorder at dispatch, 1-bit miss tag).
   */
  self_type& trace_stall_attrib(int trace_stall_attrib_);
  /**
   * Stall segmenter: re-admission of traces the store evicted.  0 (default) = legacy,
   * never re-inserted; 1 = strict, an evicted stretch re-qualifies through min_occ
   * fresh sightings; 2 = immediate, re-inserted on its next costly occurrence.
   */
  self_type& trace_stall_readmit(int trace_stall_readmit_);
  /**
   * Stall segmenter candidate table: stretch starts seen fewer than min_occ times
   * (and, with readmit 1, evicted traces re-qualifying) are remembered in a table of
   * this many entries; on overflow the least recently seen is forgotten.  Hardware
   * needs this table; the simulator otherwise keeps it unbounded.  0 (default) = unbounded.
   */
  self_type& trace_stall_filter_entries(int trace_stall_filter_entries_);
  /**
   * Stall segmenter: 1 = every mispredicted branch opens a stretch, as a u-op-cache
   * miss does.  The stretch starts at the branch and records the path after it --
   * the correct path, fetched after resolution -- hits included, up to the trace
   * depth; then the usual rules apply (costly verdict, min_occ, store keyed by the
   * branch PC).  2 = same, keyed by the branch and the direction it really went
   * (the trace_stall_mispredict key), so it is walked only when the predict-time
   * probe (trace_stall_mispredict 2/3) sees the branch predicted the other way.
   * 0 (default) = stretches open on u-op-cache misses only.
   */
  self_type& trace_stall_mispredict_open(int trace_stall_mispredict_open_);

  /**
   * Stall segmenter: a miss run longer than trace_stall_depth is not truncated --
   * the full stretch closes and recording continues in a new stretch, until a
   * u-op-cache hit ends the run.  A continuation inherits the costly verdict of
   * the segment before it, so a costly run is stored whole as consecutive traces,
   * which trace chaining links.  0 (default) = truncate at the depth.
   */
  self_type& trace_stall_split(int trace_stall_split_);

  /**
   * Run trace format: every trace is a run of sequential u-op-cache windows (a
   * start pc + a window count).  The stall segmenter cuts a miss stretch where the
   * next window is not sequential (a jump or a taken branch leaving the window)
   * or where a run reaches trace_run_max_windows; the rest continues as a new
   * trace that inherits the stretch's costly verdict.  The trace depth bounds the
   * stretch as a whole.  Consecutive runs are consecutive costly stretches, so
   * trace chaining (trace_chain_depth) links them.  2 = head run only: the
   * stretch stops recording at its first cut (its instruction range, and so its
   * costly verdict, still runs to the closing hit); one trace per stretch, so the
   * successor pointer always links stretch to stretch.  0 (default) = off.
   */
  self_type& trace_run_format(int trace_run_format_);
  self_type& trace_run_max_windows(int trace_run_max_windows_); // windows per run (default 8)

  /**
   * Stall segmenter trigger lead in cycles: each stretch is stored under the pc
   * that reached the u-op-cache check at least this many cycles before the
   * stretch opened (the newest such pc), instead of under its own first pc.  The
   * probe meets that key earlier than the stretch, so a walk's lead is a design
   * parameter rather than FTQ occupancy; after a misprediction the key lands on
   * the branch or before it.  The entry still names the stretch's first pc (its
   * distance from the key is histogrammed to size that field).  0 (default) =
   * stretches keyed by their own first pc.
   */
  self_type& trace_stall_trigger_lead(long trace_stall_trigger_lead_);

  /**
   * Stall segmenter max trace length in u-ops: a build-mode stretch longer than
   * this is truncated (default 64).
   */
  self_type& trace_stall_depth(int trace_stall_depth_);

  /**
   * Trace-cache capacity in traces (LRU). Used by all bounded fill modes
   * (miss/every/window/flush/parallel/alt); default 256.
   */
  self_type& trace_store(int trace_store_);
  self_type& trace_store_sets(int trace_store_sets_);
  self_type& trace_store_ways(int trace_store_ways_);
  self_type& trace_store_hash(int trace_store_hash_);
  /**
   * Trace store partial tags: 0 (default) = full entry-PC match; N = ways compare an
   * N-bit hashed tag (aliasing modeled: false matches walk the other trace's deltas
   * from the probing PC, inserts overwrite same-tag traces).  Set-associative only.
   */
  self_type& trace_store_tag_bits(int trace_store_tag_bits_);
  self_type& trace_store_cost_decay(int trace_store_cost_decay_);
  self_type& trace_store_cost_bits(int trace_store_cost_bits_);
  self_type& trace_stall_mispredict(int trace_stall_mispredict_);
  self_type& ftq_depth(int ftq_depth_);
  /**
   * Decoupled front end.  The IFETCH buffer is the FTQ: the branch predictor
   * inserts fetch blocks at its tail; the L1I line of a block whose window is not
   * in the u-op cache is requested at insertion (FDIP, as before).  The u-op-cache
   * lookup -- the demand access that picks stream or build mode -- happens when the
   * block reaches fetch: only the first N FTQ entries are within its reach, so a
   * build-mode stretch holds the lookups of younger blocks back and prefetches
   * launched at insertion get the FTQ residency as lead.  0 (default) = legacy:
   * every block is looked up the cycle after insertion.
   */
  self_type& ftq_lookup_window(int ftq_lookup_window_);
  /**
   * Fetch blocks the branch predictor inserts into the FTQ per cycle (a block ends
   * at a taken branch, or at two branches under the "ucp" fetch-block rule).
   * 1 (default) = one block per cycle; 2 = a two-taken-branches-per-cycle predictor.
   */
  self_type& bpu_blocks(int bpu_blocks_);
  /**
   * FDIP from the FTQ (CHAIN fill with trace_probe_ahead > 0; give baseline / UCP
   * runs the cursor with an empty trace store).  As the probe cursor -- the
   * predictor running ahead of fetch -- predicts a block whose window is not in the
   * u-op cache, its L1I line is prefetched.  With a small ifetch_buffer_size (fetch
   * stalls on an L1I miss, so the FTQ fills), this replaces the line requests the
   * large fetch buffer used to issue for every buffered block.  0 (default) = off.
   */
  self_type& ftq_fdip(int ftq_fdip_);
  /**
   * u-op-cache FDIP (CHAIN fill, probe cursor on): when the predictor puts a block in
   * the FTQ whose window is neither in the u-op cache nor staged nor being walked
   * (and no stored trace starts there), walk that window -- fetch its line and
   * pre-decode it into the staging buffer, ahead of fetch.  No storage beyond the
   * walk engine and staging buffer.  0 (default) = off.
   */
  self_type& ftq_uop_prefetch(int ftq_uop_prefetch_);
  self_type& trace_chain_parallel(int trace_chain_parallel_);
  self_type& ftq_fdip_filter(int ftq_fdip_filter_);
  self_type& ftq_fdip_uop_check(int ftq_fdip_uop_check_);
  self_type& trace_walk_nofill(int trace_walk_nofill_);
  self_type& fdip_predecode(int fdip_predecode_);
  self_type& fdip_predecode_target(int fdip_predecode_target_);
  self_type& cwt_entries(int cwt_entries_);
  self_type& rtt_mode(int rtt_mode_);
  self_type& uop_cost_mode(int uop_cost_mode_);
  self_type& uop_cost_windows(long uop_cost_windows_);
  self_type& uop_ideal_split(int uop_ideal_split_);
  self_type& uop_ideal_split_window(long uop_ideal_split_window_);
  self_type& rtt_capture(int rtt_capture_);
  self_type& rtt_len(int rtt_len_);
  self_type& rtt_max_open(int rtt_max_open_);
  self_type& rtt_table(int rtt_table_);
  self_type& rtt_gen_cond(int rtt_gen_cond_);
  self_type& rtt_fallback(int rtt_fallback_);
  self_type& rtt_boost(int rtt_boost_);
  self_type& rtt_ret(int rtt_ret_);
  self_type& rtt_key_target(int rtt_key_target_);
  self_type& rtt_cost(int rtt_cost_);
  self_type& rtt_seg_reconv(int rtt_seg_reconv_);
  self_type& rtt_conf_table(int rtt_conf_table_);
  self_type& rtt_admit_repeat(int rtt_admit_repeat_);
  self_type& rtt_cost_walk(int rtt_cost_walk_);
  self_type& rtt_seg_stable(int rtt_seg_stable_);
  self_type& rtt_l2_keep(int rtt_l2_keep_);
  self_type& rtt_cold_seq(int rtt_cold_seq_);
  self_type& rtt_cold_mode(int rtt_cold_mode_);
  self_type& rtt_cold_target(int rtt_cold_target_);
  self_type& rtt_cold_l2(int rtt_cold_l2_);
  self_type& walk_l2_mshrs(int walk_l2_mshrs_);
  self_type& walk_pq(int walk_pq_);
  self_type& walk_resident_port(int walk_resident_port_);
  self_type& walk_ooo(int walk_ooo_);
  self_type& walk_prio_head(int walk_prio_head_);
  self_type& rtt_admit_miss(int rtt_admit_miss_);
  self_type& rtt_conf_evict(int rtt_conf_evict_);
  self_type& rtt_conf_min(int rtt_conf_min_);
  self_type& rtt_cancel(int rtt_cancel_);
  self_type& walk_mshr_limit(int walk_mshr_limit_);
  self_type& rtt_format(int rtt_format_);
  self_type& rtt_max_windows(int rtt_max_windows_);
  self_type& rtt_wrong_path(int rtt_wrong_path_);
  self_type& rtt_open(int rtt_open_);
  self_type& rtt_store_filter(int rtt_store_filter_);
  self_type& ftq_shared_wp(int ftq_shared_wp_);
  self_type& trace_stall_taxonomy(int trace_stall_taxonomy_);
  self_type& l2_perfect_instr(int l2_perfect_instr_);
  self_type& l2_emissary_ways(int l2_emissary_ways_);
  self_type& l2_emissary_rand(int l2_emissary_rand_);
  self_type& rp_mode(int rp_mode_);
  self_type& rp_sets(int rp_sets_);
  self_type& rp_ways(int rp_ways_);
  self_type& rp_targets(int rp_targets_);
  self_type& rp_window(int rp_window_);
  /**
   * Fetch blocks in flight: fetch takes one block per cycle from the FTQ head into
   * the fetch (IFETCH) buffer, where it waits for its u-op-cache lookup / L1I bytes;
   * with N blocks there, fetch stalls and the predictor keeps filling the FTQ.  Use
   * with the probe cursor (the predictor running ahead) and ftq_fdip.  2-4 models a
   * 4-stage pipelined fetch issuing one block per cycle.  0 (default) = only the
   * ifetch_buffer_size instruction limit.
   */
  self_type& fetch_buffer_blocks(int fetch_buffer_blocks_);

  /**
   * ALT fill mode: walk latency in cycles from trigger to the first window
   * install (models L1I fetch + pre-decode pipe fill); default 5.
   */
  self_type& trace_walk_delay(int trace_walk_delay_);

  /**
   * ALT fill mode: maximum concurrent walks; extra trigger hits are dropped
   * (default 2).
   */
  self_type& trace_walk_max(int trace_walk_max_);

  /**
   * ALT fill mode: windows installed per walk per cycle (pre-decode width in
   * 8-uop windows; default 1 = 8 uops/cycle, 2 = 16 uops/cycle).
   */
  self_type& trace_walk_width(int trace_walk_width_);

  /**
   * ALT fill mode: when 1, a demand miss on a window held by an in-flight walk
   * stalls fetch until the window installs (hit-under-fill; no stream->build
   * switch), instead of falling to build mode.  Default 0.
   */
  self_type& trace_walk_wait(int trace_walk_wait_);

  /**
   * ALT fill mode: when 1, the walk issues real read requests for its cache
   * lines through the L1I (misses propagate down the hierarchy) and a window
   * may only install after its line arrives.  Default 0 (bytes are free).
   */
  self_type& trace_walk_l1i(int trace_walk_l1i_);

  /**
   * ALT fill mode: maximum cycles an instruction stalls waiting on a
   * walk-pending window before falling back to build mode ("wait on the fill
   * buffer, not on DRAM").  0 = unbounded.  Default 16.
   */
  self_type& trace_walk_wait_cap(int trace_walk_wait_cap_);

  /**
   * Stall segmenter L1I admission gate: when 1, a costly stretch commits only
   * if an L1I miss was observed during it -- selects stretches where u-op
   * replay saves byte fetch AND decode.  Default 0.
   */
  self_type& trace_stall_l1i_gate(int trace_stall_l1i_gate_);

  /**
   * Trace-store replacement: when 1, evict the minimum-stall-cost trace
   * (LRU tie-break) instead of plain LRU.  Default 0.
   */
  self_type& trace_store_cost_evict(int trace_store_cost_evict_);

  /**
   * HEAD fill mode: number of uops stored per trace head, served instantly to
   * the backend on a head-region hit.  Default 8.
   */
  self_type& trace_head_uops(int trace_head_uops_);

  /**
   * HEAD fill mode: taken-branch targets in the tail manifest -- the tail walk
   * may follow the recorded path through at most this many taken transfers
   * (<0 = unlimited).  Default 3.
   */
  self_type& trace_tail_targets(int trace_tail_targets_);

  /**
   * CHAIN fill mode: trace-uop staging buffer capacity in windows (walk output
   * lands here, not in the u-op cache; demand hits promote).  0 disables the
   * buffer (walks install directly).  Default 16 (= 1KB of uops).
   */
  self_type& trace_uop_buffer(int trace_uop_buffer_);

  /**
   * CHAIN fill mode: manifest window slots per trace (entry window + 16-bit
   * window deltas); traces spanning more windows are truncated.  Default 4.
   */
  self_type& trace_meta_windows(int trace_meta_windows_);

  /**
   * CHAIN fill mode: packed manifest.  >0 = slot field of this many bits with
   * per-transfer cost (sequential 2b / short 10b / long 18b), variable window
   * count; 0 (default) = legacy fixed 16-bit delta slots capped by meta_windows.
   */
  self_type& trace_meta_bits(int trace_meta_bits_);
  /**
   * CHAIN fill mode, packed manifest: signed window-delta widths of the short
   * and long jump records (defaults 8 / 16 bits = +-4KB / +-1MB at 32B
   * windows).  Ignored by the legacy format (meta_bits = 0).
   */
  self_type& trace_meta_short_bits(int trace_meta_short_bits_);
  self_type& trace_meta_long_bits(int trace_meta_long_bits_);
  /**
   * CHAIN fill mode, packed manifest: kind-code width (2 = sequential / short /
   * long / end; 1 = sequential / jump, no long class) and whether a jump that
   * no record can encode ends the trace (1, prefix kept) or rejects it (0).
   */
  self_type& trace_meta_kind_bits(int trace_meta_kind_bits_);
  self_type& trace_meta_far_truncate(int trace_meta_far_truncate_);

  /**
   * CHAIN fill mode: probe the instruction supply this many instructions AHEAD
   * of the IFETCH transfer point (models the decoupled BP/FTQ lead over the
   * fetch-point u-op-cache lookup; resets on misprediction).  0 = probe at
   * enqueue only.  Default 0.
   */
  self_type& trace_probe_ahead(int trace_probe_ahead_);
  /**
   * Deep-probe cursor speed (CHAIN fill, trace_probe_ahead > 0): fetch blocks the
   * cursor -- the branch predictor, with trace_wrong_path >= 1 -- covers per cycle.
   * 1 (default) = the same rate as the u-op-cache lookup, so the cursor only gains
   * lead while the lookup is held (IFETCH buffer full, fetch stalls); 2 = a
   * two-blocks-per-cycle predictor that runs ahead of fetch up to trace_probe_ahead.
   */
  self_type& trace_probe_blocks(int trace_probe_blocks_);
  /**
   * CHAIN fill mode: what the deep probe may do past a branch that mispredicts.
   * The instruction supply holds the CORRECT path beyond it, which a real FTQ
   * would not (it holds the predicted, wrong path until resolution).  0 (default)
   * = legacy, probes continue (walks are only flagged and counted); 1 = at the
   * misprediction, squash walks and staged windows probed past the branch and
   * freeze the cursor until it resolves.
   */
  self_type& trace_probe_squash(int trace_probe_squash_);
  /**
   * What the frontend does past a mispredicted branch.  0 (default) = legacy, the
   * deep-probe cursor keeps reading the correct path past it; 1 = the predictor
   * runs at the cursor, which stops at a branch it predicts wrong until
   * resolution; 2 = 1 + a synthetic wrong path (UCP_ISCA24 artifact model) holds
   * BTB banks and decoders until resolution; 3 = 2 + SUCP and UCP prefetch from
   * the wrong path too (walks in flight at resolution are flushed).  "ucp" = 1 +
   * the UCP_ISCA24 artifact's prediction rule: each cycle the predictor (SUCP's
   * cursor, and the fetch-point predictor of every config) stops after the first
   * taken branch or after the second branch.  "ucp2" = "ucp" + the level-3 wrong
   * path made exact: real instruction addresses (learned from the correct path),
   * the fetch-block rule, and u-op-cache lookups and fills like demand.
   */
  self_type& trace_wrong_path(std::string trace_wrong_path_);

  /**
   * DIAGNOSTIC CEILING, not a mechanism (needs trace_wrong_path >= 1).  While the
   * cursor is stopped at a branch it predicted wrong, keep probing the trace store
   * with the REAL path past the branch -- which the trace holds and hardware does
   * not -- for up to N instructions, at the cursor's bandwidth.  Its walks are what
   * a predictor naming the right trace at the moment the predictor reaches the
   * branch could launch; N bounds how far past the branch it is credited.  0
   * (default) = off (the realistic cursor); a large N approaches level 0.  Its walks
   * are flagged past-mispredict (trace-chain-shadow counts their use).
   */
  self_type& trace_oracle_peek(int trace_oracle_peek_);

  /**
   * DIAGNOSTIC, no timing effect (needs trace_wrong_path >= 1 and chain fill).  At
   * every branch prediction, unbounded tables keyed by the branch PC + its
   * predicted direction/target, optionally + 8/16/32/64 GHR bits or + the last 1/2/4
   * trace entries the cursor hit, name the trace expected to follow a misprediction
   * there ("fire").  The truth for a misprediction is the first trace the cursor
   * hits after the branch (within 192 instructions) -- what level 0 walked early.
   * Reports, per key: fires, useful (mispredicted and the named trace was right),
   * wrong.  0 (default) = off.
   */
  self_type& trace_peek_diag(int trace_peek_diag_);

  /**
   * Trigger diagnostic (CHAIN fill).  Measurement only.  Every u-op-cache miss is
   * classified -- uncovered / captured but not stored / side door (in a stored
   * trace not entered at its entry) / entry never probed / walk dropped / resident
   * at probe then gone / walk late / staged then evicted / walked but gone -- and
   * every dispatch-starve cycle is charged to the class of the miss dispatch waits
   * for.  Also: what happened at each arrival at a stored trace's entry, and the
   * probe->arrival lead.  0 (default) = off.
   */
  self_type& trace_trigger_diag(int trace_trigger_diag_);

  /**
   * Recovery key (CHAIN fill, trace_wrong_path >= 1).  Traces are recorded and
   * stored as usual (keyed by their first pc).  In addition, at each mispredicted
   * branch the first stored trace the cursor hits in the 192 instructions after it
   * is filed a second time in the same store, under a recovery key = hash(branch
   * pc, the direction/target predicted for it, trace_recovery_ghr bits of global
   * history), bit 63 set.  Every prediction forms the same key from what the
   * predictor knows at that moment and probes; a hit walks the trace at once, so
   * a recovery path is fetched while the branch is still on its way to resolve.
   * Predictions that turn out correct waste their walk (the residency filter
   * skips windows already resident).  0 (default) = off.
   */
  self_type& trace_recovery_key(int trace_recovery_key_);

  /**
   * Recovery key: global-history bits in the key (0 = branch pc + predicted
   * direction only; up to 64).  More bits = fewer wasted walks, lower coverage.
   */
  self_type& trace_recovery_ghr(int trace_recovery_ghr_);

  /**
   * Recovery key: probe only predictions the predictor flags hard-to-predict (the
   * UCP_ISCA24 signal), trading coverage for fewer wasted walks.  0 (default) = all.
   */
  self_type& trace_recovery_h2p(int trace_recovery_h2p_);

  /**
   * Trace value predictor (CHAIN fill, trace_wrong_path >= 1): a TAGE-organized
   * predictor whose value is a trace (inc/trace_vpred.h).  Traces are created and
   * stored as usual.  Every branch prediction looks the predictor up with (branch
   * pc, predicted direction/target, global history) and walks the predicted trace
   * from the regular store.  Every costly stretch (u-op-cache miss that starved
   * dispatch with the ROB empty) trains the entry of the last prediction before it
   * -- after a misprediction, the mispredicted branch with its wrong prediction, so
   * the recovery path is named while the branch is still unresolved.  0 (default) = off.
   */
  self_type& trace_vpred(int trace_vpred_);
  self_type& trace_vpred_tables(int trace_vpred_tables_);     // tagged tables above the base table (default 4)
  self_type& trace_vpred_entries(int trace_vpred_entries_);   // entries per table, power of two (default 512)
  self_type& trace_vpred_max_hist(int trace_vpred_max_hist_); // longest global history in bits, <= 64 (default 64)
  self_type& trace_vpred_tag_bits(int trace_vpred_tag_bits_); // tag width (default 12)
  self_type& trace_vpred_conf(int trace_vpred_conf_);         // confidence (0-3) needed to walk; 0 = any tag match

  /**
   * Trace chaining (CHAIN fill): every stored trace keeps a successor pointer --
   * the entry pc of the costly trace that followed it last time (consecutive
   * costly stretches, program order).  When the cursor hits a trace, the walk
   * follows the successor chain up to this many hops, walking each successor found
   * in the store.  The cursor keeps probing the FTQ as before.  0 (default) = off.
   */
  self_type& trace_chain_depth(int trace_chain_depth_);

  /**
   * Trace chaining: link two consecutive costly traces only if at most this many
   * instructions separate them.  0 (default) = any distance.
   */
  self_type& trace_chain_window(int trace_chain_window_);

  /**
   * CHAIN fill mode: per-window residency filter -- at walk launch, probe each
   * of the trace's windows in the u-op cache and the staging buffer, and
   * fetch/stage only the absent ones.  Default 0 (entry-window check only).
   */
  self_type& trace_walk_filter(int trace_walk_filter_);

  /**
   * CHAIN fill mode: per-trace launch confidence (2-bit saturating counter fed
   * back from the staging buffer: +1 demanded, -1 evicted unused; re-capture
   * retries a suppressed trace).  0 = off (default), 1 = gate walk launches on
   * conf > 0, 2 = additionally evict conf-0 traces first.
   */
  self_type& trace_launch_conf(int trace_launch_conf_);

  /**
   * CHAIN fill mode: what a staging-buffer hit does to the u-op cache.
   * 1 (default) = demand-class fill at MRU and the buffer entry is dropped;
   * 0 = serve from the buffer only, no u-op-cache fill, the entry stays;
   * 2 = fill at LRU as a prefetch-class insert (promoted on its first u-op-cache hit).
   */
  self_type& trace_buffer_promote(int trace_buffer_promote_);

  /**
   * UCP port (ISCA'24): alternate-path u-op cache prefetching on hard-to-predict
   * conditionals.  Off by default; independent of the trace_fill mode.
   */
  self_type& trace_ucp(int trace_ucp_);
  self_type& trace_ucp_threshold(int trace_ucp_threshold_);
  self_type& trace_ucp_max_ip(int trace_ucp_max_ip_);
  self_type& trace_ucp_step(int trace_ucp_step_);
  self_type& trace_ucp_altind(int trace_ucp_altind_);
  self_type& trace_ucp_artifact(int trace_ucp_artifact_);
  self_type& trace_ucp_btb_banks(int trace_ucp_btb_banks_);
  self_type& trace_warmup_train(int trace_warmup_train_);
  self_type& trace_walk_issue_cap(int trace_walk_issue_cap_);
  self_type& trace_walk_decode_cap(int trace_walk_decode_cap_);
  self_type& trace_walk_decode_shared(int trace_walk_decode_shared_);

  /**
   * Specify the maximum size of the instruction fetch buffer.
   */
  self_type& ifetch_buffer_size(std::size_t ifetch_buffer_size_);

  /**
   * Specify the maximum size of the decode buffer.
   */
  self_type& decode_buffer_size(std::size_t decode_buffer_size_);

  /**
   * Specify the maximum size of the dispatch buffer.
   */
  self_type& dispatch_buffer_size(std::size_t dispatch_buffer_size_);

  /**
   * Specify the maximum size of the DIB hit buffer.
   */
  self_type& dib_hit_buffer_size(std::size_t dib_hit_buffer_size_);

  /**
   * Specify the maximum size of the physical register file.
   */
  self_type& register_file_size(std::size_t register_file_size_);

  /**
   * Specify the maximum size of the reorder buffer.
   */
  self_type& rob_size(std::size_t rob_size_);

  /**
   * Specify the maximum size of the load queue.
   */
  self_type& lq_size(std::size_t lq_size_);

  /**
   * Specify the maximum size of the store queue.
   */
  self_type& sq_size(std::size_t sq_size_);

  /**
   * Specify the width of the instruction fetch.
   */
  self_type& fetch_width(champsim::bandwidth::maximum_type fetch_width_);

  /**
   * Specify the width of the decode.
   */
  self_type& decode_width(champsim::bandwidth::maximum_type decode_width_);

  /**
   * Specify the width of the dispatch.
   */
  self_type& dispatch_width(champsim::bandwidth::maximum_type dispatch_width_);

  /**
   * Specify the width of the scheduler.
   */
  self_type& schedule_width(champsim::bandwidth::maximum_type schedule_width_);

  /**
   * Specify the width of the execution.
   */
  self_type& execute_width(champsim::bandwidth::maximum_type execute_width_);

  /**
   * Specify the width of the load issue.
   */
  self_type& lq_width(champsim::bandwidth::maximum_type lq_width_);

  /**
   * Specify the width of the store issue.
   */
  self_type& sq_width(champsim::bandwidth::maximum_type sq_width_);

  /**
   * Specify the width of the retirement.
   */
  self_type& retire_width(champsim::bandwidth::maximum_type retire_width_);

  /**
   * Specify the maximum size of the DIB inorder width.
   */
  self_type& dib_inorder_width(champsim::bandwidth::maximum_type dib_inorder_width_);

  /**
   * Specify the reset penalty, in cycles, that follows a misprediction.
   * Note that this value is in addition to the cost of restarting the pipeline, which will depend on the number of instructions inflight at the time when the
   * misprediction is detected.
   */
  self_type& mispredict_penalty(unsigned mispredict_penalty_);

  /**
   * Specify the latency of the decode.
   */
  self_type& decode_latency(unsigned decode_latency_);

  /**
   * Specify the latency of dispatch.
   */
  self_type& dispatch_latency(unsigned dispatch_latency_);

  /**
   * Specify the latency of the scheduler.
   */
  self_type& schedule_latency(unsigned schedule_latency_);

  /**
   * Specify the latency of execution.
   */
  self_type& execute_latency(unsigned execute_latency_);

  /**
   * Specify the latency of execution.
   */
  self_type& dib_hit_latency(unsigned dib_hit_latency_);

  /**
   * Specify a pointer to the L1I cache. This is only used to transmit branch triggers for prefetcher branch hooks.
   */
  self_type& l1i(CACHE* l1i_);

  /**
   * Specify the instruction cache bandwidth.
   */
  self_type& l1i_bandwidth(champsim::bandwidth::maximum_type l1i_bw_);

  /**
   * Specify the data cache bandwidth.
   */
  self_type& l1d_bandwidth(champsim::bandwidth::maximum_type l1d_bw_);

  /**
   * Specify the downstream queues to the instruction cache.
   */
  self_type& fetch_queues(champsim::channel* fetch_queues_);

  /**
   * Specify the downstream queues to the data cache.
   */
  self_type& data_queues(champsim::channel* data_queues_);

  /**
   * Specify the branch direction predictor.
   */
  template <typename... Bs>
  core_builder<core_builder_module_type_holder<Bs...>, T> branch_predictor();

  /**
   * Specify the branch target predictor.
   */
  template <typename... Ts>
  core_builder<B, core_builder_module_type_holder<Ts...>> btb();
};
} // namespace champsim

template <typename B, typename T>
auto champsim::core_builder<B, T>::index(uint32_t cpu_) -> self_type&
{
  m_cpu = cpu_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::clock_period(champsim::chrono::picoseconds clock_period_) -> self_type&
{
  m_clock_period = clock_period_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::dib_set(std::size_t dib_set_) -> self_type&
{
  m_dib_set = dib_set_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::dib_way(std::size_t dib_way_) -> self_type&
{
  m_dib_way = dib_way_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::dib_window(std::size_t dib_window_) -> self_type&
{
  m_dib_window = dib_window_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::dib_ideal(int dib_ideal_) -> self_type&
{
  m_dib_ideal = dib_ideal_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_builder(std::string trace_builder_) -> self_type&
{
  m_trace_builder = std::move(trace_builder_);
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_fill(std::string trace_fill_) -> self_type&
{
  m_trace_fill = std::move(trace_fill_);
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_min_occ(int trace_stall_min_occ_) -> self_type&
{
  m_trace_stall_min_occ = trace_stall_min_occ_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_readmit(int trace_stall_readmit_) -> self_type&
{
  m_trace_stall_readmit = trace_stall_readmit_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_filter_entries(int trace_stall_filter_entries_) -> self_type&
{
  m_trace_stall_filter_entries = trace_stall_filter_entries_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_mispredict_open(int trace_stall_mispredict_open_) -> self_type&
{
  m_trace_stall_mispredict_open = trace_stall_mispredict_open_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_split(int trace_stall_split_) -> self_type&
{
  m_trace_stall_split = trace_stall_split_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_run_format(int trace_run_format_) -> self_type&
{
  m_trace_run_format = trace_run_format_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_run_max_windows(int trace_run_max_windows_) -> self_type&
{
  m_trace_run_max_windows = trace_run_max_windows_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_trigger_lead(long trace_stall_trigger_lead_) -> self_type&
{
  m_trace_stall_trigger_lead = trace_stall_trigger_lead_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_attrib(int trace_stall_attrib_) -> self_type&
{
  m_trace_stall_attrib = trace_stall_attrib_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_rob(int trace_stall_rob_) -> self_type&
{
  m_trace_stall_rob = trace_stall_rob_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_depth(int trace_stall_depth_) -> self_type&
{
  m_trace_stall_depth = trace_stall_depth_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_store(int trace_store_) -> self_type&
{
  m_trace_store = trace_store_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_delay(int trace_walk_delay_) -> self_type&
{
  m_trace_walk_delay = trace_walk_delay_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_max(int trace_walk_max_) -> self_type&
{
  m_trace_walk_max = trace_walk_max_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_width(int trace_walk_width_) -> self_type&
{
  m_trace_walk_width = trace_walk_width_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_wait(int trace_walk_wait_) -> self_type&
{
  m_trace_walk_wait = trace_walk_wait_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_l1i(int trace_walk_l1i_) -> self_type&
{
  m_trace_walk_l1i = trace_walk_l1i_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_wait_cap(int trace_walk_wait_cap_) -> self_type&
{
  m_trace_walk_wait_cap = trace_walk_wait_cap_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_l1i_gate(int trace_stall_l1i_gate_) -> self_type&
{
  m_trace_stall_l1i_gate = trace_stall_l1i_gate_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_store_sets(int trace_store_sets_) -> self_type&
{
  m_trace_store_sets = trace_store_sets_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_store_ways(int trace_store_ways_) -> self_type&
{
  m_trace_store_ways = trace_store_ways_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_store_tag_bits(int trace_store_tag_bits_) -> self_type&
{
  m_trace_store_tag_bits = trace_store_tag_bits_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_store_hash(int trace_store_hash_) -> self_type&
{
  m_trace_store_hash = trace_store_hash_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_store_cost_decay(int trace_store_cost_decay_) -> self_type&
{
  m_trace_store_cost_decay = trace_store_cost_decay_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_store_cost_bits(int trace_store_cost_bits_) -> self_type&
{
  m_trace_store_cost_bits = trace_store_cost_bits_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_mispredict(int trace_stall_mispredict_) -> self_type&
{
  m_trace_stall_mispredict = trace_stall_mispredict_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::ftq_depth(int ftq_depth_) -> self_type&
{
  m_ftq_depth = ftq_depth_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::ftq_lookup_window(int ftq_lookup_window_) -> self_type&
{
  m_ftq_lookup_window = ftq_lookup_window_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::bpu_blocks(int bpu_blocks_) -> self_type&
{
  m_bpu_blocks = bpu_blocks_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::fetch_buffer_blocks(int fetch_buffer_blocks_) -> self_type&
{
  m_fetch_buffer_blocks = fetch_buffer_blocks_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::ftq_uop_prefetch(int ftq_uop_prefetch_) -> self_type&
{
  m_ftq_uop_prefetch = ftq_uop_prefetch_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_chain_parallel(int trace_chain_parallel_) -> self_type&
{
  m_trace_chain_parallel = trace_chain_parallel_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::ftq_fdip_filter(int ftq_fdip_filter_) -> self_type&
{
  m_ftq_fdip_filter = ftq_fdip_filter_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::ftq_fdip_uop_check(int ftq_fdip_uop_check_) -> self_type&
{
  m_ftq_fdip_uop_check = ftq_fdip_uop_check_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::fdip_predecode(int fdip_predecode_) -> self_type&
{
  m_fdip_predecode = fdip_predecode_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::fdip_predecode_target(int fdip_predecode_target_) -> self_type&
{
  m_fdip_predecode_target = fdip_predecode_target_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::uop_cost_mode(int uop_cost_mode_) -> self_type&
{
  m_uop_cost_mode = uop_cost_mode_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::uop_cost_windows(long uop_cost_windows_) -> self_type&
{
  m_uop_cost_windows = uop_cost_windows_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::uop_ideal_split(int uop_ideal_split_) -> self_type&
{
  m_uop_ideal_split = uop_ideal_split_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::uop_ideal_split_window(long uop_ideal_split_window_) -> self_type&
{
  m_uop_ideal_split_window = uop_ideal_split_window_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_mode(int rtt_mode_) -> self_type&
{
  m_rtt_mode = rtt_mode_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_capture(int rtt_capture_) -> self_type&
{
  m_rtt_capture = rtt_capture_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_open(int rtt_open_) -> self_type&
{
  m_rtt_open = rtt_open_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_store_filter(int rtt_store_filter_) -> self_type&
{
  m_rtt_store_filter = rtt_store_filter_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_wrong_path(int rtt_wrong_path_) -> self_type&
{
  m_rtt_wrong_path = rtt_wrong_path_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_max_windows(int rtt_max_windows_) -> self_type&
{
  m_rtt_max_windows = rtt_max_windows_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_format(int rtt_format_) -> self_type&
{
  m_rtt_format = rtt_format_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_cancel(int rtt_cancel_) -> self_type&
{
  m_rtt_cancel = rtt_cancel_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::walk_mshr_limit(int walk_mshr_limit_) -> self_type&
{
  m_walk_mshr_limit = walk_mshr_limit_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_conf_min(int rtt_conf_min_) -> self_type&
{
  m_rtt_conf_min = rtt_conf_min_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_admit_miss(int rtt_admit_miss_) -> self_type&
{
  m_rtt_admit_miss = rtt_admit_miss_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_conf_evict(int rtt_conf_evict_) -> self_type&
{
  m_rtt_conf_evict = rtt_conf_evict_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::walk_prio_head(int walk_prio_head_) -> self_type&
{
  m_walk_prio_head = walk_prio_head_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::walk_ooo(int walk_ooo_) -> self_type&
{
  m_walk_ooo = walk_ooo_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::walk_resident_port(int walk_resident_port_) -> self_type&
{
  m_walk_resident_port = walk_resident_port_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::walk_l2_mshrs(int walk_l2_mshrs_) -> self_type&
{
  m_walk_l2_mshrs = walk_l2_mshrs_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::walk_pq(int walk_pq_) -> self_type&
{
  m_walk_pq = walk_pq_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_boost(int rtt_boost_) -> self_type&
{
  m_rtt_boost = rtt_boost_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_ret(int rtt_ret_) -> self_type&
{
  m_rtt_ret = rtt_ret_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_key_target(int rtt_key_target_) -> self_type&
{
  m_rtt_key_target = rtt_key_target_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_cost(int rtt_cost_) -> self_type&
{
  m_rtt_cost = rtt_cost_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_seg_reconv(int rtt_seg_reconv_) -> self_type&
{
  m_rtt_seg_reconv = rtt_seg_reconv_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_conf_table(int rtt_conf_table_) -> self_type&
{
  m_rtt_conf_table = rtt_conf_table_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_admit_repeat(int rtt_admit_repeat_) -> self_type&
{
  m_rtt_admit_repeat = rtt_admit_repeat_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_cost_walk(int rtt_cost_walk_) -> self_type&
{
  m_rtt_cost_walk = rtt_cost_walk_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_seg_stable(int rtt_seg_stable_) -> self_type&
{
  m_rtt_seg_stable = rtt_seg_stable_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_l2_keep(int rtt_l2_keep_) -> self_type&
{
  m_rtt_l2_keep = rtt_l2_keep_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_cold_seq(int rtt_cold_seq_) -> self_type&
{
  m_rtt_cold_seq = rtt_cold_seq_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_cold_mode(int rtt_cold_mode_) -> self_type&
{
  m_rtt_cold_mode = rtt_cold_mode_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_cold_target(int rtt_cold_target_) -> self_type&
{
  m_rtt_cold_target = rtt_cold_target_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_cold_l2(int rtt_cold_l2_) -> self_type&
{
  m_rtt_cold_l2 = rtt_cold_l2_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_fallback(int rtt_fallback_) -> self_type&
{
  m_rtt_fallback = rtt_fallback_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_gen_cond(int rtt_gen_cond_) -> self_type&
{
  m_rtt_gen_cond = rtt_gen_cond_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_table(int rtt_table_) -> self_type&
{
  m_rtt_table = rtt_table_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_max_open(int rtt_max_open_) -> self_type&
{
  m_rtt_max_open = rtt_max_open_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rtt_len(int rtt_len_) -> self_type&
{
  m_rtt_len = rtt_len_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::cwt_entries(int cwt_entries_) -> self_type&
{
  m_cwt_entries = cwt_entries_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_nofill(int trace_walk_nofill_) -> self_type&
{
  m_trace_walk_nofill = trace_walk_nofill_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::ftq_shared_wp(int ftq_shared_wp_) -> self_type&
{
  m_ftq_shared_wp = ftq_shared_wp_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_stall_taxonomy(int trace_stall_taxonomy_) -> self_type&
{
  m_trace_stall_taxonomy = trace_stall_taxonomy_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::l2_perfect_instr(int l2_perfect_instr_) -> self_type&
{
  m_l2_perfect_instr = l2_perfect_instr_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::l2_emissary_ways(int l2_emissary_ways_) -> self_type&
{
  m_l2_emissary_ways = l2_emissary_ways_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::l2_emissary_rand(int l2_emissary_rand_) -> self_type&
{
  m_l2_emissary_rand = l2_emissary_rand_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rp_mode(int rp_mode_) -> self_type&
{
  m_rp_mode = rp_mode_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rp_sets(int rp_sets_) -> self_type&
{
  m_rp_sets = rp_sets_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rp_ways(int rp_ways_) -> self_type&
{
  m_rp_ways = rp_ways_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rp_targets(int rp_targets_) -> self_type&
{
  m_rp_targets = rp_targets_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rp_window(int rp_window_) -> self_type&
{
  m_rp_window = rp_window_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::ftq_fdip(int ftq_fdip_) -> self_type&
{
  m_ftq_fdip = ftq_fdip_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_store_cost_evict(int trace_store_cost_evict_) -> self_type&
{
  m_trace_store_cost_evict = trace_store_cost_evict_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_head_uops(int trace_head_uops_) -> self_type&
{
  m_trace_head_uops = trace_head_uops_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_tail_targets(int trace_tail_targets_) -> self_type&
{
  m_trace_tail_targets = trace_tail_targets_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_uop_buffer(int trace_uop_buffer_) -> self_type&
{
  m_trace_uop_buffer = trace_uop_buffer_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_meta_kind_bits(int trace_meta_kind_bits_) -> self_type&
{
  m_trace_meta_kind_bits = trace_meta_kind_bits_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_meta_far_truncate(int trace_meta_far_truncate_) -> self_type&
{
  m_trace_meta_far_truncate = trace_meta_far_truncate_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_meta_short_bits(int trace_meta_short_bits_) -> self_type&
{
  m_trace_meta_short_bits = trace_meta_short_bits_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_meta_long_bits(int trace_meta_long_bits_) -> self_type&
{
  m_trace_meta_long_bits = trace_meta_long_bits_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_meta_bits(int trace_meta_bits_) -> self_type&
{
  m_trace_meta_bits = trace_meta_bits_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_meta_windows(int trace_meta_windows_) -> self_type&
{
  m_trace_meta_windows = trace_meta_windows_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_probe_ahead(int trace_probe_ahead_) -> self_type&
{
  m_trace_probe_ahead = trace_probe_ahead_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_probe_blocks(int trace_probe_blocks_) -> self_type&
{
  m_trace_probe_blocks = trace_probe_blocks_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_probe_squash(int trace_probe_squash_) -> self_type&
{
  m_trace_probe_squash = trace_probe_squash_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_wrong_path(std::string trace_wrong_path_) -> self_type&
{
  m_trace_wrong_path = trace_wrong_path_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_oracle_peek(int trace_oracle_peek_) -> self_type&
{
  m_trace_oracle_peek = trace_oracle_peek_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_peek_diag(int trace_peek_diag_) -> self_type&
{
  m_trace_peek_diag = trace_peek_diag_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_trigger_diag(int trace_trigger_diag_) -> self_type&
{
  m_trace_trigger_diag = trace_trigger_diag_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_recovery_key(int trace_recovery_key_) -> self_type&
{
  m_trace_recovery_key = trace_recovery_key_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_recovery_ghr(int trace_recovery_ghr_) -> self_type&
{
  m_trace_recovery_ghr = trace_recovery_ghr_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_recovery_h2p(int trace_recovery_h2p_) -> self_type&
{
  m_trace_recovery_h2p = trace_recovery_h2p_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_vpred(int trace_vpred_) -> self_type&
{
  m_trace_vpred = trace_vpred_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_vpred_tables(int trace_vpred_tables_) -> self_type&
{
  m_trace_vpred_tables = trace_vpred_tables_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_vpred_entries(int trace_vpred_entries_) -> self_type&
{
  m_trace_vpred_entries = trace_vpred_entries_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_vpred_max_hist(int trace_vpred_max_hist_) -> self_type&
{
  m_trace_vpred_max_hist = trace_vpred_max_hist_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_vpred_tag_bits(int trace_vpred_tag_bits_) -> self_type&
{
  m_trace_vpred_tag_bits = trace_vpred_tag_bits_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_vpred_conf(int trace_vpred_conf_) -> self_type&
{
  m_trace_vpred_conf = trace_vpred_conf_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_chain_depth(int trace_chain_depth_) -> self_type&
{
  m_trace_chain_depth = trace_chain_depth_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_chain_window(int trace_chain_window_) -> self_type&
{
  m_trace_chain_window = trace_chain_window_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_filter(int trace_walk_filter_) -> self_type&
{
  m_trace_walk_filter = trace_walk_filter_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_ucp(int trace_ucp_) -> self_type&
{
  m_trace_ucp = trace_ucp_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_ucp_threshold(int trace_ucp_threshold_) -> self_type&
{
  m_trace_ucp_threshold = trace_ucp_threshold_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_ucp_max_ip(int trace_ucp_max_ip_) -> self_type&
{
  m_trace_ucp_max_ip = trace_ucp_max_ip_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_ucp_step(int trace_ucp_step_) -> self_type&
{
  m_trace_ucp_step = trace_ucp_step_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_ucp_altind(int trace_ucp_altind_) -> self_type&
{
  m_trace_ucp_altind = trace_ucp_altind_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_ucp_artifact(int trace_ucp_artifact_) -> self_type&
{
  m_trace_ucp_artifact = trace_ucp_artifact_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_buffer_promote(int trace_buffer_promote_) -> self_type&
{
  m_trace_buffer_promote = trace_buffer_promote_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_launch_conf(int trace_launch_conf_) -> self_type&
{
  m_trace_launch_conf = trace_launch_conf_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_ucp_btb_banks(int trace_ucp_btb_banks_) -> self_type&
{
  m_trace_ucp_btb_banks = trace_ucp_btb_banks_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_warmup_train(int trace_warmup_train_) -> self_type&
{
  m_trace_warmup_train = trace_warmup_train_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_issue_cap(int trace_walk_issue_cap_) -> self_type&
{
  m_trace_walk_issue_cap = trace_walk_issue_cap_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_decode_cap(int trace_walk_decode_cap_) -> self_type&
{
  m_trace_walk_decode_cap = trace_walk_decode_cap_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::trace_walk_decode_shared(int trace_walk_decode_shared_) -> self_type&
{
  m_trace_walk_decode_shared = trace_walk_decode_shared_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::ifetch_buffer_size(std::size_t ifetch_buffer_size_) -> self_type&
{
  m_ifetch_buffer_size = ifetch_buffer_size_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::decode_buffer_size(std::size_t decode_buffer_size_) -> self_type&
{
  m_decode_buffer_size = decode_buffer_size_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::dispatch_buffer_size(std::size_t dispatch_buffer_size_) -> self_type&
{
  m_dispatch_buffer_size = dispatch_buffer_size_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::register_file_size(std::size_t register_file_size_) -> self_type&
{
  m_register_file_size = register_file_size_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::rob_size(std::size_t rob_size_) -> self_type&
{
  m_rob_size = rob_size_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::dib_hit_buffer_size(std::size_t dib_hit_buffer_size_) -> self_type&
{
  m_dib_hit_buffer_size = dib_hit_buffer_size_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::lq_size(std::size_t lq_size_) -> self_type&
{
  m_lq_size = lq_size_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::sq_size(std::size_t sq_size_) -> self_type&
{
  m_sq_size = sq_size_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::fetch_width(champsim::bandwidth::maximum_type fetch_width_) -> self_type&
{
  m_fetch_width = fetch_width_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::decode_width(champsim::bandwidth::maximum_type decode_width_) -> self_type&
{
  m_decode_width = decode_width_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::dispatch_width(champsim::bandwidth::maximum_type dispatch_width_) -> self_type&
{
  m_dispatch_width = dispatch_width_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::schedule_width(champsim::bandwidth::maximum_type schedule_width_) -> self_type&
{
  m_schedule_width = schedule_width_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::execute_width(champsim::bandwidth::maximum_type execute_width_) -> self_type&
{
  m_execute_width = execute_width_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::lq_width(champsim::bandwidth::maximum_type lq_width_) -> self_type&
{
  m_lq_width = lq_width_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::sq_width(champsim::bandwidth::maximum_type sq_width_) -> self_type&
{
  m_sq_width = sq_width_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::retire_width(champsim::bandwidth::maximum_type retire_width_) -> self_type&
{
  m_retire_width = retire_width_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::dib_inorder_width(champsim::bandwidth::maximum_type dib_inorder_width_) -> self_type&
{
  m_dib_inorder_width = dib_inorder_width_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::mispredict_penalty(unsigned mispredict_penalty_) -> self_type&
{
  m_mispredict_penalty = mispredict_penalty_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::decode_latency(unsigned decode_latency_) -> self_type&
{
  m_decode_latency = decode_latency_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::dib_hit_latency(unsigned dib_hit_latency_) -> self_type&
{
  m_dib_hit_latency = dib_hit_latency_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::dispatch_latency(unsigned dispatch_latency_) -> self_type&
{
  m_dispatch_latency = dispatch_latency_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::schedule_latency(unsigned schedule_latency_) -> self_type&
{
  m_schedule_latency = schedule_latency_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::execute_latency(unsigned execute_latency_) -> self_type&
{
  m_execute_latency = execute_latency_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::l1i(CACHE* l1i_) -> self_type&
{
  m_l1i = l1i_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::l1i_bandwidth(champsim::bandwidth::maximum_type l1i_bw_) -> self_type&
{
  m_l1i_bw = l1i_bw_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::l1d_bandwidth(champsim::bandwidth::maximum_type l1d_bw_) -> self_type&
{
  m_l1d_bw = l1d_bw_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::fetch_queues(champsim::channel* fetch_queues_) -> self_type&
{
  m_fetch_queues = fetch_queues_;
  return *this;
}

template <typename B, typename T>
auto champsim::core_builder<B, T>::data_queues(champsim::channel* data_queues_) -> self_type&
{
  m_data_queues = data_queues_;
  return *this;
}

template <typename B, typename T>
template <typename... Bs>
auto champsim::core_builder<B, T>::branch_predictor() -> champsim::core_builder<core_builder_module_type_holder<Bs...>, T>
{
  return champsim::core_builder<core_builder_module_type_holder<Bs...>, T>{*this};
}

template <typename B, typename T>
template <typename... Ts>
auto champsim::core_builder<B, T>::btb() -> champsim::core_builder<B, core_builder_module_type_holder<Ts...>>
{
  return champsim::core_builder<B, core_builder_module_type_holder<Ts...>>{*this};
}

#endif

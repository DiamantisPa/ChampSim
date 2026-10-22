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

#ifndef OOO_CPU_H
#define OOO_CPU_H

#ifdef CHAMPSIM_MODULE
#define SET_ASIDE_CHAMPSIM_MODULE
#undef CHAMPSIM_MODULE
#endif

#include <array>
#include <bitset>
#include <cstdlib>
#include <iostream>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <set>
#include <sstream>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "bandwidth.h"
#include "champsim.h"
#include "channel.h"
#include "core_builder.h"
#include "core_stats.h"
#include "instruction.h"
#include "micro_op_cache.h"
#include "modules.h"
#include "operable.h"
#include "register_allocator.h"
#include "trace_recorder.h"
#include "trace_segmenter.h"
#include "trace_stager.h"
#include "trace_stall.h"
#include "trace_store.h"
#include "trace_vpred.h"
#include "util/lru_table.h"
#include "util/to_underlying.h"

class CACHE;
class CacheBus
{
  using channel_type = champsim::channel;
  using request_type = typename channel_type::request_type;
  using response_type = typename channel_type::response_type;

  channel_type* lower_level;
  uint32_t cpu;

  friend class O3_CPU;

public:
  CacheBus(uint32_t cpu_idx, champsim::channel* ll) : lower_level(ll), cpu(cpu_idx) {}
  bool issue_read(request_type packet);
  bool issue_write(request_type packet);
};

struct LSQ_ENTRY : champsim::program_ordered<LSQ_ENTRY> {
  champsim::address virtual_address{};
  champsim::address ip{};
  champsim::chrono::clock::time_point ready_time{champsim::chrono::clock::time_point::max()};

  std::array<uint8_t, 2> asid = {std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max()};
  bool fetch_issued = false;

  uint64_t producer_id = std::numeric_limits<uint64_t>::max();
  std::vector<std::reference_wrapper<std::optional<LSQ_ENTRY>>> lq_depend_on_me{};

  LSQ_ENTRY(champsim::address addr, champsim::program_ordered<LSQ_ENTRY>::id_type id, champsim::address ip, std::array<uint8_t, 2> asid);
  void finish(ooo_model_instr& rob_entry) const;
  void finish(std::deque<ooo_model_instr>::iterator begin, std::deque<ooo_model_instr>::iterator end) const;
};

// cpu
class O3_CPU : public champsim::operable
{
public:
  uint32_t cpu = 0;

  // cycle
  champsim::chrono::clock::time_point begin_phase_time{};
  long long begin_phase_instr = 0;
  champsim::chrono::clock::time_point finish_phase_time{};
  long long finish_phase_instr = 0;
  champsim::chrono::clock::time_point last_heartbeat_time{};
  long long last_heartbeat_instr = 0;

  // instruction
  long long num_retired = 0;

  bool show_heartbeat = true;

  using stats_type = cpu_stats;

  stats_type roi_stats{}, sim_stats{};

  // instruction buffer (micro-op cache / decoded-instruction buffer)
  micro_op_cache DIB;

  // micro-op-cache front-end mode: STREAM = serving decoded u-ops from the DIB,
  // BUILD = fetching from L1I + decoding. Switching STREAM->BUILD costs a 1-cycle
  // fetch stall (modeled like UCP_ISCA24). See do_check_dib().
  enum class fetch_mode_type { STREAM, BUILD };
  fetch_mode_type fetch_mode{fetch_mode_type::STREAM};

  // frontend IPC-loss decomposition: true from a branch-misprediction recovery
  // until the front end is back to streaming (first u-op-cache hit). Used to
  // bucket u-op misses / build-mode stalls as recovery vs steady-state.
  bool in_recovery{false};
  // recovery-stall attribution: the class of the misprediction that started the current recovery --
  // 0 conditional, low confidence (H2P: what UCP anticipates)  1 conditional, confident  2 indirect jump/call
  // 3 return  4 direct jump/call (BTB miss)  5 other
  uint8_t rec_class = 5;
  [[nodiscard]] static uint8_t mispredict_class(const ooo_model_instr& b)
  {
    switch (b.branch) {
    case BRANCH_CONDITIONAL:
      return b.pred_h2p ? 0 : 1;
    case BRANCH_INDIRECT:
    case BRANCH_INDIRECT_CALL:
      return 2;
    case BRANCH_RETURN:
      return 3;
    case BRANCH_DIRECT_JUMP:
    case BRANCH_DIRECT_CALL:
      return 4;
    default:
      return 5;
    }
  }

  // stall segmenter trigger: a build-mode dispatch-starve cycle marks the in-flight
  // stretch costly when ROB occupancy <= this threshold. 0 = fully empty (tight);
  // higher broadens to partial (near-empty) stalls. See dispatch_instruction().
  std::size_t stall_rob_threshold{0};

  // Prometheus trace builders, fed from the post-merge u-op stream, run in
  // parallel so their coverage/trace stats can be diffed (seg_* / rec_* / stg_*):
  //   * segmenter -- backward oracle (512-ring + backward walk; not synthesizable)
  //   * recorder  -- forward armed-recorder (no history)
  //   * stager    -- staging-buffer (short ring + pos-lookup; synthesizable)
  trace_segmenter segmenter{};
  trace_recorder recorder{};
  trace_stager stager{};
  trace_stall stall{};
  bool trace_seg_enable{true};  // run the backward segmenter (resolved in ctor)
  bool trace_rec_enable{true};  // run the forward recorder   (resolved in ctor)
  bool trace_stg_enable{false};   // run the staging builder    (resolved in ctor)
  bool trace_stall_enable{false}; // run the stall-triggered segmenter (trace_builder "stall"/"all")
  int stall_mispredict_mode{0}; // trace_stall_mispredict: 0=off, 1=resolution-time probe, 2=predict-time alternate-key probe (capture identical)

  // Trace-fill: a bounded trace cache fed by the stager; on the appropriate
  // u-op-cache event install the trace's windows.  Mode set by "trace_fill"
  // config / PROMETHEUS_TRACE_FILL env (forces the stager on). See do_check_dib().
  //   OFF    - disabled
  //   MISS   - fill on a miss at a trace entry PC                  (option a)
  //   EVERY  - fill on hit-or-miss at a trace entry PC
  //   WINDOW - fill on a miss anywhere in a trace's footprint      (window-indexed)
  //   FLUSH  - fill on a branch misprediction, window-matched at the recovery PC
  //            (no demand fill; the UCP-style "fast refill" trigger)
  //   PERFECT- upper bound: hit on ANY window the active builder covers (unbounded,
  //            proactive, no store) -- the segmentation algorithm's IPC ceiling
  enum class fill_mode_type { OFF, MISS, EVERY, WINDOW, FLUSH, PERFECT, PARALLEL, ALT, HEAD, CHAIN };
  trace_store fill_store{};
  trace_store rtt_store{}; // rtt_table 1: the recovery traces, in a store of fill_store's geometry of their own -- SUCP's
                           // address probes (cursor, wrong path) never see them, whatever the key's bits
  fill_mode_type fill_mode{fill_mode_type::OFF};

  // ALT mode: metadata trace cache + timed pre-decode walk.  At decode of a
  // conditional branch, the ALTERNATE (not-predicted) direction's PC probes the
  // window-indexed trace store; a hit launches a "walk" that installs the trace's
  // windows into the u-op cache one window per cycle after a base delay
  // (PROMETHEUS_WALK_DELAY cycles ~ L1I + decode pipe fill).  Trace existence is
  // the trigger filter (traces exist only for historically stall-costly paths).
  // At most alt_walk_max walks are in flight; extra triggers are dropped.
  struct alt_walk {
    champsim::chrono::clock::time_point ready{};
    std::vector<std::vector<uint64_t>> windows; // per-window ip groups, in trace order
    std::size_t idx = 0;                        // next window to install
    uint64_t entry = 0;                         // trace entry PC (dedup)
    // real-L1I mode (trace_walk_l1i): the walk fetches its instruction bytes
    // through the actual L1I (misses propagate to L2/LLC/DRAM); a window may
    // only install once its cache line has arrived.
    std::deque<uint64_t> lines_pending;  // representative ip per unique line, not yet issued
    std::set<uint64_t> lines_ready;      // block numbers whose bytes have arrived
    champsim::chrono::clock::time_point last_progress{}; // watchdog: last install/issue (walk aborted if stuck)
    champsim::chrono::clock::time_point probe_time{};    // CHAIN: trigger time (slack = first demand touch - this)
    bool background = false; // wait-cap expired on this walk: demand no longer waits on it (pure background prefetch)
    bool ucp = false;        // UCP alt-path walk: installs into the u-op cache as prefetch-class (never the chain buffer)
    uint64_t probe_id = 0;   // CHAIN: instr_id of the probing instruction (0 = not from the instruction supply)
    bool oracle = false;     // CHAIN: probed past a branch that mispredicted (correct path a real FTQ would not hold)
    uint64_t lines_issued = 0; // line requests this walk has issued (oracle-exposure stat)
    bool wrong_path = false; // launched from the wrong path (trace_wrong_path 3): flushed at resolution
    bool recovery = false;   // launched by a recovery-key probe (trace_recovery_key)
    uint8_t vp = 0;          // launched by the trace value predictor: 1 = at a correct prediction, 2 = at a misprediction
    uint8_t chain_hop = 0;   // successor hop that launched it (trace_chain_depth; 0 = not chained)
    bool fdip_pd = false;        // fdip_predecode: rides FDIP's line (no read of its own); ready once the L1I holds it
    bool rtt = false;            // a recovery-trace walk launched at a (correct-path) prediction (rtt_cancel)
    std::vector<uint8_t> done;   // walk_ooo: windows already installed out of order (idx = first not installed)
    std::deque<uint64_t> lines_resident; // walk_resident_port: L1I-resident lines, read through the idle demand fetch port
    std::deque<std::pair<champsim::chrono::clock::time_point, uint64_t>> res_eta; // ...reads in flight (ready time, block)
    std::deque<std::pair<champsim::chrono::clock::time_point, uint64_t>> pf_wait; // trace_ucp_artifact 2: L1I prefetches issued (time, block), line not seen yet
    uint64_t ucp_branch = 0; // UCP: instr_id of the launching H2P branch (0 = wrong-path trigger)
    bool direct_install = false; // installs into the u-op cache (prefetch-class) instead of the staging buffer
  };
  std::deque<alt_walk> alt_walks;
  std::size_t alt_walk_max = 2;
  int alt_walk_delay_cycles = 5;
  int alt_walk_width = 1;      // windows installed per walk per cycle (pre-decode width)
  int alt_walk_wait_mode = 0;  // walk-pending miss policy: 0=miss through, 1=always wait (hit-under-fill),
                               // 2=wait ONLY if the window's line already arrived (install imminent; stalling
                               // on in-flight bytes measured slower than the pipelined slow path)
  bool alt_walk_l1i = false;   // walk fetches its bytes through the real L1I (install gated on line arrival)
  bool alt_walk_nofill_l1 = false;
  bool alt_walk_nofill = false; // trace_walk_nofill 1: walk line reads allocate in no cache (L1I/L2/LLC) and touch no replacement state; 2: only the L1I
  int alt_walk_wait_cap = 16;  // max cycles an instruction waits on a pending window before falling to build (0 = unbounded)
  // HEAD fill mode: trace = instantly-servable head (first head_uops uops stored
  // as uops, ~8B each) + a fetch manifest for the tail (the next tail_targets
  // taken-branch targets, ~8B each).  Head hits feed the backend directly; the
  // tail walk launches the same cycle to hide its fetch+decode latency behind
  // head consumption.
  int head_uops = 8;     // uops stored per trace head (served instantly)
  int tail_targets = 3;  // taken targets in the manifest: walk reach (<0 = unlimited)

  // CHAIN fill mode: metadata-only trace cache ([tag | window deltas | valid],
  // ~9B/entry), probed by ENTRY PC with each predicted pc at IFETCH enqueue (the
  // lookahead point).  A hit launches a walk that fetches the trace's <=
  // meta_windows windows through the real L1I and pre-decodes them into a small
  // trace-uop staging BUFFER (not the u-op cache -- no pollution).  A demand hit
  // in the buffer serves the uops and PROMOTES the window into the u-op cache as
  // a demand-class fill.
  struct uop_buf_entry {
    uint64_t tag = 0;                                  // window tag
    std::vector<uint64_t> ips;                         // the window's uops (promotion payload)
    champsim::chrono::clock::time_point probe_time{};  // walk trigger time (slack stat)
    uint64_t origin = 0;                               // entry PC of the trace whose walk staged it (launch-confidence feedback)
    bool served = false;                               // already served a demand hit (only kept resident when trace_buffer_promote = 0)
    uint64_t probe_id = 0;                             // instr_id that probed the staging walk (0 = not squashable)
    bool oracle = false;                               // staged by a walk probed past a mispredicted branch
    bool wrong_path = false;                           // staged by a walk launched from the wrong path
    bool recovery = false;                             // first staged by a recovery-key walk
    uint8_t vp = 0;                                    // first staged by a value-predictor walk (2 = fired at a misprediction)
    uint8_t chain_hop = 0;                             // first staged by a chained successor walk (its hop)
    bool late = false;                                 // trace_trigger_diag: demand had missed this window after the walk launched
    uint8_t pos = 0;                                   // trace_trigger_diag: its position among the windows its walk staged
  };
  std::deque<uop_buf_entry> uop_buffer; // trace-uop staging buffer (FIFO)
  // measurement only (no timing effect): the last UOP_BUF_EVICT_LOG windows
  // evicted from the staging buffer unused, so a later demand miss on one of
  // them can be attributed to buffer capacity ("evicted-then-missed" = the
  // opportunity of a larger FIFO).  Entries are consumed on first match.
  static constexpr std::size_t UOP_BUF_EVICT_LOG = 1024;
  std::deque<std::pair<uint64_t, champsim::chrono::clock::time_point>> uop_buf_evict_log; // (window tag, eviction time)
  std::unordered_map<uint64_t, champsim::chrono::clock::time_point> uop_buf_evict_map;    // tag -> latest eviction time
  int uop_buffer_windows = 16;          // capacity in windows (16 x 8 uops x 8B = 1KB; 0 = install direct to DIB)
  int meta_windows = 4;                 // manifest capacity: window slots per trace (entry + deltas)
  int meta_bits = 0;                    // packed manifest: slot-field bits (0 = legacy fixed slots, see chain_encode)
  int meta_short_bits = 8;              // packed manifest: short-jump signed delta width
  int meta_long_bits = 16;              // packed manifest: long-jump signed delta width (0 = no long class)
  int meta_kind_bits = 2;               // packed manifest: kind-code width (1 = seq/jump only)
  bool meta_far_truncate = false;       // unencodable jump ends the trace (prefix kept) instead of rejecting it
  // deep probe: scan the instruction supply up to N instructions AHEAD of the
  // transfer point (models the decoupled BP/FTQ running ahead of fetch -- the
  // lead a real FTQ-tail probe has over the fetch-point u-op-cache lookup, which
  // this simulator's collapsed frontend otherwise erases).  The cursor resets on
  // a branch misprediction (= FTQ flush), so recovery lead rebuilds gradually.
  // 0 = probe at enqueue only (no lead).
  int chain_probe_ahead = 0;
  int chain_probe_blocks = 1; // trace_probe_blocks: fetch blocks the cursor covers per cycle
  int ftq_lookup_window = 0;  // ftq_lookup_window: u-op-cache lookup within the first N FTQ entries (0 = at insertion)
  int bpu_blocks = 1;         // bpu_blocks: fetch blocks the predictor inserts per cycle
  bool ftq_fdip = false;      // ftq_fdip: the probe cursor (predictor) requests each predicted block's L1I line
  bool ftq_uop_prefetch = false; // ftq_uop_prefetch: walk each predicted window the u-op cache lacks
  std::size_t uop_retain_ways = 0; // uop_retain: costly u-op-cache lines protected per set (0 = plain LRU)
  bool chain_parallel = false;     // PROMETHEUS_CHAIN_PARALLEL (ceiling): a miss in a stored trace's window is served instantly
  bool ftq_fdip_filter = false;    // ftq_fdip_filter: FDIP skips lines already resident in the L1I (tag-filtered prefetch)
  bool ftq_fdip_uop_check = true;  // ftq_fdip_uop_check: FDIP skips windows the u-op cache holds (0 = L1I-only FDIP)
  // fdip_predecode: FDIP-triggered pre-decode of costly windows.  When the predictor reaches a
  // window that is costly and absent from the u-op cache, it is pre-decoded from the line FDIP
  // brings into the L1I (no read of its own) and installed into the u-op cache (or staged).
  // 1 = costly = a window of a stored SUCP trace (SUCP's own trace walks off); 2 = costly-window
  // table marked when a u-op miss leaves the ROB empty; 3 = 1 with SUCP's trace walks on.
  int fdip_predecode = 0;
  int fdip_predecode_target = 0;  // 0 = u-op cache (prefetch-class insert at LRU), 1 = trace-uop staging buffer
  std::size_t cwt_entries = 4096; // costly-window table (mode 2): 8-way, 12-bit partial tags, LRU
  struct cwt_entry {
    bool valid = false;
    uint64_t tag = 0;
    uint64_t lru = 0;
  };
  std::vector<cwt_entry> cwt;
  uint64_t cwt_clock = 0;
  uint64_t pd_last_window = std::numeric_limits<uint64_t>::max();
  uint64_t wp_pd_window = 0;
  std::vector<uint64_t> wp_pd_ips;
  static constexpr long FDIP_PD_DEADLINE_CYCLES = 256; // a pre-decode whose line never arrives is dropped
  [[nodiscard]] bool sucp_trace_walks() const { return fdip_predecode != 1; }
  // recovery-trace table (rtt_mode): the recovery path after each misprediction, recorded as it is
  // fetched, keyed by the branch and where it really went (conditionals) or by the branch alone;
  // walked (lines + pre-decode + stage) when the predictor reaches the branch again
  int rtt_mode = 0;         // 0 off; 1 trigger only at wrong predictions (oracle trigger); 2 at every prediction of a
                            // branch with a trace; 3 = 2 with conditionals only when low-confidence (H2P);
                            // 4 = at the RESOLUTION of a mispredicted branch (realistic: decode/execute redirect),
                            //     keyed by (branch, where it really went) for every branch type;
                            // 5 = 2 for low-confidence predictions only: H2P conditionals + indirect branches (no
                            //     confidence estimate); returns and direct jumps/calls (RAS/BTB) never look up -- and,
                            //     as nothing would look their traces up, their mispredictions record none
  int rtt_format = 0;       // rtt_table 1: 0 the window manifest (chain_encode); 1 direction bits -- meta_bits bits, one
                            //   per conditional branch the walker meets; the walker rebuilds the path from the BTB (targets,
                            //   branch identity), a RAS copy and the bits, so the trace ends where that would leave the path
  int rtt_table = 0;        // 0 unbounded table; 1 the SUCP trace store (trace_store_* geometry, chain_encode manifest
                            //   format, e.g. 8K entries x 12-bit manifest = 38 KB), keyed by the hashed key (never an address)
  int rtt_capture = 0;      // 0 = from the first u-op-cache miss of the recovery path to the next u-op-cache hit; 1 = fixed length
  std::size_t rtt_len = 128; // max instructions per recovery trace
  int rtt_max_windows = 0;   // >0: a stored trace keeps only its first N u-op-cache windows (0 = no cap)
  int rtt_open_rule = 0;    // which branches open a recording: 0 every misprediction; 1 low-confidence conditionals (UCP H2P,
                            // whatever the outcome) + mispredicted other branches (no direction confidence: a wrong target);
                            // 2 = 1, but a correctly predicted low-confidence conditional stores its ALTERNATE path instead
                            // (taken: the fall-through; not taken: the BTB target), generated rtt_len instructions deep;
                            // 3 = 2, opened at every low-confidence conditional OR mispredicted branch (any confidence, any type):
                            // mispredicted -> the correct path, correctly predicted low-confidence -> the alternate path
  int rtt_store_filter = 0; // 0 store every recording; 1 only if the ROB went empty while dispatch waited on one of its
                            // instructions (the branch's successors up to the recording's end); 2 = 1, waiting on a u-op-cache miss
  std::unordered_map<uint64_t, std::vector<uint64_t>> rtt;
  struct rtt_cap {          // a recovery path being recorded (a few may be open: a misprediction inside another's path)
    uint64_t pc = 0;        // the branch that opened it
    uint64_t branch_id = 0;
    uint64_t last_id = 0;   // its last instruction (set when the recording ends)
    bool costly = false;    // rtt_store_filter: the ROB went empty waiting on one of its instructions
    uint64_t stall_at_open = 0; // rtt_cost: empty-ROB cycle count when the recording opened
    bool missed = false;    // rtt_admit_miss: a recorded instruction missed the u-op cache
    bool far = false;       // rtt_admit_miss 2: ...and its line was not in the L1I either (a fetch-latency miss)
    bool cond = false;      // ...a conditional: keyed by where it really went
    bool keyed = false;
    uint64_t key = 0;
    bool started = false;   // capture 0: recording begins at the first u-op-cache miss
    bool done = false;
    std::size_t seen = 0;
    std::vector<uint64_t> ips;
    std::vector<uint8_t> flags; // per recorded instruction: 0 not a branch, 1 branch not taken, 2 branch taken
  };
  std::size_t rtt_max_open = 4; // recordings open at once (the oldest is ended early when another opens)
  std::vector<rtt_cap> rtt_open;
  std::vector<rtt_cap> rtt_wait; // rtt_store_filter: recorded, its instructions not all dispatched yet -- stored if the ROB empties on them
  void rtt_rob_empty();
  [[nodiscard]] uint64_t rtt_fall_through(uint64_t pc) const;
  [[nodiscard]] std::vector<uint64_t> rtt_alt_path(uint64_t start, std::size_t n);
  // rtt_key_target: a conditional's trace is keyed by where the recovery path starts (the alternate target) alone, so
  // every branch whose misprediction lands there shares one trace (fewer entries); default: (branch, target)
  bool rtt_key_target = false;
  // rtt_seg_stable: re-recording an existing trace keeps the new path only up to where it leaves the stored one
  //   (traces end where the recovery path stops repeating; a path that repeats in full grows back)
  bool rtt_seg_stable = false;
  // rtt_conf_table N: the 2-bit walk-confidence counters live in one N-entry table indexed by a hash of the trace key
  //   (shared, untagged) instead of one per trace entry -- saves 2 bits per entry
  int rtt_conf_table = 0;
  // rtt_admit_repeat N: a NEW trace is stored only the second time its recovery is recorded -- an N-entry direct-mapped
  //   filter of 12-bit partial key tags remembers first sightings (N x 12 bits); re-recordings of stored traces always pass
  int rtt_admit_repeat = 0;
  std::vector<uint16_t> rtt_repeat_tab;
  std::vector<uint8_t> rtt_conf_tab;
  [[nodiscard]] std::size_t rtt_conf_idx(uint64_t key) const { return static_cast<std::size_t>((key ^ (key >> 17) ^ (key >> 31)) % rtt_conf_tab.size()); }
  [[nodiscard]] int rtt_conf_get(uint64_t key) const
  {
    if (rtt_conf_table > 0) {
      return rtt_conf_tab[rtt_conf_idx(key)];
    }
    return rtt_store.conf_of(key | (uint64_t{1} << 63));
  }
  // a trace being stored: a new one starts weakly confident (as a fresh entry would); a re-recorded one gets at least 1
  void rtt_conf_on_insert(uint64_t key)
  {
    if (rtt_conf_table > 0) {
      auto& c = rtt_conf_tab[rtt_conf_idx(key)];
      c = (rtt_store.conf_of(key | (uint64_t{1} << 63)) < 0) ? uint8_t{2} : std::max<uint8_t>(c, 1);
    }
  }
  void rtt_conf_train(uint64_t key, bool up)
  {
    if (rtt_conf_table > 0) {
      auto& c = rtt_conf_tab[rtt_conf_idx(key)];
      c = up ? static_cast<uint8_t>(std::min(3, c + 1)) : static_cast<uint8_t>(std::max(0, c - 1));
    } else if (up) {
      rtt_store.conf_hit(key | (uint64_t{1} << 63));
    } else {
      rtt_store.conf_miss(key | (uint64_t{1} << 63));
    }
  }
  // rtt_seg_reconv: a recording ends where the recovery path reaches a window the wrong path already fetched (the paths
  //   reconverged: the rest is resident or in flight) -- traces keep only the divergent part
  bool rtt_seg_reconv = false;
  // rtt_cost: each trace carries the cost of its last recovery -- the empty-ROB cycles while its path was being fetched,
  //   log2-bucketed into the store's saturating cost field (trace_store_cost_bits; eviction by min cost if
  //   trace_store_cost_evict).  rtt_cost_walk T: a trace whose cost bucket is >= T walks even when rtt_conf_min would
  //   suppress it (a costly recovery is worth a speculative walk)
  bool rtt_cost = false;
  int rtt_cost_walk = 0;
  uint64_t rob_empty_total = 0; // empty-ROB cycles so far (any state)
  uint64_t rtt_census_tick = 0;
  [[nodiscard]] uint64_t rtt_key(uint64_t pc, uint64_t next) const
  {
    return rtt_key_target ? (((next << 1) ^ 0x2545f4914f6cdd1dULL) | (uint64_t{1} << 63)) : ((pc ^ (next << 1)) | (uint64_t{1} << 63));
  }
  bool rtt_on_predict(const ooo_model_instr& instr, bool wrong);
  void rtt_on_resolve(const ooo_model_instr& branch);
  bool rtt_wrong_path = false; // modes 2/3: the predictor also looks the table up for branches it predicts down a wrong path
  bool rtt_cancel = false;     // a branch that resolves CORRECTLY cancels its recovery walk (lines not yet issued) and drops
                               // the windows it staged that no demand used -- the alternate path is known useless
  bool rtt_probing = false;    // walks launched now are recovery-trace walks
  int walk_mshr_limit = 0;     // >0: walks issue a line only while the L1I has fewer MSHRs in use AND an empty prefetch
                               //     queue (FDIP and demand go first)
  int walk_pq = 0;            // >0: recovery-trace walk lines are issued as L1I PREFETCHES (prefetch queue, served after demand)
                              //   instead of gated demand reads -- no FDIP-first gate; at most this many in flight; a window
                              //   pre-decodes once its line is in the L1I
  int walk_l2_mshrs = 0;      // >0: walk line misses use this many walker MSHRs in the L1I (CACHE::walker_mshr), never a
                              //    regular MSHR; they go to L2 as ordinary reads and are not filled into the L1I.  Walks
                              //    then wait only for a free walker MSHR (the FDIP-first gate no longer applies)
  int walk_resident_port = 0; // 1: a walk reads its L1I-resident lines through the demand fetch port on cycles demand does
                              //    not use it (u-op cache streaming), one line per cycle, L1I latency -- no L1I request
                              //    queue, tag-check slot or MSHR; only missing lines take the L1I miss path.  2: such
                              //    lines are ready at once (bound)
  champsim::chrono::clock::time_point walk_res_stamp{};
  bool walk_ooo = false;       // walks install any window whose line has arrived (no head-of-line blocking behind a
                               //   slow line); default in order
  void late_window_classify(uint64_t rawip);
  int walk_prio_head = 0;      // walk_mshr_limit: the first N lines of each walk issue at once anyway (the recovery
                               //   path's head is where the stall is); the rest waits for FDIP/demand
  int rtt_conf_min = 0;        // rtt_table 1: >0 = a trace walks only if its entry's 2-bit counter is at least this; the
                               //   counter is trained at resolution of every looked-up branch (+1 mispredicted, -1 correct)
  uint64_t rtt_last_lookup_key = 0;
  bool rtt_gen_cond = false;   // conditionals ALWAYS walk the generated alternate path (no table lookup, no recording);
                               //   the table serves only branches whose alternate target the predictor cannot know (indirects)
  bool rtt_fallback = false;   // a low-confidence conditional whose alternate direction has no stored trace walks the
                               //   alternate path generated like UCP's (BTB targets, predictor directions, RAS copy, +4
                               //   through unseen code), rtt_len deep -- no storage
  int rtt_boost = 0;           // mode 5: >0 = a CONFIDENT conditional prediction also looks up, and walks if its trace
                               //   entry's 2-bit counter (trained at resolution, as rtt_conf_min) is at least this --
                               //   covers the branches TAGE is confident about yet keeps getting wrong
  // rtt_cold_seq N: a low-confidence (H2P) conditional predicted TAKEN whose fall-through has no recovery trace
  //   (no history: probably code that never ran) prefetches the first N lines of the fall-through into the L1I
  //   at prediction time (resident lines skipped).  rtt_cold_mode: 0 = no trace is the only test; 1 = and the
  //   branch never fell through (1 bit per BTB entry); 2 = and the fall-through window was never looked up (oracle)
  int rtt_cold_seq = 0;
  int rtt_cold_mode = 0;
  bool rtt_cold_target = false; // also a no-trace H2P predicted NOT taken: prefetch N lines from its BTB target
  bool rtt_cold_l2 = false;     // the prefetch fills the L2 only (no L1I allocation or MSHR): hides DRAM latency without L1I pollution
  void rtt_cold_prefetch(uint64_t pc, bool wrong_path, uint64_t target = 0);
  bool rtt_l2_keep = false;    // the first demand hit on a staged recovery window marks its line high-priority in the L1I and
                               //   L2 (EMISSARY bit; protected ways = l2_emissary_ways) so the next walk finds it in L2;
                               //   starvation-based EMISSARY marking is off
  int rtt_ret = 0;             // mode 5: >0 = returns record their recovery path when mispredicted (key = return pc), and a
                               //   return prediction walks it if the entry's 2-bit counter is at least this (as rtt_boost)
  int rtt_admit_miss = 0;      // 1: store a recording only if some instruction missed the u-op cache; 2: only if such a miss also
                               //   found its line outside the L1I (the misses that cost fetch latency).  Older note: // store a recording only if some of its instructions missed the u-op cache (an all-hit
                               //   recovery path needs no walk and would only take an entry)
  bool rtt_conf_evict = false; // train the entries' 2-bit counters at resolution (as rtt_conf_min) and evict conf-0
                               //   entries first (traces whose branch keeps resolving correctly)
  void rtt_cancel_walk(uint64_t branch_id);
  void rtt_on_wp_predict(uint64_t pc, bool cond, bool pred_taken, uint64_t btb_target, uint64_t fall_through, bool h2p, uint8_t type);
  [[nodiscard]] std::size_t rtt_dir_encode(const std::vector<uint64_t>& ips, const std::vector<uint8_t>& flags);
  [[nodiscard]] static bool rtt_low_conf_type(uint8_t type) { return type == BRANCH_INDIRECT || type == BRANCH_INDIRECT_CALL; }
  const std::vector<uint64_t>* rtt_lookup(uint64_t key);
  void rtt_capture_step(const ooo_model_instr& instr, bool uop_hit);
  void rtt_commit(rtt_cap& c);
  uint64_t rtt_cost_bucket(const rtt_cap& c);
  bool rtt_diag = false; // PROMETHEUS_RTT_DIAG: at each ROI misprediction, compare the stored trace with the real recovery path
  void rtt_diag_check(const ooo_model_instr& branch);
  bool oracle_recovery_seen = false;       // PROMETHEUS_ORACLE_RECOVERY_SEEN: the oracle walks only windows fetched before (history-reachable)
  std::unordered_set<uint64_t> seen_win;   // windows looked up at the u-op cache so far (warmup included)
  std::size_t oracle_recovery = 0; // PROMETHEUS_ORACLE_RECOVERY (diagnostic ceiling): when the predictor makes a wrong
                                   // prediction, walk the next N instructions of the REAL path at once
  unsigned oracle_recovery_classes = 0x3f; // PROMETHEUS_ORACLE_RECOVERY_CLASSES: bit mask of mispredict_class() values the oracle walks for
  bool cwt_find(uint64_t window, bool touch);
  void cwt_mark(uint64_t window);
  [[nodiscard]] bool window_costly(uint64_t ip);
  void fdip_predecode_window(const std::vector<uint64_t>& ips, bool wrong_path);
  [[nodiscard]] bool walk_line_ready(const alt_walk& w, uint64_t ip) const;
  bool ftq_shared_wp = false;      // ftq_shared_wp: the wrong path shares the FTQ with the correct path queued ahead of the
                                   // branch, at the predictor's block rate (implied by fetch_buffer_blocks > 0)
  bool stall_taxonomy = false;     // PROMETHEUS_STALL_TAXONOMY: charge starve cycles by recovery / first touch / serving level
  // instruction-side L2 study (JSON l2_perfect_instr / l2_emissary_ways / l2_emissary_rand)
  int l2_perfect_instr = 0;          // ceiling at L2: 1 = re-requested instruction lines always hit, 2 = all instruction lines
  std::size_t l2_emissary_ways = 0;  // EMISSARY P(N) at L2: N protected high-priority ways per set (0 = off)
  uint32_t l2_emissary_rand = 32;    // ...a starving L1I miss is marked with probability 1/r
  CACHE* l2c = nullptr;              // this core's L2 (found by name once the caches run)
  bool l2_bound = false;
  std::minstd_rand emissary_rng{0x5eed};
  void emissary_mark(champsim::address ip);
  // resteer prefetcher (JSON rp_*; PDIP-style): lines that stalled the front end on the resteer path of a
  // mispredicted branch are filed under the branch's line; when the predictor reaches that line again
  // they are prefetched into the L1I.  1 = PDIP: lines whose L1I miss starved the backend; 2 = SUCP path:
  // every L1I-missed line of a resteer window that starved the backend
  int rp_mode = 0;
  std::size_t rp_sets = 512;
  std::size_t rp_ways = 8;
  std::size_t rp_targets = 2;        // target lines per entry, each with a 4-bit mask of the next 4 lines
  int rp_window = 64;                // instructions after the resteer that count as its path
  struct rp_entry {
    bool valid = false;
    uint64_t tag = 0;
    uint64_t lru = 0;
    std::vector<std::pair<uint64_t, uint8_t>> targets; // (line number, mask of the next 4 lines)
  };
  std::vector<rp_entry> rp_table;
  uint64_t rp_clock = 0;
  uint64_t rp_win_key = 0;           // trigger of the newest resteer window
  int rp_left = 0;                   // instructions still counted in it
  struct rp_win_state {
    uint64_t key = 0;
    bool starved = false;            // SUCP path mode: the window starved the backend (its lines are filed)
    std::vector<uint64_t> lines;     // ...L1I-missed lines seen before that
  };
  std::deque<rp_win_state> rp_windows;  // recent resteer windows (the predictor runs ahead: misses of an older
                                     // window can still be in flight when the next one opens)
  uint64_t rp_last_lookup = 0;
  rp_entry* rp_find(uint64_t key, bool allocate);
  void rp_add_target(rp_entry& e, uint64_t line);
  void rp_open(uint64_t branch_ip);
  rp_win_state* rp_win(uint64_t key);
  void rp_path_line(uint64_t key, uint64_t line);
  void rp_path_starved(uint64_t key);
  void rp_lookup(uint64_t ip);
  std::unordered_set<uint64_t> tax_seen; // stall taxonomy: windows looked up so far
  uint64_t fup_last_window = ~uint64_t{0};
  int fetch_buffer_blocks = 0; // fetch_buffer_blocks: fetch blocks allowed in the fetch buffer (0 = no block limit)
  uint64_t fdip_last_line = 0;
  uint64_t fdip_last_window = 0;
  std::size_t chain_probe_cursor = 0; // next unprobed input_queue position (relative to front)
  // The instruction supply is the correct path, so past a branch that will
  // mispredict the cursor sees PCs a real FTQ would not hold until resolution.
  // chain_shadow_branch = instr_id of the mispredicted branch awaiting resolution
  // (0 = none); walks probed past it are flagged oracle.  trace_probe_squash 1
  // squashes them at the misprediction and freezes the cursor until resolution.
  int chain_probe_squash = 0;
  uint64_t chain_shadow_branch = 0;
  // trace_wrong_path: what the frontend does past a mispredicted branch.
  // 0 = legacy: the cursor keeps reading the instruction supply (the correct
  //     path) past the branch; no wrong path exists.
  // 1 = the predictor runs at the cursor (one call per instruction in program
  //     order, so the prediction sequence is unchanged), so the cursor stops at a
  //     branch it predicts wrong and resumes at resolution: no oracle, no wrong path.
  // 2 = 1 + a synthetic wrong path (BTB targets, main-predictor directions, +4
  //     sequential steps: the UCP_ISCA24 artifact's model) occupies the frontend
  //     until resolution -- it holds BTB banks against UCP walks and, when it
  //     misses the u-op cache, the decoders against all walks.
  // 3 = 2 + the wrong path is fetched like the correct path (u-op-cache misses
  //     fetch their lines through the L1I, fetch waits on lines in flight and
  //     stops when the backend is full) and triggers prefetches like it: SUCP
  //     probes the trace store and UCP fires on H2P branches along it.  Walks
  //     still in flight at resolution are flushed; fetched lines stay.
  int wrong_path_level = 0;
  // trace_wrong_path "ucp": level 1 + the UCP_ISCA24 artifact's prediction rule --
  // each cycle the predictor stops after the first taken branch or after the
  // second branch (one fetch block, at most two branches, per cycle).  Applies to
  // SUCP's cursor and to the fetch-point predictor (the predictor of the baseline
  // and UCP configs; in SUCP configs the FTQ-to-fetch step).
  bool fetch_block_rule = false;
  // trace_wrong_path "ucp2": "ucp" + the level-3 wrong path (FTQ, demand fetch with
  // L1I requests, backend cap, SUCP/UCP triggers, flush at resolution), made exact:
  // the wrong path walks real instruction addresses (each instruction's successor
  // learned from the correct path; it stops at code the trace never executed or an
  // unknown target), follows the fetch-block rule, and reads and fills the u-op
  // cache like demand (wrong-path pollution).  Predictor history is not updated.
  bool wp_exact = false;
  bool wp_uop_fill = false;
  bool wp_gen_stopped = false;                      // the wrong path cannot continue (unseen code / unknown target)
  std::unordered_map<uint64_t, uint64_t> code_next; // learned sequential successor of each executed instruction
  uint64_t code_prev_ip = 0;
  bool code_prev_seq = false;                       // the previous fetched instruction falls through to the next
  bool chain_cursor_blocked = false; // level >= 1: the predictor went down a wrong path; the cursor waits for resolution
  // trace_oracle_peek (diagnostic ceiling): the real path past the stopping branch
  int oracle_peek = 0;               // instructions past it probed (0 = off)
  std::size_t chain_peek_cursor = 0; // next one, indexed from the input-queue front like the cursor
  long chain_peek_left = 0;          // still to probe for this misprediction
  bool chain_peek_probing = false;   // walks launched now are peek walks (flagged past-mispredict)
  // trace_trigger_diag (diagnostic): why stored traces did not serve their misses.
  // The cursor's probe of a trace entry records its outcome on the instruction;
  // demand reaching the entry opens an "occurrence" (the trace's windows); every
  // u-op-cache miss is classified, and dispatch-starve cycles are charged to the
  // class of the miss dispatch waits for.  Measurement only.
  enum : uint8_t { TRIG_NONE = 0, TRIG_RESIDENT, TRIG_CONF, TRIG_CAP, TRIG_DEDUP, TRIG_LAUNCHED };
  enum : uint8_t {
    TD_UNCLASSIFIED = 0,
    TD_UNCOVERED,        // window in no segmenter trace (window history off; else split into TD_COLD..TD_COSTLY_REPEAT)
    TD_NOT_STORED,       // captured by the segmenter, not in the store
    TD_SIDE_DOOR,        // in a stored trace, but demand did not enter it through its entry
    TD_NOT_PROBED,       // occurrence whose entry the cursor never found in the store
    TD_DROPPED,          // entry probed, walk not launched (walk cap / launch confidence)
    TD_RESIDENT_AT_PROBE, // entry probed, window resident then -> no walk; gone by demand
    TD_LATE,             // walk in flight, window not staged yet
    TD_STAGED_EVICTED,   // window staged, evicted from the buffer before demand
    TD_WALKED_GONE,      // walk launched, window neither pending nor staged (promoted+evicted, filtered, aborted)
    // in no trace, by the window's history:
    TD_COLD,             // first fetch of this window ever (compulsory)
    TD_PAST_DEPTH,       // costly stretch past the depth cap (now, or the only costly sighting)
    TD_NEVER_COSTLY,     // fetched before, never in a costly stretch (its misses never emptied the ROB)
    TD_COSTLY_ONCE,      // in one costly stretch so far (the occurrence filter wants a second)
    TD_COSTLY_REPEAT     // in several costly stretches, still in no trace (different starts / another path captured)
  };
  bool trigger_diag = false;
  uint64_t td_occ_entry = 0;          // the stored trace demand is currently inside (0 = none)
  std::vector<uint64_t> td_occ_wtags; // its windows
  uint8_t td_occ_probe = TRIG_NONE;   // what the cursor did at its entry
  std::unordered_set<uint64_t> td_seen_windows; // windows fetched so far (warmup included)
  std::unordered_map<uint64_t, champsim::chrono::clock::time_point> td_last_miss; // window -> its latest demand miss
  bool td_in_run = false;             // the last fetch missed: the current miss run's first miss was classified
  uint8_t td_run_head = TD_UNCLASSIFIED;
  void trigger_diag_probe(ooo_model_instr& probed, uint8_t result);
  void fdip_note(uint64_t ip, bool wrong_path); // ftq_fdip: a block entered the FTQ
  void trigger_diag_check(ooo_model_instr& instr, bool dib_hit, bool buf_hit, bool staged_evicted);
  // trace_peek_diag (diagnostic): next-trace predictability at mispredictions
  static constexpr std::size_t PEEK_KEYS = 8; // pc, +ghr 8/16/32/64, +trace history 1/2/4
  static constexpr uint64_t PEEK_TRUTH_WINDOW = 192;
  bool peek_diag = false;
  std::array<std::unordered_map<uint64_t, uint64_t>, PEEK_KEYS> pdiag_table; // key -> trace entry that followed
  std::array<uint64_t, PEEK_KEYS> pdiag_pend_key{};
  std::array<uint64_t, PEEK_KEYS> pdiag_pend_pred{}; // the trace each key named at the pending misprediction (0 = none)
  bool pdiag_pending = false;
  uint64_t pdiag_pend_id = 0;
  uint64_t pdiag_ghr = 0;                 // conditional outcomes, program order (newest in bit 0)
  std::array<uint64_t, 4> pdiag_th{};     // last trace entries the cursor hit, newest first
  uint64_t pdiag_th_last_id = 0;
  // trace_recovery_key: recovery traces filed under (branch, predicted direction, GHR)
  bool rk_enable = false;
  int rk_ghr_bits = 0;
  bool rk_h2p = false;
  uint64_t rk_ghr = 0;          // conditional outcomes in program order (newest in bit 0)
  bool rk_pending = false;      // a misprediction waits for the first trace after it
  uint64_t rk_pend_id = 0;
  uint64_t rk_pend_key = 0;
  bool rk_probing = false;      // walks launched now come from a recovery-key probe
  // trace_vpred: TAGE-organized predictor of the next costly trace
  bool vp_enable = false;
  trace_vpred vpred{};
  uint64_t vp_ghr = 0;          // conditional outcomes in program order (newest in bit 0)
  uint8_t vp_probing = 0;       // walks launched now: 1 = vpred at a correct prediction, 2 = at a misprediction
  // trace_chain_depth: successor-pointer chaining
  int chain_depth = 0;
  uint64_t chain_window = 0;
  bool chain_prev_valid = false; // the previous costly stretch (program order)
  uint64_t chain_prev_first = 0;
  uint64_t chain_prev_end_id = 0;
  uint8_t chain_hop_probing = 0; // walks launched now: hop number of a successor walk (0 = not chained)
  // u-op-cache miss-cost profile (PROMETHEUS_MISS_PROFILE): every empty-ROB build-mode
  // stall cycle is charged to the window of the instruction dispatch waits for, when
  // that instruction missed the u-op cache
  bool miss_profile = false;
  unsigned miss_window_bits = 5;
  struct miss_site {
    uint64_t misses = 0;
    uint64_t cost = 0; // empty-ROB cycles charged
  };
  std::unordered_map<uint64_t, miss_site> miss_sites;
  uint64_t miss_prof_other = 0; // empty-ROB cycles whose oldest front-end instruction hit or was not looked up yet
  int uop_cost_mode = 0;          // per-workload costly-window oracle, no env needed: 1 = profile pass (dump the windows ranked
                                  // by empty-ROB cycles to <dir>/<trace>.win), 2 = oracle pass (the top uop_cost_windows of
                                  // that file always hit, held outside the u-op cache); dir = PROMETHEUS_PROFILE_DIR or
                                  // $HOME/uop_cost_profiles
  long uop_cost_windows = 75;     // ~5 KB at 8 u-ops x 8 B + tag per window
  // uop_ideal_split (diagnostic oracle, bit mask): a u-op-cache miss of the selected kind is served as a hit --
  //   1 steady state, line in the L1I   2 steady state, line not in the L1I
  //   4 within uop_ideal_split_window instructions after a misprediction redirect, line in the L1I   8 ... line not in the L1I
  // 15 ~ the ideal u-op cache; the four masks split its gain into decode-only vs fetch-latency, steady vs recovery
  // +16: only windows looked up before (warmup included) -- what a history-based scheme could reach
  int uop_ideal_split = 0;
  long uop_ideal_split_window = 64;
  uint64_t last_redirect_id = 0; // instr_id of the latest mispredicted branch (start of the recovery path)
  // PROMETHEUS_COLD_DIAG (timing-neutral): u-op misses whose line is not in the L1I within the first 64
  // instructions after a misprediction, split seen-before / never-seen, by line distance from the redirect
  // target and by the mispredicted branch's class
  bool cold_diag = false;
  uint64_t redirect_target_line = 0;
  [[nodiscard]] static std::string uop_cost_file();
  std::string miss_dump_path;   // PROMETHEUS_MISS_DUMP: windows ranked by cost, for the oracle pass (+ <path>.traces)
  long trace_oracle_n = 0;      // PROMETHEUS_TRACE_ORACLE: traces whose windows always hit
  static bool oracle_install_env() // PROMETHEUS_ORACLE_INSTALL: oracle windows are filled on demand (take capacity)
  {
    const char* e = std::getenv("PROMETHEUS_ORACLE_INSTALL");
    return e != nullptr && *e != '\0' && std::atoi(e) != 0;
  }
  void miss_profile_finalize();
  bool wp_active = false;            // level >= 2: a wrong path is being generated (mispredicted branch unresolved)
  bool wp_fetching = false;          // ...and fetch has reached the branch: the wrong path is being fetched
  bool wp_decoders_busy = false;     // this cycle's wrong-path fetch missed the u-op cache (decoders in build mode)
  uint64_t wp_next = 0;              // next wrong-path address the predictor generates
  struct wp_entry {
    uint64_t addr = 0;
    bool checked = false; // level 3: u-op-cache tag check done
    bool miss = false;    // level 3: missed the u-op cache and staging buffer (needs its line + decoders)
    bool branch = false;  // ucp2: the BTB identifies a branch here
    bool taken = false;   // ucp2: predicted taken (ends the fetch block and the u-op-cache entry)
  };
  std::deque<wp_entry> wp_ftq;       // wrong-path addresses generated, not yet fetched
  std::size_t wp_issue_idx = 0;      // level 3: entries [0, idx) are tag-checked and have their line requested
  std::size_t wp_consumed = 0;       // level 3: wrong-path instructions fetched into the backend (until the flush)
  std::unordered_set<uint64_t> wp_lines_requested; // level 3: blocks the wrong path asked the L1I for
  // rtt_seg_reconv: the u-op windows the wrong path of the current / latest misprediction fetched
  std::unordered_set<uint64_t> wp_windows_fetched;
  std::unordered_set<uint64_t> last_wp_windows;
  uint64_t wp_branch_id = 0;
  uint64_t last_wp_branch = 0;
  std::unordered_set<uint64_t> wp_lines_ready;     // level 3: blocks that arrived during this wrong path
  long l1i_demand_issued = 0;        // correct-path L1I requests fetch_instruction issued this cycle
  std::vector<uint64_t> wp_ras;      // wrong-path return stack (copy of the RAS at the misprediction)
  static constexpr uint64_t WP_STEP = 4; // sequential wrong-path step (artifact; exact on ARM, approximate on x86)
  // per-window residency filter: at walk launch, probe each of the trace's
  // windows in the u-op cache and the staging buffer, and fetch/stage ONLY the
  // absent ones -- kills redundant staging (walks shorten, slots free up).
  bool alt_walk_filter = false;
  int chain_buffer_promote = 1;        // trace_buffer_promote: on a staging-buffer hit, 1 = fill the u-op cache at MRU and drop the
                                       // entry; 0 = serve from the buffer only (no fill, entry stays); 2 = fill at LRU (prefetch class)
  int chain_launch_conf = 0;           // trace_launch_conf: 0 off, 1 gate launches on conf > 0, 2 = also conf-aware eviction

  // UCP port (Singh et al., ISCA'24 "Alternate Path u-op Cache Prefetching"):
  // on a hard-to-predict conditional (TAGE-SC-L confidence, classification
  // published via ucp_hooks.h), walk the NOT-predicted path -- targets from the
  // real BTB, directions from a dedicated 8KB Alt-BP (ucp_alt_tage.h), returns
  // from a snapshot Alt-RAS -- and prefetch its windows into the u-op cache as
  // prefetch-class fills through the existing alt-walk machinery (bytes through
  // the real L1I when trace_walk_l1i is set).  Independent of trace_fill; off by
  // default; the paper's no-Alt-Ind flavor (walks stop at indirect branches).
  bool ucp_enable = false;
  int ucp_threshold = 500;   // Table-I weighted stop counter limit (paper H2P_T)
  int ucp_max_ip_check = 64; // stop after this many straight-line steps without a branch (paper MAX_IP_CHECK)
  int ucp_step = 4;          // instruction stride on the alt path (ARM traces: 4B)
  class TAGE_PREDICTOR_8KB* ucp_altbp = nullptr; // dedicated Alt-BP, allocated on first branch
  bool ucp_altind_enable = false;      // 12.95KB flavor: 4KB Alt-ITTAGE walks through indirect branches
  // trace_ucp_artifact (bit mask) -- UCP_ISCA24 artifact semantics for UCP walks (7 = all):
  //   1 one alternate path at a time: a new trigger drops the lines the previous path has not issued yet
  //     (stop_previous_alternate_path); lines already requested still decode and install (uop-MSHR entries)
  //   2 lines requested as L1I PREFETCHES (PQ, prefetch priority, like FDIP), 1 line per cycle, at most
  //     32 in flight (UOP_MSHR); a window decodes once its line is in the L1I (fill or PQ hit), in fill order
  //   4 dedicated alternate decoders: DECODE_WIDTH instructions per cycle, not the demand decoders' idle slots
  // any bit: UCP windows install out of order (whichever line arrived), as the artifact's fill-order decode
  int ucp_artifact = 0;
  static constexpr int UCP_ART_LINES_PER_CYCLE = 1; // operate_on_uop_mshr_queue: to_read = 1
  static constexpr std::size_t UCP_ART_MSHR = 32;   // defines.h UOP_MSHR
  static constexpr long UCP_ART_L1I_LATENCY = 4;    // cycles: a requested line is decodable no sooner than an L1I access
  champsim::chrono::clock::time_point ucp_pf_stamp{};
  int ucp_pf_left = 0;
  // the uop-cache MSHR (UOP_CACHE_MSHR): one entry per requested line, freed in FIFO order once the line
  // arrived AND the H2P branch that launched its path resolved (pref_done && branch_resolved)
  struct ucp_mshr_entry {
    uint64_t blk = 0;
    uint64_t branch = 0; // launching branch instr_id (0 = a wrong-path trigger: no resolution to wait for)
    champsim::chrono::clock::time_point issued{};
    bool filled = false;
  };
  std::deque<ucp_mshr_entry> ucp_mshr;
  std::unordered_set<uint64_t> ucp_unresolved; // launching branches not yet executed
  uint64_t ucp_trigger_branch = 0;             // set by the caller of do_ucp_trigger (correct-path trigger)
  static constexpr long UCP_ART_MSHR_TIMEOUT = 20000; // cycles: free an entry whose line never arrives (the artifact would deadlock)
  champsim::chrono::clock::time_point ucp_dec_stamp{};
  long ucp_dec_left = 0;
  void ucp_stop_previous();
  [[nodiscard]] bool ucp_window_pending(uint64_t rawip) const;
  [[nodiscard]] bool ucp_decode_available();
  int ucp_btb_banks = 0;               // trace_ucp_btb_banks: >0 = model N BTB banks; 0 = ideal (artifact IDEAL_BTB_BANKING)
  bool trace_warmup_train = false;     // trace_warmup_train: chain/UCP train during warmup (artifact parity); default cold-start at ROI
  uint64_t ucp_demand_bank_mask = 0;   // BTB banks touched by demand predictions this cycle
  champsim::chrono::clock::time_point ucp_bank_mask_stamp{}; // cycle the mask belongs to
  // global walk line-issue port model: at most this many walk line reads may enter
  // the L1I per cycle, ACROSS all walks and trigger-time bursts (UCP's paper budgets
  // 1 prefetch/cycle).  0 = legacy schema (per-walk 1/cycle, unlimited aggregate).
  int alt_walk_issue_cap = 0;
  int walk_issue_count = 0;                              // issues consumed this cycle
  champsim::chrono::clock::time_point walk_issue_stamp{}; // cycle the count belongs to
  // walk pre-decode model: at most trace_walk_decode_cap windows may install per
  // cycle ACROSS all walks (the width of the decode resource serving the walks;
  // 1 window = 8 uops ~= a 6-8 wide decoder).  0 = legacy per-walk schema.
  // trace_walk_decode_shared additionally requires the DEMAND path to be in
  // stream mode (u-op-cache hits, regular decoders idle) for walks to install at
  // all -- UCP's "SharedDecoders" winner-takes-all rule: no dedicated hardware,
  // walks borrow the existing decoders in their idle cycles only.
  int alt_walk_decode_cap = 0;
  int alt_walk_decode_shared = 0;     // 1 = walks pre-decode only in stream mode; 2 = also when demand left the decoders idle this cycle
  uint64_t demand_decoded_now = 0;    // legacy-decoded instructions this cycle (decode runs before the walks drain)
  uint64_t wp_decoded_now = 0;        // wrong-path instructions decoded this cycle (trace_wrong_path >= 2; wp_fetch runs before the walks drain)
  int walk_decode_count = 0;                               // installs consumed this cycle
  long walk_decode_slots = 0;                              // decode_shared 2: decode width demand left this cycle
  champsim::chrono::clock::time_point walk_slots_stamp{};  // cycle the slots belong to
  champsim::chrono::clock::time_point walk_decode_stamp{}; // cycle the count belongs to
  class alt_ittage* ucp_altind = nullptr; // dedicated Alt-Ind, allocated on first branch

  static constexpr int ALT_WALK_ABORT_CYCLES = 4096; // watchdog: abort a walk with no progress for this long
  static constexpr int STALL_L1I_MISS_CYCLES = 8;    // fetch completion slower than this => it missed L1I (hit ~4-6 cyc end-to-end)
  unsigned dib_window_bits = 0;

  // Resolve the trace-fill mode from the "trace_fill" config field, overridden
  // by PROMETHEUS_TRACE_FILL when set.  Accepts off/miss/every/window (and the
  // legacy 0/1 -> off/miss for back-compat).
  static fill_mode_type resolve_fill_mode(const std::string& cfg)
  {
    const char* e = std::getenv("PROMETHEUS_TRACE_FILL");
    const std::string v = (e != nullptr && *e != '\0') ? std::string{e} : cfg;
    if (v == "every") {
      return fill_mode_type::EVERY;
    }
    if (v == "window") {
      return fill_mode_type::WINDOW;
    }
    if (v == "flush") {
      return fill_mode_type::FLUSH;
    }
    if (v == "perfect") {
      return fill_mode_type::PERFECT;
    }
    if (v == "parallel") {
      return fill_mode_type::PARALLEL;
    }
    if (v == "alt") {
      return fill_mode_type::ALT;
    }
    if (v == "head") {
      return fill_mode_type::HEAD;
    }
    if (v == "chain") {
      return fill_mode_type::CHAIN;
    }
    if (v == "miss" || v == "1") {
      return fill_mode_type::MISS;
    }
    return fill_mode_type::OFF; // "off", "0", "" or anything else
  }

  // Resolve which trace builder(s) run from the "trace_builder" config field,
  // overridden by the PROMETHEUS_TRACE env var when set.  Returns {seg, rec, stg}.
  // Accepted values: "both" (default = seg+rec), "backward"/"seg", "forward"/"rec",
  // "staging"/"stg", "all" (seg+rec+stg), "off"/"none".
  static std::tuple<bool, bool, bool> resolve_trace_builder(const std::string& cfg)
  {
    const char* e = std::getenv("PROMETHEUS_TRACE");
    const std::string v = (e != nullptr && *e != '\0') ? std::string{e} : cfg;
    const bool seg = (v == "both" || v == "backward" || v == "seg" || v == "all");
    const bool rec = (v == "both" || v == "forward" || v == "rec" || v == "all");
    const bool stg = (v == "staging" || v == "stg" || v == "all");
    return {seg, rec, stg};
  }

  // reorder buffer, load/store queue, register file
  std::deque<ooo_model_instr> IFETCH_BUFFER;
  std::deque<ooo_model_instr> DISPATCH_BUFFER;
  std::deque<ooo_model_instr> DECODE_BUFFER;
  std::deque<ooo_model_instr> ROB;
  std::deque<ooo_model_instr> DIB_HIT_BUFFER;

  std::vector<std::optional<LSQ_ENTRY>> LQ;
  std::deque<LSQ_ENTRY> SQ;

  // Constants
  const std::size_t IFETCH_BUFFER_SIZE, DISPATCH_BUFFER_SIZE, DECODE_BUFFER_SIZE, REGISTER_FILE_SIZE, ROB_SIZE, SQ_SIZE, DIB_HIT_BUFFER_SIZE;
  champsim::bandwidth::maximum_type FETCH_WIDTH, DECODE_WIDTH, DISPATCH_WIDTH, SCHEDULER_SIZE, EXEC_WIDTH, DIB_INORDER_WIDTH;
  champsim::bandwidth::maximum_type LQ_WIDTH, SQ_WIDTH;
  champsim::bandwidth::maximum_type RETIRE_WIDTH;
  champsim::chrono::clock::duration BRANCH_MISPREDICT_PENALTY;
  champsim::chrono::clock::duration DISPATCH_LATENCY;
  champsim::chrono::clock::duration DECODE_LATENCY;
  champsim::chrono::clock::duration SCHEDULING_LATENCY;
  champsim::chrono::clock::duration EXEC_LATENCY;
  champsim::chrono::clock::duration DIB_HIT_LATENCY;

  champsim::bandwidth::maximum_type L1I_BANDWIDTH, L1D_BANDWIDTH;

  RegisterAllocator reg_allocator{REGISTER_FILE_SIZE};

  // branch
  champsim::chrono::clock::time_point fetch_resume_time{};

  // instruction-supply lookahead (FTQ model): json ftq_depth > 0 overrides the
  // legacy 2 x fetch_width; env PROMETHEUS_FTQ_DEPTH overrides both.  This is
  // the decoupled-BPU runahead depth -- probes (trace_probe_ahead) are clamped
  // by it, and it is truncated at every mispredict (cursor reset), so direction
  // is oracle only within the inter-mispredict window.
  static std::size_t resolve_ifetch_size(std::size_t json_size)
  {
    if (const char* e = std::getenv("PROMETHEUS_IFETCH_SIZE"); e != nullptr && *e != '\0' && std::atol(e) > 0) {
      return static_cast<std::size_t>(std::atol(e));
    }
    return json_size;
  }
  static long resolve_ftq_depth(int json_depth, long legacy)
  {
    int d = json_depth;
    if (const char* e = std::getenv("PROMETHEUS_FTQ_DEPTH"); e != nullptr && *e != '\0') {
      d = std::atoi(e);
    }
    return (d > 0) ? d : legacy;
  }
  const long IN_QUEUE_SIZE;
  std::deque<ooo_model_instr> input_queue;

  CacheBus L1I_bus, L1D_bus;
  CACHE* l1i;

  void initialize() final;
  long operate() final;
  void begin_phase() final;
  void end_phase(unsigned cpu) final;

  void initialize_instruction();
  long check_dib();
  void drain_alt_walks();
  void do_alt_trigger(uint64_t alt_pc);
  void do_head_trigger(const std::vector<uint64_t>& ips, std::size_t start_idx);
  uint8_t do_chain_trigger(const std::vector<uint64_t>& ips, uint64_t probe_id = 0, bool wrong_path = false); // returns TRIG_*
  void chain_mispredict_shadow(uint64_t branch_id);
  void peek_diag_branch(const ooo_model_instr& instr, bool mispredicted); // trace_peek_diag: fire + record at a prediction
  void peek_diag_probe(uint64_t instr_id, uint64_t hit_entry);           // ...and learn the truth from the cursor's probes
  void peek_diag_finalize(uint64_t truth);
  uint64_t recovery_key(const ooo_model_instr& instr) const;            // trace_recovery_key
  void recovery_on_predict(const ooo_model_instr& instr, bool mispredicted);
  void recovery_learn(uint64_t instr_id, const std::vector<uint64_t>* ips);
  void vpred_on_predict(const ooo_model_instr& instr, bool mispredicted); // trace_vpred: look up + walk at a prediction
  void vpred_train_from_stall();                                         // ...train on the segmenter's costly verdicts
  void chain_follow(uint64_t entry, uint64_t probe_id);                  // trace_chain_depth: walk successors of a hit trace
  void uop_buffer_push(const std::vector<uint64_t>& window_ips, champsim::chrono::clock::time_point probe_time, uint64_t origin, uint64_t probe_id = 0,
                       bool oracle = false, bool wrong_path = false, bool recovery = false, uint8_t vp = 0, uint8_t chain_hop = 0, uint8_t pos = 0);
  void predict_instruction(ooo_model_instr& instr);              // BTB + direction lookup (no training)
  [[nodiscard]] bool branch_mispredicts(const ooo_model_instr& instr) const;
  bool predict_early(ooo_model_instr& instr);                    // level >= 1: predict + train at the cursor
  void wp_begin(const ooo_model_instr& branch);                  // level >= 2: start the wrong path past a mispredicted branch
  void wp_generate();                                            // predictor side: extend the wrong path
  void wp_fetch();                                               // fetch side: consume it (u-op cache / decoder occupancy)
  void wp_fetch_lines();                                         // level 3: fetch with L1I lines and backend backpressure
  [[nodiscard]] bool uop_resident(uint64_t ip) const;            // window in the u-op cache or the staging buffer
  void wp_resolve();                                             // resolution: flush the wrong path and its walks
  void wp_ucp_trigger(uint64_t pc, bool predicted_taken, uint64_t target); // level 3: UCP on a wrong-path H2P branch
  [[nodiscard]] std::vector<uint64_t> chain_encode(const std::vector<uint64_t>& ips, bool& truncated, bool& encodable);
  [[nodiscard]] bool alt_window_pending(uint64_t rawip) const;
  [[nodiscard]] alt_walk* alt_pending_walk(uint64_t rawip); // like alt_window_pending, but skips background walks
  [[nodiscard]] bool walk_wait_worthwhile(const alt_walk* holder, uint64_t rawip) const; // wait-mode-2 gate: line already arrived?
  [[nodiscard]] bool walk_issue_available(uint64_t walk_lines_issued = std::numeric_limits<uint64_t>::max()); // under the issue cap this cycle?
  void walk_issue_note();                    // consume one issue slot (on successful issue)
  void ucp_note_demand_bank(uint64_t ip);          // demand prediction occupies its BTB bank this cycle
  [[nodiscard]] bool ucp_bank_conflict(uint64_t ip); // is this IP's BTB bank held by demand this cycle?
  [[nodiscard]] bool walk_decode_available(std::size_t uops = 1); // may a walk install a window (of this many uops) this cycle?
  void walk_decode_note(std::size_t uops = 1);                    // consume one install slot
  void do_ucp_branch(const ooo_model_instr& arch_instr, uint64_t btb_target, bool always_taken, bool h2p);
  [[nodiscard]] std::vector<uint64_t> ucp_generate_alt_path(uint64_t pc, uint8_t branch_type, bool prediction, uint64_t start);
  void do_ucp_trigger(uint64_t h2p_ip, const std::vector<uint64_t>& ips, bool wrong_path = false);
  long fetch_instruction();
  long promote_to_decode();
  long decode_instruction();
  long dispatch_instruction();
  void feed_stall_traces(std::size_t before); // push stall-segmenter commits/re-captures into the trace store
  void advance_stall_attrib();                // dispatch-side attribution: finalize stretches dispatch has passed
  long schedule_instruction();
  long execute_instruction();
  long operate_lsq();
  long complete_inflight_instruction();
  long handle_memory_return();
  long retire_rob();

  bool do_init_instruction(ooo_model_instr& instr);
  bool do_predict_branch(ooo_model_instr& instr);
  void do_check_dib(ooo_model_instr& instr);
  void do_flush_fill(const ooo_model_instr& branch); // FLUSH mode: install recovery-PC trace on a mispredict
  void do_event_trigger(const ooo_model_instr& branch); // mispredict-keyed probe + capture arm (trace_stall_mispredict)
  void finalize_coverage_stats(); // end-of-phase: derived trace-coverage attribution
  bool do_fetch_instruction(std::deque<ooo_model_instr>::iterator begin, std::deque<ooo_model_instr>::iterator end);
  void do_dib_update(const ooo_model_instr& instr);
  void do_scheduling(ooo_model_instr& instr);
  void do_execution(ooo_model_instr& instr);
  void do_memory_scheduling(ooo_model_instr& instr);
  void do_complete_execution(ooo_model_instr& instr);
  void do_sq_forward_to_lq(LSQ_ENTRY& sq_entry, LSQ_ENTRY& lq_entry);

  void do_finish_store(const LSQ_ENTRY& sq_entry);
  bool do_complete_store(const LSQ_ENTRY& sq_entry);
  bool execute_load(const LSQ_ENTRY& lq_entry);

  [[nodiscard]] auto roi_instr() const { return roi_stats.instrs(); }
  [[nodiscard]] auto roi_cycle() const { return roi_stats.cycles(); }
  [[nodiscard]] auto sim_instr() const { return num_retired - begin_phase_instr; }
  [[nodiscard]] auto sim_cycle() const { return (current_time.time_since_epoch() / clock_period) - sim_stats.begin_cycles; }

  void print_deadlock() final;

#include "module_decl.inc"

  struct branch_module_concept {
    virtual ~branch_module_concept() = default;

    virtual void impl_initialize_branch_predictor() = 0;
    virtual void impl_last_branch_result(champsim::address ip, champsim::address target, bool taken, uint8_t branch_type) = 0;
    virtual bool impl_predict_branch(champsim::address ip, champsim::address predicted_target, bool always_taken, uint8_t branch_type) = 0;
  };

  struct btb_module_concept {
    virtual ~btb_module_concept() = default;

    virtual void impl_initialize_btb() = 0;
    virtual void impl_update_btb(champsim::address ip, champsim::address predicted_target, bool taken, uint8_t branch_type) = 0;
    virtual std::pair<champsim::address, bool> impl_btb_prediction(champsim::address ip, uint8_t branch_type) = 0;
  };

  template <typename... Bs>
  struct branch_module_model final : branch_module_concept {
    std::tuple<Bs...> intern_;
    explicit branch_module_model(O3_CPU* cpu) : intern_(Bs{cpu}...) { (void)cpu; /* silence -Wunused-but-set-parameter when sizeof...(Bs) == 0 */ }

    void impl_initialize_branch_predictor() final;
    void impl_last_branch_result(champsim::address ip, champsim::address target, bool taken, uint8_t branch_type) final;
    [[nodiscard]] bool impl_predict_branch(champsim::address ip, champsim::address predicted_target, bool always_taken, uint8_t branch_type) final;
  };

  template <typename... Ts>
  struct btb_module_model final : btb_module_concept {
    std::tuple<Ts...> intern_;
    explicit btb_module_model(O3_CPU* cpu) : intern_(Ts{cpu}...) { (void)cpu; /* silence -Wunused-but-set-parameter when sizeof...(Ts) == 0 */ }

    void impl_initialize_btb() final;
    void impl_update_btb(champsim::address ip, champsim::address predicted_target, bool taken, uint8_t branch_type) final;
    [[nodiscard]] std::pair<champsim::address, bool> impl_btb_prediction(champsim::address ip, uint8_t branch_type) final;
  };

  std::unique_ptr<branch_module_concept> branch_module_pimpl;
  std::unique_ptr<btb_module_concept> btb_module_pimpl;

  // NOLINTBEGIN(readability-make-member-function-const): legacy modules use non-const hooks
  void impl_initialize_branch_predictor() const;
  void impl_last_branch_result(champsim::address ip, champsim::address target, bool taken, uint8_t branch_type) const;
  [[nodiscard]] bool impl_predict_branch(champsim::address ip, champsim::address predicted_target, bool always_taken, uint8_t branch_type) const;

  void impl_initialize_btb() const;
  void impl_update_btb(champsim::address ip, champsim::address predicted_target, bool taken, uint8_t branch_type) const;
  [[nodiscard]] std::pair<champsim::address, bool> impl_btb_prediction(champsim::address ip, uint8_t branch_type) const;
  // NOLINTEND(readability-make-member-function-const)

  template <typename... Bs, typename... Ts>
  explicit O3_CPU(champsim::core_builder<champsim::core_builder_module_type_holder<Bs...>, champsim::core_builder_module_type_holder<Ts...>> b)
      : champsim::operable(b.m_clock_period), cpu(b.m_cpu),
        DIB(b.m_dib_set, b.m_dib_way, champsim::data::bits{champsim::lg2(b.m_dib_window)}, b.m_dib_ideal),
        LQ(b.m_lq_size), IFETCH_BUFFER_SIZE(resolve_ifetch_size(b.m_ifetch_buffer_size)), DISPATCH_BUFFER_SIZE(b.m_dispatch_buffer_size), DECODE_BUFFER_SIZE(b.m_decode_buffer_size),
        REGISTER_FILE_SIZE(b.m_register_file_size), ROB_SIZE(b.m_rob_size), SQ_SIZE(b.m_sq_size), DIB_HIT_BUFFER_SIZE(b.m_dib_hit_buffer_size),
        FETCH_WIDTH(b.m_fetch_width), DECODE_WIDTH(b.m_decode_width), DISPATCH_WIDTH(b.m_dispatch_width), SCHEDULER_SIZE(b.m_schedule_width),
        EXEC_WIDTH(b.m_execute_width), DIB_INORDER_WIDTH(b.m_dib_inorder_width), LQ_WIDTH(b.m_lq_width), SQ_WIDTH(b.m_sq_width), RETIRE_WIDTH(b.m_retire_width),
        BRANCH_MISPREDICT_PENALTY(b.m_mispredict_penalty * b.m_clock_period), DISPATCH_LATENCY(b.m_dispatch_latency * b.m_clock_period),
        DECODE_LATENCY(b.m_decode_latency * b.m_clock_period), SCHEDULING_LATENCY(b.m_schedule_latency * b.m_clock_period),
        EXEC_LATENCY(b.m_execute_latency * b.m_clock_period), DIB_HIT_LATENCY(b.m_dib_hit_latency * b.m_clock_period), L1I_BANDWIDTH(b.m_l1i_bw),
        L1D_BANDWIDTH(b.m_l1d_bw), IN_QUEUE_SIZE(resolve_ftq_depth(b.m_ftq_depth, 2 * champsim::to_underlying(b.m_fetch_width))), L1I_bus(b.m_cpu, b.m_fetch_queues),
        L1D_bus(b.m_cpu, b.m_data_queues), l1i(b.m_l1i), branch_module_pimpl(std::make_unique<branch_module_model<Bs...>>(this)),
        btb_module_pimpl(std::make_unique<btb_module_model<Ts...>>(this))
  {
    std::tie(trace_seg_enable, trace_rec_enable, trace_stg_enable) = resolve_trace_builder(b.m_trace_builder);
    {
      const char* e = std::getenv("PROMETHEUS_TRACE");
      const std::string v = (e != nullptr && *e != '\0') ? std::string{e} : b.m_trace_builder;
      trace_stall_enable = (v == "stall" || v == "all");
    }
    if (trace_stall_enable) {
      int min_occ = b.m_trace_stall_min_occ; // config "trace_stall_min_occ"
      if (const char* e = std::getenv("PROMETHEUS_STALL_MIN_OCC"); e != nullptr && *e != '\0') {
        min_occ = std::atoi(e); // env override for sweeps
      }
      stall.configure(min_occ);
      int rob_th = b.m_trace_stall_rob; // config "trace_stall_rob" (partial-stall trigger threshold)
      if (const char* e = std::getenv("PROMETHEUS_STALL_ROB"); e != nullptr && *e != '\0') {
        rob_th = std::atoi(e); // env override for sweeps
      }
      stall_rob_threshold = (rob_th < 0) ? 0 : static_cast<std::size_t>(rob_th);
      int attrib = b.m_trace_stall_attrib;
      if (const char* e = std::getenv("PROMETHEUS_STALL_ATTRIB"); e != nullptr && *e != '\0') {
        attrib = std::atoi(e); // 0 = open stretch at the lookup, 1 = dispatch-side attribution
      }
      stall.set_attrib(std::clamp(attrib, 0, 1));
      int readmit = b.m_trace_stall_readmit;
      if (const char* e = std::getenv("PROMETHEUS_STALL_READMIT"); e != nullptr && *e != '\0') {
        readmit = std::atoi(e); // 0 = never (legacy), 1 = strict, 2 = immediate
      }
      stall.set_readmit(std::clamp(readmit, 0, 2));
      int filter_entries = b.m_trace_stall_filter_entries;
      if (const char* e = std::getenv("PROMETHEUS_STALL_FILTER"); e != nullptr && *e != '\0') {
        filter_entries = std::atoi(e); // candidate table entries (0 = unbounded)
      }
      stall.set_filter_cap(static_cast<std::size_t>(std::max(0, filter_entries)));
      int mp_open = b.m_trace_stall_mispredict_open;
      if (const char* e = std::getenv("PROMETHEUS_STALL_MP_OPEN"); e != nullptr && *e != '\0') {
        mp_open = std::atoi(e); // 1 = mispredicted branches open stretches (like u-op-cache misses)
      }
      stall.set_mispredict_open(std::clamp(mp_open, 0, 2)); // 2 = keyed by (branch, direction taken) for the predict-time probe
      int split = b.m_trace_stall_split;
      if (const char* e = std::getenv("PROMETHEUS_STALL_SPLIT"); e != nullptr && *e != '\0') {
        split = std::atoi(e); // 1 = split miss runs past the depth into consecutive (chained) traces
      }
      stall.set_split(split != 0);
      int run_fmt = b.m_trace_run_format;
      if (const char* e = std::getenv("PROMETHEUS_RUN_FORMAT"); e != nullptr && *e != '\0') {
        run_fmt = std::atoi(e); // 1 = traces are runs of sequential windows, cut at jumps
      }
      int run_win = b.m_trace_run_max_windows;
      if (const char* e = std::getenv("PROMETHEUS_RUN_MAX_WINDOWS"); e != nullptr && *e != '\0') {
        run_win = std::atoi(e);
      }
      stall.set_run_format(std::clamp(run_fmt, 0, 2), run_win); // 1 = all runs stored, 2 = head run only
      long trig_lead = b.m_trace_stall_trigger_lead;
      if (const char* e = std::getenv("PROMETHEUS_STALL_TRIGGER_LEAD"); e != nullptr && *e != '\0') {
        trig_lead = std::atol(e); // cycles; 0 = stretches keyed by their own first pc
      }
      stall.set_trigger_lead(static_cast<uint64_t>(std::max(trig_lead, 0L)));
      int depth = b.m_trace_stall_depth; // config "trace_stall_depth" (max trace length)
      if (const char* e = std::getenv("PROMETHEUS_STALL_DEPTH"); e != nullptr && *e != '\0') {
        depth = std::atoi(e); // env override for sweeps
      }
      stall.set_max_uops(depth);
      int l1i_gate = b.m_trace_stall_l1i_gate;
      if (const char* e = std::getenv("PROMETHEUS_STALL_L1I_GATE"); e != nullptr && *e != '\0') {
        l1i_gate = std::atoi(e); // only commit stretches that also missed L1I
      }
      stall.set_l1i_gate(l1i_gate != 0);
    }
    fill_mode = resolve_fill_mode(b.m_trace_fill);
    if (fill_mode != fill_mode_type::OFF) {
      // trace-fill is fed by whichever trace builder is selected (stall or stager,
      // per trace_builder); if the user picked neither, default to the stager.
      if (!trace_stall_enable && !trace_stg_enable) {
        trace_stg_enable = true;
      }
      // CHAIN keeps the window index in SHADOW mode: maintained for the miss-
      // classification instrumentation (mid-trace trigger-gap sizing) but never
      // consulted by chain's probe or fill decisions.
      const bool window_indexed = (fill_mode == fill_mode_type::WINDOW || fill_mode == fill_mode_type::FLUSH || fill_mode == fill_mode_type::PARALLEL
                                   || fill_mode == fill_mode_type::ALT || fill_mode == fill_mode_type::HEAD || fill_mode == fill_mode_type::CHAIN);
      fill_store.configure(static_cast<unsigned>(champsim::lg2(b.m_dib_window)), window_indexed);
      dib_window_bits = static_cast<unsigned>(champsim::lg2(b.m_dib_window));
      stall.set_window_bits(dib_window_bits);
      // capacity/walk knobs: JSON value, overridden by env (env > JSON > default)
      int store_cap = b.m_trace_store;
      if (const char* e = std::getenv("PROMETHEUS_TRACE_STORE"); e != nullptr && *e != '\0') {
        store_cap = std::atoi(e);
      }
      fill_store.set_capacity(static_cast<std::size_t>(store_cap < 1 ? 1 : store_cap));
      int store_sets = b.m_trace_store_sets;
      if (const char* e = std::getenv("PROMETHEUS_STORE_SETS"); e != nullptr && *e != '\0') {
        store_sets = std::atoi(e);
      }
      int store_ways = b.m_trace_store_ways;
      if (const char* e = std::getenv("PROMETHEUS_STORE_WAYS"); e != nullptr && *e != '\0') {
        store_ways = std::atoi(e); // 0 = fully associative (legacy); N = N-way set-associative
      }
      // fully-assoc mode: a legacy PROMETHEUS_TRACE_STORE capacity sweep keeps
      // authority over a JSON-baked sets value (env sets still wins if given)
      if (store_ways == 0 && std::getenv("PROMETHEUS_STORE_SETS") == nullptr && std::getenv("PROMETHEUS_TRACE_STORE") != nullptr) {
        store_sets = 0;
      }
      fill_store.set_geometry(static_cast<std::size_t>(store_sets < 0 ? 0 : store_sets), static_cast<unsigned>(store_ways < 0 ? 0 : store_ways));
      int store_hash = b.m_trace_store_hash;
      if (const char* e = std::getenv("PROMETHEUS_STORE_HASH"); e != nullptr && *e != '\0') {
        store_hash = std::atoi(e); // 0 = plain modulo index, 1 = xor-fold
      }
      fill_store.set_hash(store_hash != 0);
      int tag_bits = b.m_trace_store_tag_bits;
      if (const char* e = std::getenv("PROMETHEUS_STORE_TAG_BITS"); e != nullptr && *e != '\0') {
        tag_bits = std::atoi(e); // partial-tag width (0 = full match)
      }
      fill_store.set_tag_bits(static_cast<unsigned>(std::clamp(tag_bits, 0, 63)));
      alt_walk_delay_cycles = b.m_trace_walk_delay;
      if (const char* e = std::getenv("PROMETHEUS_WALK_DELAY"); e != nullptr && *e != '\0') {
        alt_walk_delay_cycles = std::atoi(e); // base install delay in cycles (~L1I + decode)
      }
      alt_walk_max = static_cast<std::size_t>(b.m_trace_walk_max < 1 ? 1 : b.m_trace_walk_max);
      if (const char* e = std::getenv("PROMETHEUS_WALK_MAX"); e != nullptr && *e != '\0') {
        alt_walk_max = static_cast<std::size_t>(std::atoi(e)); // concurrent walks
      }
      alt_walk_width = (b.m_trace_walk_width < 1) ? 1 : b.m_trace_walk_width;
      if (const char* e = std::getenv("PROMETHEUS_WALK_WIDTH"); e != nullptr && *e != '\0') {
        alt_walk_width = std::max(1, std::atoi(e)); // windows installed per walk per cycle
      }
      alt_walk_wait_mode = (b.m_trace_walk_wait < 0) ? 0 : b.m_trace_walk_wait;
      if (const char* e = std::getenv("PROMETHEUS_WALK_WAIT"); e != nullptr && *e != '\0') {
        alt_walk_wait_mode = std::max(0, std::atoi(e)); // 0=off, 1=always wait, 2=line-ready-gated
      }
      alt_walk_l1i = (b.m_trace_walk_l1i != 0);
      if (const char* e = std::getenv("PROMETHEUS_WALK_L1I"); e != nullptr && *e != '\0') {
        alt_walk_l1i = (std::atoi(e) != 0); // walk fetches bytes through the real L1I
      }
      alt_walk_wait_cap = b.m_trace_walk_wait_cap;
      if (const char* e = std::getenv("PROMETHEUS_WAIT_CAP"); e != nullptr && *e != '\0') {
        alt_walk_wait_cap = std::atoi(e); // max wait cycles before falling to build (0 = unbounded)
      }
      int cost_evict = b.m_trace_store_cost_evict;
      if (const char* e = std::getenv("PROMETHEUS_STORE_COST_EVICT"); e != nullptr && *e != '\0') {
        cost_evict = std::atoi(e); // evict min-stall-cost trace instead of LRU
      }
      fill_store.set_cost_policy(cost_evict != 0);
      int cost_decay = b.m_trace_store_cost_decay;
      if (const char* e = std::getenv("PROMETHEUS_STORE_COST_DECAY"); e != nullptr && *e != '\0') {
        cost_decay = std::atoi(e); // halve costs every N insertions (0 = off)
      }
      fill_store.set_cost_decay(static_cast<uint64_t>(cost_decay < 0 ? 0 : cost_decay));
      int cost_bits = b.m_trace_store_cost_bits;
      if (const char* e = std::getenv("PROMETHEUS_STORE_COST_BITS"); e != nullptr && *e != '\0') {
        cost_bits = std::atoi(e); // saturating counter width (0 = unbounded)
      }
      fill_store.set_cost_bits(static_cast<unsigned>(cost_bits < 0 ? 0 : cost_bits));
      head_uops = (b.m_trace_head_uops < 0) ? 0 : b.m_trace_head_uops;
      if (const char* e = std::getenv("PROMETHEUS_HEAD_UOPS"); e != nullptr && *e != '\0') {
        head_uops = std::max(0, std::atoi(e)); // instantly-servable head length
      }
      tail_targets = b.m_trace_tail_targets;
      if (const char* e = std::getenv("PROMETHEUS_TAIL_TARGETS"); e != nullptr && *e != '\0') {
        tail_targets = std::atoi(e); // manifest reach in taken targets (<0 = unlimited)
      }
      uop_buffer_windows = (b.m_trace_uop_buffer < 0) ? 0 : b.m_trace_uop_buffer;
      if (const char* e = std::getenv("PROMETHEUS_UOP_BUFFER"); e != nullptr && *e != '\0') {
        uop_buffer_windows = std::max(0, std::atoi(e)); // staging buffer capacity in windows
      }
      meta_windows = (b.m_trace_meta_windows < 1) ? 1 : b.m_trace_meta_windows;
      meta_bits = std::max(0, b.m_trace_meta_bits);
      if (const char* e = std::getenv("PROMETHEUS_META_BITS"); e != nullptr && *e != '\0') {
        meta_bits = std::max(0, std::atoi(e)); // packed manifest slot-field width (0 = legacy)
      }
      meta_short_bits = b.m_trace_meta_short_bits;
      if (const char* e = std::getenv("PROMETHEUS_META_SHORT_BITS"); e != nullptr && *e != '\0') {
        meta_short_bits = std::atoi(e); // short-jump signed delta width
      }
      meta_long_bits = b.m_trace_meta_long_bits;
      if (const char* e = std::getenv("PROMETHEUS_META_LONG_BITS"); e != nullptr && *e != '\0') {
        meta_long_bits = std::atoi(e); // long-jump signed delta width
      }
      meta_kind_bits = b.m_trace_meta_kind_bits;
      if (const char* e = std::getenv("PROMETHEUS_META_KIND_BITS"); e != nullptr && *e != '\0') {
        meta_kind_bits = std::atoi(e); // kind-code width (1 or 2)
      }
      meta_far_truncate = (b.m_trace_meta_far_truncate != 0);
      if (const char* e = std::getenv("PROMETHEUS_META_FAR_TRUNCATE"); e != nullptr && *e != '\0') {
        meta_far_truncate = (std::atoi(e) != 0); // unencodable jump: truncate (1) or reject (0)
      }
      meta_kind_bits = std::clamp(meta_kind_bits, 1, 2);
      meta_short_bits = std::clamp(meta_short_bits, 2, 24);
      if (meta_long_bits != 0) {
        meta_long_bits = std::clamp(meta_long_bits, meta_short_bits, 48); // a long record must reach at least as far as a short one
      }
      if (const char* e = std::getenv("PROMETHEUS_META_WINDOWS"); e != nullptr && *e != '\0') {
        meta_windows = std::max(1, std::atoi(e)); // manifest window slots per trace
      }
      chain_probe_ahead = (b.m_trace_probe_ahead < 0) ? 0 : b.m_trace_probe_ahead;
      if (const char* e = std::getenv("PROMETHEUS_PROBE_AHEAD"); e != nullptr && *e != '\0') {
        chain_probe_ahead = std::max(0, std::atoi(e)); // deep-probe lead in instructions (0 = enqueue only)
      }
      chain_probe_blocks = std::max(1, b.m_trace_probe_blocks);
      if (const char* e = std::getenv("PROMETHEUS_PROBE_BLOCKS"); e != nullptr && *e != '\0') {
        chain_probe_blocks = std::max(1, std::atoi(e)); // fetch blocks the cursor covers per cycle
      }
      chain_probe_squash = std::clamp(b.m_trace_probe_squash, 0, 1);
      if (const char* e = std::getenv("PROMETHEUS_PROBE_SQUASH"); e != nullptr && *e != '\0') {
        chain_probe_squash = std::clamp(std::atoi(e), 0, 1); // 0 = legacy (flag only), 1 = squash past a mispredict
      }
      alt_walk_filter = (b.m_trace_walk_filter != 0);
      if (const char* e = std::getenv("PROMETHEUS_WALK_FILTER"); e != nullptr && *e != '\0') {
        alt_walk_filter = (std::atoi(e) != 0); // per-window residency filter at walk launch
      }
      chain_launch_conf = std::clamp(b.m_trace_launch_conf, 0, 2);
      if (const char* e = std::getenv("PROMETHEUS_LAUNCH_CONF"); e != nullptr && *e != '\0') {
        chain_launch_conf = std::clamp(std::atoi(e), 0, 2); // 0 off, 1 gate launches, 2 + conf-aware eviction
      }
      fill_store.set_conf_evict(chain_launch_conf == 2);
      chain_buffer_promote = std::clamp(b.m_trace_buffer_promote, 0, 2);
      if (const char* e = std::getenv("PROMETHEUS_BUFFER_PROMOTE"); e != nullptr && *e != '\0') {
        chain_buffer_promote = std::clamp(std::atoi(e), 0, 2); // 1 MRU promote, 0 no u-op-cache fill, 2 LRU promote
      }
    }

    stall_mispredict_mode = (b.m_trace_stall_mispredict < 0) ? 0 : b.m_trace_stall_mispredict;
    if (const char* e = std::getenv("PROMETHEUS_STALL_MISPREDICT"); e != nullptr && *e != '\0') {
      stall_mispredict_mode = std::max(0, std::atoi(e)); // 0=off, 1=resolution probe, 2=predict-time alternate probe, 3=2 H2P-gated, 4=H2P alternate probe of the regular store
    }

    // UCP port knobs: independent of the trace-fill mode (trace_ucp composes with
    // any fill mode, including off = pure UCP)
    ucp_enable = (b.m_trace_ucp != 0);
    if (const char* e = std::getenv("PROMETHEUS_UCP"); e != nullptr && *e != '\0') {
      ucp_enable = (std::atoi(e) != 0); // UCP alternate-path u-op prefetching
    }
    ucp_threshold = (b.m_trace_ucp_threshold < 1) ? 500 : b.m_trace_ucp_threshold;
    if (const char* e = std::getenv("PROMETHEUS_UCP_T"); e != nullptr && *e != '\0') {
      ucp_threshold = std::max(1, std::atoi(e)); // weighted stop-counter threshold
    }
    ucp_max_ip_check = (b.m_trace_ucp_max_ip < 1) ? 64 : b.m_trace_ucp_max_ip;
    if (const char* e = std::getenv("PROMETHEUS_UCP_MAX_IP"); e != nullptr && *e != '\0') {
      ucp_max_ip_check = std::max(1, std::atoi(e)); // straight-line run limit
    }
    ucp_step = (b.m_trace_ucp_step < 1) ? 4 : b.m_trace_ucp_step;
    if (const char* e = std::getenv("PROMETHEUS_UCP_STEP"); e != nullptr && *e != '\0') {
      ucp_step = std::max(1, std::atoi(e)); // alt-path instruction stride (bytes)
    }
    ucp_altind_enable = (b.m_trace_ucp_altind != 0);
    if (const char* e = std::getenv("PROMETHEUS_UCP_ALTIND"); e != nullptr && *e != '\0') {
      ucp_altind_enable = (std::atoi(e) != 0); // 4KB Alt-ITTAGE (12.95KB flavor)
    }
    ucp_artifact = std::clamp(b.m_trace_ucp_artifact, 0, 7);
    if (const char* e = std::getenv("PROMETHEUS_UCP_ARTIFACT"); e != nullptr && *e != '\0') {
      ucp_artifact = std::clamp(std::atoi(e), 0, 7); // UCP_ISCA24 semantics bit mask (7 = all)
    }
    ucp_btb_banks = (b.m_trace_ucp_btb_banks < 0) ? 0 : b.m_trace_ucp_btb_banks;
    if (const char* e = std::getenv("PROMETHEUS_UCP_BTB_BANKS"); e != nullptr && *e != '\0') {
      ucp_btb_banks = std::max(0, std::atoi(e)); // model N BTB banks (0 = ideal)
    }
    ucp_btb_banks = std::min(ucp_btb_banks, 64); // bank mask is one uint64_t
    while (ucp_btb_banks & (ucp_btb_banks - 1)) {
      ucp_btb_banks &= ucp_btb_banks - 1; // round down to a power of two (bank index is a bit-select)
    }
    trace_warmup_train = (b.m_trace_warmup_train != 0);
    if (const char* e = std::getenv("PROMETHEUS_WARMUP_TRAIN"); e != nullptr && *e != '\0') {
      trace_warmup_train = (std::atoi(e) != 0); // mechanisms train during warmup (UCP_ISCA24 artifact behavior)
    }
    alt_walk_issue_cap = (b.m_trace_walk_issue_cap < 0) ? 0 : b.m_trace_walk_issue_cap;
    if (const char* e = std::getenv("PROMETHEUS_WALK_ISSUE_CAP"); e != nullptr && *e != '\0') {
      alt_walk_issue_cap = std::max(0, std::atoi(e)); // global walk line-issues per cycle (0 = unlimited)
    }
    alt_walk_decode_cap = (b.m_trace_walk_decode_cap < 0) ? 0 : b.m_trace_walk_decode_cap;
    if (const char* e = std::getenv("PROMETHEUS_WALK_DECODE_CAP"); e != nullptr && *e != '\0') {
      alt_walk_decode_cap = std::max(0, std::atoi(e)); // global walk window-installs per cycle (0 = unlimited)
    }
    alt_walk_decode_shared = std::clamp(b.m_trace_walk_decode_shared, 0, 2);
    if (const char* e = std::getenv("PROMETHEUS_WALK_DECODE_SHARED"); e != nullptr && *e != '\0') {
      alt_walk_decode_shared = std::clamp(std::atoi(e), 0, 2); // shared decoders: 1 = stream-mode cycles only, 2 = + idle build-mode cycles
    }
    {
      // "ucp" = level 1 + the UCP_ISCA24 fetch-block rule; otherwise a level 0-3
      std::string wp = b.m_trace_wrong_path;
      if (const char* e = std::getenv("PROMETHEUS_WRONG_PATH"); e != nullptr && *e != '\0') {
        wp = e; // 0 legacy, 1 stop at mispredicts, ucp = 1 + fetch-block rule, 2 + wrong-path contention, 3 + wrong-path prefetch
      }
      if (wp != "0" && wp != "1" && wp != "2" && wp != "3" && wp != "ucp" && wp != "ucp2") {
        // an unknown value would silently fall back to level 0 (the correct-path oracle)
        std::cerr << "trace_wrong_path: unknown value \"" << wp << "\" (expected 0, 1, 2, 3, ucp or ucp2)" << std::endl;
        std::exit(1);
      }
      fetch_block_rule = (wp == "ucp" || wp == "ucp2");
      wp_exact = wp_uop_fill = (wp == "ucp2");
      wrong_path_level = wp_exact ? 3 : fetch_block_rule ? 1 : std::clamp(std::atoi(wp.c_str()), 0, 3);
    }
    oracle_peek = std::max(0, b.m_trace_oracle_peek);
    if (const char* e = std::getenv("PROMETHEUS_ORACLE_PEEK"); e != nullptr && *e != '\0') {
      oracle_peek = std::max(0, std::atoi(e)); // diagnostic ceiling: instrs of the real path probed past a stopping mispredict
    }
    peek_diag = (b.m_trace_peek_diag != 0);
    if (const char* e = std::getenv("PROMETHEUS_PEEK_DIAG"); e != nullptr && *e != '\0') {
      peek_diag = (std::atoi(e) != 0); // diagnostic: next-trace predictability at mispredictions
    }
    trigger_diag = (b.m_trace_trigger_diag != 0);
    if (const char* e = std::getenv("PROMETHEUS_TRIGGER_DIAG"); e != nullptr && *e != '\0') {
      trigger_diag = (std::atoi(e) != 0); // diagnostic: why stored traces did not serve their misses
    }
    stall.set_window_history(trigger_diag);
    rk_enable = (b.m_trace_recovery_key != 0);
    if (const char* e = std::getenv("PROMETHEUS_RECOVERY_KEY"); e != nullptr && *e != '\0') {
      rk_enable = (std::atoi(e) != 0); // recovery traces filed under (branch, predicted direction, GHR)
    }
    rk_ghr_bits = std::clamp(b.m_trace_recovery_ghr, 0, 64);
    if (const char* e = std::getenv("PROMETHEUS_RECOVERY_GHR"); e != nullptr && *e != '\0') {
      rk_ghr_bits = std::clamp(std::atoi(e), 0, 64);
    }
    rk_h2p = (b.m_trace_recovery_h2p != 0);
    if (const char* e = std::getenv("PROMETHEUS_RECOVERY_H2P"); e != nullptr && *e != '\0') {
      rk_h2p = (std::atoi(e) != 0);
    }
    {
      const auto knob = [](int json, const char* env) {
        if (const char* e = std::getenv(env); e != nullptr && *e != '\0') {
          return std::atoi(e);
        }
        return json;
      };
      chain_depth = std::max(0, knob(b.m_trace_chain_depth, "PROMETHEUS_CHAIN_DEPTH"));
      chain_window = static_cast<uint64_t>(std::max(0, knob(b.m_trace_chain_window, "PROMETHEUS_CHAIN_WINDOW")));
      vp_enable = (knob(b.m_trace_vpred, "PROMETHEUS_VPRED") != 0);
      vpred.configure(knob(b.m_trace_vpred_tables, "PROMETHEUS_VPRED_TABLES"), knob(b.m_trace_vpred_entries, "PROMETHEUS_VPRED_ENTRIES"),
                      knob(4, "PROMETHEUS_VPRED_MIN_HIST"), knob(b.m_trace_vpred_max_hist, "PROMETHEUS_VPRED_MAX_HIST"),
                      knob(b.m_trace_vpred_tag_bits, "PROMETHEUS_VPRED_TAG_BITS"), knob(b.m_trace_vpred_conf, "PROMETHEUS_VPRED_CONF"));
    }
    if (ucp_enable && fill_mode == fill_mode_type::OFF) {
      // pure-UCP runs still need the walk pacing knobs (normally resolved with the fill block)
      alt_walk_max = (b.m_trace_walk_max < 1) ? 1 : static_cast<std::size_t>(b.m_trace_walk_max);
      if (const char* e = std::getenv("PROMETHEUS_WALK_MAX"); e != nullptr && *e != '\0') {
        alt_walk_max = static_cast<std::size_t>(std::max(1, std::atoi(e)));
      }
      alt_walk_width = (b.m_trace_walk_width < 1) ? 1 : b.m_trace_walk_width;
      if (const char* e = std::getenv("PROMETHEUS_WALK_WIDTH"); e != nullptr && *e != '\0') {
        alt_walk_width = std::max(1, std::atoi(e));
      }
      alt_walk_delay_cycles = (b.m_trace_walk_delay < 0) ? 0 : b.m_trace_walk_delay;
      if (const char* e = std::getenv("PROMETHEUS_WALK_DELAY"); e != nullptr && *e != '\0') {
        alt_walk_delay_cycles = std::max(0, std::atoi(e));
      }
      alt_walk_l1i = (b.m_trace_walk_l1i != 0);
      if (const char* e = std::getenv("PROMETHEUS_WALK_L1I"); e != nullptr && *e != '\0') {
        alt_walk_l1i = (std::atoi(e) != 0);
      }
    }

    // decoupled front end (all fill modes: baseline and UCP too)
    fetch_buffer_blocks = std::max(0, b.m_fetch_buffer_blocks);
    if (const char* e = std::getenv("PROMETHEUS_FETCH_BLOCKS"); e != nullptr && *e != '\0') {
      fetch_buffer_blocks = std::max(0, std::atoi(e)); // fetch blocks in flight in the fetch buffer
    }
    ftq_fdip = (b.m_ftq_fdip != 0);
    ftq_uop_prefetch = (b.m_ftq_uop_prefetch != 0);
    if (const char* e = std::getenv("PROMETHEUS_FTQ_UOP_PF"); e != nullptr && *e != '\0') {
      ftq_uop_prefetch = (std::atoi(e) != 0); // u-op FDIP: walk each predicted window the u-op cache lacks
    }
    if (const char* e = std::getenv("PROMETHEUS_FTQ_FDIP"); e != nullptr && *e != '\0') {
      ftq_fdip = (std::atoi(e) != 0); // FDIP: prefetch L1I lines of blocks as the cursor predicts them
    }
    if (const char* e = std::getenv("PROMETHEUS_UOP_RETAIN"); e != nullptr && *e != '\0') {
      uop_retain_ways = static_cast<std::size_t>(std::max(0, std::atoi(e))); // cost-aware u-op-cache replacement
    }
    DIB.set_retain(uop_retain_ways);
    if (const char* e = std::getenv("PROMETHEUS_DIB_IDEAL"); e != nullptr && *e != '\0') {
      DIB.set_ideal(std::atoi(e)); // 1 = every lookup hits, 2 = only first-touch windows miss
    }
    chain_parallel = (b.m_trace_chain_parallel != 0);
    ftq_shared_wp = (b.m_ftq_shared_wp != 0);
    ftq_fdip_filter = (b.m_ftq_fdip_filter != 0);
    alt_walk_nofill = (b.m_trace_walk_nofill != 0);
    fdip_predecode = b.m_fdip_predecode;
    fdip_predecode_target = b.m_fdip_predecode_target;
    cwt_entries = static_cast<std::size_t>(std::max(8, b.m_cwt_entries));
    rtt_mode = b.m_rtt_mode;
    rtt_store = fill_store; // same geometry, policy and accounting (still empty here)
    rtt_capture = b.m_rtt_capture;
    rtt_len = static_cast<std::size_t>(std::max(1, b.m_rtt_len));
    if (const char* e = std::getenv("PROMETHEUS_RTT_MODE"); e != nullptr && *e != '\0') {
      rtt_mode = std::atoi(e);
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_CAPTURE"); e != nullptr && *e != '\0') {
      rtt_capture = std::atoi(e);
    }
    rtt_open_rule = b.m_rtt_open;
    rtt_max_windows = std::max(0, b.m_rtt_max_windows);
    if (const char* e = std::getenv("PROMETHEUS_RTT_MAX_WINDOWS"); e != nullptr && *e != '\0') {
      rtt_max_windows = std::max(0, std::atoi(e));
    }
    rtt_table = b.m_rtt_table;
    rtt_format = b.m_rtt_format;
    if (const char* e = std::getenv("PROMETHEUS_RTT_FORMAT"); e != nullptr && *e != '\0') {
      rtt_format = std::atoi(e);
    }
    rtt_wrong_path = b.m_rtt_wrong_path != 0;
    rtt_cancel = b.m_rtt_cancel != 0;
    walk_mshr_limit = std::max(0, b.m_walk_mshr_limit);
    walk_prio_head = std::max(0, b.m_walk_prio_head);
    walk_ooo = b.m_walk_ooo != 0;
    walk_resident_port = std::max(0, b.m_walk_resident_port);
    walk_l2_mshrs = std::max(0, b.m_walk_l2_mshrs);
    walk_pq = std::max(0, b.m_walk_pq);
    if (const char* e = std::getenv("PROMETHEUS_WALK_PQ"); e != nullptr && *e != '\0') {
      walk_pq = std::max(0, std::atoi(e));
    }
    if (const char* e = std::getenv("PROMETHEUS_WALK_L2_MSHRS"); e != nullptr && *e != '\0') {
      walk_l2_mshrs = std::max(0, std::atoi(e));
    }
    if (const char* e = std::getenv("PROMETHEUS_WALK_RESIDENT_PORT"); e != nullptr && *e != '\0') {
      walk_resident_port = std::max(0, std::atoi(e));
    }
    if (const char* e = std::getenv("PROMETHEUS_WALK_OOO"); e != nullptr && *e != '\0') {
      walk_ooo = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("PROMETHEUS_WALK_PRIO_HEAD"); e != nullptr && *e != '\0') {
      walk_prio_head = std::max(0, std::atoi(e));
    }
    rtt_conf_min = std::max(0, b.m_rtt_conf_min);
    rtt_admit_miss = std::max(0, b.m_rtt_admit_miss);
    rtt_boost = std::max(0, b.m_rtt_boost);
    rtt_ret = std::max(0, b.m_rtt_ret);
    rtt_key_target = b.m_rtt_key_target != 0;
    rtt_conf_table = std::max(0, b.m_rtt_conf_table);
    rtt_admit_repeat = std::max(0, b.m_rtt_admit_repeat);
    if (const char* e = std::getenv("PROMETHEUS_RTT_ADMIT_REPEAT"); e != nullptr && *e != '\0') {
      rtt_admit_repeat = std::max(0, std::atoi(e));
    }
    if (rtt_admit_repeat > 0) {
      rtt_repeat_tab.assign(static_cast<std::size_t>(rtt_admit_repeat), 0);
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_CONF_TABLE"); e != nullptr && *e != '\0') {
      rtt_conf_table = std::max(0, std::atoi(e));
    }
    if (rtt_conf_table > 0) {
      rtt_conf_tab.assign(static_cast<std::size_t>(rtt_conf_table), 2); // weakly confident, as a fresh trace entry
    }
    rtt_seg_reconv = b.m_rtt_seg_reconv != 0;
    if (const char* e = std::getenv("PROMETHEUS_RTT_SEG_RECONV"); e != nullptr && *e != '\0') {
      rtt_seg_reconv = std::atoi(e) != 0;
    }
    rtt_cost = b.m_rtt_cost != 0;
    rtt_cost_walk = std::max(0, b.m_rtt_cost_walk);
    if (const char* e = std::getenv("PROMETHEUS_RTT_COST"); e != nullptr && *e != '\0') {
      rtt_cost = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_COST_WALK"); e != nullptr && *e != '\0') {
      rtt_cost_walk = std::max(0, std::atoi(e));
    }
    rtt_seg_stable = b.m_rtt_seg_stable != 0;
    if (const char* e = std::getenv("PROMETHEUS_RTT_KEY_TARGET"); e != nullptr && *e != '\0') {
      rtt_key_target = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_SEG_STABLE"); e != nullptr && *e != '\0') {
      rtt_seg_stable = std::atoi(e) != 0;
    }
    rtt_l2_keep = b.m_rtt_l2_keep != 0;
    rtt_cold_seq = std::max(0, b.m_rtt_cold_seq);
    rtt_cold_mode = std::clamp(b.m_rtt_cold_mode, 0, 2);
    if (const char* e = std::getenv("PROMETHEUS_RTT_COLD_SEQ"); e != nullptr && *e != '\0') {
      rtt_cold_seq = std::max(0, std::atoi(e));
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_COLD_MODE"); e != nullptr && *e != '\0') {
      rtt_cold_mode = std::clamp(std::atoi(e), 0, 2);
    }
    rtt_cold_target = b.m_rtt_cold_target != 0;
    rtt_cold_l2 = b.m_rtt_cold_l2 != 0;
    if (const char* e = std::getenv("PROMETHEUS_RTT_COLD_L2"); e != nullptr && *e != '\0') {
      rtt_cold_l2 = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_COLD_TARGET"); e != nullptr && *e != '\0') {
      rtt_cold_target = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_L2_KEEP"); e != nullptr && *e != '\0') {
      rtt_l2_keep = std::atoi(e) != 0;
    }
    rtt_fallback = b.m_rtt_fallback != 0;
    rtt_gen_cond = b.m_rtt_gen_cond != 0;
    if (const char* e = std::getenv("PROMETHEUS_RTT_GEN_COND"); e != nullptr && *e != '\0') {
      rtt_gen_cond = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_FALLBACK"); e != nullptr && *e != '\0') {
      rtt_fallback = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_BOOST"); e != nullptr && *e != '\0') {
      rtt_boost = std::max(0, std::atoi(e));
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_RET"); e != nullptr && *e != '\0') {
      rtt_ret = std::max(0, std::atoi(e));
    }
    rtt_conf_evict = b.m_rtt_conf_evict != 0;
    if (const char* e = std::getenv("PROMETHEUS_RTT_ADMIT_MISS"); e != nullptr && *e != '\0') {
      rtt_admit_miss = std::max(0, std::atoi(e));
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_CONF_EVICT"); e != nullptr && *e != '\0') {
      rtt_conf_evict = std::atoi(e) != 0;
    }
    if (rtt_conf_evict) {
      rtt_store.set_conf_evict(true);
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_CONF_MIN"); e != nullptr && *e != '\0') {
      rtt_conf_min = std::max(0, std::atoi(e));
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_CANCEL"); e != nullptr && *e != '\0') {
      rtt_cancel = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("PROMETHEUS_WALK_MSHR_LIMIT"); e != nullptr && *e != '\0') {
      walk_mshr_limit = std::max(0, std::atoi(e));
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_WRONG_PATH"); e != nullptr && *e != '\0') {
      rtt_wrong_path = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_TABLE"); e != nullptr && *e != '\0') {
      rtt_table = std::atoi(e);
    }
    rtt_max_open = static_cast<std::size_t>(std::max(1, b.m_rtt_max_open));
    if (const char* e = std::getenv("PROMETHEUS_RTT_MAX_OPEN"); e != nullptr && *e != '\0') {
      rtt_max_open = static_cast<std::size_t>(std::max(1, std::atoi(e)));
    }
    rtt_store_filter = b.m_rtt_store_filter;
    if (const char* e = std::getenv("PROMETHEUS_RTT_OPEN"); e != nullptr && *e != '\0') {
      rtt_open_rule = std::atoi(e);
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_STORE_FILTER"); e != nullptr && *e != '\0') {
      rtt_store_filter = std::atoi(e);
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_LEN"); e != nullptr && *e != '\0') {
      rtt_len = static_cast<std::size_t>(std::max(1, std::atoi(e)));
    }
    if (const char* e = std::getenv("PROMETHEUS_ORACLE_RECOVERY_SEEN"); e != nullptr && *e != '\0') {
      oracle_recovery_seen = std::atoi(e) != 0;
    }
    if (const char* e = std::getenv("PROMETHEUS_RTT_DIAG"); e != nullptr && *e != '\0') {
      rtt_diag = std::atoi(e) != 0;
    }
    if (rtt_mode != 0) {
      std::cout << "Prometheus recovery-trace table: trigger "
                << (rtt_mode == 1   ? "wrong predictions only (oracle)"
                    : rtt_mode == 2 ? "every prediction"
                    : rtt_mode == 3 ? "low-confidence conditionals"
                    : rtt_mode == 4 ? "resolution of a mispredicted branch"
                                    : "low-confidence predictions (H2P conditionals + indirects)")
                << ((rtt_mode == 2 || rtt_mode == 3 || rtt_mode == 5) ? (rtt_wrong_path ? " (+ wrong-path predictions)" : " (correct path only)") : "")
                << (rtt_cancel ? "  cancel-on-correct-resolution" : "") << (rtt_conf_min > 0 ? "  walk if entry confidence >= " + std::to_string(rtt_conf_min) : std::string{})
                << (rtt_fallback ? "  generated alternate path when no trace" : "") << (rtt_gen_cond ? "  conditionals: generated path only (table for indirects)" : "")
                << (rtt_boost > 0 ? "  confident conditionals walk at entry confidence >= " + std::to_string(rtt_boost) : std::string{})
                << (rtt_ret > 0 ? "  returns walk at entry confidence >= " + std::to_string(rtt_ret) : std::string{})
                << (rtt_key_target ? "  conditionals keyed by recovery target only (shared traces)" : "") << (rtt_seg_stable ? "  stable-prefix re-recording" : "") << (rtt_conf_table > 0 ? "  shared confidence table " + std::to_string(rtt_conf_table) + " x 2 bits" : std::string{})
                << (rtt_admit_repeat > 0 ? "  new traces stored on their 2nd recording (filter " + std::to_string(rtt_admit_repeat) + " x 12 bits)" : std::string{}) << (rtt_seg_reconv ? "  recording ends at reconvergence with the wrong path" : "")
                << (rtt_cost ? "  trace cost = empty-ROB cycles of its recovery" : "") << (rtt_cost_walk > 0 ? "  cost bucket >= " + std::to_string(rtt_cost_walk) + " walks despite low confidence" : std::string{})
                << (rtt_cold_seq > 0 ? "  no-trace taken H2P: prefetch " + std::to_string(rtt_cold_seq) + " fall-through lines (cold mode " + std::to_string(rtt_cold_mode) + (rtt_cold_target ? ", + taken side" : "") + (rtt_cold_l2 ? ", into the L2 only" : "") + ")" : std::string{})
                << (rtt_admit_miss == 1 ? "  store only recordings with a u-op miss" : rtt_admit_miss >= 2 ? "  store only recordings with a u-op miss whose line missed the L1I" : "") << (rtt_conf_evict ? "  usefulness-aware eviction" : "") << (walk_mshr_limit > 0 ? "  walks yield at " + std::to_string(walk_mshr_limit) + " L1I MSHRs" + (walk_prio_head > 0 ? " (first " + std::to_string(walk_prio_head) + " lines exempt)" : std::string{}) : std::string{})
                << "  table " << (rtt_table == 1 ? (rtt_format == 1 ? "SUCP trace store (bounded, direction-bit traces)" : "SUCP trace store (bounded, manifest-encoded)") : "unbounded")
                << "  capture " << (rtt_capture == 0 ? "first u-op miss -> next u-op hit" : "fixed length") << "  max " << rtt_len << " instrs"
                << (rtt_max_windows > 0 ? "  stored: first " + std::to_string(rtt_max_windows) + " windows" : std::string{})
                << "  opened at " << (rtt_open_rule == 0 ? "every misprediction" : rtt_open_rule == 1 ? "low-confidence conditionals + mispredicted other branches"
                                                                         : rtt_open_rule == 2 ? "low-confidence conditionals (correct prediction: alternate path) + mispredicted other branches"
                                                                                              : "low-confidence or mispredicted branches (mispredicted: correct path, else alternate path)")
                << "  max open " << rtt_max_open << "  stored " << (rtt_store_filter == 0 ? "always" : rtt_store_filter == 1 ? "if the ROB went empty on it" : "if the ROB went empty on a u-op miss in it") << std::endl;
    }
    if (const char* e = std::getenv("PROMETHEUS_ORACLE_RECOVERY"); e != nullptr && *e != '\0') {
      oracle_recovery = static_cast<std::size_t>(std::max(0, std::atoi(e)));
      std::cout << "Prometheus oracle recovery walk: " << oracle_recovery << " real-path instructions per misprediction" << std::endl;
    }
    if (const char* e = std::getenv("PROMETHEUS_ORACLE_RECOVERY_CLASSES"); e != nullptr && *e != '\0') {
      oracle_recovery_classes = static_cast<unsigned>(std::strtoul(e, nullptr, 0));
      std::cout << "Prometheus oracle recovery classes: 0x" << std::hex << oracle_recovery_classes << std::dec
                << " (1 cond-H2P, 2 cond-confident, 4 indirect, 8 return, 0x10 direct, 0x20 other)" << std::endl;
    }
    if (const char* e = std::getenv("PROMETHEUS_FDIP_PREDECODE"); e != nullptr && *e != '\0') {
      fdip_predecode = std::atoi(e);
    }
    if (const char* e = std::getenv("PROMETHEUS_FDIP_PREDECODE_TARGET"); e != nullptr && *e != '\0') {
      fdip_predecode_target = std::atoi(e);
    }
    if (const char* e = std::getenv("PROMETHEUS_CWT_ENTRIES"); e != nullptr && *e != '\0') {
      cwt_entries = static_cast<std::size_t>(std::max(8, std::atoi(e)));
    }
    if (fdip_predecode == 2) {
      cwt.resize((cwt_entries / 8) * 8);
    }
    if (fdip_predecode != 0) {
      std::cout << "Prometheus fdip-predecode: mode " << fdip_predecode << " target " << (fdip_predecode_target == 0 ? "u-op cache" : "staging buffer");
      if (fdip_predecode == 2) {
        std::cout << "  costly-window table " << cwt.size() << " entries x 16 bits = " << static_cast<double>(cwt.size() * 16) / 8192.0 << " KB";
      }
      std::cout << std::endl;
    }
    int walk_nofill_mode = b.m_trace_walk_nofill;
    if (const char* e = std::getenv("PROMETHEUS_WALK_NOFILL"); e != nullptr && *e != '\0') {
      walk_nofill_mode = std::atoi(e);
    }
    alt_walk_nofill = (walk_nofill_mode != 0);       // 1: walk reads allocate in no cache level
    alt_walk_nofill_l1 = (walk_nofill_mode == 2);    // 2: ...only the L1I skips them (L2/LLC allocate)
    ftq_fdip_uop_check = (b.m_ftq_fdip_uop_check != 0);
    if (const char* e = std::getenv("PROMETHEUS_FTQ_FDIP_UOP_CHECK"); e != nullptr && *e != '\0') {
      ftq_fdip_uop_check = (std::atoi(e) != 0);
    }
    if (const char* e = std::getenv("PROMETHEUS_FTQ_FDIP_FILTER"); e != nullptr && *e != '\0') {
      ftq_fdip_filter = (std::atoi(e) != 0);
    }
    if (const char* e = std::getenv("PROMETHEUS_FTQ_SHARED_WP"); e != nullptr && *e != '\0') {
      ftq_shared_wp = (std::atoi(e) != 0);
    }
    if (const char* e = std::getenv("PROMETHEUS_CHAIN_PARALLEL"); e != nullptr && *e != '\0') {
      chain_parallel = (std::atoi(e) != 0); // CHAIN with perfect delivery of the stored traces (same front end)
    }
    if (const char* e = std::getenv("PROMETHEUS_UOP_INCLUSIVE"); e != nullptr && *e != '\0' && std::atoi(e) != 0) {
      DIB.set_line_present([cache = l1i](champsim::address a) { return cache->has_virtual_block(a); });
      std::cout << "Prometheus u-op cache: inclusive of the L1I" << std::endl;
    }
    stall_taxonomy = (b.m_trace_stall_taxonomy != 0);
    l2_perfect_instr = b.m_l2_perfect_instr;
    l2_emissary_ways = static_cast<std::size_t>(std::max(0, b.m_l2_emissary_ways));
    l2_emissary_rand = static_cast<uint32_t>(std::max(1, b.m_l2_emissary_rand));
    rp_mode = b.m_rp_mode;
    rp_sets = static_cast<std::size_t>(std::max(1, b.m_rp_sets));
    rp_ways = static_cast<std::size_t>(std::max(1, b.m_rp_ways));
    rp_targets = static_cast<std::size_t>(std::max(1, b.m_rp_targets));
    rp_window = std::max(1, b.m_rp_window);
    if (const char* e = std::getenv("PROMETHEUS_L2_PERFECT_INSTR"); e != nullptr && *e != '\0') {
      l2_perfect_instr = std::atoi(e);
    }
    if (const char* e = std::getenv("PROMETHEUS_L2_EMISSARY_WAYS"); e != nullptr && *e != '\0') {
      l2_emissary_ways = static_cast<std::size_t>(std::max(0, std::atoi(e)));
    }
    if (const char* e = std::getenv("PROMETHEUS_L2_EMISSARY_RAND"); e != nullptr && *e != '\0') {
      l2_emissary_rand = static_cast<uint32_t>(std::max(1, std::atoi(e)));
    }
    if (const char* e = std::getenv("PROMETHEUS_RP_MODE"); e != nullptr && *e != '\0') {
      rp_mode = std::atoi(e);
    }
    if (const char* e = std::getenv("PROMETHEUS_RP_TARGETS"); e != nullptr && *e != '\0') {
      rp_targets = static_cast<std::size_t>(std::max(1, std::atoi(e)));
    }
    if (const char* e = std::getenv("PROMETHEUS_RP_SETS"); e != nullptr && *e != '\0') {
      rp_sets = static_cast<std::size_t>(std::max(1, std::atoi(e)));
    }
    if (const char* e = std::getenv("PROMETHEUS_RP_WINDOW"); e != nullptr && *e != '\0') {
      rp_window = std::max(1, std::atoi(e));
    }
    if (rp_mode != 0) {
      rp_table.resize(rp_sets * rp_ways);
    }
    if (l2_perfect_instr != 0 || l2_emissary_ways > 0 || rp_mode != 0) {
      // PDIP accounting: 10-bit tag + 1 LRU bit + per target a 34-bit line address and a 4-bit mask
      const double rp_kb = (rp_mode != 0) ? static_cast<double>(rp_sets * rp_ways * (11 + rp_targets * 38)) / 8192.0 : 0.0;
      std::cout << "Prometheus L2/resteer knobs: l2_perfect_instr=" << l2_perfect_instr << " l2_emissary_ways=" << l2_emissary_ways
                << " l2_emissary_rand=" << l2_emissary_rand << " rp_mode=" << rp_mode << " rp_sets=" << rp_sets << " rp_ways=" << rp_ways
                << " rp_targets=" << rp_targets << " rp_window=" << rp_window << " rp_storage_kb=" << rp_kb << std::endl;
    }
    if (const char* e = std::getenv("PROMETHEUS_STALL_TAXONOMY"); e != nullptr && *e != '\0') {
      stall_taxonomy = (std::atoi(e) != 0);
    }
    if (const char* e = std::getenv("PROMETHEUS_DECODE_WIDTH"); e != nullptr && *e != '\0' && std::atol(e) > 0) {
      DECODE_WIDTH = champsim::bandwidth::maximum_type{std::atol(e)}; // legacy-decode width
    }
    if (const char* e = std::getenv("PROMETHEUS_DECODE_LATENCY"); e != nullptr && *e != '\0' && std::atol(e) >= 0) {
      DECODE_LATENCY = std::atol(e) * b.m_clock_period; // legacy-decode pipeline depth (cycles)
    }
    ftq_lookup_window = std::max(0, b.m_ftq_lookup_window);
    if (const char* e = std::getenv("PROMETHEUS_FTQ_LOOKUP"); e != nullptr && *e != '\0') {
      ftq_lookup_window = std::max(0, std::atoi(e)); // u-op-cache lookup within the first N FTQ entries (0 = at insertion)
    }
    bpu_blocks = std::max(1, b.m_bpu_blocks);
    if (const char* e = std::getenv("PROMETHEUS_BPU_BLOCKS"); e != nullptr && *e != '\0') {
      bpu_blocks = std::max(1, std::atoi(e)); // fetch blocks the predictor inserts per cycle
    }
    // self-report the RESOLVED knob values (env > JSON > default) so every log
    // records the exact configuration that produced it (stale-binary insurance)
    miss_window_bits = static_cast<unsigned>(champsim::lg2(b.m_dib_window));
    uop_cost_mode = b.m_uop_cost_mode;
    uop_cost_windows = b.m_uop_cost_windows;
    if (const char* e = std::getenv("PROMETHEUS_UOP_COST_MODE"); e != nullptr && *e != '\0') {
      uop_cost_mode = std::atoi(e);
    }
    if (const char* e = std::getenv("PROMETHEUS_UOP_COST_WINDOWS"); e != nullptr && *e != '\0') {
      uop_cost_windows = std::atol(e);
    }
    if (const char* e = std::getenv("PROMETHEUS_COLD_DIAG"); e != nullptr && *e != '\0') {
      cold_diag = std::atoi(e) != 0;
    }
    uop_ideal_split = b.m_uop_ideal_split;
    uop_ideal_split_window = b.m_uop_ideal_split_window;
    if (const char* e = std::getenv("PROMETHEUS_UOP_IDEAL_SPLIT"); e != nullptr && *e != '\0') {
      uop_ideal_split = std::atoi(e);
    }
    if (const char* e = std::getenv("PROMETHEUS_UOP_IDEAL_SPLIT_WINDOW"); e != nullptr && *e != '\0') {
      uop_ideal_split_window = std::atol(e);
    }
    if (uop_ideal_split != 0) {
      std::cout << "Prometheus u-op ideal split: mask " << uop_ideal_split << " (1 steady/L1I-resident, 2 steady/L1I-missing, 4 recovery/resident, 8 recovery/missing, 16 seen-before windows only)"
                << "  recovery = first " << uop_ideal_split_window << " instructions after a misprediction" << std::endl;
    }
    if (const char* e = std::getenv("PROMETHEUS_MISS_PROFILE"); e != nullptr && *e != '\0') {
      miss_profile = (std::atoi(e) != 0);
    }
    if (const char* e = std::getenv("PROMETHEUS_MISS_DUMP"); e != nullptr && *e != '\0') {
      miss_dump_path = e;
      miss_profile = true;
    }
    if (const char* e = std::getenv("PROMETHEUS_UOP_ORACLE"); e != nullptr && *e != '\0') {
      // diagnostic: the first N windows of a miss-profile dump always hit
      long n = std::numeric_limits<long>::max();
      if (const char* en = std::getenv("PROMETHEUS_UOP_ORACLE_N"); en != nullptr && *en != '\0') {
        n = std::atol(en);
      }
      std::unordered_set<uint64_t> windows;
      std::ifstream in(e);
      uint64_t w = 0;
      std::string rest;
      while (static_cast<long>(windows.size()) < n && in >> w) {
        windows.insert(w);
        std::getline(in, rest);
      }
      if (!in.eof() && windows.empty()) {
        std::cerr << "PROMETHEUS_UOP_ORACLE: cannot read " << e << std::endl;
        std::exit(1);
      }
      DIB.set_oracle_windows(std::move(windows), oracle_install_env());
    }
    if (const char* e = std::getenv("PROMETHEUS_TRACE_ORACLE"); e != nullptr && *e != '\0') {
      // diagnostic: every window of the first N traces of a trace dump (<miss dump>.traces,
      // ranked by cost) always hits -- prices an N-entry store prefetched perfectly
      if (DIB.oracle_windows() != 0) {
        std::cerr << "PROMETHEUS_TRACE_ORACLE and PROMETHEUS_UOP_ORACLE are exclusive" << std::endl;
        std::exit(1);
      }
      long n = std::numeric_limits<long>::max();
      if (const char* en = std::getenv("PROMETHEUS_TRACE_ORACLE_N"); en != nullptr && *en != '\0') {
        n = std::atol(en);
      }
      std::ifstream in(e);
      if (!in) {
        std::cerr << "PROMETHEUS_TRACE_ORACLE: cannot read " << e << std::endl;
        std::exit(1);
      }
      long head = 0; // PROMETHEUS_TRACE_ORACLE_HEAD: only each trace's first W windows (0 = all)
      if (const char* eh = std::getenv("PROMETHEUS_TRACE_ORACLE_HEAD"); eh != nullptr && *eh != '\0') {
        head = std::max(0L, std::atol(eh));
      }
      std::unordered_set<uint64_t> windows;
      std::string line;
      while (trace_oracle_n < n && std::getline(in, line)) {
        std::istringstream ls(line);
        uint64_t cost = 0;
        uint64_t w = 0;
        if (!(ls >> cost)) {
          continue;
        }
        long taken = 0;
        while ((head == 0 || taken < head) && ls >> w) {
          windows.insert(w);
          ++taken;
        }
        ++trace_oracle_n;
      }
      std::cout << "Prometheus trace oracle: " << trace_oracle_n << " traces, " << windows.size() << " windows (head " << head << ")" << std::endl;
      DIB.set_oracle_windows(std::move(windows), oracle_install_env());
    }
    if (const char* e = std::getenv("PROMETHEUS_UOP_INSERT_FILTER"); e != nullptr && *e != '\0') {
      // diagnostic (profile-guided insertion): the u-op cache allocates only windows of the
      // first N traces of a trace dump; mode 0 bypasses every other window, mode 1 inserts it at LRU
      long n = std::numeric_limits<long>::max();
      if (const char* en = std::getenv("PROMETHEUS_UOP_INSERT_FILTER_N"); en != nullptr && *en != '\0') {
        n = std::atol(en);
      }
      int mode = 0;
      if (const char* em = std::getenv("PROMETHEUS_UOP_INSERT_MODE"); em != nullptr && *em != '\0') {
        mode = std::atoi(em);
      }
      std::ifstream in(e);
      if (!in) {
        std::cerr << "PROMETHEUS_UOP_INSERT_FILTER: cannot read " << e << std::endl;
        std::exit(1);
      }
      std::unordered_set<uint64_t> windows;
      std::string line;
      long traces = 0;
      while (traces < n && std::getline(in, line)) {
        std::istringstream ls(line);
        uint64_t cost = 0;
        uint64_t w = 0;
        if (!(ls >> cost)) {
          continue;
        }
        while (ls >> w) {
          windows.insert(w);
        }
        ++traces;
      }
      std::cout << "Prometheus insert filter: " << traces << " traces, " << windows.size() << " windows, mode " << mode << std::endl;
      DIB.set_insert_filter(std::move(windows), mode);
    }
    std::cout << "Prometheus knobs: fill=" << b.m_trace_fill << " min_occ=" << stall.min_occ() << " rob=" << stall_rob_threshold << " stall_attrib=" << stall.attrib() << " readmit=" << stall.readmit() << " mp_open=" << stall.mispredict_open() << " split=" << stall.split() << " run_format=" << stall.run_format() << " run_max_windows=" << stall.run_max_windows() << " trig_lead=" << stall.trigger_lead()
              << " depth=" << stall.max_trace_uops() << " l1i_gate=" << stall.l1i_gated() << " meta_windows=" << meta_windows
              << " meta_bits=" << meta_bits << " short_bits=" << meta_short_bits << " long_bits=" << meta_long_bits
              << " kind_bits=" << meta_kind_bits << " far_truncate=" << meta_far_truncate
              << " store=" << fill_store.cap() << " sets=" << fill_store.sets() << " ways=" << fill_store.ways() << " hash=" << fill_store.hash_mode() << " tag_bits=" << fill_store.tag_bits()
              << " cost_evict=" << fill_store.cost_policy() << " cost_decay=" << fill_store.cost_decay() << " cost_bits=" << fill_store.cost_bits()
              << " walk_max=" << alt_walk_max
              << " walk_width=" << alt_walk_width
              << " walk_delay=" << alt_walk_delay_cycles << " walk_l1i=" << alt_walk_l1i << " walk_wait=" << alt_walk_wait_mode
              << " wait_cap=" << alt_walk_wait_cap << " issue_cap=" << alt_walk_issue_cap << " decode_cap=" << alt_walk_decode_cap
              << " decode_shared=" << alt_walk_decode_shared << " walk_filter=" << alt_walk_filter << " launch_conf=" << chain_launch_conf
              << " buffer_promote=" << chain_buffer_promote
              << " uop_buffer=" << uop_buffer_windows
              << " probe_ahead=" << chain_probe_ahead << " probe_blocks=" << chain_probe_blocks << " ftq_lookup=" << ftq_lookup_window << " bpu_blocks=" << bpu_blocks << " ftq_fdip=" << ftq_fdip << " fdip_filter=" << ftq_fdip_filter << " fdip_uop_check=" << ftq_fdip_uop_check << " walk_nofill=" << alt_walk_nofill << " ftq_uop_pf=" << ftq_uop_prefetch << " uop_retain=" << uop_retain_ways << " chain_parallel=" << chain_parallel << " decode_width=" << static_cast<long>(DECODE_WIDTH) << " decode_latency=" << (DECODE_LATENCY / b.m_clock_period) << " ifetch=" << IFETCH_BUFFER_SIZE << " fetch_blocks=" << fetch_buffer_blocks << " ftq_shared_wp=" << ftq_shared_wp << " probe_squash=" << chain_probe_squash << " wrong_path=" << (wp_exact ? std::string{"ucp2"} : fetch_block_rule ? std::string{"ucp"} : std::to_string(wrong_path_level)) << " oracle_peek=" << oracle_peek << " peek_diag=" << peek_diag << " trigger_diag=" << trigger_diag
              << " recovery_key=" << rk_enable << " recovery_ghr=" << rk_ghr_bits << " recovery_h2p=" << rk_h2p
 << " chain_depth=" << chain_depth << " chain_window=" << chain_window
              << " miss_profile=" << miss_profile << " uop_oracle=" << DIB.oracle_windows() << " trace_oracle=" << trace_oracle_n << " oracle_install=" << DIB.oracle_install()
              << " vpred=" << vp_enable << " vpred_tables=" << vpred.tables() << " vpred_entries=" << vpred.entries()
              << " vpred_hist=" << vpred.hist(vpred.tables() - 1) << " vpred_kbytes=" << (vpred.storage_bits() / 8192.0) << " ftq_depth=" << IN_QUEUE_SIZE << " mispredict_trigger=" << stall_mispredict_mode
              << " head_uops=" << head_uops << " tail_targets=" << tail_targets
              << " ucp=" << ucp_enable << " ucp_T=" << ucp_threshold << " ucp_max_ip=" << ucp_max_ip_check << " ucp_step=" << ucp_step
              << " ucp_altind=" << ucp_altind_enable << " ucp_btb_banks=" << ucp_btb_banks << " ucp_artifact=" << ucp_artifact
              << " warmup_train=" << trace_warmup_train << std::endl;
    if (fill_mode == fill_mode_type::CHAIN && trace_stall_enable) {
      // storage estimate of the SUCP state (the simulator matches full pcs and keeps the
      // candidate table unbounded unless trace_stall_filter_entries is set; hardware
      // keeps a partial tag and a bounded table)
      const auto lg = [](uint64_t x) {
        int b = 0;
        while ((uint64_t{1} << b) < x) {
          ++b;
        }
        return b;
      };
      const uint64_t entries = fill_store.cap();
      const int tag = (fill_store.tag_bits() > 0) ? static_cast<int>(fill_store.tag_bits()) : 16;
      const int manifest = (meta_bits > 0) ? meta_bits : meta_windows * 16;
      const int cost = static_cast<int>(fill_store.cost_bits());
      const int lru = (fill_store.ways() > 1) ? lg(fill_store.ways()) : 0;
      const int succ = (chain_depth > 0) ? lg(entries) + 8 : 0; // successor as store slot + 8-bit check tag
      const int conf = (chain_launch_conf > 0) ? 2 : 0;
      const int per = tag + manifest + cost + lru + 1 + succ + conf;
      const double store_kb = static_cast<double>(entries) * per / 8192.0;
      const uint64_t filt = stall.filter_cap();
      const int filt_bits = tag + 2 + cost; // candidate: tag + occurrence count + cost
      const double filt_kb = static_cast<double>(filt) * filt_bits / 8192.0;
      const double stage_kb = static_cast<double>(uop_buffer_windows) * 8 * 64 / 8192.0; // 8 u-ops x 8 B per window
      std::cout << "Prometheus storage: store " << entries << " x " << per << " bits = " << store_kb << " KB (tag " << tag << " manifest " << manifest
                << " cost " << cost << " lru " << lru << " valid 1 successor " << succ << " conf " << conf << ")  candidate table " << filt
                << (filt == 0 ? " (UNBOUNDED, not counted)" : "") << " x " << filt_bits << " bits = " << filt_kb << " KB  staging " << uop_buffer_windows
                << " windows = " << stage_kb << " KB  SUCP total " << (store_kb + filt_kb + stage_kb) << " KB" << std::endl;
    }
  }
};

template <typename... Bs>
void O3_CPU::branch_module_model<Bs...>::impl_initialize_branch_predictor()
{
  [[maybe_unused]] auto process_one = [&](auto& b) {
    using namespace champsim::modules;
    if constexpr (branch_predictor::has_initialize<decltype(b)>)
      b.initialize_branch_predictor();
  };

  std::apply([&](auto&... b) { (..., process_one(b)); }, intern_);
}

template <typename... Bs>
void O3_CPU::branch_module_model<Bs...>::impl_last_branch_result(champsim::address ip, champsim::address target, bool taken, uint8_t branch_type)
{
  [[maybe_unused]] auto process_one = [&](auto& b) {
    using namespace champsim::modules;
    if constexpr (branch_predictor::has_last_branch_result<decltype(b), uint64_t, uint64_t, bool, uint8_t>)
      b.last_branch_result(ip.to<uint64_t>(), target.to<uint64_t>(), taken, branch_type);
    if constexpr (branch_predictor::has_last_branch_result<decltype(b), champsim::address, champsim::address, bool, uint8_t>)
      b.last_branch_result(ip, target, taken, branch_type);
  };

  std::apply([&](auto&... b) { (..., process_one(b)); }, intern_);
}

template <typename... Bs>
bool O3_CPU::branch_module_model<Bs...>::impl_predict_branch(champsim::address ip, champsim::address predicted_target, bool always_taken, uint8_t branch_type)
{
  using return_type = bool;
  [[maybe_unused]] auto process_one = [&](auto& b) {
    using namespace champsim::modules;
    /* Strong addresses, full size */
    if constexpr (branch_predictor::has_predict_branch<decltype(b), champsim::address, champsim::address, bool, uint8_t>)
      return return_type{b.predict_branch(ip, predicted_target, always_taken, branch_type)};

    /* Raw integer addresses, full size */
    if constexpr (branch_predictor::has_predict_branch<decltype(b), uint64_t, uint64_t, bool, uint8_t>)
      return return_type{b.predict_branch(ip.to<uint64_t>(), predicted_target.to<uint64_t>(), always_taken, branch_type)};

    /* Strong addresses, short size */
    if constexpr (branch_predictor::has_predict_branch<decltype(b), champsim::address>)
      return return_type{b.predict_branch(ip)};

    /* Raw integer addresses, short size */
    if constexpr (branch_predictor::has_predict_branch<decltype(b), uint64_t>)
      return return_type{b.predict_branch(ip.to<uint64_t>())};

    return return_type{};
  };

  if constexpr (sizeof...(Bs)) {
    return std::apply([&](auto&... b) { return (..., process_one(b)); }, intern_);
  }
  return return_type{};
}

template <typename... Ts>
void O3_CPU::btb_module_model<Ts...>::impl_initialize_btb()
{
  [[maybe_unused]] auto process_one = [&](auto& t) {
    using namespace champsim::modules;
    if constexpr (btb::has_initialize<decltype(t)>)
      t.initialize_btb();
  };

  std::apply([&](auto&... t) { (..., process_one(t)); }, intern_);
}

template <typename... Ts>
void O3_CPU::btb_module_model<Ts...>::impl_update_btb(champsim::address ip, champsim::address predicted_target, bool taken, uint8_t branch_type)
{
  [[maybe_unused]] auto process_one = [&](auto& t) {
    using namespace champsim::modules;
    if constexpr (btb::has_update_btb<decltype(t), champsim::address, champsim::address, bool, uint8_t>)
      t.update_btb(ip, predicted_target, taken, branch_type);
    if constexpr (btb::has_update_btb<decltype(t), uint64_t, uint64_t, bool, uint8_t>)
      t.update_btb(ip.to<uint64_t>(), predicted_target.to<uint64_t>(), taken, branch_type);
  };

  std::apply([&](auto&... t) { (..., process_one(t)); }, intern_);
}

template <typename... Ts>
std::pair<champsim::address, bool> O3_CPU::btb_module_model<Ts...>::impl_btb_prediction(champsim::address ip, uint8_t branch_type)
{
  using return_type = std::pair<champsim::address, bool>;
  [[maybe_unused]] auto process_one = [&](auto& t) {
    using namespace champsim::modules;

    /* Strong addresses, full size */
    if constexpr (btb::has_btb_prediction<decltype(t), champsim::address, uint8_t>)
      return return_type{t.btb_prediction(ip, branch_type)};

    /* Strong addresses, short size */
    if constexpr (btb::has_btb_prediction<decltype(t), champsim::address>)
      return return_type{t.btb_prediction(ip)};

    /* Raw integer addresses, full size */
    if constexpr (btb::has_btb_prediction<decltype(t), uint64_t, uint8_t>)
      return return_type{t.btb_prediction(ip.to<uint64_t>(), branch_type)};

    /* Raw integer addresses, short size */
    if constexpr (btb::has_btb_prediction<decltype(t), uint64_t>)
      return return_type{t.btb_prediction(ip.to<uint64_t>())};

    return return_type{};
  };

  if constexpr (sizeof...(Ts) > 0) {
    return std::apply([&](auto&... t) { return (..., process_one(t)); }, intern_);
  }
  return return_type{};
}

#ifdef SET_ASIDE_CHAMPSIM_MODULE
#undef SET_ASIDE_CHAMPSIM_MODULE
#define CHAMPSIM_MODULE
#endif

#endif

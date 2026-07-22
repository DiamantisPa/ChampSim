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
#include <set>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
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
  };
  std::deque<alt_walk> alt_walks;
  std::size_t alt_walk_max = 2;
  int alt_walk_delay_cycles = 5;
  int alt_walk_width = 1;      // windows installed per walk per cycle (pre-decode width)
  bool alt_walk_wait = false;  // miss on a walk-pending window stalls fetch (hit-under-fill) instead of switching to build
  bool alt_walk_l1i = false;   // walk fetches its bytes through the real L1I (install gated on line arrival)
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
  };
  std::deque<uop_buf_entry> uop_buffer; // trace-uop staging buffer (FIFO)
  int uop_buffer_windows = 16;          // capacity in windows (16 x 8 uops x 8B = 1KB; 0 = install direct to DIB)
  int meta_windows = 4;                 // manifest capacity: window slots per trace (entry + deltas)
  // deep probe: scan the instruction supply up to N instructions AHEAD of the
  // transfer point (models the decoupled BP/FTQ running ahead of fetch -- the
  // lead a real FTQ-tail probe has over the fetch-point u-op-cache lookup, which
  // this simulator's collapsed frontend otherwise erases).  The cursor resets on
  // a branch misprediction (= FTQ flush), so recovery lead rebuilds gradually.
  // 0 = probe at enqueue only (no lead).
  int chain_probe_ahead = 0;
  std::size_t chain_probe_cursor = 0; // next unprobed input_queue position (relative to front)
  // per-window residency filter: at walk launch, probe each of the trace's
  // windows in the u-op cache and the staging buffer, and fetch/stage ONLY the
  // absent ones -- kills redundant staging (walks shorten, slots free up).
  bool alt_walk_filter = false;

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
  bool alt_walk_decode_shared = false;
  int walk_decode_count = 0;                               // installs consumed this cycle
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
  void do_chain_trigger(const std::vector<uint64_t>& ips);
  void uop_buffer_push(const std::vector<uint64_t>& window_ips, champsim::chrono::clock::time_point probe_time);
  [[nodiscard]] std::vector<uint64_t> chain_encode(const std::vector<uint64_t>& ips, bool& truncated, bool& encodable) const;
  [[nodiscard]] bool alt_window_pending(uint64_t rawip) const;
  [[nodiscard]] alt_walk* alt_pending_walk(uint64_t rawip); // like alt_window_pending, but skips background walks
  [[nodiscard]] bool walk_issue_available(); // under the issue cap this cycle?
  void walk_issue_note();                    // consume one issue slot (on successful issue)
  [[nodiscard]] bool walk_decode_available(); // may a walk install a window this cycle?
  void walk_decode_note();                    // consume one install slot
  void do_ucp_branch(const ooo_model_instr& arch_instr, uint64_t btb_target, bool always_taken, bool h2p);
  [[nodiscard]] std::vector<uint64_t> ucp_generate_alt_path(const ooo_model_instr& h2p_instr, uint64_t start);
  void do_ucp_trigger(uint64_t h2p_ip, const std::vector<uint64_t>& ips);
  long fetch_instruction();
  long promote_to_decode();
  long decode_instruction();
  long dispatch_instruction();
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
        LQ(b.m_lq_size), IFETCH_BUFFER_SIZE(b.m_ifetch_buffer_size), DISPATCH_BUFFER_SIZE(b.m_dispatch_buffer_size), DECODE_BUFFER_SIZE(b.m_decode_buffer_size),
        REGISTER_FILE_SIZE(b.m_register_file_size), ROB_SIZE(b.m_rob_size), SQ_SIZE(b.m_sq_size), DIB_HIT_BUFFER_SIZE(b.m_dib_hit_buffer_size),
        FETCH_WIDTH(b.m_fetch_width), DECODE_WIDTH(b.m_decode_width), DISPATCH_WIDTH(b.m_dispatch_width), SCHEDULER_SIZE(b.m_schedule_width),
        EXEC_WIDTH(b.m_execute_width), DIB_INORDER_WIDTH(b.m_dib_inorder_width), LQ_WIDTH(b.m_lq_width), SQ_WIDTH(b.m_sq_width), RETIRE_WIDTH(b.m_retire_width),
        BRANCH_MISPREDICT_PENALTY(b.m_mispredict_penalty * b.m_clock_period), DISPATCH_LATENCY(b.m_dispatch_latency * b.m_clock_period),
        DECODE_LATENCY(b.m_decode_latency * b.m_clock_period), SCHEDULING_LATENCY(b.m_schedule_latency * b.m_clock_period),
        EXEC_LATENCY(b.m_execute_latency * b.m_clock_period), DIB_HIT_LATENCY(b.m_dib_hit_latency * b.m_clock_period), L1I_BANDWIDTH(b.m_l1i_bw),
        L1D_BANDWIDTH(b.m_l1d_bw), IN_QUEUE_SIZE(2 * champsim::to_underlying(b.m_fetch_width)), L1I_bus(b.m_cpu, b.m_fetch_queues),
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
      const bool window_indexed = (fill_mode == fill_mode_type::WINDOW || fill_mode == fill_mode_type::FLUSH || fill_mode == fill_mode_type::PARALLEL
                                   || fill_mode == fill_mode_type::ALT || fill_mode == fill_mode_type::HEAD);
      fill_store.configure(static_cast<unsigned>(champsim::lg2(b.m_dib_window)), window_indexed);
      dib_window_bits = static_cast<unsigned>(champsim::lg2(b.m_dib_window));
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
      alt_walk_wait = (b.m_trace_walk_wait != 0);
      if (const char* e = std::getenv("PROMETHEUS_WALK_WAIT"); e != nullptr && *e != '\0') {
        alt_walk_wait = (std::atoi(e) != 0); // stall fetch on walk-pending misses
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
      if (const char* e = std::getenv("PROMETHEUS_META_WINDOWS"); e != nullptr && *e != '\0') {
        meta_windows = std::max(1, std::atoi(e)); // manifest window slots per trace
      }
      chain_probe_ahead = (b.m_trace_probe_ahead < 0) ? 0 : b.m_trace_probe_ahead;
      if (const char* e = std::getenv("PROMETHEUS_PROBE_AHEAD"); e != nullptr && *e != '\0') {
        chain_probe_ahead = std::max(0, std::atoi(e)); // deep-probe lead in instructions (0 = enqueue only)
      }
      alt_walk_filter = (b.m_trace_walk_filter != 0);
      if (const char* e = std::getenv("PROMETHEUS_WALK_FILTER"); e != nullptr && *e != '\0') {
        alt_walk_filter = (std::atoi(e) != 0); // per-window residency filter at walk launch
      }
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
    alt_walk_issue_cap = (b.m_trace_walk_issue_cap < 0) ? 0 : b.m_trace_walk_issue_cap;
    if (const char* e = std::getenv("PROMETHEUS_WALK_ISSUE_CAP"); e != nullptr && *e != '\0') {
      alt_walk_issue_cap = std::max(0, std::atoi(e)); // global walk line-issues per cycle (0 = unlimited)
    }
    alt_walk_decode_cap = (b.m_trace_walk_decode_cap < 0) ? 0 : b.m_trace_walk_decode_cap;
    if (const char* e = std::getenv("PROMETHEUS_WALK_DECODE_CAP"); e != nullptr && *e != '\0') {
      alt_walk_decode_cap = std::max(0, std::atoi(e)); // global walk window-installs per cycle (0 = unlimited)
    }
    alt_walk_decode_shared = (b.m_trace_walk_decode_shared != 0);
    if (const char* e = std::getenv("PROMETHEUS_WALK_DECODE_SHARED"); e != nullptr && *e != '\0') {
      alt_walk_decode_shared = (std::atoi(e) != 0); // walks may install only in stream-mode cycles (shared decoders)
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

    // self-report the RESOLVED knob values (env > JSON > default) so every log
    // records the exact configuration that produced it (stale-binary insurance)
    std::cout << "Prometheus knobs: fill=" << b.m_trace_fill << " min_occ=" << stall.min_occ() << " rob=" << stall_rob_threshold
              << " depth=" << stall.max_trace_uops() << " l1i_gate=" << stall.l1i_gated() << " meta_windows=" << meta_windows
              << " store=" << fill_store.cap() << " sets=" << fill_store.sets() << " ways=" << fill_store.ways() << " hash=" << fill_store.hash_mode()
              << " cost_evict=" << fill_store.cost_policy() << " walk_max=" << alt_walk_max << " walk_width=" << alt_walk_width
              << " walk_delay=" << alt_walk_delay_cycles << " walk_l1i=" << alt_walk_l1i << " walk_wait=" << alt_walk_wait
              << " wait_cap=" << alt_walk_wait_cap << " issue_cap=" << alt_walk_issue_cap << " decode_cap=" << alt_walk_decode_cap
              << " decode_shared=" << alt_walk_decode_shared << " walk_filter=" << alt_walk_filter << " uop_buffer=" << uop_buffer_windows
              << " probe_ahead=" << chain_probe_ahead << " head_uops=" << head_uops << " tail_targets=" << tail_targets
              << " ucp=" << ucp_enable << " ucp_T=" << ucp_threshold << " ucp_max_ip=" << ucp_max_ip_check << " ucp_step=" << ucp_step
              << " ucp_altind=" << ucp_altind_enable << std::endl;
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

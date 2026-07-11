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
  int m_trace_stall_depth{64};         // stall segmenter: max trace length in u-ops (truncation cap)
  int m_trace_store{256};              // trace-cache capacity in traces (bounded fill modes)
  int m_trace_walk_delay{5};           // alt fill: trigger-to-first-install latency in cycles
  int m_trace_walk_max{2};             // alt fill: max concurrent walks
  int m_trace_walk_width{1};           // alt fill: windows installed per walk per cycle
  int m_trace_walk_wait{0};            // alt fill: 1 = miss on a walk-pending window stalls fetch instead of switching to build
  int m_trace_walk_l1i{0};             // alt fill: 1 = walk fetches bytes through the real L1I (installs gated on line arrival)
  int m_trace_walk_wait_cap{16};       // alt fill: max cycles an instruction waits on a pending window (0 = unbounded)
  int m_trace_stall_l1i_gate{0};       // stall segmenter: 1 = only commit stretches that also missed L1I
  int m_trace_store_cost_evict{0};     // trace store: 1 = evict minimum-stall-cost trace instead of LRU
  int m_trace_head_uops{8};            // head fill: uops stored per trace head (served instantly)
  int m_trace_tail_targets{3};         // head fill: taken targets in the tail manifest (<0 = unlimited)
  int m_trace_uop_buffer{16};          // chain fill: trace-uop staging buffer capacity in windows (0 = install direct)
  int m_trace_meta_windows{4};         // chain fill: manifest window slots per trace (entry + 16-bit deltas)
  int m_trace_probe_ahead{0};          // chain fill: deep-probe lead in instructions ahead of enqueue (0 = enqueue only)
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
   * Stall segmenter max trace length in u-ops: a build-mode stretch longer than
   * this is truncated (default 64).
   */
  self_type& trace_stall_depth(int trace_stall_depth_);

  /**
   * Trace-cache capacity in traces (LRU). Used by all bounded fill modes
   * (miss/every/window/flush/parallel/alt); default 256.
   */
  self_type& trace_store(int trace_store_);

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
   * CHAIN fill mode: probe the instruction supply this many instructions AHEAD
   * of the IFETCH transfer point (models the decoupled BP/FTQ lead over the
   * fetch-point u-op-cache lookup; resets on misprediction).  0 = probe at
   * enqueue only.  Default 0.
   */
  self_type& trace_probe_ahead(int trace_probe_ahead_);

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

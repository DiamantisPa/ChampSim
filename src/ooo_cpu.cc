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

#include "ooo_cpu.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <numeric>
#include <string>
#include <fmt/chrono.h>
#include <fmt/core.h>
#include <fmt/ranges.h>

#include "cache.h"
#include "champsim.h"
#include "deadlock.h"
#include "event_listeners.h"
#include "instruction.h"
#include "util/span.h"

long O3_CPU::operate()
{
  long progress{0};
  progress += retire_rob();                    // retire
  progress += complete_inflight_instruction(); // finalize execution
  progress += execute_instruction();           // execute instructions
  progress += schedule_instruction();          // schedule instructions
  progress += handle_memory_return();          // finalize memory transactions
  progress += operate_lsq();                   // execute memory transactions

  progress += dispatch_instruction(); // dispatch
  progress += decode_instruction();   // decode
  progress += promote_to_decode();

  progress += fetch_instruction(); // fetch
  progress += check_dib();
  initialize_instruction();

  return progress;
}

void O3_CPU::initialize()
{
  // BRANCH PREDICTOR & BTB
  impl_initialize_branch_predictor();
  impl_initialize_btb();
}

void O3_CPU::begin_phase()
{
  begin_phase_instr = num_retired;
  begin_phase_time = current_time;

  // Record where the next phase begins
  stats_type stats;
  stats.name = "CPU " + std::to_string(cpu);
  stats.begin_instrs = num_retired;
  stats.begin_cycles = begin_phase_time.time_since_epoch() / clock_period;
  sim_stats = stats;
}

void O3_CPU::end_phase(unsigned finished_cpu)
{
  // Record where the phase ended (overwrite if this is later)
  sim_stats.end_instrs = num_retired;
  sim_stats.end_cycles = current_time.time_since_epoch() / clock_period;

  if (finished_cpu == this->cpu) {
    finish_phase_instr = num_retired;
    finish_phase_time = current_time;

    finalize_coverage_stats();
    roi_stats = sim_stats;
  }
}

// End-of-phase: compute the derived trace-coverage attribution (time-agnostic
// covered-final, never-covered loss buckets, and cross-builder dynamic diffs)
// from the builders' accumulated per-IP state into sim_stats.
void O3_CPU::finalize_coverage_stats()
{
  if (trace_seg_enable) {
    segmenter.finalize_coverage(sim_stats);
  }
  if (trace_rec_enable) {
    recorder.finalize_coverage(sim_stats);
  }
  if (trace_stg_enable) {
    stager.finalize_coverage(sim_stats);
  }
  if (trace_stall_enable) {
    stall.finalize_buckets(sim_stats);
  }

  // cross-builder dynamic-weighted diffs: the dynamic stream is identical across
  // builders, so iterate any enabled builder's dyn_counts and compare covered sets
  const std::unordered_map<uint64_t, uint64_t>* dyn = nullptr;
  if (trace_seg_enable) {
    dyn = &segmenter.dyn_counts();
  } else if (trace_stg_enable) {
    dyn = &stager.dyn_counts();
  } else if (trace_rec_enable) {
    dyn = &recorder.dyn_counts();
  }
  if (dyn != nullptr) {
    for (const auto& [ip, cnt] : *dyn) {
      if (trace_seg_enable && trace_stg_enable && segmenter.covers(ip) && !stager.covers(ip)) {
        sim_stats.xcov_seg_not_stg += cnt;
      }
      if (trace_seg_enable && trace_rec_enable && segmenter.covers(ip) && !recorder.covers(ip)) {
        sim_stats.xcov_seg_not_rec += cnt;
      }
      if (trace_stg_enable && trace_rec_enable && stager.covers(ip) && !recorder.covers(ip)) {
        sim_stats.xcov_stg_not_rec += cnt;
      }
    }
  }
}

void O3_CPU::initialize_instruction()
{
  champsim::bandwidth instrs_to_read_this_cycle{
      std::min(FETCH_WIDTH, champsim::bandwidth::maximum_type{static_cast<long>(IFETCH_BUFFER_SIZE - std::size(IFETCH_BUFFER))})};

  bool stop_fetch = false;
  while (current_time >= fetch_resume_time && instrs_to_read_this_cycle.has_remaining() && !stop_fetch && !std::empty(input_queue)) {
    instrs_to_read_this_cycle.consume();

    stop_fetch = do_init_instruction(input_queue.front());

    // CHAIN trace-fill, no deep probe (probe_ahead == 0), or an instruction the
    // deep cursor has not reached yet (post-mispredict rebuild): probe at enqueue.
    if (fill_mode == fill_mode_type::CHAIN && !warmup && chain_probe_cursor == 0) {
      if (const auto* ips = fill_store.lookup_entry(input_queue.front().ip.to<uint64_t>()); ips != nullptr && !ips->empty()) {
        do_chain_trigger(*ips);
      }
    }

    // Add to IFETCH_BUFFER
    IFETCH_BUFFER.push_back(input_queue.front());
    input_queue.pop_front();
    if (chain_probe_cursor > 0) {
      --chain_probe_cursor; // the cursor indexes relative to the queue front
    }

    IFETCH_BUFFER.back().ready_time = current_time;
  }

  // CHAIN deep probe: advance the cursor through the instruction supply, probing
  // each pc once, up to chain_probe_ahead instructions beyond the transfer point.
  // This models the decoupled BP/FTQ running ahead of fetch -- the lead a real
  // FTQ-tail probe has over the fetch-point u-op-cache lookup.  The cursor is
  // reset on branch mispredictions (= FTQ flush), so recovery lead rebuilds at
  // probe bandwidth, honestly.
  if (fill_mode == fill_mode_type::CHAIN && !warmup && chain_probe_ahead > 0) {
    const std::size_t limit = std::min(std::size(input_queue), static_cast<std::size_t>(chain_probe_ahead));
    for (champsim::bandwidth probe_bw{FETCH_WIDTH}; probe_bw.has_remaining() && chain_probe_cursor < limit; probe_bw.consume()) {
      if (const auto* ips = fill_store.lookup_entry(input_queue[chain_probe_cursor].ip.to<uint64_t>()); ips != nullptr && !ips->empty()) {
        do_chain_trigger(*ips);
      }
      ++chain_probe_cursor;
    }
  }
}

namespace
{
void do_stack_pointer_folding(ooo_model_instr& arch_instr)
{
  // The exact, true value of the stack pointer for any given instruction can usually be determined immediately after the instruction is decoded without
  // waiting for the stack pointer's dependency chain to be resolved.
  bool writes_sp = (std::count(std::begin(arch_instr.destination_registers), std::end(arch_instr.destination_registers), champsim::REG_STACK_POINTER) > 0);
  if (writes_sp) {
    // Avoid creating register dependencies on the stack pointer for calls, returns, pushes, and pops, but not for variable-sized changes in the
    // stack pointer position. reads_other indicates that the stack pointer is being changed by a variable amount, which can't be determined before
    // execution.
    bool reads_other =
        (std::count_if(std::begin(arch_instr.source_registers), std::end(arch_instr.source_registers),
                       [](auto r) { return r != champsim::REG_STACK_POINTER && r != champsim::REG_FLAGS && r != champsim::REG_INSTRUCTION_POINTER; })
         > 0);
    if ((arch_instr.is_branch) || !(std::empty(arch_instr.destination_memory) && std::empty(arch_instr.source_memory)) || (!reads_other)) {
      auto nonsp_end = std::remove(std::begin(arch_instr.destination_registers), std::end(arch_instr.destination_registers), champsim::REG_STACK_POINTER);
      arch_instr.destination_registers.erase(nonsp_end, std::end(arch_instr.destination_registers));
    }
  }
}
} // namespace

bool O3_CPU::do_predict_branch(ooo_model_instr& arch_instr)
{
  bool stop_fetch = false;

  // handle branch prediction for all instructions as at this point we do not know if the instruction is a branch
  sim_stats.total_branch_types.increment(arch_instr.branch);
  auto [predicted_branch_target, always_taken] = impl_btb_prediction(arch_instr.ip, arch_instr.branch);
  arch_instr.branch_prediction = impl_predict_branch(arch_instr.ip, predicted_branch_target, always_taken, arch_instr.branch) || always_taken;
  if (!arch_instr.branch_prediction) {
    predicted_branch_target = champsim::address{};
  }

  if (arch_instr.is_branch) {
    if constexpr (champsim::debug_print) {
      fmt::print("[BRANCH] instr_id: {} ip: {} taken: {}\n", arch_instr.instr_id, arch_instr.ip, arch_instr.branch_taken);
    }

    // call code prefetcher every time the branch predictor is used
    l1i->impl_prefetcher_branch_operate(arch_instr.ip, arch_instr.branch, predicted_branch_target);

    if (predicted_branch_target != arch_instr.branch_target
        || (((arch_instr.branch == BRANCH_CONDITIONAL) || (arch_instr.branch == BRANCH_OTHER))
            && arch_instr.branch_taken != arch_instr.branch_prediction)) { // conditional branches are re-evaluated at decode when the target is computed
      sim_stats.total_rob_occupancy_at_branch_mispredict += std::size(ROB);
      sim_stats.branch_type_misses.increment(arch_instr.branch);
      if (!warmup) {
        fetch_resume_time = champsim::chrono::clock::time_point::max();
        stop_fetch = true;
        arch_instr.branch_mispredicted = true;
      }
    } else {
      stop_fetch = arch_instr.branch_taken; // if correctly predicted taken, then we can't fetch anymore instructions this cycle
    }

    impl_update_btb(arch_instr.ip, arch_instr.branch_target, arch_instr.branch_taken, arch_instr.branch);
    impl_last_branch_result(arch_instr.ip, arch_instr.branch_target, arch_instr.branch_taken, arch_instr.branch);
  }

  return stop_fetch;
}

bool O3_CPU::do_init_instruction(ooo_model_instr& arch_instr)
{
  // fast warmup eliminates register dependencies between instructions branch predictor, cache contents, and prefetchers are still warmed up
  if (warmup) {
    arch_instr.source_registers.clear();
    arch_instr.destination_registers.clear();
  }

  ::do_stack_pointer_folding(arch_instr);
  return do_predict_branch(arch_instr);
}

long O3_CPU::check_dib()
{
  // ALT/HEAD/CHAIN trace-fill: advance in-flight pre-decode walks (timed installs) once per cycle
  if ((fill_mode == fill_mode_type::ALT || fill_mode == fill_mode_type::HEAD || fill_mode == fill_mode_type::CHAIN) && !alt_walks.empty()) {
    drain_alt_walks();
  }
  // scan through IFETCH_BUFFER to find instructions that hit in the decoded instruction buffer
  auto begin = std::find_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), [](const ooo_model_instr& x) { return !x.dib_checked; });
  auto [window_begin, window_end] = champsim::get_span(begin, std::end(IFETCH_BUFFER), champsim::bandwidth{FETCH_WIDTH});
  long progress{0};
  for (auto it = window_begin; it != window_end; ++it) {
    this->do_check_dib(*it);
    if (!it->dib_checked) {
      break; // ALT wait-on-pending: fetch stalls here this cycle; younger entries stay in order
    }
    ++progress;
  }
  return progress;
}

// ALT trace-fill: install ready windows of in-flight walks, one window per walk per
// cycle after the base delay (models L1I fetch + pre-decode through idle decode slots).
void O3_CPU::drain_alt_walks()
{
  for (auto& w : alt_walks) {
    // real-L1I mode: issue one pending line per walk per cycle through the L1I
    // (contends with demand for rq slots and MSHRs; misses go to L2/LLC/DRAM)
    if (alt_walk_l1i && !w.lines_pending.empty()) {
      CacheBus::request_type line_pkt;
      line_pkt.v_address = champsim::address{w.lines_pending.front()};
      line_pkt.ip = champsim::address{w.lines_pending.front()};
      line_pkt.instr_id = 0; // no dependent instructions: response marks the line ready
      if (L1I_bus.issue_read(line_pkt)) {
        w.lines_pending.pop_front();
        ++sim_stats.alt_lines_issued;
        w.last_progress = current_time;
      }
    }
    while (w.idx < w.windows.size() && w.ready <= current_time) {
      // real-L1I mode: a window may only install once its line has arrived
      if (alt_walk_l1i
          && w.lines_ready.count(champsim::block_number{champsim::address{w.windows[w.idx].front()}}.to<uint64_t>()) == 0) {
        ++sim_stats.alt_line_stalls;
        break; // pre-decode stalls on the byte fetch; retry next cycle
      }
      // install up to alt_walk_width windows per cycle (pre-decode width)
      for (int k = 0; k < alt_walk_width && w.idx < w.windows.size(); ++k) {
        if (alt_walk_l1i && k > 0
            && w.lines_ready.count(champsim::block_number{champsim::address{w.windows[w.idx].front()}}.to<uint64_t>()) == 0) {
          break;
        }
        if (fill_mode == fill_mode_type::CHAIN && uop_buffer_windows > 0) {
          // CHAIN: stage into the trace-uop buffer (no u-op-cache pollution);
          // demand hits promote from there
          uop_buffer_push(w.windows[w.idx], w.probe_time);
          ++sim_stats.alt_installed_windows;
        } else {
          sim_stats.alt_installed_windows += DIB.install(w.windows[w.idx], /*prefetch=*/true);
        }
        ++w.idx;
        w.last_progress = current_time;
      }
      w.ready += clock_period;
    }
  }
  // watchdog: abort a walk that has made no progress for a long time (e.g. a line
  // response that never arrived).  Frees the slot and un-wedges any waiter; the
  // one-shot diagnostic identifies the lost state for root-causing.
  alt_walks.erase(std::remove_if(std::begin(alt_walks), std::end(alt_walks),
                                 [this](const auto& w) {
                                   if (w.idx >= w.windows.size()) {
                                     return true; // complete
                                   }
                                   if (current_time - w.last_progress > ALT_WALK_ABORT_CYCLES * clock_period) {
                                     ++sim_stats.alt_walk_aborts;
                                     fmt::print("[ALT] walk abort: entry {:#x} idx {}/{} next-blk {:#x} lines_pending {} lines_ready {} cycle {}\n",
                                                w.entry, w.idx, w.windows.size(),
                                                champsim::block_number{champsim::address{w.windows[w.idx].front()}}.to<uint64_t>(),
                                                w.lines_pending.size(), w.lines_ready.size(),
                                                current_time.time_since_epoch() / clock_period);
                                     return true; // stuck: abort
                                   }
                                   return false;
                                 }),
                  std::end(alt_walks));
}

// is rawip's window held by an in-flight walk but not yet installed?
bool O3_CPU::alt_window_pending(uint64_t rawip) const
{
  const uint64_t wtag = rawip >> dib_window_bits;
  for (const auto& w : alt_walks) {
    for (std::size_t i = w.idx; i < w.windows.size(); ++i) {
      if ((w.windows[i].front() >> dib_window_bits) == wtag) {
        return true;
      }
    }
  }
  return false;
}

// CHAIN metadata format encoder: group the trace's uops into aligned windows,
// keep the first meta_windows window slots (truncating longer traces), and
// verify every consecutive window delta fits a signed 16-bit field (±2^15
// windows = ±1MB).  Returns the kept uops; encodable=false rejects the trace.
std::vector<uint64_t> O3_CPU::chain_encode(const std::vector<uint64_t>& ips, bool& truncated, bool& encodable) const
{
  std::vector<uint64_t> kept;
  kept.reserve(ips.size());
  truncated = false;
  encodable = true;
  int windows_used = 0;
  uint64_t last_tag = 0;
  bool have_tag = false;
  for (uint64_t raw : ips) {
    const uint64_t tag = raw >> dib_window_bits;
    if (!have_tag || tag != last_tag) { // a new window slot
      if (windows_used == meta_windows) {
        truncated = true;
        break;
      }
      if (have_tag) {
        const int64_t delta = static_cast<int64_t>(tag) - static_cast<int64_t>(last_tag);
        if (delta < -32768 || delta > 32767) {
          encodable = false; // far transfer: not representable in the 16-bit delta
          return kept;
        }
      }
      ++windows_used;
      have_tag = true;
      last_tag = tag;
    }
    kept.push_back(raw);
  }
  return kept;
}

// CHAIN walk launch: probe hit at IFETCH enqueue on a trace entry.  The manifest
// is already format-enforced (<= meta_windows windows, deltas encodable), so the
// walk covers the whole stored trace.  All line addresses are known immediately
// (entry pc + deltas); in real-L1I mode the first lines issue THIS cycle.
void O3_CPU::do_chain_trigger(const std::vector<uint64_t>& ips)
{
  const uint64_t entry = ips.front();
  if (DIB.probe(champsim::address{entry})) {
    return; // entry window already resident: nothing to prefetch
  }
  const uint64_t etag = entry >> dib_window_bits;
  if (std::any_of(std::begin(uop_buffer), std::end(uop_buffer), [etag](const auto& e) { return e.tag == etag; })) {
    return; // already staged
  }
  if (std::any_of(std::begin(alt_walks), std::end(alt_walks), [entry](const auto& w) { return w.entry == entry; })) {
    return; // already being walked
  }
  if (alt_walks.size() >= alt_walk_max) {
    ++sim_stats.alt_drops;
    return;
  }
  alt_walk w;
  w.entry = entry;
  w.ready = current_time + alt_walk_delay_cycles * clock_period;
  w.last_progress = current_time;
  w.probe_time = current_time;
  uint64_t last = 0;
  bool have_last = false;
  for (uint64_t raw : ips) {
    const uint64_t tag = raw >> dib_window_bits;
    if (have_last && tag == last) {
      w.windows.back().push_back(raw);
      continue;
    }
    have_last = true;
    last = tag;
    w.windows.push_back({raw});
  }
  if (alt_walk_l1i) {
    uint64_t last_blk = 0;
    bool have_blk = false;
    for (const auto& win : w.windows) {
      const uint64_t blk = champsim::block_number{champsim::address{win.front()}}.to<uint64_t>();
      if (have_blk && blk == last_blk) {
        continue;
      }
      have_blk = true;
      last_blk = blk;
      w.lines_pending.push_back(win.front());
    }
    // all line addresses are known from the manifest: issue up to two this cycle
    for (int n = 0; n < 2 && !w.lines_pending.empty(); ++n) {
      CacheBus::request_type line_pkt;
      line_pkt.v_address = champsim::address{w.lines_pending.front()};
      line_pkt.ip = champsim::address{w.lines_pending.front()};
      line_pkt.instr_id = 0;
      if (!L1I_bus.issue_read(line_pkt)) {
        break;
      }
      w.lines_pending.pop_front();
      ++sim_stats.alt_lines_issued;
    }
  }
  alt_walks.push_back(std::move(w));
  ++sim_stats.alt_triggers;
}

// CHAIN staging buffer: FIFO of pre-decoded windows awaiting demand.  A refresh
// of a resident tag replaces it; capacity eviction discards the oldest window
// (counted as overshoot if it was never demanded -- hits erase their entry).
void O3_CPU::uop_buffer_push(const std::vector<uint64_t>& window_ips, champsim::chrono::clock::time_point probe_time)
{
  const uint64_t tag = window_ips.front() >> dib_window_bits;
  if (auto it = std::find_if(std::begin(uop_buffer), std::end(uop_buffer), [tag](const auto& e) { return e.tag == tag; });
      it != std::end(uop_buffer)) {
    it->ips = window_ips; // refresh
    it->probe_time = probe_time;
    return;
  }
  while (uop_buffer.size() >= static_cast<std::size_t>(uop_buffer_windows)) {
    uop_buffer.pop_front();
    ++sim_stats.chain_buf_evict_unused; // anything still resident was never demanded
  }
  uop_buffer.push_back({tag, window_ips, probe_time});
}

// wait-eligible variant: the walk holding the window, or nullptr -- background
// walks (wait-cap expired: their bytes proved far) are not waited on.
O3_CPU::alt_walk* O3_CPU::alt_pending_walk(uint64_t rawip)
{
  const uint64_t wtag = rawip >> dib_window_bits;
  for (auto& w : alt_walks) {
    if (w.background) {
      continue;
    }
    for (std::size_t i = w.idx; i < w.windows.size(); ++i) {
      if ((w.windows[i].front() >> dib_window_bits) == wtag) {
        return &w;
      }
    }
  }
  return nullptr;
}

// ALT trace-fill trigger: a demand u-op-cache miss PC probes the metadata trace
// cache.  Trace existence is the cost filter -- traces are only captured for
// stretches that stalled the backend.  A hit launches a timed walk that begins
// at the MISSED window within the trace (installing windows demand has already
// passed would be wasted), unless this trace is already being walked or the
// walk limit is reached.
void O3_CPU::do_alt_trigger(uint64_t alt_pc)
{
  const auto* ips = fill_store.lookup_window(alt_pc);
  if (ips == nullptr || ips->empty()) {
    return;
  }
  const uint64_t entry = ips->front();
  if (std::any_of(std::begin(alt_walks), std::end(alt_walks), [entry](const auto& w) { return w.entry == entry; })) {
    return; // this trace is already being walked
  }
  if (alt_walks.size() >= alt_walk_max) {
    ++sim_stats.alt_drops;
    return;
  }
  alt_walk w;
  w.entry = entry;
  w.ready = current_time + alt_walk_delay_cycles * clock_period;
  w.last_progress = current_time;
  uint64_t last = 0;
  bool have_last = false;
  for (uint64_t raw : *ips) { // group the trace's ips into distinct aligned windows, in order
    const uint64_t tag = raw >> dib_window_bits;
    if (have_last && tag == last) {
      w.windows.back().push_back(raw);
      continue;
    }
    have_last = true;
    last = tag;
    w.windows.push_back({raw});
  }
  // start at the missed window: skip windows demand has already gone past
  const uint64_t miss_tag = alt_pc >> dib_window_bits;
  for (std::size_t i = 0; i < w.windows.size(); ++i) {
    if ((w.windows[i].front() >> dib_window_bits) == miss_tag) {
      w.idx = i;
      break;
    }
  }
  // real-L1I mode: collect the unique cache lines of the remaining windows; the
  // walk issues them through the L1I (run-lengths known up front -> line-level MLP)
  if (alt_walk_l1i) {
    uint64_t last_blk = 0;
    bool have_blk = false;
    for (std::size_t i = w.idx; i < w.windows.size(); ++i) {
      const uint64_t rep = w.windows[i].front();
      const uint64_t blk = champsim::block_number{champsim::address{rep}}.to<uint64_t>();
      if (have_blk && blk == last_blk) {
        continue;
      }
      have_blk = true;
      last_blk = blk;
      w.lines_pending.push_back(rep);
    }
  }
  alt_walks.push_back(std::move(w));
  ++sim_stats.alt_triggers;
}

// HEAD trace-fill walk: pre-decode the trace's TAIL, from start_idx, following
// the recorded path through at most tail_targets taken transfers (the manifest's
// reach; a discontinuity in consecutive ips = a taken transfer, 4B instructions).
// In real-L1I mode the first line is issued THIS cycle (same cycle as the head
// hit) so the tail's fetch latency overlaps head consumption.
void O3_CPU::do_head_trigger(const std::vector<uint64_t>& ips, std::size_t start_idx)
{
  if (start_idx >= ips.size()) {
    return; // trace is all head, or start beyond the trace
  }
  const uint64_t entry = ips.front();
  if (std::any_of(std::begin(alt_walks), std::end(alt_walks), [entry](const auto& w) { return w.entry == entry; })) {
    return; // this trace's tail is already being walked
  }
  if (alt_walks.size() >= alt_walk_max) {
    ++sim_stats.alt_drops;
    return;
  }
  // manifest truncation: stop after tail_targets taken transfers past start_idx
  std::size_t end_idx = ips.size();
  if (tail_targets >= 0) {
    int taken = 0;
    for (std::size_t i = start_idx + 1; i < ips.size(); ++i) {
      if (ips[i] != ips[i - 1] + 4) { // discontinuity = taken transfer
        if (++taken > tail_targets) {
          end_idx = i;
          break;
        }
      }
    }
  }
  alt_walk w;
  w.entry = entry;
  w.ready = current_time + alt_walk_delay_cycles * clock_period;
  w.last_progress = current_time;
  uint64_t last = 0;
  bool have_last = false;
  for (std::size_t i = start_idx; i < end_idx; ++i) {
    const uint64_t raw = ips[i];
    const uint64_t tag = raw >> dib_window_bits;
    if (have_last && tag == last) {
      w.windows.back().push_back(raw);
      continue;
    }
    have_last = true;
    last = tag;
    w.windows.push_back({raw});
  }
  if (w.windows.empty()) {
    return;
  }
  if (alt_walk_l1i) {
    uint64_t last_blk = 0;
    bool have_blk = false;
    for (const auto& win : w.windows) {
      const uint64_t rep = win.front();
      const uint64_t blk = champsim::block_number{champsim::address{rep}}.to<uint64_t>();
      if (have_blk && blk == last_blk) {
        continue;
      }
      have_blk = true;
      last_blk = blk;
      w.lines_pending.push_back(rep);
    }
    // issue the first tail line in the SAME cycle as the head hit
    if (!w.lines_pending.empty()) {
      CacheBus::request_type line_pkt;
      line_pkt.v_address = champsim::address{w.lines_pending.front()};
      line_pkt.ip = champsim::address{w.lines_pending.front()};
      line_pkt.instr_id = 0;
      if (L1I_bus.issue_read(line_pkt)) {
        w.lines_pending.pop_front();
        ++sim_stats.alt_lines_issued;
      }
    }
  }
  alt_walks.push_back(std::move(w));
  ++sim_stats.alt_triggers;
}

void O3_CPU::do_check_dib(ooo_model_instr& instr)
{
  // Check the micro-op cache to see if we recently decoded this window
  bool was_prefetched = false;
  bool hit = DIB.check_hit(instr.ip, &was_prefetched);
  // ALT: on a demand miss, probe the trace store and launch the walk FIRST (dedup
  // and walk-cap inside do_alt_trigger), then wait-on-pending.  Trigger-before-wait
  // means even the walk's own triggering miss is served as a delayed hit: the
  // frontend never switches to build mode for a covered miss (no switch stall per
  // walk).  Uncovered or walk-capped misses fall through to the normal miss path.
  // The wait leaves the entry un-checked (re-checks next cycle, L1I fetch held);
  // placed before any counter so re-checks are side-effect-free.
  if (fill_mode == fill_mode_type::ALT && !hit && !warmup) {
    do_alt_trigger(instr.ip.to<uint64_t>());
    // wait-cap: wait on the fill buffer, not on DRAM.  An instruction stalls at
    // most alt_walk_wait_cap cycles on a pending window (enough to cover an
    // L1I-hit fill); if the cap expires the bytes proved far, so the WHOLE WALK
    // is demoted to a background prefetcher -- no instruction waits on it again,
    // demand proceeds through build mode (its fetch MSHR-merges with the walk's
    // line requests) while the walk keeps installing ahead.  Cap 0 = unbounded.
    if (alt_walk_wait) {
      if (alt_walk* holder = alt_pending_walk(instr.ip.to<uint64_t>()); holder != nullptr) {
        if (alt_walk_wait_cap == 0 || instr.dib_wait_cycles < alt_walk_wait_cap) {
          ++instr.dib_wait_cycles;
          ++sim_stats.alt_wait_cycles;
          return; // dib_checked stays false -> retried next cycle (hit-under-fill)
        }
        holder->background = true; // bytes are far: stop waiting on this walk
      }
    }
  }
  // HEAD trace-fill: a trace is an instantly-servable HEAD (first head_uops uops,
  // held as uops in the store) plus a fetch MANIFEST for the tail (the next
  // tail_targets taken targets).  A miss that lands in a resident trace's head
  // region is served directly to the backend (hit, no DIB install, no bytes),
  // and the tail walk -- real L1I/L2 line fetches + pre-decode -- launches the
  // SAME cycle so its latency hides behind head consumption.  A miss in the tail
  // region launches the walk from that point and uses the ALT wait machinery.
  if (fill_mode == fill_mode_type::HEAD && !hit && !warmup) {
    const uint64_t rawip = instr.ip.to<uint64_t>();
    if (const auto* ips = fill_store.lookup_window(rawip); ips != nullptr && !ips->empty()) {
      const uint64_t wtag = rawip >> dib_window_bits;
      std::size_t idx = ips->size(); // first trace position in the missed window
      for (std::size_t i = 0; i < ips->size(); ++i) {
        if (((*ips)[i] >> dib_window_bits) == wtag) {
          idx = i;
          break;
        }
      }
      if (idx < static_cast<std::size_t>(head_uops)) {
        ++sim_stats.uop_trace_fill_hits; // head uops feed the backend directly
        hit = true;
        do_head_trigger(*ips, static_cast<std::size_t>(head_uops)); // tail walk, same cycle
      } else if (idx < ips->size()) {
        do_head_trigger(*ips, idx); // tail-region miss: walk from here
      }
    }
    if (!hit && alt_walk_wait) {
      if (alt_walk* holder = alt_pending_walk(rawip); holder != nullptr) {
        if (alt_walk_wait_cap == 0 || instr.dib_wait_cycles < alt_walk_wait_cap) {
          ++instr.dib_wait_cycles;
          ++sim_stats.alt_wait_cycles;
          return; // hit-under-fill: wait for the in-flight tail
        }
        holder->background = true;
      }
    }
  }
  // CHAIN trace-fill: on a u-op-cache miss, probe the trace-uop staging buffer
  // (walk output).  A hit serves the uops directly and PROMOTES the window into
  // the u-op cache as a demand-class fill; the slack histogram records how far
  // ahead of demand the walk ran.  A miss whose window is in an in-flight walk
  // uses the wait machinery.
  if (fill_mode == fill_mode_type::CHAIN && !hit && !warmup) {
    const uint64_t rawip = instr.ip.to<uint64_t>();
    const uint64_t wtag = rawip >> dib_window_bits;
    auto bit = std::find_if(std::begin(uop_buffer), std::end(uop_buffer), [wtag](const auto& e) { return e.tag == wtag; });
    if (bit != std::end(uop_buffer)) {
      ++sim_stats.chain_buf_hits;
      hit = true;
      const auto slack = (current_time - bit->probe_time) / clock_period;
      const std::size_t bucket = (slack <= 4) ? 0 : (slack <= 8) ? 1 : (slack <= 16) ? 2 : (slack <= 32) ? 3 : (slack <= 64) ? 4 : 5;
      ++sim_stats.chain_slack[bucket];
      DIB.install(bit->ips, /*prefetch=*/false); // promote: demand-class fill at MRU
      uop_buffer.erase(bit);
    } else if (alt_walk_wait) {
      if (alt_walk* holder = alt_pending_walk(rawip); holder != nullptr) {
        if (alt_walk_wait_cap == 0 || instr.dib_wait_cycles < alt_walk_wait_cap) {
          ++instr.dib_wait_cycles;
          ++sim_stats.alt_wait_cycles;
          return; // walk in flight: hit-under-fill
        }
        holder->background = true;
      }
    }
  }
  if (was_prefetched) {
    ++sim_stats.alt_useful_hits; // first demand hit on a walk-installed window
  }
  ++sim_stats.uop_cache_reads;
  // Trace-fill: install a stored trace's windows into the DIB.  The triggering
  // event depends on the mode (see fill_mode_type).  A miss turned into a hit by
  // the install is served without a build-mode switch.  FLUSH does no demand fill
  // here -- it installs only on a misprediction (see do_flush_fill).
  // PERFECT trace cache (upper bound): a miss is served as a hit if the active
  // builder's traces cover this window -- unbounded, proactive, no store.  Measures
  // the IPC ceiling of the segmentation algorithm itself (staging / stall / filtered).
  if (fill_mode == fill_mode_type::PERFECT && !hit) {
    const uint64_t ip64 = instr.ip.to<uint64_t>();
    if ((trace_stall_enable && stall.covers(ip64)) || (trace_stg_enable && stager.covers(ip64))) {
      ++sim_stats.uop_trace_fill_hits;
      hit = true;
    }
  }
  // PARALLEL trace cache: probed alongside the u-op cache, feed-only.  On a u-op-cache
  // miss, if the *bounded* trace store covers this window, the fetch is served from the
  // trace cache (hit) WITHOUT installing into the DIB -- the u-op cache's contents are
  // never disturbed (no eviction/pollution).  If neither hits, it falls through to the
  // genuine-miss path (build mode -> L1I).  This models a realistic trace cache sitting
  // in parallel with the u-op cache, unlike WINDOW which installs into the DIB.
  if (fill_mode == fill_mode_type::PARALLEL && !hit) {
    if (fill_store.lookup_window(instr.ip.to<uint64_t>()) != nullptr) {
      ++sim_stats.uop_trace_fill_hits;
      hit = true;
    }
  }
  if (fill_mode != fill_mode_type::OFF && fill_mode != fill_mode_type::FLUSH && fill_mode != fill_mode_type::PERFECT
      && fill_mode != fill_mode_type::PARALLEL && fill_mode != fill_mode_type::ALT && fill_mode != fill_mode_type::HEAD
      && fill_mode != fill_mode_type::CHAIN) {
    const std::vector<uint64_t>* ips = nullptr;
    if (fill_mode == fill_mode_type::EVERY) {
      ips = fill_store.lookup_entry(instr.ip.to<uint64_t>()); // hit-or-miss at a trace entry
    } else if (!hit) {                                        // MISS / WINDOW: only on a miss
      ips = (fill_mode == fill_mode_type::WINDOW) ? fill_store.lookup_window(instr.ip.to<uint64_t>())
                                                  : fill_store.lookup_entry(instr.ip.to<uint64_t>());
    }
    if (ips != nullptr) {
      const std::size_t installed = DIB.install(*ips);
      sim_stats.uop_trace_fill_windows += installed;
      if (!hit && installed > 0) { // count only misses turned into hits (no-op when no_uop)
        ++sim_stats.uop_trace_fill_hits;
        hit = true;
      }
    }
  }
  if (hit) {
    ++sim_stats.uop_cache_hits;
    // The u-ops are in the DIB, so we can mark this as complete
    instr.fetch_completed = true;

    // Also mark it as decoded
    instr.decoded = true;

    // It can be acted on immediately
    instr.ready_time = current_time;

    // Any hit (re)enters stream mode (no hysteresis, as in UCP_ISCA24)
    fetch_mode = fetch_mode_type::STREAM;
    in_recovery = false; // back to streaming: the misprediction refill is complete
  } else {
    // genuine u-op-cache miss -> build mode. Bucket it as recovery vs steady-state,
    // and sub-count whether the missing IP is covered by a stored trace (the
    // ceiling for what trace-fill could serve).
    const uint64_t rawip = instr.ip.to<uint64_t>();
    // ALT: miss on a window that an in-flight walk holds but has not yet installed --
    // the trigger fired, the walk was just too slow.  Measures the timing loss.
    // (With alt_walk_wait on, these become alt_wait_cycles instead and this stays ~0.)
    if (fill_mode == fill_mode_type::ALT && alt_window_pending(rawip)) {
      ++sim_stats.alt_late_misses;
    }
    const bool traced = (trace_stg_enable && stager.covers(rawip)) || (trace_stall_enable && stall.covers(rawip));
    if (in_recovery) {
      ++sim_stats.uop_miss_recovery;
      if (traced) {
        ++sim_stats.uop_miss_recovery_traced;
      }
    } else {
      ++sim_stats.uop_miss_steady;
      if (traced) {
        ++sim_stats.uop_miss_steady_traced;
      }
    }
    if (fetch_mode == fetch_mode_type::STREAM) {
      // stream -> build switch on the first miss: pay a 1-cycle fetch stall
      fetch_mode = fetch_mode_type::BUILD;
      if (!warmup) {
        fetch_resume_time = std::max(fetch_resume_time, current_time + clock_period);
      }
      ++sim_stats.switch_stalls;
    }
  }

  // stall-triggered segmenter: feed the fetch stream (hit closes/commits a costly
  // build-mode stretch; miss extends the current one). See inc/trace_stall.h.  When
  // it commits a new trace and trace-fill is on, push it into the trace cache -- so
  // trace_builder=stall makes the stall segmenter the fill source.
  if (trace_stall_enable && !warmup) {
    const std::size_t before = stall.get_traces().size();
    stall.on_dib(instr.ip.to<uint64_t>(), hit, sim_stats);
    if (fill_mode != fill_mode_type::OFF) {
      if (stall.get_traces().size() > before) {
        const auto& t = stall.get_traces().back();
        if (fill_mode == fill_mode_type::CHAIN) {
          // enforce the metadata format at insert: <= meta_windows window slots,
          // every window delta encodable in 16 bits (else the trace is rejected)
          bool truncated = false;
          bool encodable = true;
          auto enc = chain_encode(t.ips, truncated, encodable);
          if (encodable) {
            if (truncated) {
              ++sim_stats.chain_truncated;
            }
            fill_store.insert(t.entry, enc, t.cost);
          } else {
            ++sim_stats.chain_unencodable;
          }
        } else {
          fill_store.insert(t.entry, t.ips, t.cost);
        }
      } else if (stall.touched()) {
        // re-capture of a stored stretch: refresh its accumulated cost so the
        // cost-aware eviction policy sees current values
        fill_store.update_cost(stall.touch_entry(), stall.touch_cost());
      }
    }
  }

  instr.dib_checked = true;

  if constexpr (champsim::debug_print) {
    fmt::print("[DIB] {} instr_id: {} ip: {} hit: {} cycle: {}\n", __func__, instr.instr_id, instr.ip, hit,
               current_time.time_since_epoch() / clock_period);
  }
}

void O3_CPU::do_flush_fill(const ooo_model_instr& branch)
{
  // FLUSH mode: on a branch misprediction, proactively install the trace covering
  // the recovery PC so the frontend refills the pipeline from the u-op cache.
  // The recovery PC is branch_target (the resolved next-IP) for a taken branch;
  // it is empty for a not-taken branch (fall-through window is already resident).
  // Window-matched: the recovery PC is an in-path address, almost never a trace
  // entry.  Models a zero-latency prefetch (optimistic upper bound).
  if (fill_mode != fill_mode_type::FLUSH) {
    return;
  }
  const uint64_t recovery = branch.branch_target.to<uint64_t>();
  if (recovery == 0) {
    return;
  }
  if (const auto* ips = fill_store.lookup_window(recovery)) {
    const std::size_t installed = DIB.install(*ips);
    sim_stats.uop_trace_fill_windows += installed;
    if (installed > 0) {
      ++sim_stats.uop_trace_fill_hits; // here: count of mispredicts served a recovery trace
    }
  }
}

long O3_CPU::fetch_instruction()
{
  long progress{0};

  // Fetch a single cache line
  auto fetch_ready = [](const ooo_model_instr& x) {
    return x.dib_checked && !x.fetch_issued;
  };

  // Find the chunk of instructions in the block
  auto no_match_ip = [](const auto& lhs, const auto& rhs) {
    return champsim::block_number{lhs.ip} != champsim::block_number{rhs.ip};
  };

  auto l1i_req_begin = std::find_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), fetch_ready);
  for (champsim::bandwidth l1i_bw{L1I_BANDWIDTH}; l1i_bw.has_remaining() && l1i_req_begin != std::end(IFETCH_BUFFER); l1i_bw.consume()) {
    auto l1i_req_end = std::adjacent_find(l1i_req_begin, std::end(IFETCH_BUFFER), no_match_ip);
    if (l1i_req_end != std::end(IFETCH_BUFFER)) {
      l1i_req_end = std::next(l1i_req_end); // adjacent_find returns the first of the non-equal elements
    }

    // Issue to L1I
    auto success = do_fetch_instruction(l1i_req_begin, l1i_req_end);
    if (success) {
      std::for_each(l1i_req_begin, l1i_req_end, [t = current_time](auto& x) {
        x.fetch_issued = true;
        x.fetch_issue_time = t; // completion latency > L1I hit => the fetch missed L1I
      });
      ++progress;
    }

    l1i_req_begin = std::find_if(l1i_req_end, std::end(IFETCH_BUFFER), fetch_ready);
  }

  return progress;
}

bool O3_CPU::do_fetch_instruction(std::deque<ooo_model_instr>::iterator begin, std::deque<ooo_model_instr>::iterator end)
{
  CacheBus::request_type fetch_packet;
  fetch_packet.v_address = begin->ip;
  fetch_packet.instr_id = begin->instr_id;
  fetch_packet.ip = begin->ip;

  std::transform(begin, end, std::back_inserter(fetch_packet.instr_depend_on_me), [](const auto& instr) { return instr.instr_id; });

  if constexpr (champsim::debug_print) {
    fmt::print("[IFETCH] {} instr_id: {} ip: {} dependents: {} event_cycle: {}\n", __func__, begin->instr_id, begin->ip,
               std::size(fetch_packet.instr_depend_on_me), begin->ready_time.time_since_epoch() / clock_period);
  }

  return L1I_bus.issue_read(fetch_packet);
}

long O3_CPU::promote_to_decode()
{
  auto is_decoded = [](const ooo_model_instr& x) {
    return x.decoded;
  };

  auto fetch_complete_and_ready = [time = current_time](const auto& x) {
    return x.fetch_completed && x.ready_time <= time;
  };

  champsim::bandwidth available_fetch_bandwidth{
      std::min(FETCH_WIDTH, std::min(champsim::bandwidth::maximum_type{static_cast<long>(DIB_HIT_BUFFER_SIZE - std::size(DIB_HIT_BUFFER))},
                                     champsim::bandwidth::maximum_type{static_cast<long>(DECODE_BUFFER_SIZE - std::size(DECODE_BUFFER))}))};

  auto fetched_check_end = std::find_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), [](const ooo_model_instr& x) { return !x.fetch_completed; });
  // find the first not fetch completed
  auto [window_begin, window_end] = champsim::get_span_p(std::begin(IFETCH_BUFFER), fetched_check_end, available_fetch_bandwidth, fetch_complete_and_ready);
  auto decoded_window_end = std::stable_partition(window_begin, window_end, is_decoded); // reorder instructions
  auto mark_for_decode = [time = current_time, lat = DECODE_LATENCY, warmup = warmup](auto& x) {
    return x.ready_time = time + (warmup ? champsim::chrono::clock::duration{} : lat);
  };
  // to DIB_HIT_BUFFER
  auto mark_for_dib = [time = current_time, lat = DIB_HIT_LATENCY, warmup = warmup](auto& x) {
    return x.ready_time = time + lat;
  };

  std::for_each(window_begin, decoded_window_end, mark_for_dib); // assume DECODE_LATENCY = DIB_HIT_LATENCY
  std::move(window_begin, decoded_window_end, std::back_inserter(DIB_HIT_BUFFER));
  // to DECODE_BUFFER

  std::for_each(decoded_window_end, window_end, mark_for_decode);
  std::move(decoded_window_end, window_end, std::back_inserter(DECODE_BUFFER));

  long progress{std::distance(window_begin, window_end)};
  IFETCH_BUFFER.erase(window_begin, window_end);
  return progress;
}
long O3_CPU::decode_instruction()
{
  auto is_ready = [time = current_time](const auto& x) {
    return x.ready_time <= time;
  };

  auto dib_hit_buffer_begin = std::begin(DIB_HIT_BUFFER);
  auto dib_hit_buffer_end = dib_hit_buffer_begin;
  auto decode_buffer_begin = std::begin(DECODE_BUFFER);
  auto decode_buffer_end = decode_buffer_begin;

  champsim::bandwidth available_decode_bandwidth{DECODE_WIDTH};

  // bw move instructions to dispatch_buffer
  champsim::bandwidth available_dib_inorder_bandwidth{
      std::min(DIB_INORDER_WIDTH, champsim::bandwidth::maximum_type{static_cast<long>(DISPATCH_BUFFER_SIZE - std::size(DISPATCH_BUFFER))})};

  // conditions choose how many instructions sent to dispatch_buffer
  while (dib_hit_buffer_end != std::end(DIB_HIT_BUFFER) && decode_buffer_end != std::end(DECODE_BUFFER) && available_dib_inorder_bandwidth.has_remaining()
         && available_decode_bandwidth.has_remaining() && is_ready(std::min(*dib_hit_buffer_end, *decode_buffer_end, ooo_model_instr::program_order))) {
    if (ooo_model_instr::program_order(*dib_hit_buffer_end, *decode_buffer_end)) {
      dib_hit_buffer_end++;
      available_dib_inorder_bandwidth.consume();
    } else {
      decode_buffer_end++;
      available_dib_inorder_bandwidth.consume();
      available_decode_bandwidth.consume();
    }
  }
  while (dib_hit_buffer_end != std::end(DIB_HIT_BUFFER) && available_dib_inorder_bandwidth.has_remaining() && is_ready(*dib_hit_buffer_end)
         && (decode_buffer_end == std::end(DECODE_BUFFER) || ooo_model_instr::program_order(*dib_hit_buffer_end, *decode_buffer_end))) {
    dib_hit_buffer_end++;
    available_dib_inorder_bandwidth.consume();
  }
  while (decode_buffer_end != std::end(DECODE_BUFFER) && available_dib_inorder_bandwidth.has_remaining() && available_decode_bandwidth.has_remaining()
         && is_ready(*decode_buffer_end)
         && (dib_hit_buffer_end == std::end(DIB_HIT_BUFFER) || ooo_model_instr::program_order(*decode_buffer_end, *dib_hit_buffer_end))) {
    decode_buffer_end++;
    available_dib_inorder_bandwidth.consume();
    available_decode_bandwidth.consume();
  }

  // decode instructions have not decoded, merge instructions with dib_hit_buffer then send to dispatch_buffer
  auto do_decode = [&, this](auto& db_entry) {
    this->do_dib_update(db_entry);

    // Resume fetch
    if (db_entry.branch_mispredicted) {
      // These branches detect the misprediction at decode
      if ((db_entry.branch == BRANCH_DIRECT_JUMP) || (db_entry.branch == BRANCH_DIRECT_CALL)
          || (((db_entry.branch == BRANCH_CONDITIONAL) || (db_entry.branch == BRANCH_OTHER)) && db_entry.branch_taken == db_entry.branch_prediction)) {
        // clear the branch_mispredicted bit so we don't attempt to resume fetch again at execute
        db_entry.branch_mispredicted = 0;
        // pay misprediction penalty
        this->fetch_resume_time = this->current_time + BRANCH_MISPREDICT_PENALTY;
        this->do_flush_fill(db_entry); // FLUSH mode: warm the recovery path
        this->in_recovery = true;      // frontend now refilling the recovery path
        this->chain_probe_cursor = 0;  // CHAIN deep probe: FTQ flush -- lead rebuilds
      }
    }
    // Add to dispatch
    db_entry.ready_time = this->current_time + (this->warmup ? champsim::chrono::clock::duration{} : this->DISPATCH_LATENCY);

    if constexpr (champsim::debug_print) {
      fmt::print("[DECODE] do_decode instr_id: {} time: {}\n", db_entry.instr_id, this->current_time.time_since_epoch() / this->clock_period);
    }
  };

  auto do_dib_hit = [&, this](auto& dib_entry) {
    dib_entry.ready_time = this->current_time + (this->warmup ? champsim::chrono::clock::duration{} : this->DISPATCH_LATENCY);
  };

  std::for_each(decode_buffer_begin, decode_buffer_end, do_decode);
  std::for_each(dib_hit_buffer_begin, dib_hit_buffer_end, do_dib_hit);

  long progress{std::distance(dib_hit_buffer_begin, dib_hit_buffer_end) + std::distance(decode_buffer_begin, decode_buffer_end)};

  auto dispatch_prev_size = std::size(DISPATCH_BUFFER);
  std::merge(dib_hit_buffer_begin, dib_hit_buffer_end, decode_buffer_begin, decode_buffer_end, std::back_inserter(DISPATCH_BUFFER),
             ooo_model_instr::program_order);

  // Prometheus trace segmentation: observe the post-merge u-op stream (u-op
  // cache hits + decoded u-ops, in program order) as it enters dispatch.  Which
  // builder(s) run is resolved once in the constructor from the "trace_builder"
  // config field, overridable at run time by the PROMETHEUS_TRACE env var (see
  // O3_CPU::resolve_trace_builder).
  if (!warmup && (trace_seg_enable || trace_rec_enable || trace_stg_enable)) {
    for (auto idx = dispatch_prev_size; idx < std::size(DISPATCH_BUFFER); ++idx) {
      if (trace_seg_enable) {
        segmenter.push(DISPATCH_BUFFER[idx], sim_stats);
      }
      if (trace_rec_enable) {
        recorder.push(DISPATCH_BUFFER[idx], sim_stats);
      }
      if (trace_stg_enable) {
        const std::size_t before = stager.get_traces().size();
        stager.push(DISPATCH_BUFFER[idx], sim_stats);
        if (fill_mode != fill_mode_type::OFF && stager.get_traces().size() > before) {
          const auto& t = stager.get_traces().back();
          fill_store.insert(t.entry, t.ips);
        }
      }
    }
  }
  DECODE_BUFFER.erase(decode_buffer_begin, decode_buffer_end);
  DIB_HIT_BUFFER.erase(dib_hit_buffer_begin, dib_hit_buffer_end);

  return progress;
}

void O3_CPU::do_dib_update(const ooo_model_instr& instr)
{
  // Build a u-op-cache entry; terminate it on a taken branch (the paper's intent;
  // UCP's demand path used only BRANCH_DIRECT_JUMP) with a 2-branch-per-entry cap.
  const bool taken_end = instr.is_branch && instr.branch_taken;
  DIB.fill(instr.ip, taken_end, instr.is_branch);
}

long O3_CPU::dispatch_instruction()
{
  champsim::bandwidth available_dispatch_bandwidth{DISPATCH_WIDTH};

  // dispatch DISPATCH_WIDTH instructions into the ROB
  while (available_dispatch_bandwidth.has_remaining() && !std::empty(DISPATCH_BUFFER) && DISPATCH_BUFFER.front().ready_time <= current_time
         && std::size(ROB) != ROB_SIZE
         && ((std::size_t)std::count_if(std::begin(LQ), std::end(LQ), [](const auto& lq_entry) { return !lq_entry.has_value(); })
             >= std::size(DISPATCH_BUFFER.front().source_memory))
         && ((std::size(DISPATCH_BUFFER.front().destination_memory) + std::size(SQ)) <= SQ_SIZE)) {
    ROB.push_back(std::move(DISPATCH_BUFFER.front()));
    DISPATCH_BUFFER.pop_front();
    do_memory_scheduling(ROB.back());

    available_dispatch_bandwidth.consume();
    ROB.back().ready_time = current_time + (warmup ? champsim::chrono::clock::duration{} : SCHEDULING_LATENCY);
  }

  // frontend IPC-loss accounting: a cycle where nothing dispatched and the front
  // end had no ready instruction to give, while in BUILD mode -- i.e. the stall is
  // attributable to a u-op-cache miss (not a backend structural stall, and not the
  // branch-mispredict fetch penalty itself).  Two bounds, each split recovery vs
  // steady:  fe_stall_* = ROB had room (dispatch starvation, upper bound on IPC
  // loss); rob_idle_* = ROB fully empty (backend idle, tight lower bound that
  // reconciles with the real->ideal gap).
  if (available_dispatch_bandwidth.amount_consumed() == 0 && fetch_mode == fetch_mode_type::BUILD
      && (std::empty(DISPATCH_BUFFER) || DISPATCH_BUFFER.front().ready_time > current_time)) {
    if (std::size(ROB) != ROB_SIZE) {
      if (in_recovery) {
        ++sim_stats.fe_stall_recovery;
      } else {
        ++sim_stats.fe_stall_steady;
      }
    }
    if (std::empty(ROB)) {
      if (in_recovery) {
        ++sim_stats.rob_idle_recovery;
      } else {
        ++sim_stats.rob_idle_steady;
      }
    }
    // stall-segmenter trigger: a build-mode dispatch-starve cycle where the ROB has
    // drained to at/below stall_rob_threshold marks the in-flight stretch costly.
    // threshold 0 = fully empty (the tight condition); a higher value broadens the
    // trigger to partial (near-empty) stalls, capturing recurring stretches the
    // fully-empty trigger misses.
    if (trace_stall_enable && !warmup && std::size(ROB) <= stall_rob_threshold) {
      stall.note_stall();
    }
  }

  return available_dispatch_bandwidth.amount_consumed();
}

long O3_CPU::schedule_instruction()
{
  champsim::bandwidth search_bw{SCHEDULER_SIZE};
  int progress{0};
  for (auto rob_it = std::begin(ROB); rob_it != std::end(ROB) && search_bw.has_remaining(); ++rob_it) {
    // if there aren't enough physical registers available for the next instruction, stop scheduling
    unsigned long sources_to_allocate = std::count_if(rob_it->source_registers.begin(), rob_it->source_registers.end(),
                                                      [&alloc = std::as_const(reg_allocator)](auto srcreg) { return !alloc.isAllocated(srcreg); });
    if (reg_allocator.count_free_registers() < (sources_to_allocate + rob_it->destination_registers.size())) {
      break;
    }
    if (!rob_it->scheduled && rob_it->ready_time <= current_time) {
      do_scheduling(*rob_it);
      ++progress;
    }

    if (!rob_it->executed) {
      search_bw.consume();
    }
  }

  return progress;
}

void O3_CPU::do_scheduling(ooo_model_instr& instr)
{
  // Mark register dependencies
  for (auto& src_reg : instr.source_registers) {
    // rename source register
    src_reg = reg_allocator.rename_src_register(src_reg);
  }

  for (auto& dreg : instr.destination_registers) {
    // rename destination register
    dreg = reg_allocator.rename_dest_register(dreg, instr.instr_id);
  }

  instr.scheduled = true;
}

long O3_CPU::execute_instruction()
{
  champsim::bandwidth exec_bw{EXEC_WIDTH};
  for (auto rob_it = std::begin(ROB); rob_it != std::end(ROB) && exec_bw.has_remaining(); ++rob_it) {
    if (rob_it->scheduled && !rob_it->executed && rob_it->ready_time <= current_time) {
      bool ready = std::all_of(std::begin(rob_it->source_registers), std::end(rob_it->source_registers),
                               [&alloc = std::as_const(reg_allocator)](auto srcreg) { return alloc.isValid(srcreg); });
      if (ready) {
        do_execution(*rob_it);
        exec_bw.consume();
      }
    }
  }

  return exec_bw.amount_consumed();
}

void O3_CPU::do_execution(ooo_model_instr& instr)
{
  instr.executed = true;
  instr.ready_time = current_time + (warmup ? champsim::chrono::clock::duration{} : EXEC_LATENCY);

  // Mark LQ entries as ready to translate
  for (auto& lq_entry : LQ) {
    if (lq_entry.has_value() && lq_entry->instr_id == instr.instr_id) {
      lq_entry->ready_time = current_time + (warmup ? champsim::chrono::clock::duration{} : EXEC_LATENCY);
    }
  }

  // Mark SQ entries as ready to translate
  for (auto& sq_entry : SQ) {
    if (sq_entry.instr_id == instr.instr_id) {
      sq_entry.ready_time = current_time + (warmup ? champsim::chrono::clock::duration{} : EXEC_LATENCY);
    }
  }

  if constexpr (champsim::debug_print) {
    fmt::print("[ROB] {} instr_id: {} ready_time: {}\n", __func__, instr.instr_id, instr.ready_time.time_since_epoch() / clock_period);
  }
}

void O3_CPU::do_memory_scheduling(ooo_model_instr& instr)
{
  // load
  for (auto& smem : instr.source_memory) {
    auto q_entry = std::find_if_not(std::begin(LQ), std::end(LQ), [](const auto& lq_entry) { return lq_entry.has_value(); });
    assert(q_entry != std::end(LQ));
    q_entry->emplace(smem, instr.instr_id, instr.ip, instr.asid); // add it to the load queue

    // Check for forwarding
    auto sq_it = std::max_element(std::begin(SQ), std::end(SQ), [smem](const auto& lhs, const auto& rhs) {
      return lhs.virtual_address != smem || (rhs.virtual_address == smem && LSQ_ENTRY::program_order(lhs, rhs));
    });
    if (sq_it != std::end(SQ) && sq_it->virtual_address == smem) {
      if (sq_it->fetch_issued) { // Store already executed
        (*q_entry)->finish(instr);
        q_entry->reset();
      } else {
        assert(sq_it->instr_id < instr.instr_id);      // The found SQ entry is a prior store
        sq_it->lq_depend_on_me.emplace_back(*q_entry); // Forward the load when the store finishes
        (*q_entry)->producer_id = sq_it->instr_id;     // The load waits on the store to finish

        if constexpr (champsim::debug_print) {
          fmt::print("[DISPATCH] {} instr_id: {} waits on: {}\n", __func__, instr.instr_id, sq_it->instr_id);
        }
      }
    }
  }

  // store
  for (auto& dmem : instr.destination_memory) {
    SQ.emplace_back(dmem, instr.instr_id, instr.ip, instr.asid); // add it to the store queue
  }

  if constexpr (champsim::debug_print) {
    fmt::print("[DISPATCH] {} instr_id: {} loads: {} stores: {} cycle: {}\n", __func__, instr.instr_id, std::size(instr.source_memory),
               std::size(instr.destination_memory), current_time.time_since_epoch() / clock_period);
  }
}

long O3_CPU::operate_lsq()
{
  champsim::bandwidth store_bw{SQ_WIDTH};

  const auto complete_id = std::empty(ROB) ? std::numeric_limits<uint64_t>::max() : ROB.front().instr_id;
  auto do_complete = [time = current_time, finished = LSQ_ENTRY::precedes(complete_id), this](const auto& x) {
    return finished(x) && x.ready_time <= time && this->do_complete_store(x);
  };

  auto unfetched_begin = std::partition_point(std::begin(SQ), std::end(SQ), [](const auto& x) { return x.fetch_issued; });
  auto [fetch_begin, fetch_end] =
      champsim::get_span_p(unfetched_begin, std::end(SQ), store_bw, [time = current_time](const auto& x) { return !x.fetch_issued && x.ready_time <= time; });
  store_bw.consume(std::distance(fetch_begin, fetch_end));
  std::for_each(fetch_begin, fetch_end, [time = current_time, this](auto& sq_entry) {
    this->do_finish_store(sq_entry);
    sq_entry.fetch_issued = true;
    sq_entry.ready_time = time;
  });

  auto [complete_begin, complete_end] = champsim::get_span_p(std::cbegin(SQ), std::cend(SQ), store_bw, do_complete);
  store_bw.consume(std::distance(complete_begin, complete_end));
  SQ.erase(complete_begin, complete_end);

  champsim::bandwidth load_bw{LQ_WIDTH};

  for (auto& lq_entry : LQ) {
    if (load_bw.has_remaining() && lq_entry.has_value() && lq_entry->producer_id == std::numeric_limits<uint64_t>::max() && !lq_entry->fetch_issued
        && lq_entry->ready_time < current_time) {
      auto success = execute_load(*lq_entry);
      if (success) {
        load_bw.consume();
        lq_entry->fetch_issued = true;
      }
    }
  }

  return store_bw.amount_consumed() + load_bw.amount_consumed();
}

void O3_CPU::do_finish_store(const LSQ_ENTRY& sq_entry)
{
  if constexpr (champsim::debug_print) {
    fmt::print("[SQ] {} instr_id: {} vaddr: {}\n", __func__, sq_entry.instr_id, sq_entry.virtual_address);
  }

  sq_entry.finish(std::begin(ROB), std::end(ROB));

  // Release dependent loads
  for (std::optional<LSQ_ENTRY>& dependent : sq_entry.lq_depend_on_me) {
    assert(dependent.has_value()); // LQ entry is still allocated
    assert(dependent->producer_id == sq_entry.instr_id);

    dependent->finish(std::begin(ROB), std::end(ROB));
    dependent.reset();
  }
}

bool O3_CPU::do_complete_store(const LSQ_ENTRY& sq_entry)
{
  CacheBus::request_type data_packet;
  data_packet.v_address = sq_entry.virtual_address;
  data_packet.instr_id = sq_entry.instr_id;
  data_packet.ip = sq_entry.ip;

  if constexpr (champsim::debug_print) {
    fmt::print("[SQ] {} instr_id: {} vaddr: {}\n", __func__, data_packet.instr_id, data_packet.v_address);
  }

  return L1D_bus.issue_write(data_packet);
}

bool O3_CPU::execute_load(const LSQ_ENTRY& lq_entry)
{
  CacheBus::request_type data_packet;
  data_packet.v_address = lq_entry.virtual_address;
  data_packet.instr_id = lq_entry.instr_id;
  data_packet.ip = lq_entry.ip;

  if constexpr (champsim::debug_print) {
    fmt::print("[LQ] {} instr_id: {} vaddr: {}\n", __func__, data_packet.instr_id, data_packet.v_address);
  }

  return L1D_bus.issue_read(data_packet);
}

void O3_CPU::do_complete_execution(ooo_model_instr& instr)
{
  for (auto dreg : instr.destination_registers) {
    // mark physical register's data as valid
    reg_allocator.complete_dest_register(dreg);
  }

  instr.completed = true;

  if (instr.branch_mispredicted) {
    fetch_resume_time = current_time + BRANCH_MISPREDICT_PENALTY;
    do_flush_fill(instr); // FLUSH mode: warm the recovery path
    in_recovery = true;   // frontend now refilling the recovery path
    chain_probe_cursor = 0; // CHAIN deep probe: FTQ flush -- lead rebuilds
  }
}

long O3_CPU::complete_inflight_instruction()
{
  // update ROB entries with completed executions
  champsim::bandwidth complete_bw{EXEC_WIDTH};
  for (auto rob_it = std::begin(ROB); rob_it != std::end(ROB) && complete_bw.has_remaining(); ++rob_it) {
    if (rob_it->executed && !rob_it->completed && (rob_it->ready_time <= current_time) && rob_it->completed_mem_ops == rob_it->num_mem_ops()) {
      do_complete_execution(*rob_it);
      complete_bw.consume();
    }
  }

  return complete_bw.amount_consumed();
}

long O3_CPU::handle_memory_return()
{
  long progress{0};

  for (champsim::bandwidth fetch_bw{FETCH_WIDTH}, l1i_bw{L1I_BANDWIDTH};
       fetch_bw.has_remaining() && l1i_bw.has_remaining() && !L1I_bus.lower_level->returned.empty(); l1i_bw.consume()) {
    auto& l1i_entry = L1I_bus.lower_level->returned.front();

    // real-L1I walk mode: any arriving line (walk-issued or demand, incl. MSHR merges)
    // marks that block ready in all in-flight walks
    if ((fill_mode == fill_mode_type::ALT || fill_mode == fill_mode_type::HEAD || fill_mode == fill_mode_type::CHAIN) && alt_walk_l1i
        && !alt_walks.empty()) {
      const uint64_t blk = champsim::block_number{l1i_entry.v_address}.to<uint64_t>();
      for (auto& w : alt_walks) {
        w.lines_ready.insert(blk);
      }
    }

    while (fetch_bw.has_remaining() && !l1i_entry.instr_depend_on_me.empty()) {
      auto fetched = std::find_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), ooo_model_instr::matches_id(l1i_entry.instr_depend_on_me.front()));
      if (fetched != std::end(IFETCH_BUFFER) && champsim::block_number{fetched->ip} == champsim::block_number{l1i_entry.v_address} && fetched->fetch_issued) {
        fetched->fetch_completed = true;
        // L1I-miss admission gate: a fetch that took longer than an L1I hit round
        // trip missed L1I; mark the segmenter's in-flight stretch (loose
        // attribution, same style as the ROB-drain note_stall signal).
        if (trace_stall_enable && !warmup && current_time - fetched->fetch_issue_time > STALL_L1I_MISS_CYCLES * clock_period) {
          stall.note_l1i_miss();
        }
        fetch_bw.consume();
        ++progress;

        if constexpr (champsim::debug_print) {
          fmt::print("[IFETCH] {} instr_id: {} fetch completed\n", __func__, fetched->instr_id);
        }
      }

      l1i_entry.instr_depend_on_me.erase(std::begin(l1i_entry.instr_depend_on_me));
    }

    // remove this entry if we have serviced all of its instructions
    if (l1i_entry.instr_depend_on_me.empty()) {
      L1I_bus.lower_level->returned.pop_front();
      ++progress;
    }
  }

  auto l1d_it = std::begin(L1D_bus.lower_level->returned);
  for (champsim::bandwidth l1d_bw{L1D_BANDWIDTH}; l1d_bw.has_remaining() && l1d_it != std::end(L1D_bus.lower_level->returned); l1d_bw.consume(), ++l1d_it) {
    for (auto& lq_entry : LQ) {
      if (lq_entry.has_value() && lq_entry->fetch_issued && champsim::block_number{lq_entry->virtual_address} == champsim::block_number{l1d_it->v_address}) {
        lq_entry->finish(std::begin(ROB), std::end(ROB));
        lq_entry.reset();
        ++progress;
      }
    }
    ++progress;
  }
  L1D_bus.lower_level->returned.erase(std::begin(L1D_bus.lower_level->returned), l1d_it);

  return progress;
}

long O3_CPU::retire_rob()
{
  auto [retire_begin, retire_end] =
      champsim::get_span_p(std::cbegin(ROB), std::cend(ROB), champsim::bandwidth{RETIRE_WIDTH}, [](const auto& x) { return x.completed; });
  assert(std::distance(retire_begin, retire_end) >= 0); // end succeeds begin
  if constexpr (champsim::debug_print) {
    std::for_each(retire_begin, retire_end, [cycle = current_time.time_since_epoch() / clock_period](const auto& x) {
      fmt::print("[ROB] retire_rob instr_id: {} is retired cycle: {}\n", x.instr_id, cycle);
    });
  }

  // commit register writes to backend RAT
  // and recycle the old physical registers
  for (auto rob_it = retire_begin; rob_it != retire_end; ++rob_it) {
    for (auto dreg : rob_it->destination_registers) {
      reg_allocator.retire_dest_register(dreg);
    }
  }

  uint64_t cycles = current_time.time_since_epoch() / clock_period;
  handle_event<Event::RETIRE>(cpu, retire_begin, retire_end, cycles);

  auto retire_count = std::distance(retire_begin, retire_end);
  num_retired += retire_count;
  ROB.erase(retire_begin, retire_end);

  return retire_count;
}

void O3_CPU::impl_initialize_branch_predictor() const { branch_module_pimpl->impl_initialize_branch_predictor(); }

void O3_CPU::impl_last_branch_result(champsim::address ip, champsim::address target, bool taken, uint8_t branch_type) const
{
  branch_module_pimpl->impl_last_branch_result(ip, target, taken, branch_type);
}

bool O3_CPU::impl_predict_branch(champsim::address ip, champsim::address predicted_target, bool always_taken, uint8_t branch_type) const
{
  return branch_module_pimpl->impl_predict_branch(ip, predicted_target, always_taken, branch_type);
}

void O3_CPU::impl_initialize_btb() const { btb_module_pimpl->impl_initialize_btb(); }

void O3_CPU::impl_update_btb(champsim::address ip, champsim::address predicted_target, bool taken, uint8_t branch_type) const
{
  btb_module_pimpl->impl_update_btb(ip, predicted_target, taken, branch_type);
}

std::pair<champsim::address, bool> O3_CPU::impl_btb_prediction(champsim::address ip, uint8_t branch_type) const
{
  return btb_module_pimpl->impl_btb_prediction(ip, branch_type);
}

// LCOV_EXCL_START Exclude the following function from LCOV
void O3_CPU::print_deadlock()
{
  fmt::print("DEADLOCK! CPU {} cycle {}\n", cpu, current_time.time_since_epoch() / clock_period);

  auto instr_pack = [period = clock_period, this](const auto& entry) {
    return std::tuple{entry.instr_id,
                      entry.fetch_issued,
                      entry.fetch_completed,
                      entry.scheduled,
                      entry.executed,
                      entry.completed,
                      reg_allocator.count_reg_dependencies(entry),
                      entry.num_mem_ops() - entry.completed_mem_ops,
                      entry.ready_time.time_since_epoch() / period};
  };
  std::string_view instr_fmt{
      "instr_id: {} fetch_issued: {} fetch_completed: {} scheduled: {} executed: {} completed: {} num_reg_dependent: {} num_mem_ops: {} event: {}"};
  champsim::range_print_deadlock(IFETCH_BUFFER, "cpu" + std::to_string(cpu) + "_IFETCH", instr_fmt, instr_pack);
  champsim::range_print_deadlock(DECODE_BUFFER, "cpu" + std::to_string(cpu) + "_DECODE", instr_fmt, instr_pack);
  champsim::range_print_deadlock(DISPATCH_BUFFER, "cpu" + std::to_string(cpu) + "_DISPATCH", instr_fmt, instr_pack);
  champsim::range_print_deadlock(ROB, "cpu" + std::to_string(cpu) + "_ROB", instr_fmt, instr_pack);

  // print occupied physical registers
  reg_allocator.print_deadlock();

  // print LQ entry
  auto lq_pack = [period = clock_period](const auto& entry) {
    std::string depend_id{"-"};
    if (entry->producer_id != std::numeric_limits<uint64_t>::max()) {
      depend_id = std::to_string(entry->producer_id);
    }
    return std::tuple{entry->instr_id, entry->virtual_address, entry->fetch_issued, entry->ready_time.time_since_epoch() / period, depend_id};
  };
  std::string_view lq_fmt{"instr_id: {} address: {} fetch_issued: {} event_cycle: {} waits on {}"};

  auto sq_pack = [period = clock_period](const auto& entry) {
    std::vector<uint64_t> depend_ids;
    std::transform(std::begin(entry.lq_depend_on_me), std::end(entry.lq_depend_on_me), std::back_inserter(depend_ids),
                   [](const std::optional<LSQ_ENTRY>& lq_entry) { return lq_entry->producer_id; });
    return std::tuple{entry.instr_id, entry.virtual_address, entry.fetch_issued, entry.ready_time.time_since_epoch() / period, depend_ids};
  };
  std::string_view sq_fmt{"instr_id: {} address: {} fetch_issued: {} event_cycle: {} LQ waiting: {}"};
  champsim::range_print_deadlock(LQ, "cpu" + std::to_string(cpu) + "_LQ", lq_fmt, lq_pack);
  champsim::range_print_deadlock(SQ, "cpu" + std::to_string(cpu) + "_SQ", sq_fmt, sq_pack);
}
// LCOV_EXCL_STOP

LSQ_ENTRY::LSQ_ENTRY(champsim::address addr, champsim::program_ordered<LSQ_ENTRY>::id_type id, champsim::address local_ip, std::array<uint8_t, 2> local_asid)
    : champsim::program_ordered<LSQ_ENTRY>{id}, virtual_address(addr), ip(local_ip), asid(local_asid)
{
}

void LSQ_ENTRY::finish(std::deque<ooo_model_instr>::iterator begin, std::deque<ooo_model_instr>::iterator end) const
{
  auto rob_entry = std::partition_point(begin, end, ooo_model_instr::precedes(this->instr_id));
  assert(rob_entry != end);
  finish(*rob_entry);
}

void LSQ_ENTRY::finish(ooo_model_instr& rob_entry) const
{
  assert(rob_entry.instr_id == this->instr_id);

  ++rob_entry.completed_mem_ops;
  assert(rob_entry.completed_mem_ops <= rob_entry.num_mem_ops());

  if constexpr (champsim::debug_print) {
    fmt::print("[LSQ] {} instr_id: {} full_address: {} remain_mem_ops: {}\n", __func__, instr_id, virtual_address,
               rob_entry.num_mem_ops() - rob_entry.completed_mem_ops);
  }
}

bool CacheBus::issue_read(request_type data_packet)
{
  data_packet.address = data_packet.v_address;
  data_packet.is_translated = false;
  data_packet.cpu = cpu;
  data_packet.type = access_type::LOAD;

  return lower_level->add_rq(data_packet);
}

bool CacheBus::issue_write(request_type data_packet)
{
  data_packet.address = data_packet.v_address;
  data_packet.is_translated = false;
  data_packet.cpu = cpu;
  data_packet.type = access_type::WRITE;
  data_packet.response_requested = false;

  return lower_level->add_wq(data_packet);
}

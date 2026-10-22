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

#include <filesystem>
#include "prometheus_run.h"
#include "ooo_cpu.h"

#include <algorithm>
#include <limits>
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
#include "ucp_alt_tage.h"   // UCP port: dedicated 8KB Alt-BP (TAGE_PREDICTOR_8KB)
#include "ucp_alt_ittage.h" // UCP port: dedicated 4KB Alt-Ind (12.95KB flavor) -- must follow ucp_alt_tage.h
#include "ucp_hooks.h"    // UCP port: BTB probe / RAS snapshot / H2P flag from the modules
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
  advance_stall_attrib();
  progress += decode_instruction();   // decode
  progress += promote_to_decode();

  progress += fetch_instruction(); // fetch
  progress += check_dib();
  initialize_instruction();

  // cumulative store counters mirrored into stats each cycle so the ROI
  // subtraction (sim - warmup snapshot) yields per-phase deltas
  sim_stats.store_evictions = fill_store.evictions();
  sim_stats.store_conflict_evictions = fill_store.conflict_evictions();
  sim_stats.store_occupancy = fill_store.size();
  sim_stats.store_win_added = fill_store.windows_added();
  sim_stats.store_win_shared = fill_store.windows_shared();
  sim_stats.store_lookups = fill_store.lookups();
  sim_stats.store_alias_hits = fill_store.alias_hits();
  sim_stats.store_alias_overwrites = fill_store.alias_overwrites();
  if (rtt_mode != 0 && rtt_table == 1 && (++rtt_census_tick & ((uint64_t{1} << 20) - 1)) == 0) {
    rtt_store.window_census(dib_window_bits, sim_stats.rtt_win_total, sim_stats.rtt_win_distinct, sim_stats.rtt_win_shared); // snapshot (not a delta)
  }
  sim_stats.alt_pf_evicted_unused = DIB.pf_evicted_unused();
  sim_stats.retain_marks = DIB.retain_marks();
  sim_stats.uop_incl_invalidations = DIB.inclusive_invalidations();
  sim_stats.retain_diverted = DIB.retain_diverted();
  sim_stats.retain_costly_evicted = DIB.retain_costly_evicted();
  if (!l2_bound && (l2_perfect_instr != 0 || l2_emissary_ways > 0)) {
    if (CACHE* c = CACHE::find_live("cpu" + std::to_string(cpu) + "_L2C"); c != nullptr) {
      l2c = c;
      l2c->perfect_instr = l2_perfect_instr;
      l2c->emissary_ways = l2_emissary_ways;
      l2_bound = true;
      std::cout << "Prometheus L2: " << l2c->NAME << " perfect_instr=" << l2_perfect_instr << " emissary_ways=" << l2_emissary_ways << std::endl;
    }
  }
  if (l2c != nullptr) {
    sim_stats.l2_perfect_hits = l2c->perfect_instr_hits;
    sim_stats.l2_emissary_marks = l2c->emissary_marks;
    sim_stats.l2_emissary_diverted = l2c->emissary_diverted;
    sim_stats.l2_emissary_prot_evicted = l2c->emissary_prot_evicted;
  }

  return progress;
}

void O3_CPU::initialize()
{
  // BRANCH PREDICTOR & BTB
  impl_initialize_branch_predictor();
  impl_initialize_btb();
  if (alt_walk_nofill_l1) {
    CACHE::nofill_instr_only = true; // walk line reads skip the L1I only
  }

  // costly-window oracle (uop_cost_mode): the per-workload profile file is named after the trace,
  // known only now (the cores are built before the command line is parsed)
  if (uop_cost_mode == 1) {
    const std::string f = uop_cost_file();
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path{f}.parent_path(), ec);
    miss_profile = true;
    miss_dump_path = f;
    std::cout << "Prometheus costly-window profile pass: ROI windows ranked by empty-ROB cycles -> " << f << std::endl;
  } else if (uop_cost_mode == 2) {
    const std::string f = uop_cost_file();
    std::ifstream in(f);
    if (!in) {
      std::cerr << "costly-window oracle: no profile " << f << " (run the profile pass first)" << std::endl;
      std::exit(1);
    }
    std::unordered_set<uint64_t> windows;
    uint64_t w = 0;
    std::string rest;
    while (static_cast<long>(windows.size()) < uop_cost_windows && in >> w) {
      windows.insert(w);
      std::getline(in, rest);
    }
    std::cout << "Prometheus costly-window oracle: " << windows.size() << " costliest windows always hit, held outside the u-op cache (~"
              << (windows.size() * 68 + 1023) / 1024 << " KB at 8 u-ops x 8 B + 4 B tag) from " << f << std::endl;
    DIB.set_oracle_windows(std::move(windows), false);
  }
}

std::string O3_CPU::uop_cost_file()
{
  std::string dir;
  if (const char* e = std::getenv("PROMETHEUS_PROFILE_DIR"); e != nullptr && *e != '\0') {
    dir = e;
  } else {
    const char* home = std::getenv("HOME");
    dir = std::string{(home != nullptr) ? home : "."} + "/uop_cost_profiles";
  }
  return dir + "/" + std::filesystem::path{prometheus::trace_name}.filename().string() + ".win";
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
  miss_profile_finalize();
  if (trace_stall_enable) {
    stall.finalize_buckets(sim_stats);
    // where captured traces start, at window / line granularity: would indexing
    // the store by start window (one lookup per window a fetch block covers)
    // merge distinct traces?  (end-of-run, no timing effect)
    std::unordered_map<uint64_t, uint64_t> per_window;
    std::unordered_set<uint64_t> starts;
    std::unordered_set<uint64_t> lines;
    for (const auto& t : stall.get_traces()) {
      const uint64_t first = t.ips.empty() ? t.entry : t.ips.front();
      if (!starts.insert(first).second) {
        continue;
      }
      ++per_window[first >> 5];
      lines.insert(first >> 6);
    }
    sim_stats.starts_traces = starts.size();
    sim_stats.starts_windows = per_window.size();
    sim_stats.starts_lines = lines.size();
    for (const auto& [w, n] : per_window) {
      if (n >= 2) {
        ++sim_stats.starts_multi_windows;
      }
      sim_stats.starts_max_per_window = std::max(sim_stats.starts_max_per_window, n);
    }
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
      std::min(champsim::bandwidth::maximum_type{static_cast<long>(FETCH_WIDTH) * bpu_blocks},
               champsim::bandwidth::maximum_type{static_cast<long>(IFETCH_BUFFER_SIZE - std::size(IFETCH_BUFFER))})};

  bool stop_fetch = false;
  int block_branches = 0; // trace_wrong_path "ucp": branches taken into this cycle's fetch block
  int blocks_done = 0;    // bpu_blocks: fetch blocks predicted this cycle
  // fetch_buffer_blocks: fetch holds at most that many blocks in flight; when full it
  // takes nothing from the FTQ this cycle (the predictor keeps running ahead)
  if (fetch_buffer_blocks > 0
      && std::count_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), [](const auto& x) { return x.fetch_block_start; }) >= fetch_buffer_blocks) {
    stop_fetch = true;
  }
  bool block_open = false; // an instruction of the current block was taken this cycle
  while (current_time >= fetch_resume_time && instrs_to_read_this_cycle.has_remaining() && !stop_fetch && !std::empty(input_queue)) {
    instrs_to_read_this_cycle.consume();

    if (rp_mode != 0) { // resteer prefetcher: the instruction belongs to the open resteer path
      input_queue.front().rp_key = (rp_left > 0) ? rp_win_key : 0;
      if (rp_left > 0) {
        --rp_left;
      }
    }
    stop_fetch = do_init_instruction(input_queue.front());
    if (wp_exact || stall_mispredict_mode == 4) {
      // learn instruction boundaries from the correct path: an instruction that does
      // not transfer control is followed by its fall-through (exact x86 length)
      const uint64_t fetched_ip = input_queue.front().ip.to<uint64_t>();
      if (code_prev_seq) {
        code_next[code_prev_ip] = fetched_ip;
      }
      code_prev_ip = fetched_ip;
      code_prev_seq = !(input_queue.front().is_branch && input_queue.front().branch_taken);
    }
    if (fetch_block_rule && input_queue.front().is_branch && ++block_branches == 2) {
      stop_fetch = true; // UCP_ISCA24: at most two branches per fetch block (a taken one already ends it)
    }
    if (ucp_enable && ucp_btb_banks > 0) {
      ucp_note_demand_bank(input_queue.front().ip.to<uint64_t>()); // demand prediction holds its BTB bank this cycle
    }

    // CHAIN trace-fill, no deep probe (probe_ahead == 0), or an instruction the
    // deep cursor has not reached yet (post-mispredict rebuild): probe at enqueue.
    if (fill_mode == fill_mode_type::CHAIN && (!warmup || trace_warmup_train) && chain_probe_cursor == 0) {
      const uint64_t probed_ip = input_queue.front().ip.to<uint64_t>();
      const auto* ips = sucp_trace_walks() ? fill_store.lookup_entry(probed_ip) : nullptr; // fdip_predecode 1: the store only marks costly windows
      const bool store_hit = (ips != nullptr && !ips->empty());
      if (store_hit) {
        trigger_diag_probe(input_queue.front(), do_chain_trigger(*ips, input_queue.front().instr_id));
      }
      peek_diag_probe(input_queue.front().instr_id, store_hit ? probed_ip : 0);
      recovery_learn(input_queue.front().instr_id, store_hit ? ips : nullptr);
      if (store_hit) {
        chain_follow(probed_ip, input_queue.front().instr_id);
      }
    }

    // Add to IFETCH_BUFFER
    IFETCH_BUFFER.push_back(input_queue.front());
    input_queue.pop_front();
    IFETCH_BUFFER.back().fetch_block_start = !block_open; // first instruction of this cycle's fetch block
    block_open = true;
    if (chain_probe_cursor > 0) {
      --chain_probe_cursor; // the cursor indexes relative to the queue front
    }
    if (chain_peek_cursor > 0) {
      --chain_peek_cursor;
    }

    IFETCH_BUFFER.back().ready_time = current_time;
    if (ftq_lookup_window > 0) {
      // decoupled front end: FDIP requests the line now if the block's window is not in the u-op cache
      IFETCH_BUFFER.back().pre_miss = !DIB.probe(IFETCH_BUFFER.back().ip);
    }
    // bpu_blocks: a block ended (taken branch / block rule) -- the predictor goes on
    // with the next one this cycle; a misprediction (fetch_resume_time) still stops it
    if (stop_fetch && bpu_blocks > 1 && current_time >= fetch_resume_time && ++blocks_done < bpu_blocks) {
      stop_fetch = false;
      block_branches = 0;
      block_open = false; // the next instruction starts a new fetch block
    }
  }

  // CHAIN deep probe: advance the cursor through the instruction supply, probing
  // each pc once, up to chain_probe_ahead instructions beyond the transfer point.
  // This models the decoupled BP/FTQ running ahead of fetch -- the lead a real
  // FTQ-tail probe has over the fetch-point u-op-cache lookup.  The cursor is
  // reset on branch mispredictions (= FTQ flush), so recovery lead rebuilds at
  // probe bandwidth, honestly.
  // trace_probe_squash 1: while a mispredicted branch awaits resolution the FTQ
  // holds its wrong path, so the cursor (reset at the misprediction) stays frozen.
  // trace_wrong_path >= 1: the predictor runs here, at the cursor (the FTQ tail);
  // when it predicts a branch wrong, nothing past that branch is on the correct
  // path, so the cursor stops until the branch resolves.
  if (fill_mode == fill_mode_type::CHAIN && (!warmup || trace_warmup_train) && chain_probe_ahead > 0
      && !(chain_probe_squash == 1 && chain_shadow_branch != 0) && !chain_cursor_blocked) {
    const std::size_t limit = std::min(std::size(input_queue), static_cast<std::size_t>(chain_probe_ahead));
    int cursor_branches = 0; // trace_wrong_path "ucp": branches in the block the cursor predicts this cycle
    int cursor_blocks = 0;   // trace_probe_blocks: blocks covered this cycle
    for (champsim::bandwidth probe_bw{champsim::bandwidth::maximum_type{static_cast<long>(FETCH_WIDTH) * chain_probe_blocks}}; probe_bw.has_remaining() && chain_probe_cursor < limit;
         probe_bw.consume()) {
      auto& probed = input_queue[chain_probe_cursor];
      const uint64_t probed_ip = probed.ip.to<uint64_t>();
      if (ftq_fdip) {
        fdip_note(probed_ip, false);
      }
      if (rp_mode != 0) {
        rp_lookup(probed_ip);
      }
      if (ftq_fdip && fdip_predecode != 0 && (probed_ip >> dib_window_bits) != pd_last_window) {
        // FDIP-triggered pre-decode: the predictor entered a window; only this block's
        // instructions in the window are known to it
        pd_last_window = probed_ip >> dib_window_bits;
        std::vector<uint64_t> win_ips;
        for (std::size_t k = chain_probe_cursor; k < limit; ++k) {
          const auto& x = input_queue[k];
          if ((x.ip.to<uint64_t>() >> dib_window_bits) != pd_last_window) {
            break;
          }
          win_ips.push_back(x.ip.to<uint64_t>());
          if (x.is_branch && x.branch_taken) {
            break; // the block ends here
          }
        }
        fdip_predecode_window(win_ips, false);
      }
      const auto* ips = sucp_trace_walks() ? fill_store.lookup_entry(probed_ip) : nullptr; // fdip_predecode 1: the store only marks costly windows
      const bool store_hit = (ips != nullptr && !ips->empty());
      if (store_hit) {
        trigger_diag_probe(probed, do_chain_trigger(*ips, probed.instr_id));
      }
      if (ftq_uop_prefetch && !store_hit && (probed_ip >> dib_window_bits) != fup_last_window) {
        // u-op FDIP: the predictor entered a window; walk it if the u-op cache lacks it
        // (do_chain_trigger skips windows resident, staged or already being walked).
        // Only this block's instructions in the window are known to the predictor.
        fup_last_window = probed_ip >> dib_window_bits;
        std::vector<uint64_t> win_ips;
        for (std::size_t k = chain_probe_cursor; k < limit; ++k) {
          const auto& x = input_queue[k];
          if ((x.ip.to<uint64_t>() >> dib_window_bits) != fup_last_window) {
            break;
          }
          win_ips.push_back(x.ip.to<uint64_t>());
          if (x.is_branch && x.branch_taken) {
            break; // the block ends here
          }
        }
        if (!win_ips.empty() && do_chain_trigger(win_ips, probed.instr_id) == TRIG_LAUNCHED) {
          ++sim_stats.fup_walks;
        }
      }
      peek_diag_probe(probed.instr_id, store_hit ? probed_ip : 0);
      recovery_learn(probed.instr_id, store_hit ? ips : nullptr);
      if (store_hit) {
        chain_follow(probed_ip, probed.instr_id);
      }
      ++chain_probe_cursor;
      // warmup never stalls on a misprediction (and never resolves one), so the cursor only stops in the ROI
      if (wrong_path_level >= 1 && !probed.early_predicted && predict_early(probed) && !warmup) {
        chain_cursor_blocked = true;
        if (rtt_diag) {
          rtt_diag_check(probed);
        }
        if (oracle_recovery > 0 && fill_mode == fill_mode_type::CHAIN && ((oracle_recovery_classes >> mispredict_class(probed)) & 1u) != 0) {
          // ORACLE (diagnostic ceiling): the predictor knows this prediction is wrong and where
          // execution really goes -- walk the real recovery path now (lines + pre-decode + stage)
          std::vector<uint64_t> real_path;
          for (std::size_t k = chain_probe_cursor; k < std::size(input_queue) && k - chain_probe_cursor < oracle_recovery; ++k) {
            const uint64_t ip = input_queue[k].ip.to<uint64_t>();
            if (!oracle_recovery_seen || seen_win.count(ip >> dib_window_bits) != 0) {
              real_path.push_back(ip); // _SEEN: only code fetched before (what any history-based scheme could hold)
            }
          }
          if (!real_path.empty()) {
            ++sim_stats.oracle_recovery_triggers;
            chain_peek_probing = true; // flagged past-mispredict
            if (do_chain_trigger(real_path, probed.instr_id) == TRIG_LAUNCHED) {
              ++sim_stats.oracle_recovery_walks;
            }
            chain_peek_probing = false;
          }
        }
        chain_peek_cursor = chain_probe_cursor; // trace_oracle_peek: the real path starts right past the branch
        chain_peek_left = oracle_peek;
        ++sim_stats.wp_cursor_stops;
        if (wrong_path_level >= 2) {
          wp_begin(probed); // the predictor continues down the wrong path from here
        }
        break;
      }
      // UCP_ISCA24 prediction rule: one fetch block per cycle -- it ends at the
      // first taken branch, and holds at most two branches (trace_probe_blocks:
      // the cursor covers that many blocks per cycle)
      if (fetch_block_rule && probed.is_branch && (probed.branch_taken || ++cursor_branches == 2)) {
        if (++cursor_blocks >= chain_probe_blocks) {
          break;
        }
        cursor_branches = 0;
      }
    }
  }

  // trace_oracle_peek (DIAGNOSTIC CEILING, not a mechanism): while the cursor is
  // stopped at a mispredicted branch, keep probing the REAL path past it -- the
  // trace holds it, hardware does not -- for up to N instructions, at the cursor's
  // bandwidth and block rule.  Its walks are what a predictor naming the right
  // trace(s) when the predictor reaches the branch could launch; they are flagged
  // past-mispredict, so trace-chain-shadow reports their use.  No branch is
  // predicted here: the cursor still predicts and stops at every later branch.
  if (fill_mode == fill_mode_type::CHAIN && (!warmup || trace_warmup_train) && oracle_peek > 0 && chain_cursor_blocked && chain_peek_left > 0) {
    const std::size_t limit = std::min(std::size(input_queue), static_cast<std::size_t>(chain_probe_ahead));
    int peek_branches = 0;
    chain_peek_probing = true;
    for (champsim::bandwidth peek_bw{FETCH_WIDTH}; peek_bw.has_remaining() && chain_peek_cursor < limit && chain_peek_left > 0; peek_bw.consume()) {
      const auto& peeked = input_queue[chain_peek_cursor];
      ++sim_stats.peek_probes;
      if (const auto* ips = fill_store.lookup_entry(peeked.ip.to<uint64_t>()); ips != nullptr && !ips->empty()) {
        ++sim_stats.peek_store_hits;
        do_chain_trigger(*ips, peeked.instr_id);
      }
      ++chain_peek_cursor;
      --chain_peek_left;
      if (fetch_block_rule && peeked.is_branch && (peeked.branch_taken || ++peek_branches == 2)) {
        break;
      }
    }
    chain_peek_probing = false;
  }

  // trace_wrong_path >= 2: the predictor extends the wrong path while the
  // mispredicted branch is unresolved
  if (wp_active) {
    wp_generate();
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

// handle branch prediction for all instructions as at this point we do not know
// if the instruction is a branch.  Lookup only: training follows in the caller.
void O3_CPU::predict_instruction(ooo_model_instr& arch_instr)
{
  auto [predicted_branch_target, always_taken] = impl_btb_prediction(arch_instr.ip, arch_instr.branch);
  arch_instr.branch_prediction = impl_predict_branch(arch_instr.ip, predicted_branch_target, always_taken, arch_instr.branch) || always_taken;
  // UCP: capture the H2P classification and the raw BTB target now -- the flag is
  // per-prediction (next predict overwrites it)
  arch_instr.pred_h2p = ucp::bp_last_h2p;
  arch_instr.pred_always_taken = always_taken;
  arch_instr.pred_btb_target = predicted_branch_target.to<uint64_t>();
}

bool O3_CPU::branch_mispredicts(const ooo_model_instr& instr) const
{
  const champsim::address predicted_branch_target{instr.branch_prediction ? instr.pred_btb_target : 0};
  return instr.is_branch
         && (predicted_branch_target != instr.branch_target
             || (((instr.branch == BRANCH_CONDITIONAL) || (instr.branch == BRANCH_OTHER)) && instr.branch_taken != instr.branch_prediction));
}

// trace_wrong_path >= 1: predict (and train) an instruction when the deep-probe
// cursor reaches it -- the predictor sits at the FTQ tail.  The predictor is
// queried once per instruction and trained right after, in program order, exactly
// as at fetch, so the prediction sequence is unchanged; fetch reuses the stored
// result.  Returns whether the branch is predicted wrong.
bool O3_CPU::predict_early(ooo_model_instr& instr)
{
  predict_instruction(instr);
  instr.vp_ctx = vpred.last_seq(); // trace_vpred: the prediction this instruction follows
  instr.early_predicted = true;
  if (!instr.is_branch) {
    return false;
  }
  // hybrid UCP + SUCP: UCP sits at the predictor and must see pre-training state
  if (ucp_enable && (!warmup || trace_warmup_train)) {
    do_ucp_branch(instr, instr.pred_btb_target, instr.pred_always_taken, instr.pred_h2p);
  }
  impl_update_btb(instr.ip, instr.branch_target, instr.branch_taken, instr.branch);
  impl_last_branch_result(instr.ip, instr.branch_target, instr.branch_taken, instr.branch);
  const bool wrong = branch_mispredicts(instr);
  if (rtt_mode != 0 && fill_mode == fill_mode_type::CHAIN && (!warmup || trace_warmup_train)) {
    rtt_last_lookup_key = 0;
    instr.rtt_walked = rtt_on_predict(instr, wrong);
    instr.rtt_key = rtt_last_lookup_key;
  }
  peek_diag_branch(instr, wrong);
  recovery_on_predict(instr, wrong);
  vpred_on_predict(instr, wrong);
  return wrong;
}

// recovery-trace table: the predictor reaches a branch.  Mode 1 (oracle trigger) walks the
// recovery trace only when this prediction is wrong, keyed by where the branch really goes;
// modes 2/3 walk, at every prediction (3: of low-confidence conditionals and of all other
// branches), the trace recorded for the OTHER outcome -- the path fetch restarts on if
// this prediction turns out wrong.  The walk is the SUCP walk (real L1I reads, shared
// pre-decode, staging), launched from the predictor, so it runs alongside FDIP.
bool O3_CPU::rtt_on_predict(const ooo_model_instr& instr, bool wrong)
{
  if (rtt_mode == 4) {
    return false; // triggered at resolution (rtt_on_resolve)
  }
  if (rtt_mode == 1 && !wrong) {
    return false;
  }
  const uint64_t pc = instr.ip.to<uint64_t>();
  const bool cond = (instr.branch == BRANCH_CONDITIONAL) || (instr.branch == BRANCH_OTHER);
  if (rtt_mode == 3 && cond && !instr.pred_h2p) {
    return false;
  }
  bool boost_only = false; // rtt_boost: a confident conditional, walked only if its entry keeps being right to walk
  const bool ret_gated = (rtt_mode == 5 && rtt_ret > 0 && instr.branch == BRANCH_RETURN); // rtt_ret: likewise for returns
  if (rtt_mode == 5 && (cond ? !instr.pred_h2p : !rtt_low_conf_type(instr.branch))) {
    if (rtt_boost > 0 && cond) {
      boost_only = true;
    } else if (!ret_gated) {
      return false; // a confident prediction (TAGE high confidence, RAS, BTB direct target): no lookup
    }
  }
  uint64_t key = pc; // other branches: a wrong target, keyed by the branch alone
  if (cond) {
    uint64_t next = 0;
    const bool fall_through = (rtt_mode == 1) ? !instr.branch_taken : instr.branch_prediction; // where the trace starts
    if (fall_through) {
      const auto it = code_next.find(pc);
      next = (it != code_next.end()) ? it->second : 0; // the learned fall-through (0 = never seen)
      if (next == 0 && rtt_open_rule >= 2) {
        next = pc + WP_STEP; // alternate-path traces are keyed by +4 when the fall-through was never seen
      }
    } else {
      next = (rtt_mode == 1) ? instr.branch_target.to<uint64_t>() : instr.pred_btb_target;
    }
    if (next == 0) {
      if (rtt_cold_seq > 0 && fall_through && !boost_only && rtt_mode == 5) {
        rtt_cold_prefetch(pc, false); // never fell through: certainly no trace for it
      }
      return false;
    }
    key = rtt_key(pc, next);
  }
  ++sim_stats.rtt_lookups;
  const auto* ips = (rtt_gen_cond && cond) ? nullptr : rtt_lookup(key);
  if (ips == nullptr || ips->empty()) {
    if (rtt_cold_seq > 0 && cond && instr.branch_prediction && !boost_only && rtt_mode == 5) {
      rtt_cold_prefetch(pc, false); // no recovery trace for the fall-through
    } else if (rtt_cold_seq > 0 && rtt_cold_target && cond && !instr.branch_prediction && !boost_only && rtt_mode == 5 && instr.pred_btb_target != 0) {
      rtt_cold_prefetch(pc, false, instr.pred_btb_target); // predicted not taken, no recovery trace for the taken path
    }
    if ((rtt_fallback || rtt_gen_cond) && cond && !boost_only) {
      // no recorded recovery path: walk the alternate path generated from the BTB and the predictor
      uint64_t alt = 0;
      if (instr.branch_prediction) {
        const auto it = code_next.find(pc);
        alt = (it != code_next.end()) ? it->second : pc + WP_STEP;
      } else {
        alt = instr.pred_btb_target;
      }
      if (alt != 0) {
        auto gen = rtt_alt_path(alt, rtt_len);
        if (!gen.empty()) {
          rtt_probing = true;
          const bool launched = (do_chain_trigger(gen, instr.instr_id) == TRIG_LAUNCHED);
          rtt_probing = false;
          if (launched) {
            ++sim_stats.rtt_fb_walks;
            return true;
          }
        }
      }
    }
    return false;
  }
  ++sim_stats.rtt_hits;
  if ((rtt_conf_min > 0 || rtt_conf_evict || rtt_boost > 0 || rtt_ret > 0) && rtt_table == 1) {
    rtt_last_lookup_key = key; // trained when this branch resolves
    const int conf = rtt_conf_get(key);
    if (boost_only) {
      ++sim_stats.rtt_boost_hits;
      if (conf < rtt_boost) {
        return false; // confident, and this branch's recent predictions were right: no walk
      }
    }
    if (ret_gated) {
      ++sim_stats.rtt_ret_hits;
      if (conf < rtt_ret) {
        return false; // the RAS has been right for this return lately: no walk
      }
    }
    if (rtt_conf_min > 0 && conf >= 0 && conf < rtt_conf_min) {
      if (rtt_cost_walk > 0 && rtt_store.cost_of(key | (uint64_t{1} << 63)) >= rtt_cost_walk) {
        ++sim_stats.rtt_cost_rescued; // mostly right lately, but its recovery is costly: walk anyway
      } else {
        ++sim_stats.rtt_conf_suppressed; // this branch's recent low-confidence predictions were mostly right
        return false;
      }
    }
  }
  rtt_probing = true;
  const bool launched = (do_chain_trigger(*ips, instr.instr_id) == TRIG_LAUNCHED);
  rtt_probing = false;
  if (launched) {
    ++sim_stats.rtt_walks;
    if (boost_only) {
      ++sim_stats.rtt_boost_walks;
    }
    if (ret_gated) {
      ++sim_stats.rtt_ret_walks;
    }
  }
  return launched;
}

// modes 2/3 with rtt_wrong_path: the predictor running down a wrong path (before the misprediction
// resolves) looks the table up for the branches it predicts there too, exactly as on the correct
// path; the walks it launches are wrong-path walks (flushed at resolution).
void O3_CPU::rtt_on_wp_predict(uint64_t pc, bool cond, bool pred_taken, uint64_t btb_target, uint64_t fall_through, bool h2p, uint8_t type)
{
  const bool wp_boost_only = (rtt_mode == 5 && rtt_boost > 0 && cond && !h2p);
  if ((rtt_mode == 3 || rtt_mode == 5) && cond && !h2p && !wp_boost_only) {
    return;
  }
  const bool wp_ret_gated = (rtt_mode == 5 && rtt_ret > 0 && type == BRANCH_RETURN);
  if (rtt_mode == 5 && !cond && !rtt_low_conf_type(type) && !wp_ret_gated) {
    return;
  }
  uint64_t key = pc;
  if (cond) {
    const uint64_t alt = pred_taken ? fall_through : btb_target; // the other direction
    if (alt == 0) {
      if (rtt_cold_seq > 0 && pred_taken && !wp_boost_only && rtt_mode == 5) {
        rtt_cold_prefetch(pc, true);
      }
      return;
    }
    key = rtt_key(pc, alt);
  }
  ++sim_stats.rtt_wp_lookups;
  const auto* ips = (rtt_gen_cond && cond) ? nullptr : rtt_lookup(key);
  if (ips == nullptr || ips->empty()) {
    if (rtt_cold_seq > 0 && cond && pred_taken && !wp_boost_only && rtt_mode == 5) {
      rtt_cold_prefetch(pc, true);
    } else if (rtt_cold_seq > 0 && rtt_cold_target && cond && !pred_taken && !wp_boost_only && rtt_mode == 5 && btb_target != 0) {
      rtt_cold_prefetch(pc, true, btb_target);
    }
    if ((rtt_fallback || rtt_gen_cond) && cond && !wp_boost_only) {
      const uint64_t alt = pred_taken ? (fall_through != 0 ? fall_through : pc + WP_STEP) : btb_target;
      if (alt != 0) {
        auto gen = rtt_alt_path(alt, rtt_len);
        if (!gen.empty() && do_chain_trigger(gen, 0, /*wrong_path=*/true) == TRIG_LAUNCHED) {
          ++sim_stats.rtt_fb_wp_walks;
        }
      }
    }
    return;
  }
  ++sim_stats.rtt_wp_hits;
  if (wp_boost_only && rtt_table == 1 && rtt_conf_get(key) < rtt_boost) {
    return;
  }
  if (wp_ret_gated && rtt_table == 1 && rtt_conf_get(key) < rtt_ret) {
    return;
  }
  if (rtt_conf_min > 0 && rtt_table == 1) {
    if (const int conf = rtt_conf_get(key); conf >= 0 && conf < rtt_conf_min) {
      if (rtt_cost_walk > 0 && rtt_store.cost_of(key | (uint64_t{1} << 63)) >= rtt_cost_walk) {
        ++sim_stats.rtt_cost_rescued;
      } else {
        ++sim_stats.rtt_conf_suppressed;
        return;
      }
    }
  }
  if (do_chain_trigger(*ips, 0, /*wrong_path=*/true) == TRIG_LAUNCHED) {
    ++sim_stats.rtt_wp_walks;
  }
}

// rtt_cold_seq: the predicted-taken H2P conditional at pc has no recovery trace for its fall-through, so the
// fall-through is probably code that never ran (nothing to walk): bring its first lines into the L1I now, so a
// misprediction does not wait on DRAM for them
void O3_CPU::rtt_cold_prefetch(uint64_t pc, bool wrong_path, uint64_t target)
{
  const auto it = code_next.find(pc);
  if (target == 0 && rtt_cold_mode >= 1 && it != code_next.end()) {
    return; // the branch has fallen through before (BTB "ever not-taken" bit)
  }
  const uint64_t start = (target != 0) ? target : (it != code_next.end()) ? it->second : pc + WP_STEP; // target: the taken side (rtt_cold_target)
  if (rtt_cold_mode >= 2 && seen_win.count(start >> dib_window_bits) != 0) {
    return; // oracle: the fall-through window was looked up before
  }
  ++sim_stats.rtt_cold_triggers;
  if (wrong_path) {
    ++sim_stats.rtt_cold_wp_triggers;
  }
  const uint64_t first = champsim::block_number{champsim::address{start}}.to<uint64_t>();
  for (int k = 0; k < rtt_cold_seq; ++k) {
    const champsim::address line{champsim::block_number{first + static_cast<uint64_t>(k)}};
    if (l1i->has_virtual_block(line)) {
      ++sim_stats.rtt_cold_resident;
      continue;
    }
    if (l1i->prefetch_line(line, !rtt_cold_l2, 0)) { // rtt_cold_l2: translated and tag-checked at the L1I, filled in the L2 only
      ++sim_stats.rtt_cold_lines;
    }
  }
}

// rtt_cancel: the branch whose prediction launched a recovery walk resolved correctly -- stop the walk
// (its remaining lines are never issued) and drop the windows it staged that no demand has used
void O3_CPU::rtt_cancel_walk(uint64_t branch_id)
{
  bool found = false;
  alt_walks.erase(std::remove_if(std::begin(alt_walks), std::end(alt_walks),
                                 [this, branch_id, &found](const auto& w) {
                                   if (!w.rtt || w.probe_id != branch_id) {
                                     return false;
                                   }
                                   found = true;
                                   sim_stats.rtt_cancel_lines += w.lines_pending.size();
                                   return true;
                                 }),
                  std::end(alt_walks));
  const auto before = uop_buffer.size();
  uop_buffer.erase(std::remove_if(std::begin(uop_buffer), std::end(uop_buffer),
                                  [branch_id](const auto& e) { return !e.served && e.probe_id == branch_id && !e.wrong_path; }),
                   std::end(uop_buffer));
  sim_stats.rtt_cancel_windows += before - uop_buffer.size();
  if (found || before != uop_buffer.size()) {
    ++sim_stats.rtt_cancelled;
  }
}

// the stored trace for a key: the unbounded table, or the bounded store (rtt_store, fill_store's
// geometry, separate so SUCP's address probes can never launch these)
const std::vector<uint64_t>* O3_CPU::rtt_lookup(uint64_t key)
{
  if (rtt_table == 1) {
    return rtt_store.lookup_entry(key | (uint64_t{1} << 63));
  }
  const auto it = rtt.find(key);
  return (it == rtt.end()) ? nullptr : &it->second;
}

// rtt_mode 4 (realistic): the branch just resolved as mispredicted (at decode or execute) -- now the
// hardware knows the misprediction and where execution really goes.  Look the trace up and walk it
// (real L1I reads, shared pre-decode, staging) while fetch restarts down the same path.
void O3_CPU::rtt_on_resolve(const ooo_model_instr& branch)
{
  if (rtt_mode != 4 || fill_mode != fill_mode_type::CHAIN) {
    return;
  }
  const uint64_t pc = branch.ip.to<uint64_t>();
  uint64_t next = 0;
  if (!input_queue.empty() && input_queue.front().instr_id == branch.instr_id + 1) {
    next = input_queue.front().ip.to<uint64_t>(); // fetch stopped right past the branch: the redirect target
  } else if (branch.branch_taken) {
    next = branch.branch_target.to<uint64_t>();
  } else if (const auto it = code_next.find(pc); it != code_next.end()) {
    next = it->second;
  }
  if (next == 0) {
    return;
  }
  ++sim_stats.rtt_lookups;
  const auto* ips = rtt_lookup(rtt_key(pc, next));
  if (ips == nullptr || ips->empty()) {
    return;
  }
  ++sim_stats.rtt_hits;
  if (do_chain_trigger(*ips, branch.instr_id) == TRIG_LAUNCHED) {
    ++sim_stats.rtt_walks;
  }
}

// recovery-trace table capture, in program order at the u-op-cache check: a misprediction
// opens a recording of the path that follows it (the recovery path).  Capture 0 skips the
// leading u-op-cache hits, records from the first miss and closes at the next hit (only
// hits on the u-op cache itself count: a window our own walk staged would truncate every
// re-recording); capture 1 records a fixed rtt_len instructions right past the branch.
void O3_CPU::rtt_capture_step(const ooo_model_instr& instr, bool uop_hit)
{
  const uint64_t ip = instr.ip.to<uint64_t>();
  for (auto& c : rtt_open) {
    if (rtt_seg_reconv && c.branch_id == last_wp_branch && c.started && !c.ips.empty() && last_wp_windows.count(ip >> dib_window_bits) != 0
        && (c.ips.back() >> dib_window_bits) != (ip >> dib_window_bits)) {
      c.done = true; // the recovery path rejoined the code the wrong path fetched: the rest is already there
      ++sim_stats.rtt_reconv_cut;
      continue;
    }
    if (!c.keyed) {
      c.key = (c.cond || rtt_mode == 4) ? rtt_key(c.pc, ip) : c.pc; // the first instruction past the branch: where it really went
      c.keyed = true;
    }
    ++c.seen;
    if (!c.started) {
      if (rtt_capture == 1 || !uop_hit) {
        c.started = true;
      } else if (c.seen >= rtt_len) {
        c.done = true; // recovered without a u-op-cache miss: nothing to record
        continue;
      }
    }
    if (c.started) {
      if (rtt_capture == 0 && uop_hit) {
        c.done = true;
        continue;
      }
      c.ips.push_back(ip);
      c.flags.push_back(instr.is_branch ? (instr.branch_taken ? 2 : 1) : 0);
      if (!uop_hit) {
        c.missed = true;
        if (rtt_admit_miss >= 2 && !c.far && !l1i->has_virtual_block(instr.ip)) {
          c.far = true; // ...and the bytes were not in the L1I: this recovery paid fetch latency
        }
      }
      if (c.ips.size() >= rtt_len) {
        c.done = true;
      }
    }
  }
  auto finish = [this, &instr](rtt_cap& c) {
    c.last_id = instr.instr_id;
    if (rtt_store_filter == 0 || c.costly) {
      rtt_commit(c);
    } else if (!c.ips.empty()) {
      rtt_wait.push_back(std::move(c)); // its instructions may still starve the backend before they dispatch
    }
  };
  for (auto& c : rtt_open) {
    if (c.done) {
      finish(c);
    }
  }
  rtt_open.erase(std::remove_if(std::begin(rtt_open), std::end(rtt_open), [](const auto& c) { return c.done; }), std::end(rtt_open));
  const bool cond = (instr.branch == BRANCH_CONDITIONAL) || (instr.branch == BRANCH_OTHER);
  const bool opens = instr.is_branch && (rtt_mode != 5 || cond || rtt_low_conf_type(instr.branch) || (rtt_ret > 0 && instr.branch == BRANCH_RETURN))
                     && !(rtt_gen_cond && cond)
                     && (rtt_open_rule == 0   ? instr.pred_wrong
                         : rtt_open_rule == 3 ? (instr.pred_wrong || (cond && instr.pred_h2p))
                                              : (cond ? instr.pred_h2p : instr.pred_wrong));
  if (opens && rtt_open_rule >= 2 && cond && !instr.pred_wrong) {
    // correctly predicted low-confidence conditional: the trace is the path it did NOT take --
    // taken: the fall-through; not taken: the BTB target -- generated now, rtt_len instructions deep.
    // Stored under the filter like a recording of the instructions that follow the branch.
    ++sim_stats.rtt_opens;
    const uint64_t start = instr.branch_taken ? rtt_fall_through(ip) : instr.pred_btb_target;
    if (start == 0) {
      ++sim_stats.rtt_alt_nobtb;
      return;
    }
    ++sim_stats.rtt_alt;
    rtt_cap c;
    c.stall_at_open = rob_empty_total;
    c.pc = ip;
    c.branch_id = instr.instr_id;
    c.last_id = instr.instr_id + rtt_len;
    c.cond = true;
    c.keyed = true;
    c.key = rtt_key(ip, start);
    c.started = true;
    c.done = true;
    c.ips = rtt_alt_path(start, rtt_len);
    if (rtt_store_filter == 0) {
      rtt_commit(c);
    } else {
      rtt_wait.push_back(std::move(c)); // stored if the ROB empties on the instructions that follow the branch
    }
    return;
  }
  if (opens) {
    if (rtt_open.size() >= rtt_max_open) {
      finish(rtt_open.front());
      rtt_open.erase(std::begin(rtt_open));
    }
    ++sim_stats.rtt_opens;
    if (instr.pred_wrong) {
      ++sim_stats.rtt_opens_wrong;
    }
    rtt_cap c;
    c.stall_at_open = rob_empty_total;
    c.pc = ip;
    c.branch_id = instr.instr_id;
    c.cond = cond;
    rtt_open.push_back(std::move(c));
  }
}

// the branch's fall-through: its learned successor (exact x86 length), +4 if it never fell through
uint64_t O3_CPU::rtt_fall_through(uint64_t pc) const
{
  const auto it = code_next.find(pc);
  return (it != code_next.end()) ? it->second : pc + WP_STEP;
}

// rtt_open 2: the alternate path from `start`, n instructions, the way the wrong path is generated:
// the BTB identifies branches at their addresses, the predictor gives conditional directions
// (lookup only), returns use a copy of the RAS, anything else continues to its learned successor --
// or +4 through code never seen executing (UCP's step), so the path can reach first-touch code.
std::vector<uint64_t> O3_CPU::rtt_alt_path(uint64_t start, std::size_t n)
{
  std::vector<uint64_t> ips;
  std::vector<uint64_t> ras = ucp::btb_ras_snapshot ? ucp::btb_ras_snapshot() : std::vector<uint64_t>{};
  uint64_t addr = start;
  while (ips.size() < n && addr != 0) {
    ips.push_back(addr);
    uint64_t next = rtt_fall_through(addr);
    if (ucp::btb_probe) {
      if (const auto pb = ucp::btb_probe(addr); pb.has_value()) {
        bool taken = true;
        uint64_t tgt = pb->target;
        if (pb->conditional) {
          taken = impl_predict_branch(champsim::address{addr}, champsim::address{tgt}, false, BRANCH_CONDITIONAL);
        }
        if (pb->branch_type == BRANCH_RETURN) {
          tgt = 0;
          if (!ras.empty()) {
            tgt = ras.back();
            ras.pop_back();
          }
        } else if (pb->branch_type == BRANCH_DIRECT_CALL || pb->branch_type == BRANCH_INDIRECT_CALL) {
          ras.push_back(next);
        }
        if (taken) {
          next = tgt; // 0 = unknown target: the path ends
        }
      }
    }
    addr = next;
  }
  return ips;
}

// rtt_store_filter: a dispatch cycle with an empty ROB.  The instruction dispatch waits for is the
// oldest one still in the front end; a recording that holds it (open, or ended but not yet fully
// dispatched) starved the backend and is stored.  Ended recordings whose instructions have all
// dispatched without that are dropped.
void O3_CPU::rtt_rob_empty()
{
  const ooo_model_instr* oldest = nullptr;
  for (const auto* q : {&DISPATCH_BUFFER, &DECODE_BUFFER, &IFETCH_BUFFER, &input_queue}) {
    if (!q->empty()) {
      oldest = &q->front();
      break;
    }
  }
  const uint64_t wait_id = (oldest != nullptr) ? oldest->instr_id : std::numeric_limits<uint64_t>::max();
  if (std::empty(ROB) && oldest != nullptr && (rtt_store_filter != 2 || (oldest->dib_checked && oldest->uop_miss))) {
    for (auto& c : rtt_open) {
      if (wait_id > c.branch_id) {
        c.costly = true;
      }
    }
    for (auto& c : rtt_wait) {
      if (wait_id > c.branch_id && wait_id <= c.last_id) {
        c.costly = true;
        rtt_commit(c);
      }
    }
  }
  rtt_wait.erase(std::remove_if(std::begin(rtt_wait), std::end(rtt_wait),
                                [this, wait_id](const auto& c) {
                                  if (c.costly) {
                                    return true; // stored above
                                  }
                                  if (wait_id > c.last_id) {
                                    ++sim_stats.rtt_filtered; // all its instructions dispatched: the ROB never emptied on it
                                    return true;
                                  }
                                  return false;
                                }),
                 std::end(rtt_wait));
}

// diagnostic: the cursor just predicted `branch` wrong, and input_queue[chain_probe_cursor..] is the
// real recovery path.  Look up the trace stored for (branch, where it really goes) and compare:
// prefix match (instructions), recall = real-path windows absent from the u-op cache and the
// staging buffer that the stored trace holds, precision = absent stored windows on the real path.
// Both over the first rtt_len real-path instructions.
void O3_CPU::rtt_diag_check(const ooo_model_instr& branch)
{
  if (chain_probe_cursor >= std::size(input_queue)) {
    return;
  }
  const uint64_t pc = branch.ip.to<uint64_t>();
  const bool cond = (branch.branch == BRANCH_CONDITIONAL) || (branch.branch == BRANCH_OTHER);
  const uint64_t key = cond ? rtt_key(pc, input_queue[chain_probe_cursor].ip.to<uint64_t>()) : pc;
  ++sim_stats.rtt_dg_lookups;
  const auto it = rtt.find(key);
  static const std::vector<uint64_t> none;
  const auto& ips = (it != rtt.end()) ? it->second : none;
  {
    // every misprediction: the real path's missing windows by where their line is and whether seen before
    std::unordered_set<uint64_t> stored;
    for (uint64_t ip : ips) {
      stored.insert(ip >> dib_window_bits);
    }
    std::unordered_set<uint64_t> done;
    for (std::size_t k = chain_probe_cursor; k < std::size(input_queue) && k - chain_probe_cursor < rtt_len; ++k) {
      const uint64_t ip = input_queue[k].ip.to<uint64_t>();
      const uint64_t w = ip >> dib_window_bits;
      if (!done.insert(w).second || DIB.probe(champsim::address{ip})
          || std::any_of(std::begin(uop_buffer), std::end(uop_buffer), [w](const auto& e) { return e.tag == w; })) {
        continue;
      }
      const bool far = !l1i->has_virtual_block(champsim::address{ip});
      const bool first = seen_win.count(w) == 0;
      const bool cov = stored.count(w) != 0;
      ++(far ? sim_stats.rtt_dg_far : sim_stats.rtt_dg_near);
      if (first) {
        ++(far ? sim_stats.rtt_dg_far_first : sim_stats.rtt_dg_near_first);
      }
      if (cov) {
        ++(far ? sim_stats.rtt_dg_far_cov : sim_stats.rtt_dg_near_cov);
      }
    }
  }
  if (ips.empty()) {
    return;
  }
  ++sim_stats.rtt_dg_found;
  std::size_t prefix = 0;
  while (prefix < ips.size() && chain_probe_cursor + prefix < std::size(input_queue)
         && ips[prefix] == input_queue[chain_probe_cursor + prefix].ip.to<uint64_t>()) {
    ++prefix;
  }
  sim_stats.rtt_dg_prefix += prefix;
  if (prefix == ips.size()) {
    ++sim_stats.rtt_dg_full;
  }
  auto absent = [this](uint64_t tag) {
    return !DIB.probe(champsim::address{tag << dib_window_bits})
           && std::none_of(std::begin(uop_buffer), std::end(uop_buffer), [tag](const auto& e) { return e.tag == tag; });
  };
  std::unordered_set<uint64_t> real_win;
  for (std::size_t k = chain_probe_cursor; k < std::size(input_queue) && k - chain_probe_cursor < rtt_len; ++k) {
    real_win.insert(input_queue[k].ip.to<uint64_t>() >> dib_window_bits);
  }
  std::unordered_set<uint64_t> stored_win;
  for (uint64_t ip : ips) {
    stored_win.insert(ip >> dib_window_bits);
  }
  for (uint64_t w : real_win) {
    if (absent(w)) {
      ++sim_stats.rtt_dg_real_win;
      if (stored_win.count(w) != 0) {
        ++sim_stats.rtt_dg_real_cov;
      }
    }
  }
  for (uint64_t w : stored_win) {
    if (absent(w)) {
      ++sim_stats.rtt_dg_stored_win;
      if (real_win.count(w) != 0) {
        ++sim_stats.rtt_dg_stored_use;
      }
    }
  }
}

// rtt_format 1: how much of a recorded path a walker rebuilds from (start, meta_bits direction bits).
// The walker runs like the predictor down the path: the BTB names the branches and their targets, a
// RAS copy the returns, and each conditional it meets takes the next stored bit.  The trace ends at
// the branch where that would leave the recorded path: the bits are used up, a taken branch the BTB
// does not know, or a BTB target that is not where the path went.  Returns: the kept length.
std::size_t O3_CPU::rtt_dir_encode(const std::vector<uint64_t>& ips, const std::vector<uint8_t>& flags)
{
  const int budget = std::max(1, meta_bits);
  int bits = 0;
  for (std::size_t i = 0; i + 1 < ips.size(); ++i) {
    const bool branch = flags[i] != 0;
    const bool taken = flags[i] == 2;
    const uint64_t next = ips[i + 1];
    const auto pb = (ucp::btb_probe && branch) ? ucp::btb_probe(ips[i]) : std::nullopt;
    if (!pb.has_value()) {
      if (taken) {
        ++sim_stats.rtt_dir_btb_miss; // the walker sees no branch here and falls through
        sim_stats.rtt_dir_bits_used += static_cast<uint64_t>(bits);
        return i + 1;
      }
      continue;
    }
    if (pb->conditional) {
      if (bits == budget) {
        ++sim_stats.rtt_dir_bits_out; // one more conditional than the bits can steer
        sim_stats.rtt_dir_bits_used += static_cast<uint64_t>(bits);
        return i + 1;
      }
      ++bits;
      if (taken && pb->target != next) {
        ++sim_stats.rtt_dir_target;
        sim_stats.rtt_dir_bits_used += static_cast<uint64_t>(bits);
        return i + 1;
      }
      continue;
    }
    if (pb->branch_type != BRANCH_RETURN && pb->target != next) {
      ++sim_stats.rtt_dir_target; // a jump/call/indirect whose BTB target is not where the path went
      sim_stats.rtt_dir_bits_used += static_cast<uint64_t>(bits);
      return i + 1;
    }
  }
  ++sim_stats.rtt_dir_full;
  sim_stats.rtt_dir_bits_used += static_cast<uint64_t>(bits);
  return ips.size();
}

// rtt_cost: the recording's cost -- empty-ROB cycles while its path was fetched, log2-bucketed (0 = never emptied)
uint64_t O3_CPU::rtt_cost_bucket(const rtt_cap& c)
{
  if (!rtt_cost) {
    return 0;
  }
  const uint64_t cyc = rob_empty_total - c.stall_at_open;
  const uint64_t b = (cyc == 0) ? 0 : static_cast<uint64_t>(64 - __builtin_clzll(cyc)); // bit width
  if (b == 0) {
    ++sim_stats.rtt_cost_zero;
  }
  sim_stats.rtt_cost_sum += b;
  return b;
}

void O3_CPU::rtt_commit(rtt_cap& c)
{
  if (!c.keyed || c.ips.empty()) {
    return;
  }
  if ((rtt_admit_miss == 1 && !c.missed) || (rtt_admit_miss >= 2 && !c.far)) {
    ++sim_stats.rtt_admit_skipped; // the whole recovery path hit the u-op cache (2: or its misses found their lines in the L1I)
    return;
  }
  if (rtt_admit_repeat > 0 && rtt_table == 1 && rtt_store.conf_of(c.key | (uint64_t{1} << 63)) < 0) {
    // a new trace: store it only if this recovery was seen (and qualified) before
    const uint64_t h = c.key ^ (c.key >> 21) ^ (c.key >> 43);
    auto& slot = rtt_repeat_tab[static_cast<std::size_t>(h % rtt_repeat_tab.size())];
    const auto tag = static_cast<uint16_t>(((h / rtt_repeat_tab.size()) & 0xfff) | 0x1000); // 12-bit tag + present flag
    if (slot != tag) {
      slot = tag; // first sighting: remember it, store next time
      ++sim_stats.rtt_repeat_held;
      return;
    }
    slot = 0;
  }
  if (rtt_max_windows > 0) {
    // keep the first rtt_max_windows windows of the path (a window = a run of instructions in one u-op-cache window)
    int windows = 0;
    uint64_t last = 0;
    std::size_t keep = c.ips.size();
    for (std::size_t i = 0; i < c.ips.size(); ++i) {
      const uint64_t w = c.ips[i] >> dib_window_bits;
      if (i == 0 || w != last) {
        if (++windows > rtt_max_windows) {
          keep = i;
          break;
        }
        last = w;
      }
    }
    c.ips.resize(keep);
    if (c.flags.size() > keep) {
      c.flags.resize(keep);
    }
  }
  if (rtt_table == 1 && rtt_format == 1 && c.flags.size() == c.ips.size()) {
    // bounded, direction-bit traces: keep what the walker rebuilds
    const std::size_t kept = rtt_dir_encode(c.ips, c.flags);
    if (kept < c.ips.size()) {
      ++sim_stats.rtt_truncated;
    }
    std::vector<uint64_t> enc(c.ips.begin(), c.ips.begin() + static_cast<std::ptrdiff_t>(kept));
    if (rtt_seg_stable) {
      // keep the new path only as far as it repeats the stored one; a full repeat grows the trace back
      if (const auto* old = rtt_store.lookup_entry(c.key | (uint64_t{1} << 63)); old != nullptr && !old->empty()) {
        ++sim_stats.rtt_seg_rerecords;
        const std::size_t n = std::min(old->size(), enc.size());
        std::size_t p = 0;
        while (p < n && (*old)[p] == enc[p]) {
          ++p;
        }
        if (p < n && p > 0) {
          enc.resize(p);
          ++sim_stats.rtt_seg_trimmed;
        }
        sim_stats.rtt_seg_kept_instrs += enc.size();
      }
    }
    ++sim_stats.rtt_captures;
    sim_stats.rtt_cap_instrs += c.ips.size();
    sim_stats.rtt_stored_instrs += enc.size();
    rtt_conf_on_insert(c.key);
    rtt_store.insert(c.key | (uint64_t{1} << 63), enc, rtt_cost_bucket(c));
    c.ips.clear();
    sim_stats.rtt_table_size = rtt_store.size();
    return;
  }
  if (rtt_table == 1) {
    // bounded: the SUCP trace store, the manifest format enforced at insert (truncates)
    bool truncated = false;
    bool encodable = true;
    auto enc = chain_encode(c.ips, truncated, encodable);
    if (!encodable || enc.empty()) {
      ++sim_stats.rtt_unencodable;
      return;
    }
    if (truncated) {
      ++sim_stats.rtt_truncated;
    }
    ++sim_stats.rtt_captures;
    sim_stats.rtt_cap_instrs += c.ips.size();
    sim_stats.rtt_stored_instrs += enc.size();
    rtt_conf_on_insert(c.key);
    rtt_store.insert(c.key | (uint64_t{1} << 63), enc);
    c.ips.clear();
    sim_stats.rtt_table_size = rtt_store.size();
    return;
  }
  ++sim_stats.rtt_captures;
  sim_stats.rtt_cap_instrs += c.ips.size();
  rtt[c.key] = std::move(c.ips); // the latest recovery path
  c.ips.clear();
  sim_stats.rtt_table_size = rtt.size();
}

bool O3_CPU::do_predict_branch(ooo_model_instr& arch_instr)
{
  bool stop_fetch = false;

  sim_stats.total_branch_types.increment(arch_instr.branch);
  if (!arch_instr.early_predicted) {
    predict_instruction(arch_instr);
    arch_instr.vp_ctx = vpred.last_seq(); // trace_vpred: the prediction this instruction follows
  }
  const bool ucp_h2p_now = ucp_enable && arch_instr.pred_h2p;
  const uint64_t ucp_btb_target = arch_instr.pred_btb_target;
  const bool always_taken = arch_instr.pred_always_taken;
  const champsim::address predicted_branch_target{arch_instr.branch_prediction ? ucp_btb_target : 0};

  if (arch_instr.is_branch) {
    if constexpr (champsim::debug_print) {
      fmt::print("[BRANCH] instr_id: {} ip: {} taken: {}\n", arch_instr.instr_id, arch_instr.ip, arch_instr.branch_taken);
    }

    // mispredict-trigger modes 2/3: predict-time alternate-path probe.  The stored
    // key is hash(branch, restart) where restart = the path the failed prediction
    // rejected; the same quantity is computable HERE, before resolution, as the
    // alternate of the current prediction (fallthrough if predicted taken, BTB
    // target if predicted not-taken).  A hit means this branch has repeatedly
    // mispredicted from this direction with a costly recovery -- prefetch that
    // recovery path now, with fetch-to-resolution lead.  Conditionals only: an
    // indirect's alternate target is not enumerable at predict time.  Mode 3 adds
    // the UCP H2P gate (CTR_SC_BIMH, published by tage_sc_l on every prediction):
    // unfiltered probing fires on every occurrence of a stored branch, so ~the
    // branch's correct-prediction fraction of walks is pure speculation waste --
    // measured -0.061pp (mode 2, qualcomm).  Gating on hard-to-predict recovers
    // the conversion rate, exactly UCP's rationale for its classifier.
    if ((stall_mispredict_mode == 2 || (stall_mispredict_mode == 3 && arch_instr.pred_h2p)) && (!warmup || trace_warmup_train) && fill_mode == fill_mode_type::CHAIN
        && ((arch_instr.branch == BRANCH_CONDITIONAL) || (arch_instr.branch == BRANCH_OTHER))) {
      const uint64_t bip = arch_instr.ip.to<uint64_t>();
      const uint64_t alt = arch_instr.branch_prediction ? bip + 4 : ucp_btb_target;
      if (alt != 0) {
        const uint64_t key = (bip ^ (alt << 1)) | (uint64_t{1} << 63);
        ++sim_stats.event_probes;
        if (const auto* ips = fill_store.lookup_entry(key); ips != nullptr && !ips->empty()) {
          ++sim_stats.event_probe_hits;
          do_chain_trigger(*ips);
        }
      }
    }

    // mode 4: predict-time alternate probe of the REGULAR store.  When a hard-to-
    // predict conditional is predicted, the first instruction of its other
    // direction -- where fetch restarts if this prediction is wrong -- is probed as
    // an ordinary trace entry: a recovery path opens with a u-op-cache miss at that
    // address, so the stall segmenter has usually stored it already.  The walk gets
    // the prediction-to-resolution lead (UCP's) with a recorded path, and needs no
    // event traces.  The fall-through is the branch's learned successor (exact x86
    // length); none = never seen falling through.
    if (stall_mispredict_mode == 4 && arch_instr.pred_h2p && (!warmup || trace_warmup_train) && fill_mode == fill_mode_type::CHAIN
        && ((arch_instr.branch == BRANCH_CONDITIONAL) || (arch_instr.branch == BRANCH_OTHER))) {
      uint64_t alt = ucp_btb_target; // predicted not-taken: the other direction is the BTB target
      if (arch_instr.branch_prediction) {
        const auto it = code_next.find(arch_instr.ip.to<uint64_t>());
        alt = (it != code_next.end()) ? it->second : 0; // predicted taken: the other direction is the fall-through
      }
      ++sim_stats.event_probes;
      if (alt != 0) {
        if (const auto* ips = fill_store.lookup_entry(alt); ips != nullptr && !ips->empty()) {
          ++sim_stats.event_probe_hits;
          do_chain_trigger(*ips);
        }
      }
    }

    // call code prefetcher every time the branch predictor is used
    l1i->impl_prefetcher_branch_operate(arch_instr.ip, arch_instr.branch, predicted_branch_target);

    if (branch_mispredicts(arch_instr)) { // conditional branches are re-evaluated at decode when the target is computed
      arch_instr.pred_wrong = true;
      if (rp_mode != 0) {
        rp_open(arch_instr.ip.to<uint64_t>()); // what follows is this branch's resteer path
      }
      sim_stats.total_rob_occupancy_at_branch_mispredict += std::size(ROB);
      sim_stats.branch_type_misses.increment(arch_instr.branch);
      if (!warmup) {
        fetch_resume_time = champsim::chrono::clock::time_point::max();
        stop_fetch = true;
        arch_instr.branch_mispredicted = true;
        if (fill_mode == fill_mode_type::CHAIN) {
          chain_mispredict_shadow(arch_instr.instr_id); // walks probed past this branch saw the correct path
        }
        if (wrong_path_level >= 1) {
          chain_cursor_blocked = true; // predicted here (the cursor had not reached it): same stop
          if (!arch_instr.early_predicted) {
            chain_peek_cursor = 1; // trace_oracle_peek: past the branch (the queue front, popped this cycle)
            chain_peek_left = oracle_peek;
          }
        }
        if (wrong_path_level >= 2) {
          if (!wp_active) {
            wp_begin(arch_instr); // no cursor ahead of fetch: the predictor turns down the wrong path here
          }
          wp_fetching = true; // fetch has reached the branch and follows the wrong path until resolution
        }
      }
    } else {
      stop_fetch = arch_instr.branch_taken; // if correctly predicted taken, then we can't fetch anymore instructions this cycle
    }

    // predicted at the cursor (trace_wrong_path >= 1): UCP and training already ran there
    if (!arch_instr.early_predicted) {
      // UCP: classify/trigger BEFORE the predictor and BTB update (artifact order --
      // the walk must see pre-update state); Alt-BP training happens inside
      if (ucp_enable && (!warmup || trace_warmup_train)) {
        do_ucp_branch(arch_instr, ucp_btb_target, always_taken, ucp_h2p_now);
      }

      impl_update_btb(arch_instr.ip, arch_instr.branch_target, arch_instr.branch_taken, arch_instr.branch);
      impl_last_branch_result(arch_instr.ip, arch_instr.branch_target, arch_instr.branch_taken, arch_instr.branch);
      peek_diag_branch(arch_instr, branch_mispredicts(arch_instr));
      recovery_on_predict(arch_instr, branch_mispredicts(arch_instr));
      vpred_on_predict(arch_instr, branch_mispredicts(arch_instr));
    }
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
  // trace_wrong_path >= 2: fetch follows the wrong path (sets decoder occupancy for this cycle's walks)
  if (wp_fetching) {
    wp_fetch();
  }
  // ALT/HEAD/CHAIN trace-fill: advance in-flight pre-decode walks (timed installs) once per cycle
  if ((fill_mode == fill_mode_type::ALT || fill_mode == fill_mode_type::HEAD || fill_mode == fill_mode_type::CHAIN || ucp_enable) && !alt_walks.empty()) {
    drain_alt_walks();
  }
  // scan through IFETCH_BUFFER to find instructions that hit in the decoded instruction buffer
  auto begin = std::find_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), [](const ooo_model_instr& x) { return !x.dib_checked; });
  auto [window_begin, window_end] = champsim::get_span(begin, std::end(IFETCH_BUFFER), champsim::bandwidth{FETCH_WIDTH});
  if (ftq_lookup_window > 0) {
    // decoupled front end: the lookup happens when a block reaches fetch -- only the
    // first ftq_lookup_window FTQ entries are within reach
    const auto reach = std::min<std::ptrdiff_t>(ftq_lookup_window, static_cast<std::ptrdiff_t>(std::size(IFETCH_BUFFER)));
    if (std::distance(std::begin(IFETCH_BUFFER), window_end) > reach) {
      window_end = std::next(std::begin(IFETCH_BUFFER), reach);
    }
    if (std::distance(std::begin(IFETCH_BUFFER), window_begin) >= reach) {
      window_begin = window_end; // the oldest unchecked block is not at fetch yet
    }
  }
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
// global walk line-issue port model (trace_walk_issue_cap): lazily reset the
// per-cycle budget; 0 = unlimited (legacy schema).
bool O3_CPU::walk_issue_available(uint64_t walk_lines_issued)
{
  if (walk_l2_mshrs > 0) {
    sim_stats.walker_mshr_peak = std::max<uint64_t>(sim_stats.walker_mshr_peak, l1i->walker_mshr_peak);
    if (l1i->walker_inflight() >= static_cast<std::size_t>(walk_l2_mshrs)) {
      ++sim_stats.walker_mshr_full;
      return false; // every walker MSHR busy
    }
  }
  const auto walk_pq_busy = [this] {
    const auto pq = l1i->get_pq_occupancy();
    return std::accumulate(std::begin(pq), std::end(pq), std::size_t{0}) > 0;
  };
  if (walk_l2_mshrs == 0 && walk_mshr_limit > 0 && walk_lines_issued >= static_cast<uint64_t>(walk_prio_head) // walker MSHRs: walk misses never take a regular MSHR
      && (l1i->get_mshr_occupancy() >= static_cast<std::size_t>(walk_mshr_limit) || walk_pq_busy())) {
    ++sim_stats.walk_mshr_yield; // FDIP and demand keep the L1I's misses
    return false;
  }
  if (alt_walk_issue_cap <= 0) {
    return true;
  }
  if (walk_issue_stamp != current_time) {
    walk_issue_stamp = current_time;
    walk_issue_count = 0;
  }
  return walk_issue_count < alt_walk_issue_cap;
}

void O3_CPU::walk_issue_note()
{
  if (alt_walk_issue_cap > 0) {
    ++walk_issue_count;
  }
}

// walk pre-decode model (trace_walk_decode_cap / trace_walk_decode_shared): a
// window install = one pre-decode.  The cap is a per-cycle budget across all
// walks (incl. catch-up bursts); shared mode additionally requires the demand
// path to be in stream mode (regular decoders idle) -- UCP's SharedDecoders rule.
bool O3_CPU::walk_decode_available(std::size_t uops)
{
  if (alt_walk_decode_shared == 2) {
    // shared decoders by age: demand (older) decodes first; a walk window
    // pre-decodes in the decode width demand left this cycle, in any fetch mode
    if (walk_slots_stamp != current_time) {
      walk_slots_stamp = current_time;
      // the wrong path, while fetch follows it, decodes on the same decoders
      const long wp_used = wp_fetching ? static_cast<long>(wp_decoded_now) : 0;
      walk_decode_slots = std::max<long>(0, static_cast<long>(DECODE_WIDTH) - static_cast<long>(demand_decoded_now) - wp_used);
    }
    if (walk_decode_slots < std::min<long>(static_cast<long>(uops), static_cast<long>(DECODE_WIDTH))) {
      return false;
    }
  } else if (alt_walk_decode_shared == 1) {
    // demand owns the decoders this cycle; while fetch follows a wrong path
    // (trace_wrong_path >= 2), whether it does depends on that path's u-op-cache hits
    if (wp_fetching ? wp_decoders_busy : fetch_mode != fetch_mode_type::STREAM) {
      return false;
    }
  }
  if (alt_walk_decode_cap <= 0) {
    return true;
  }
  if (walk_decode_stamp != current_time) {
    walk_decode_stamp = current_time;
    walk_decode_count = 0;
  }
  return walk_decode_count < alt_walk_decode_cap;
}

void O3_CPU::walk_decode_note(std::size_t uops)
{
  if (alt_walk_decode_shared == 2) {
    walk_decode_slots -= std::min<long>(static_cast<long>(uops), static_cast<long>(DECODE_WIDTH));
  }
  if (alt_walk_decode_cap > 0) {
    ++walk_decode_count;
  }
}

// UCP BTB banking model (trace_ucp_btb_banks): each demand prediction occupies its
// BTB bank for the cycle and a UCP walk may only install a window whose lead-IP
// bank is free.  Banks are FETCH-BLOCK granular (bank = (ip>>6) mod banks): real
// BTBs index per fetch block, so a 16-wide sequential demand run touches ~1-2
// banks/cycle and small realistic bank counts (2/4/8) are meaningful.  (The
// UCP_ISCA24 artifact instead hashes per-instruction, ip>>2, and compensates with
// 32 banks -- roughly the same collision odds at 16x the bank count.)  Demand
// always wins; the artifact's probabilistic alternate-priority heuristic
// (BTB_CONF_PROB) is omitted, making this conservative for UCP.  0 = ideal
// porting (the artifact's shipped IDEAL_BTB_BANKING).
void O3_CPU::ucp_note_demand_bank(uint64_t ip)
{
  if (ucp_bank_mask_stamp != current_time) {
    ucp_bank_mask_stamp = current_time;
    ucp_demand_bank_mask = 0;
  }
  ucp_demand_bank_mask |= 1ULL << ((ip >> 6) & (static_cast<uint64_t>(ucp_btb_banks) - 1));
}

bool O3_CPU::ucp_bank_conflict(uint64_t ip)
{
  // walks drain (check_dib) before this cycle's demand predictions are noted
  // (initialize_instruction), so the freshest available mask is last cycle's --
  // accept it: fetch is sequential, so consecutive cycles touch correlated banks
  if (ucp_btb_banks <= 0 || ucp_bank_mask_stamp + clock_period < current_time) {
    return false; // banking off, or no recent demand predictions
  }
  return (ucp_demand_bank_mask >> ((ip >> 6) & (static_cast<uint64_t>(ucp_btb_banks) - 1))) & 1ULL;
}

void O3_CPU::drain_alt_walks()
{
  if (ucp_artifact & 2) {
    // uop-MSHR: mark arrived lines, then free from the front while the front entry's line arrived and
    // its branch resolved (operate_on_uop_mshr_queue)
    for (auto& m : ucp_mshr) {
      if (!m.filled && current_time >= m.issued + UCP_ART_L1I_LATENCY * clock_period
          && (l1i->has_virtual_block(champsim::address{champsim::block_number{m.blk}}) || current_time >= m.issued + UCP_ART_MSHR_TIMEOUT * clock_period)) {
        m.filled = true;
      }
    }
    while (!ucp_mshr.empty() && ucp_mshr.front().filled
           && (ucp_mshr.front().branch == 0 || ucp_unresolved.count(ucp_mshr.front().branch) == 0
               || current_time >= ucp_mshr.front().issued + UCP_ART_MSHR_TIMEOUT * clock_period)) {
      ucp_mshr.pop_front();
    }
  }
  std::size_t walk_pq_inflight = 0; // walk_pq: recovery-walk lines requested, not yet seen in the L1I
  if (walk_pq > 0) {
    for (const auto& w : alt_walks) {
      walk_pq_inflight += w.ucp ? 0 : w.pf_wait.size();
    }
  }
  for (auto& w : alt_walks) {
    if (!w.ucp && walk_pq > 0 && alt_walk_l1i) {
      // walk_pq: a requested line is decodable once the L1I holds it (fill or prefetch hit), never sooner than an L1I access
      for (auto it = std::begin(w.pf_wait); it != std::end(w.pf_wait);) {
        if (current_time >= it->first + UCP_ART_L1I_LATENCY * clock_period && l1i->has_virtual_block(champsim::address{champsim::block_number{it->second}})) {
          w.lines_ready.insert(it->second);
          w.last_progress = current_time;
          --walk_pq_inflight;
          it = w.pf_wait.erase(it);
        } else {
          ++it;
        }
      }
      // issue into the L1I prefetch queue: the cache serves it after demand; only the per-cycle walk issue cap applies
      if (!w.lines_pending.empty()) {
        if (walk_issue_stamp != current_time) {
          walk_issue_stamp = current_time;
          walk_issue_count = 0;
        }
        if (alt_walk_issue_cap <= 0 || walk_issue_count < alt_walk_issue_cap) {
          const uint64_t blk = champsim::block_number{champsim::address{w.lines_pending.front()}}.to<uint64_t>();
          if (walk_pq_inflight < static_cast<std::size_t>(walk_pq) && l1i->prefetch_line(champsim::address{w.lines_pending.front()}, true, 0)) {
            ++walk_pq_inflight;
            w.pf_wait.emplace_back(current_time, blk);
            w.lines_pending.pop_front();
            ++sim_stats.alt_lines_issued;
            ++sim_stats.walk_pq_issued;
            ++w.lines_issued;
            if (w.oracle) {
              ++sim_stats.chain_shadow_lines;
            }
            if (w.wrong_path) {
              ++sim_stats.wp_lines;
            }
            w.last_progress = current_time;
            walk_issue_note();
          } else {
            ++sim_stats.walk_pq_full;
          }
        }
      }
    } else if (w.ucp && (ucp_artifact & 2) && alt_walk_l1i) {
      // UCP_ISCA24: a requested line feeds the alternate decoders once the L1I holds it -- a fill, or
      // the prefetch found it resident (L1I_PREF_AT_TRY_HIT) -- never sooner than an L1I access
      for (auto it = std::begin(w.pf_wait); it != std::end(w.pf_wait);) {
        if (current_time >= it->first + UCP_ART_L1I_LATENCY * clock_period && l1i->has_virtual_block(champsim::address{champsim::block_number{it->second}})) {
          w.lines_ready.insert(it->second);
          w.last_progress = current_time;
          it = w.pf_wait.erase(it);
        } else {
          ++it;
        }
      }
      // operate_on_uop_mshr_queue: one line per cycle into the L1I prefetch queue, at most UOP_MSHR in flight
      if (!w.lines_pending.empty()) {
        if (ucp_pf_stamp != current_time) {
          ucp_pf_stamp = current_time;
          ucp_pf_left = UCP_ART_LINES_PER_CYCLE;
        }
        if (ucp_pf_left > 0) {
          const uint64_t blk = champsim::block_number{champsim::address{w.lines_pending.front()}}.to<uint64_t>();
          if (ucp_mshr.size() < UCP_ART_MSHR && l1i->prefetch_line(champsim::address{w.lines_pending.front()}, true, 0)) {
            --ucp_pf_left;
            ucp_mshr.push_back({blk, w.ucp_branch, current_time, false});
            w.pf_wait.emplace_back(current_time, blk);
            w.lines_pending.pop_front();
            ++sim_stats.alt_lines_issued;
            ++sim_stats.ucp_art_pf_issued;
            ++w.lines_issued;
            if (w.wrong_path) {
              ++sim_stats.wp_lines;
            }
            w.last_progress = current_time;
          } else {
            ++sim_stats.ucp_art_pf_full;
          }
        }
      }
    }
    // real-L1I mode: issue one pending line per walk per cycle through the L1I
    // (contends with demand for rq slots and MSHRs; misses go to L2/LLC/DRAM)
    else if (alt_walk_l1i && !w.lines_pending.empty() && walk_issue_available(w.lines_issued)) {
      CacheBus::request_type line_pkt;
      line_pkt.v_address = champsim::address{w.lines_pending.front()};
      line_pkt.ip = champsim::address{w.lines_pending.front()};
      line_pkt.instr_id = 0; // no dependent instructions: response marks the line ready
      line_pkt.no_fill = alt_walk_nofill && !w.ucp; // SUCP walk bytes allocate in no cache level
      line_pkt.walker = (walk_l2_mshrs > 0 && !w.ucp);
      if (line_pkt.walker) {
        line_pkt.no_fill = true; // handed to the walk, not filled into the L1I (L2/LLC still allocate)
      }
      if (L1I_bus.issue_read(line_pkt)) {
        w.lines_pending.pop_front();
        ++sim_stats.alt_lines_issued;
        ++w.lines_issued;
        if (w.oracle) {
          ++sim_stats.chain_shadow_lines;
        }
        if (w.wrong_path) {
          ++sim_stats.wp_lines;
        }
        w.last_progress = current_time;
        walk_issue_note();
      }
    }
    // walk_resident_port: resident lines read through the demand fetch port when demand leaves it idle
    while (!w.res_eta.empty() && w.res_eta.front().first <= current_time) {
      w.lines_ready.insert(w.res_eta.front().second);
      w.res_eta.pop_front();
    }
    if (walk_resident_port == 1 && !w.lines_resident.empty() && fetch_mode == fetch_mode_type::STREAM && walk_res_stamp != current_time) {
      walk_res_stamp = current_time; // one read per cycle for all walks
      const uint64_t ip = w.lines_resident.front();
      w.lines_resident.pop_front();
      if (l1i->has_virtual_block(champsim::address{ip})) {
        w.res_eta.emplace_back(current_time + 4 * clock_period, champsim::block_number{champsim::address{ip}}.to<uint64_t>()); // L1I hit latency
        ++sim_stats.walk_res_read;
      } else {
        ++sim_stats.walk_res_evicted;
        w.lines_pending.push_back(ip); // evicted since launch: fetch it like a missing line
      }
      w.last_progress = current_time;
    }
    while (w.idx < w.windows.size() && w.ready <= current_time) {
      if (((walk_ooo && !w.ucp) || (w.ucp && ucp_artifact != 0)) && !w.fdip_pd) {
        // out of order: any window whose line has arrived installs (pre-decode width permitting),
        // so one slow line does not hold back the rest of the walk
        if (w.done.size() != w.windows.size()) {
          w.done.assign(w.windows.size(), 0);
        }
        const bool ucp_dedicated = w.ucp && (ucp_artifact & 4) != 0; // own decoders: their width is the only limit
        int installed = 0;
        for (std::size_t i = w.idx; i < w.windows.size() && (ucp_dedicated || installed < alt_walk_width); ++i) {
          if (w.done[i] != 0) {
            continue;
          }
          if (!walk_line_ready(w, w.windows[i].front())) {
            if (i == w.idx) {
              ++sim_stats.alt_line_stalls;
            }
            continue;
          }
          if (w.ucp && ucp_bank_conflict(w.windows[i].front())) {
            ++sim_stats.ucp_bank_conflicts;
            break; // BTB bank held by a demand prediction this cycle; retry next cycle
          }
          if (ucp_dedicated ? !ucp_decode_available() : !walk_decode_available(w.windows[i].size())) {
            break;
          }
          const std::size_t n_uops = w.windows[i].size();
          if (fill_mode == fill_mode_type::CHAIN && uop_buffer_windows > 0 && !w.ucp && !w.direct_install) {
            uop_buffer_push(w.windows[i], w.probe_time, w.entry, w.probe_id, w.oracle, w.wrong_path, w.recovery, w.vp, w.chain_hop,
                            static_cast<uint8_t>(std::min<std::size_t>(i, 255)));
            ++sim_stats.alt_installed_windows;
          } else {
            sim_stats.alt_installed_windows += DIB.install(w.windows[i], /*prefetch=*/true);
          }
          w.done[i] = 1;
          ++installed;
          w.last_progress = current_time;
          if (ucp_dedicated) {
            ucp_dec_left -= static_cast<long>(n_uops); // a window wider than the decoders spills into the next cycle
          } else {
            walk_decode_note(n_uops);
          }
        }
        while (w.idx < w.windows.size() && w.done[w.idx] != 0) {
          ++w.idx;
        }
        w.ready += clock_period;
        continue;
      }
      if (w.ucp && ucp_bank_conflict(w.windows[w.idx].front())) {
        ++sim_stats.ucp_bank_conflicts;
        break; // BTB bank held by a demand prediction this cycle; retry next cycle
      }
      // real-L1I mode: a window may only install once its line has arrived
      if (!walk_line_ready(w, w.windows[w.idx].front())) {
        ++sim_stats.alt_line_stalls;
        break; // pre-decode stalls on the byte fetch; retry next cycle
      }
      if (!walk_decode_available(w.windows[w.idx].size())) {
        break; // no pre-decode slot this cycle (cap reached / demand owns shared decoders); retry next cycle
      }
      // install up to alt_walk_width windows per cycle (pre-decode width)
      for (int k = 0; k < alt_walk_width && w.idx < w.windows.size() && walk_decode_available(w.windows[w.idx].size()); ++k) {
        const std::size_t n_uops = w.windows[w.idx].size();
        if (k > 0 && !walk_line_ready(w, w.windows[w.idx].front())) {
          break;
        }
        if (w.fdip_pd) {
          ++sim_stats.fdip_pd_installed;
        }
        if (fill_mode == fill_mode_type::CHAIN && uop_buffer_windows > 0 && !w.ucp && !w.direct_install) {
          // CHAIN: stage into the trace-uop buffer (no u-op-cache pollution);
          // demand hits promote from there
          uop_buffer_push(w.windows[w.idx], w.probe_time, w.entry, w.probe_id, w.oracle, w.wrong_path, w.recovery, w.vp, w.chain_hop,
                          static_cast<uint8_t>(std::min<std::size_t>(w.idx, 255)));
          ++sim_stats.alt_installed_windows;
        } else {
          sim_stats.alt_installed_windows += DIB.install(w.windows[w.idx], /*prefetch=*/true);
        }
        ++w.idx;
        w.last_progress = current_time;
        walk_decode_note(n_uops);
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
                                   if (w.fdip_pd && current_time - w.probe_time > FDIP_PD_DEADLINE_CYCLES * clock_period) {
                                     ++sim_stats.fdip_pd_expired; // FDIP's line never arrived (prefetch dropped or late)
                                     return true;
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

// trace_ucp_artifact 1 (stop_previous_alternate_path): a new UCP trigger drops what the earlier
// paths have not requested yet; their requested lines stay in flight and still decode and install
void O3_CPU::ucp_stop_previous()
{
  for (auto& w : alt_walks) {
    if (!w.ucp || w.lines_pending.empty()) {
      continue;
    }
    if (w.done.size() != w.windows.size()) {
      w.done.assign(w.windows.size(), 0);
      std::fill(std::begin(w.done), std::begin(w.done) + static_cast<std::ptrdiff_t>(w.idx), 1);
    }
    std::set<uint64_t> unissued;
    for (uint64_t ip : w.lines_pending) {
      unissued.insert(champsim::block_number{champsim::address{ip}}.to<uint64_t>());
    }
    for (const auto& [t, blk] : w.pf_wait) {
      unissued.erase(blk); // requested earlier (a later window revisits the line)
    }
    for (uint64_t blk : w.lines_ready) {
      unissued.erase(blk);
    }
    for (std::size_t i = w.idx; i < w.windows.size(); ++i) {
      if (w.done[i] == 0 && unissued.count(champsim::block_number{champsim::address{w.windows[i].front()}}.to<uint64_t>()) != 0) {
        w.done[i] = 1; // dropped, never installed
        ++sim_stats.ucp_art_dropped;
      }
    }
    w.lines_pending.clear();
    while (w.idx < w.windows.size() && w.done[w.idx] != 0) {
      ++w.idx;
    }
  }
}

// is rawip's window requested by an in-flight UCP path but not installed yet?
bool O3_CPU::ucp_window_pending(uint64_t rawip) const
{
  const uint64_t wtag = rawip >> dib_window_bits;
  for (const auto& w : alt_walks) {
    if (!w.ucp) {
      continue;
    }
    for (std::size_t i = w.idx; i < w.windows.size(); ++i) {
      if ((w.done.empty() || w.done[i] == 0) && (w.windows[i].front() >> dib_window_bits) == wtag) {
        return true;
      }
    }
  }
  return false;
}

// trace_ucp_artifact 4: the alternate decoders decode DECODE_WIDTH instructions per cycle
bool O3_CPU::ucp_decode_available()
{
  if (ucp_dec_stamp != current_time) {
    ucp_dec_stamp = current_time;
    ucp_dec_left = std::min<long>(ucp_dec_left, 0) + static_cast<long>(DECODE_WIDTH);
  }
  return ucp_dec_left > 0;
}

// is rawip's window held by an in-flight walk but not yet installed?
// a demand u-op-cache miss on a window a walk holds but has not installed: why is it late?
void O3_CPU::late_window_classify(uint64_t rawip)
{
  const uint64_t wtag = rawip >> dib_window_bits;
  for (const auto& w : alt_walks) {
    for (std::size_t i = w.idx; i < w.windows.size(); ++i) {
      if ((!w.done.empty() && w.done[i] != 0) || (w.windows[i].front() >> dib_window_bits) != wtag) {
        continue;
      }
      if (walk_line_ready(w, w.windows[i].front())) {
        ++(i > w.idx && w.done.empty() ? sim_stats.late_hol : sim_stats.late_decode);
      } else {
        const uint64_t blk = champsim::block_number{champsim::address{w.windows[i].front()}}.to<uint64_t>();
        const bool unissued = std::any_of(std::begin(w.lines_pending), std::end(w.lines_pending),
                                          [blk](uint64_t ip) { return champsim::block_number{champsim::address{ip}}.to<uint64_t>() == blk; });
        ++(unissued ? sim_stats.late_unissued : sim_stats.late_memory);
      }
      return;
    }
  }
}

bool O3_CPU::alt_window_pending(uint64_t rawip) const
{
  const uint64_t wtag = rawip >> dib_window_bits;
  for (const auto& w : alt_walks) {
    for (std::size_t i = w.idx; i < w.windows.size(); ++i) {
      if (!w.done.empty() && w.done[i] != 0) {
        continue; // installed out of order already
      }
      if ((w.windows[i].front() >> dib_window_bits) == wtag) {
        return true;
      }
    }
  }
  return false;
}

// CHAIN metadata format encoder: group the trace's uops into aligned windows
// and enforce the manifest format, truncating longer traces.  Two formats:
//   meta_bits == 0 (legacy): fixed slots, one 16-bit window delta per window
//     beyond the entry, at most meta_windows windows;
//   meta_bits > 0 (packed): a fixed-width slot field of meta_bits bits holding
//     a variable number of windows, each costed by its transfer kind with a
//     meta_kind_bits kind code:
//       kind 2 bits: sequential (delta +1) 2, short 2+meta_short_bits, long
//                    2+meta_long_bits (long_bits 0 = no long class);
//       kind 1 bit:  sequential 1, jump 1+meta_short_bits (no long class);
//     signed window deltas (defaults 8 / 16 = +-4KB / +-1MB).  Windows are
//     packed until the budget is exhausted; the end needs no length field
//     (kind 2: an end code; kind 1: a jump of delta 0, or 1-padding too short
//     to hold a jump record).
// A delta beyond the widest class (legacy: 16 bits) rejects the trace, or with
// meta_far_truncate ends it there and keeps the prefix.  Every transfer is
// also histogrammed by kind and by signed bits needed (packed-format sizing).
// Returns the kept uops; encodable=false rejects the trace.
std::vector<uint64_t> O3_CPU::chain_encode(const std::vector<uint64_t>& ips, bool& truncated, bool& encodable)
{
  // packed format: configurable kind-code and delta widths; legacy slots are a
  // flat 16-bit delta (classified 8/16 only for the histograms)
  const bool packed = meta_bits > 0;
  const int kind_bits = packed ? meta_kind_bits : 2;
  const int short_delta_bits = packed ? meta_short_bits : 8;
  const bool has_long = !packed || (kind_bits == 2 && meta_long_bits > 0);
  const int long_delta_bits = !packed ? 16 : (has_long ? meta_long_bits : 0);
  const int widest_delta_bits = has_long ? long_delta_bits : short_delta_bits;
  const int SEQ_BITS = kind_bits; // kind code only: next window = previous + 1
  const int SHORT_BITS = kind_bits + short_delta_bits;
  const int LONG_BITS = kind_bits + long_delta_bits;
  // smallest n with -2^(n-1) <= d <= 2^(n-1)-1: the signed field width a delta needs
  const auto signed_bits = [](int64_t d) {
    int n = 1;
    while (d < -(int64_t{1} << (n - 1)) || d > (int64_t{1} << (n - 1)) - 1) {
      ++n;
    }
    return n;
  };
  std::vector<uint64_t> kept;
  kept.reserve(ips.size());
  truncated = false;
  encodable = true;
  int windows_used = 0;
  int bits_used = 0;
  int hardest_kind = 0; // over the STORED transfers: 0 none (single window), 1 sequential, 2 short, 3 long
  int need_bits = 0;    // packed cost of the WHOLE captured stretch (demand-side sizing of the slot field)
  int need_windows = 0; // windows the whole captured stretch spans
  int reach_max = -1;   // farthest taken transfer in the stretch, as a reach bucket (see reach_bucket); -1 = none
  bool stop = false;    // manifest full: keep scanning for the need histogram, stop storing
  uint64_t last_tag = 0;
  bool have_tag = false;
  // delta-width sizing: |delta| in windows -> bucket 0 <=128 (8b, +-4KB), 1 <=4096 (13b, +-128KB),
  // 2 <=8192 (14b, +-256KB), 3 <=16384 (15b, +-512KB), 4 <=32768 (16b, +-1MB), 5 beyond
  const auto reach_bucket = [](int64_t delta) {
    const int64_t m = (delta < 0) ? -delta : delta;
    return (m <= 128) ? 0 : (m <= 4096) ? 1 : (m <= 8192) ? 2 : (m <= 16384) ? 3 : (m <= 32768) ? 4 : 5;
  };
  for (uint64_t raw : ips) {
    const uint64_t tag = raw >> dib_window_bits;
    if (!have_tag || tag != last_tag) { // a new window slot
      int slot_bits = 0;
      int kind = 0;
      if (have_tag) {
        const int64_t delta = static_cast<int64_t>(tag) - static_cast<int64_t>(last_tag);
        const int need = signed_bits(delta);
        if (delta != 1) {
          const int rb = reach_bucket(delta);
          ++sim_stats.chain_reach_xfer[rb];
          reach_max = std::max(reach_max, rb);
          ++sim_stats.chain_delta_bits[static_cast<std::size_t>(std::clamp(need, 3, 17))];
        }
        if (delta != 1 && need > widest_delta_bits) {
          if (stop) {
            break; // beyond the stored part: the need histogram stops here (unchanged storage decision)
          }
          ++sim_stats.chain_delta_far;
          if (meta_far_truncate) {
            ++sim_stats.chain_far_truncated; // keep the prefix up to the unencodable jump
            truncated = true;
            stop = true;
            break;
          }
          ++sim_stats.chain_reach_trace[reach_max]; // the stretch's reach so far (this transfer's bucket or wider)
          encodable = false; // far transfer: not representable in the widest delta
          return kept;
        }
        if (delta == 1) {
          ++sim_stats.chain_delta_seq;
          slot_bits = SEQ_BITS;
          kind = 1;
        } else if (need <= short_delta_bits) {
          ++sim_stats.chain_delta_short;
          slot_bits = SHORT_BITS;
          kind = 2;
        } else {
          ++sim_stats.chain_delta_long;
          slot_bits = LONG_BITS;
          kind = 3;
        }
      }
      need_bits += slot_bits;
      ++need_windows;
      if (!stop) {
        if (meta_bits > 0) {
          if (bits_used + slot_bits > meta_bits) {
            truncated = true; // slot field full
            stop = true;
          } else {
            bits_used += slot_bits;
          }
        } else if (windows_used == meta_windows) {
          truncated = true;
          stop = true;
        }
        if (!stop) {
          hardest_kind = std::max(hardest_kind, kind);
          ++windows_used;
        }
      }
      have_tag = true;
      last_tag = tag;
    }
    if (!stop) {
      kept.push_back(raw);
    }
  }
  // demand-side sizing: what the whole captured stretch would have needed
  // (capped by trace_stall_depth at capture, so raise depth to see the true need)
  {
    const std::size_t bb = (need_bits <= 16) ? 0 : (need_bits <= 24) ? 1 : (need_bits <= 32) ? 2 : (need_bits <= 48) ? 3 : (need_bits <= 64) ? 4 : 5;
    ++sim_stats.chain_need_bits[bb];
    const std::size_t wb = (need_windows <= 2) ? 0 : (need_windows <= 3) ? 1 : (need_windows <= 4) ? 2 : (need_windows <= 6) ? 3 : 4;
    ++sim_stats.chain_need_windows[wb];
    if (truncated) {
      sim_stats.chain_trunc_windows += static_cast<uint64_t>(need_windows - windows_used);
    }
    // per stretch: the narrowest delta width that keeps it whole (sequential-only stretches need none)
    if (reach_max >= 0) {
      ++sim_stats.chain_reach_trace[reach_max];
    } else {
      ++sim_stats.chain_reach_none;
    }
  }
  switch (hardest_kind) { // per stored manifest: what the slot field had to express
  case 0:
    ++sim_stats.chain_trace_single;
    break;
  case 1:
    ++sim_stats.chain_trace_seq_only;
    break;
  case 2:
    ++sim_stats.chain_trace_with_short;
    break;
  default:
    ++sim_stats.chain_trace_with_long;
    break;
  }
  return kept;
}

// CHAIN walk launch: probe hit at IFETCH enqueue on a trace entry.  The manifest
// is already format-enforced (<= meta_windows windows, deltas encodable), so the
// walk covers the whole stored trace.  All line addresses are known immediately
// (entry pc + deltas); in real-L1I mode the first lines issue THIS cycle.
uint8_t O3_CPU::do_chain_trigger(const std::vector<uint64_t>& ips, uint64_t probe_id, bool wrong_path)
{
  const uint64_t entry = ips.front();
  if (std::any_of(std::begin(alt_walks), std::end(alt_walks), [entry](const auto& w) { return w.entry == entry; })) {
    return TRIG_DEDUP; // already being walked
  }
  if (!alt_walk_filter) {
    // legacy check: skip the walk only when the ENTRY window is resident/staged
    if (DIB.probe(champsim::address{entry})) {
      return TRIG_RESIDENT;
    }
    const uint64_t etag = entry >> dib_window_bits;
    if (std::any_of(std::begin(uop_buffer), std::end(uop_buffer), [etag](const auto& e) { return e.tag == etag; })) {
      return TRIG_RESIDENT;
    }
  }
  // group the trace's uops into aligned windows, in path order
  std::vector<std::vector<uint64_t>> groups;
  uint64_t last = 0;
  bool have_last = false;
  for (uint64_t raw : ips) {
    const uint64_t tag = raw >> dib_window_bits;
    if (have_last && tag == last) {
      groups.back().push_back(raw);
      continue;
    }
    have_last = true;
    last = tag;
    groups.push_back({raw});
  }
  if (alt_walk_filter) {
    // per-window residency filter: fetch/stage ONLY windows absent from both the
    // u-op cache and the staging buffer.  Redundant staging is the main source of
    // wasted line fetches, hogged walk slots, and unconsumed buffer entries.
    groups.erase(std::remove_if(std::begin(groups), std::end(groups),
                                [this](const auto& g) {
                                  const uint64_t tag = g.front() >> dib_window_bits;
                                  const bool resident = DIB.probe(champsim::address{g.front()})
                                                        || std::any_of(std::begin(uop_buffer), std::end(uop_buffer),
                                                                       [tag](const auto& e) { return e.tag == tag; });
                                  if (resident) {
                                    ++sim_stats.chain_filtered_windows;
                                  }
                                  return resident;
                                }),
                 std::end(groups));
    if (groups.empty()) {
      return TRIG_RESIDENT; // fully resident: nothing to walk (not a drop, not a trigger)
    }
  }
  if (chain_launch_conf > 0 && fill_store.conf_of(entry) == 0) {
    ++sim_stats.chain_conf_suppressed; // this trace's staged windows keep dying unused: hold the walk until it is re-captured
    return TRIG_CONF;
  }
  if (alt_walks.size() >= alt_walk_max) {
    ++sim_stats.alt_drops;
    return TRIG_CAP;
  }
  alt_walk w;
  w.entry = entry;
  w.ready = current_time + alt_walk_delay_cycles * clock_period;
  w.last_progress = current_time;
  w.probe_time = current_time;
  w.windows = std::move(groups);
  w.probe_id = probe_id;
  w.oracle = (chain_shadow_branch != 0 && probe_id > chain_shadow_branch) || chain_peek_probing; // probed during an unresolved misprediction
  if (w.oracle) {
    ++sim_stats.chain_shadow_walks;
  }
  w.wrong_path = wrong_path;
  w.rtt = rtt_probing && !wrong_path;
  w.recovery = rk_probing;
  w.vp = vp_probing;
  w.chain_hop = chain_hop_probing;
  if (wrong_path) {
    ++sim_stats.wp_walks;
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
      if (walk_resident_port != 0 && l1i->has_virtual_block(champsim::address{win.front()})) {
        ++sim_stats.walk_res_lines;
        if (walk_resident_port == 2) {
          w.lines_ready.insert(blk); // bound: resident bytes are there at once
        } else {
          w.lines_resident.push_back(win.front());
        }
        continue;
      }
      if (walk_resident_port != 0) {
        ++sim_stats.walk_miss_lines;
      }
      w.lines_pending.push_back(win.front());
    }
    // all line addresses are known from the manifest: issue up to two this cycle
    for (int n = 0; n < 2 && walk_pq == 0 && !w.lines_pending.empty() && walk_issue_available(w.lines_issued); ++n) {
      CacheBus::request_type line_pkt;
      line_pkt.v_address = champsim::address{w.lines_pending.front()};
      line_pkt.ip = champsim::address{w.lines_pending.front()};
      line_pkt.instr_id = 0;
      line_pkt.no_fill = alt_walk_nofill && !w.ucp; // SUCP walk bytes allocate in no cache level
      line_pkt.walker = (walk_l2_mshrs > 0 && !w.ucp);
      if (line_pkt.walker) {
        line_pkt.no_fill = true; // handed to the walk, not filled into the L1I (L2/LLC still allocate)
      }
      if (!L1I_bus.issue_read(line_pkt)) {
        break;
      }
      w.lines_pending.pop_front();
      ++sim_stats.alt_lines_issued;
      ++w.lines_issued;
      if (w.oracle) {
        ++sim_stats.chain_shadow_lines;
      }
      if (w.wrong_path) {
        ++sim_stats.wp_lines;
      }
      walk_issue_note();
    }
  }
  if (rk_probing) {
    ++sim_stats.rk_walks;
  }
  if (chain_hop_probing != 0) {
    ++sim_stats.chain_walks_hop[chain_hop_probing - 1];
  }
  if (vp_probing != 0) {
    ++sim_stats.vp_walks;
    if (vp_probing == 2) {
      ++sim_stats.vp_walks_mp;
    }
  }
  alt_walks.push_back(std::move(w));
  ++sim_stats.alt_triggers;
  return TRIG_LAUNCHED;
}

// trace_peek_diag: could a table keyed by what the predictor knows at a branch --
// its pc and predicted direction/target, optionally + global history or the
// recent trace sequence -- name the trace that follows when that branch turns out
// mispredicted?  Called once per branch, in program order, at its prediction (the
// cursor, or fetch when the cursor had not reached it).  Every prediction whose
// key has an entry "fires" (= a walk a real design would launch); the truth for a
// misprediction comes later from the cursor's own probes (peek_diag_probe).
namespace
{
uint64_t peek_mix(uint64_t x)
{
  x ^= x >> 33;
  x *= 0xff51afd7ed558ccdULL;
  x ^= x >> 33;
  x *= 0xc4ceb9fe1a85ec53ULL;
  x ^= x >> 33;
  return x;
}
} // namespace

void O3_CPU::peek_diag_branch(const ooo_model_instr& instr, bool mispredicted)
{
  if (!peek_diag || fill_mode != fill_mode_type::CHAIN || wrong_path_level < 1 || !instr.is_branch) {
    return;
  }
  const bool cond = (instr.branch == BRANCH_CONDITIONAL) || (instr.branch == BRANCH_OTHER);
  const uint64_t pc = instr.ip.to<uint64_t>();
  // what was predicted: a direction for a conditional, a target otherwise
  const uint64_t sig = cond ? (instr.branch_prediction ? 1 : 2) : (instr.branch_prediction ? instr.pred_btb_target : 3);
  const uint64_t base = peek_mix(pc ^ peek_mix(sig + 0x9e3779b97f4a7c15ULL));
  std::array<uint64_t, PEEK_KEYS> keys{};
  keys[0] = base;
  keys[1] = peek_mix(base ^ ((pdiag_ghr & 0xffULL) | (uint64_t{1} << 60)));
  keys[2] = peek_mix(base ^ ((pdiag_ghr & 0xffffULL) | (uint64_t{2} << 60)));
  keys[3] = peek_mix(base ^ ((pdiag_ghr & 0xffffffffULL) | (uint64_t{3} << 60)));
  keys[4] = peek_mix(base ^ peek_mix(pdiag_ghr + 4));
  uint64_t th = 0;
  for (std::size_t n = 0; n < pdiag_th.size(); ++n) {
    th = peek_mix(th ^ pdiag_th[n]);
    if (n == 0) {
      keys[5] = peek_mix(base ^ th ^ 5);
    } else if (n == 1) {
      keys[6] = peek_mix(base ^ th ^ 6);
    } else if (n == 3) {
      keys[7] = peek_mix(base ^ th ^ 7);
    }
  }
  std::array<uint64_t, PEEK_KEYS> pred{};
  for (std::size_t i = 0; i < PEEK_KEYS; ++i) {
    if (const auto it = pdiag_table[i].find(keys[i]); it != pdiag_table[i].end()) {
      pred[i] = it->second;
      ++sim_stats.pdiag_fires[i];
    }
  }
  if (mispredicted) {
    if (pdiag_pending) {
      peek_diag_finalize(0); // the previous misprediction met no trace before this one
    }
    pdiag_pending = true;
    pdiag_pend_id = instr.instr_id;
    pdiag_pend_key = keys;
    pdiag_pend_pred = pred;
    ++sim_stats.pdiag_mispredicts;
  }
  if (cond) {
    pdiag_ghr = (pdiag_ghr << 1) | (instr.branch_taken ? 1 : 0); // the real outcome: history is repaired at recovery
  }
}

// every cursor/enqueue probe, in program order: the first trace hit past a pending
// misprediction is its truth (what level 0 walked early); a hit also extends the
// trace history (once per instruction -- the cursor re-probes after a reset)
void O3_CPU::peek_diag_probe(uint64_t instr_id, uint64_t hit_entry)
{
  if (!peek_diag) {
    return;
  }
  if (pdiag_pending && instr_id > pdiag_pend_id) {
    if (hit_entry != 0) {
      peek_diag_finalize(hit_entry);
    } else if (instr_id > pdiag_pend_id + PEEK_TRUTH_WINDOW) {
      peek_diag_finalize(0);
    }
  }
  if (hit_entry != 0 && instr_id > pdiag_th_last_id) {
    for (std::size_t n = pdiag_th.size() - 1; n > 0; --n) {
      pdiag_th[n] = pdiag_th[n - 1];
    }
    pdiag_th[0] = hit_entry;
    pdiag_th_last_id = instr_id;
  }
}

// score each key's guess for the pending misprediction, then learn what followed
void O3_CPU::peek_diag_finalize(uint64_t truth)
{
  pdiag_pending = false;
  if (truth != 0) {
    ++sim_stats.pdiag_with_trace;
  }
  for (std::size_t i = 0; i < PEEK_KEYS; ++i) {
    if (pdiag_pend_pred[i] != 0) {
      if (truth != 0 && pdiag_pend_pred[i] == truth) {
        ++sim_stats.pdiag_useful[i];
      } else {
        ++sim_stats.pdiag_wrong[i];
      }
    }
    if (truth != 0) {
      pdiag_table[i][pdiag_pend_key[i]] = truth;
    }
  }
}

// trace_recovery_key: the key a prediction forms -- branch pc, what was predicted
// (direction, or target for other branches), optionally global history -- with
// bit 63 set so it never collides with a first-pc key in the shared store
uint64_t O3_CPU::recovery_key(const ooo_model_instr& instr) const
{
  const bool cond = (instr.branch == BRANCH_CONDITIONAL) || (instr.branch == BRANCH_OTHER);
  const uint64_t sig = cond ? (instr.branch_prediction ? 1 : 2) : (instr.branch_prediction ? instr.pred_btb_target : 3);
  uint64_t key = peek_mix(instr.ip.to<uint64_t>() ^ peek_mix(sig + 0x9e3779b97f4a7c15ULL));
  if (rk_ghr_bits > 0) {
    const uint64_t mask = (rk_ghr_bits >= 64) ? ~uint64_t{0} : ((uint64_t{1} << rk_ghr_bits) - 1);
    key = peek_mix(key ^ peek_mix((rk_ghr & mask) + static_cast<uint64_t>(rk_ghr_bits)));
  }
  return key | (uint64_t{1} << 63);
}

// at every branch prediction, in program order (the cursor, or fetch for a branch
// the cursor had not reached): probe the recovery key and walk its trace at once;
// a misprediction arms the filing of the trace that will follow it
void O3_CPU::recovery_on_predict(const ooo_model_instr& instr, bool mispredicted)
{
  if (!rk_enable || fill_mode != fill_mode_type::CHAIN || wrong_path_level < 1 || !instr.is_branch || (warmup && !trace_warmup_train)) {
    return;
  }
  const uint64_t key = recovery_key(instr);
  if (!rk_h2p || instr.pred_h2p) {
    ++sim_stats.rk_probes;
    if (const auto* ips = fill_store.lookup_entry(key); ips != nullptr && !ips->empty()) {
      ++sim_stats.rk_hits;
      rk_probing = true;
      do_chain_trigger(*ips, instr.instr_id);
      rk_probing = false;
    }
  }
  if (mispredicted) {
    rk_pending = true; // a pending filing with no trace yet is dropped
    rk_pend_id = instr.instr_id;
    rk_pend_key = key;
    ++sim_stats.rk_mispredicts;
  }
  if ((instr.branch == BRANCH_CONDITIONAL) || (instr.branch == BRANCH_OTHER)) {
    rk_ghr = (rk_ghr << 1) | (instr.branch_taken ? 1 : 0); // the real outcome: history is repaired at recovery
  }
}

// every cursor/enqueue probe, in program order: the first stored trace hit past a
// pending misprediction (within 192 instructions) is filed under its recovery key
void O3_CPU::recovery_learn(uint64_t instr_id, const std::vector<uint64_t>* ips)
{
  if (!rk_pending || instr_id <= rk_pend_id) {
    return;
  }
  if (ips != nullptr) {
    const std::vector<uint64_t> trace = *ips; // the store may move entries on insert
    fill_store.insert(rk_pend_key, trace);
    ++sim_stats.rk_filed;
    rk_pending = false;
  } else if (instr_id > rk_pend_id + PEEK_TRUTH_WINDOW) {
    rk_pending = false; // no stored trace followed closely enough
  }
}

// trace_vpred: at every branch prediction, in program order -- look the value
// predictor up with what the predictor knows now and walk the trace it names
// (inc/trace_vpred.h).  mispredicted only labels the statistics.
void O3_CPU::vpred_on_predict(const ooo_model_instr& instr, bool mispredicted)
{
  if (!vp_enable || fill_mode != fill_mode_type::CHAIN || wrong_path_level < 1 || !instr.is_branch || (warmup && !trace_warmup_train)) {
    return;
  }
  const bool cond = (instr.branch == BRANCH_CONDITIONAL) || (instr.branch == BRANCH_OTHER);
  const uint64_t sig = cond ? (instr.branch_prediction ? 1 : 2) : (instr.branch_prediction ? instr.pred_btb_target : 3);
  if (const uint64_t trace_pc = vpred.predict(instr.ip.to<uint64_t>(), sig, vp_ghr, mispredicted, sim_stats); trace_pc != 0) {
    if (const auto* ips = fill_store.lookup_entry(trace_pc); ips != nullptr && !ips->empty()) {
      ++sim_stats.vp_store_hits;
      vp_probing = mispredicted ? 2 : 1;
      do_chain_trigger(*ips, instr.instr_id);
      vp_probing = 0;
    }
  }
  if (cond) {
    vp_ghr = (vp_ghr << 1) | (instr.branch_taken ? 1 : 0); // the real outcome: history is repaired at recovery
  }
}

// u-op-cache miss-cost profile, end of run: rank the windows that missed by the
// empty-ROB cycles charged to them; report how much the costliest 1K..8K hold,
// how much the most frequently missing 1K..8K hold (cost is not frequency), and
// how much of it lies in windows the stall segmenter's traces contain (does the
// segmentation heuristic capture the costly misses?).  Optionally dump the
// ranking (window cost misses traced) for the oracle pass (PROMETHEUS_UOP_ORACLE).
void O3_CPU::miss_profile_finalize()
{
  if (!miss_profile || miss_sites.empty()) {
    return;
  }
  std::vector<std::pair<uint64_t, miss_site>> sites(miss_sites.begin(), miss_sites.end());
  std::unordered_set<uint64_t> traced;
  if (trace_stall_enable) {
    for (const auto& t : stall.get_traces()) {
      for (uint64_t ip : t.ips) {
        traced.insert(ip >> miss_window_bits);
      }
    }
  }
  sim_stats.mp_windows = sites.size();
  sim_stats.mp_misses = 0;
  sim_stats.mp_cost = 0;
  sim_stats.mp_traced_all = 0;
  for (const auto& [w, s] : sites) {
    sim_stats.mp_misses += s.misses;
    sim_stats.mp_cost += s.cost;
    if (traced.count(w) != 0) {
      sim_stats.mp_traced_all += s.cost;
    }
  }
  sim_stats.mp_other = miss_prof_other;
  static constexpr std::array<std::size_t, 4> TOP = {1024, 2048, 4096, 8192};
  const auto cumulate = [&](std::array<uint64_t, 4>& out, std::array<uint64_t, 4>* traced_out) {
    uint64_t run = 0, run_traced = 0;
    std::size_t bi = 0;
    for (std::size_t k = 0; k < sites.size() && bi < TOP.size(); ++k) {
      run += sites[k].second.cost;
      if (traced_out != nullptr && traced.count(sites[k].first) != 0) {
        run_traced += sites[k].second.cost;
      }
      while (bi < TOP.size() && (k + 1) >= TOP[bi]) {
        out[bi] = run;
        if (traced_out != nullptr) {
          (*traced_out)[bi] = run_traced;
        }
        ++bi;
      }
    }
    for (; bi < TOP.size(); ++bi) { // fewer windows than the bucket: all of them
      out[bi] = run;
      if (traced_out != nullptr) {
        (*traced_out)[bi] = run_traced;
      }
    }
  };
  std::sort(sites.begin(), sites.end(), [](const auto& a, const auto& b) { return a.second.misses > b.second.misses; });
  cumulate(sim_stats.mp_cost_freq, nullptr);
  std::sort(sites.begin(), sites.end(), [](const auto& a, const auto& b) { return a.second.cost > b.second.cost; });
  cumulate(sim_stats.mp_cost_top, &sim_stats.mp_traced_top);
  if (!miss_dump_path.empty()) {
    std::ofstream out(miss_dump_path);
    for (const auto& [w, s] : sites) {
      if (s.cost == 0) {
        break; // ranked by cost: the rest never stalled the core
      }
      out << w << ' ' << s.cost << ' ' << s.misses << ' ' << (traced.count(w) != 0 ? 1 : 0) << '\n';
    }
  }
  if (!miss_dump_path.empty() && trace_stall_enable) {
    // the segmenter's traces ranked by cost, one per line: cost, then the trace's
    // windows -- the trace-oracle pass makes every window of the first N always hit
    const auto& tr = stall.get_traces();
    std::vector<std::size_t> order(tr.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return tr[a].cost > tr[b].cost; });
    std::ofstream out(miss_dump_path + ".traces");
    for (std::size_t i : order) {
      if (tr[i].cost == 0) {
        break;
      }
      out << tr[i].cost;
      uint64_t last = std::numeric_limits<uint64_t>::max();
      for (uint64_t ip : tr[i].ips) {
        const uint64_t w = ip >> miss_window_bits;
        if (w != last) {
          out << ' ' << w;
          last = w;
        }
      }
      out << '\n';
    }
  }
}

// ftq_fdip: the predictor has put the block holding `ip` in the FTQ; prefetch its
// L1I line unless the u-op cache holds the window (checked once per window, issued
// once per line).  wrong_path: a block the predictor fetched past a mispredicted
// branch (trace_wrong_path ucp2) -- the prefetch is real traffic, usually wasted.
// costly-window table (fdip_predecode 2): set-associative, 8-way, 12-bit partial tags, LRU
bool O3_CPU::cwt_find(uint64_t window, bool touch)
{
  const std::size_t sets = cwt.size() / 8;
  if (sets == 0) {
    return false;
  }
  const std::size_t set = static_cast<std::size_t>(window % sets);
  const uint64_t tag = (window / sets) & 0xfff;
  auto begin = std::next(std::begin(cwt), static_cast<std::ptrdiff_t>(set * 8));
  auto end = std::next(begin, 8);
  auto it = std::find_if(begin, end, [tag](const auto& e) { return e.valid && e.tag == tag; });
  if (it == end) {
    return false;
  }
  if (touch) {
    it->lru = ++cwt_clock;
  }
  return true;
}

void O3_CPU::cwt_mark(uint64_t window)
{
  if (cwt_find(window, true)) {
    return;
  }
  const std::size_t sets = cwt.size() / 8;
  const std::size_t set = static_cast<std::size_t>(window % sets);
  auto begin = std::next(std::begin(cwt), static_cast<std::ptrdiff_t>(set * 8));
  auto end = std::next(begin, 8);
  auto victim = std::find_if(begin, end, [](const auto& e) { return !e.valid; });
  if (victim == end) {
    victim = std::min_element(begin, end, [](const auto& a, const auto& b) { return a.lru < b.lru; });
  }
  victim->valid = true;
  victim->tag = (window / sets) & 0xfff;
  victim->lru = ++cwt_clock;
  ++sim_stats.cwt_marks;
}

// is this window costly?  mode 2: the costly-window table; modes 1/3: it lies in a stored SUCP trace
bool O3_CPU::window_costly(uint64_t ip)
{
  if (fdip_predecode == 2) {
    return cwt_find(ip >> dib_window_bits, true);
  }
  return fill_store.peek_window(ip);
}

// FDIP-triggered pre-decode of a costly window the predictor just reached: no read of its own --
// it waits for FDIP's line in the L1I, pre-decodes in the decode slots demand leaves, and
// installs into the u-op cache as a prefetch (or stages).  Skips windows already cached,
// staged or being walked.
void O3_CPU::fdip_predecode_window(const std::vector<uint64_t>& ips, bool wrong_path)
{
  if (ips.empty() || (warmup && !trace_warmup_train)) {
    return;
  }
  ++sim_stats.fdip_pd_windows;
  const uint64_t rawip = ips.front();
  const uint64_t wtag = rawip >> dib_window_bits;
  if (DIB.probe(champsim::address{rawip}) || alt_window_pending(rawip)
      || std::any_of(std::begin(uop_buffer), std::end(uop_buffer), [wtag](const auto& e) { return e.tag == wtag; })) {
    return;
  }
  if (!window_costly(rawip)) {
    return;
  }
  ++sim_stats.fdip_pd_costly;
  if (alt_walks.size() >= alt_walk_max) {
    ++sim_stats.fdip_pd_dropped;
    return;
  }
  alt_walk w;
  w.entry = rawip;
  w.fdip_pd = true;
  w.direct_install = (fdip_predecode_target == 0);
  w.wrong_path = wrong_path;
  w.ready = current_time;
  w.last_progress = current_time;
  w.probe_time = current_time;
  w.windows.push_back(ips);
  alt_walks.push_back(std::move(w));
  ++sim_stats.fdip_pd_launched;
}

// may this walk pre-decode the window holding ip?  FDIP-riding pre-decodes need the line in the
// L1I; real-L1I walks need their own read to have returned
bool O3_CPU::walk_line_ready(const alt_walk& w, uint64_t ip) const
{
  if (w.fdip_pd) {
    return l1i->has_virtual_block(champsim::address{ip});
  }
  return !alt_walk_l1i || w.lines_ready.count(champsim::block_number{champsim::address{ip}}.to<uint64_t>()) != 0;
}

// EMISSARY: mark the line high-priority in the L1I and in the L2 (the bit survives while the
// line stays in either cache)
void O3_CPU::emissary_mark(champsim::address ip)
{
  champsim::address paddr{};
  if (l1i->mark_priority_virtual(ip, paddr)) {
    ++sim_stats.emissary_mark_events;
    if (l2c != nullptr) {
      l2c->mark_priority_physical(paddr);
    }
  }
}

// resteer prefetcher table: set-associative, 10-bit partial tags, LRU
O3_CPU::rp_entry* O3_CPU::rp_find(uint64_t key, bool allocate)
{
  const std::size_t set = static_cast<std::size_t>(key % rp_sets);
  const uint64_t tag = (key / rp_sets) & 0x3ff;
  auto begin = std::next(std::begin(rp_table), static_cast<std::ptrdiff_t>(set * rp_ways));
  auto end = std::next(begin, static_cast<std::ptrdiff_t>(rp_ways));
  if (auto it = std::find_if(begin, end, [tag](const auto& e) { return e.valid && e.tag == tag; }); it != end) {
    it->lru = ++rp_clock;
    return &*it;
  }
  if (!allocate) {
    return nullptr;
  }
  auto victim = std::find_if(begin, end, [](const auto& e) { return !e.valid; });
  if (victim == end) {
    victim = std::min_element(begin, end, [](const auto& a, const auto& b) { return a.lru < b.lru; });
  }
  victim->valid = true;
  victim->tag = tag;
  victim->lru = ++rp_clock;
  victim->targets.clear();
  return &*victim;
}

// file a line: folded into a target's mask when it is one of the 4 lines after it; else a new
// target, replacing the oldest when the entry is full
void O3_CPU::rp_add_target(rp_entry& e, uint64_t line)
{
  for (auto& [base, mask] : e.targets) {
    if (line == base) {
      return;
    }
    if (line > base && line - base <= 4) {
      mask = static_cast<uint8_t>(mask | (1u << (line - base - 1)));
      return;
    }
  }
  if (e.targets.size() >= rp_targets) {
    e.targets.erase(std::begin(e.targets));
  }
  e.targets.emplace_back(line, uint8_t{0});
}

void O3_CPU::rp_open(uint64_t branch_ip)
{
  rp_win_key = champsim::block_number{champsim::address{branch_ip}}.to<uint64_t>();
  rp_left = rp_window;
  if (rp_mode == 2) {
    rp_windows.push_back(rp_win_state{rp_win_key, false, {}});
    if (rp_windows.size() > 8) {
      rp_windows.pop_front();
    }
  }
}

O3_CPU::rp_win_state* O3_CPU::rp_win(uint64_t key)
{
  auto it = std::find_if(std::rbegin(rp_windows), std::rend(rp_windows), [key](const auto& w) { return w.key == key; });
  return (it == std::rend(rp_windows)) ? nullptr : &*it;
}

// SUCP path mode: every L1I-missed line of a resteer window that starved the backend is filed
// under its trigger; lines that arrive before the first starvation wait in the window
void O3_CPU::rp_path_line(uint64_t key, uint64_t line)
{
  rp_win_state* w = rp_win(key);
  if (w == nullptr) {
    return;
  }
  if (w->starved) {
    if (rp_entry* e = rp_find(key, true); e != nullptr) {
      rp_add_target(*e, line);
    }
  } else if (w->lines.size() < 64 && (w->lines.empty() || w->lines.back() != line)) {
    w->lines.push_back(line);
  }
}

// ...the first starvation files the path: it replaces the entry's targets (the latest path wins)
void O3_CPU::rp_path_starved(uint64_t key)
{
  rp_win_state* w = rp_win(key);
  if (w == nullptr || w->starved) {
    return;
  }
  w->starved = true;
  if (rp_entry* e = rp_find(key, true); e != nullptr) {
    e->targets.clear();
    for (uint64_t line : w->lines) {
      rp_add_target(*e, line);
    }
    ++sim_stats.rp_records;
  }
  w->lines.clear();
}

// the predictor reached a new line: if it is a filed trigger, prefetch its targets into the L1I
void O3_CPU::rp_lookup(uint64_t ip)
{
  const uint64_t key = champsim::block_number{champsim::address{ip}}.to<uint64_t>();
  if (key == rp_last_lookup) {
    return;
  }
  rp_last_lookup = key;
  ++sim_stats.rp_lookups;
  const rp_entry* e = rp_find(key, false);
  if (e == nullptr || e->targets.empty()) {
    return;
  }
  ++sim_stats.rp_hits;
  for (const auto& [base, mask] : e->targets) {
    for (unsigned k = 0; k <= 4; ++k) {
      if (k > 0 && (mask & (1u << (k - 1))) == 0) {
        continue;
      }
      const champsim::address line_addr{champsim::block_number{base + k}};
      if (l1i->prefetch_line(line_addr, true, 0)) {
        ++sim_stats.rp_prefetches;
      } else {
        ++sim_stats.rp_dropped;
      }
    }
  }
}

void O3_CPU::fdip_note(uint64_t ip, bool wrong_path)
{
  const uint64_t win = ip >> dib_window_bits;
  const uint64_t line = champsim::block_number{champsim::address{ip}}.to<uint64_t>();
  if (win == fdip_last_window) {
    return;
  }
  fdip_last_window = win;
  // ftq_fdip_uop_check 1: a window the u-op cache holds needs no bytes; 0: FDIP looks at the L1I only
  if (line == fdip_last_line || (ftq_fdip_uop_check && DIB.probe(champsim::address{ip}))) {
    return;
  }
  fdip_last_line = line;
  if (ftq_fdip_filter && l1i->has_virtual_block(champsim::address{ip})) {
    ++sim_stats.fdip_filtered; // the line is already in the L1I: no prefetch (tag-filtered FDIP)
    return;
  }
  if (l1i->prefetch_line(champsim::address{ip}, true, 0)) {
    ++sim_stats.fdip_issued;
    if (wrong_path) {
      ++sim_stats.fdip_wp_issued;
    }
  } else {
    ++sim_stats.fdip_dropped;
  }
}

// trace_trigger_diag: the cursor found `probed` to be a stored trace's entry.  Keep
// the best outcome over its probes (a launch over a dedup over no walk) and the
// cycle it was first reached -- the lead demand gave the walk.
void O3_CPU::trigger_diag_probe(ooo_model_instr& probed, uint8_t result)
{
  if (!trigger_diag) {
    return;
  }
  if (!warmup) {
    ++sim_stats.td_probes[result];
  }
  if (result > probed.trig_probe) {
    probed.trig_probe = result;
    probed.trig_probe_cycle = static_cast<uint64_t>(current_time.time_since_epoch() / clock_period);
  }
}

// trace_trigger_diag: classify this fetch.  Demand reaching a stored trace's entry
// opens an occurrence (that trace's windows + what the cursor did at the entry);
// demand leaving those windows closes it.  Every u-op-cache miss gets a class,
// which dispatch-starve cycles are later charged to (see dispatch_instruction).
void O3_CPU::trigger_diag_check(ooo_model_instr& instr, bool dib_hit, bool buf_hit, bool staged_evicted)
{
  const uint64_t rawip = instr.ip.to<uint64_t>();
  const uint64_t wtag = rawip >> dib_window_bits;
  const auto in_occ = [this, wtag] { return std::find(std::begin(td_occ_wtags), std::end(td_occ_wtags), wtag) != std::end(td_occ_wtags); };
  bool at_entry = false;
  if (const auto* ips = fill_store.peek_ips(rawip); ips != nullptr && !ips->empty()) {
    at_entry = true;
    td_occ_entry = rawip;
    td_occ_probe = instr.trig_probe;
    td_occ_wtags.clear();
    for (uint64_t v : *ips) {
      if (td_occ_wtags.empty() || td_occ_wtags.back() != (v >> dib_window_bits)) {
        td_occ_wtags.push_back(v >> dib_window_bits);
      }
    }
  } else if (td_occ_entry != 0 && !in_occ()) {
    td_occ_entry = 0; // demand left the trace's windows: the occurrence is over
    td_occ_wtags.clear();
  }
  uint8_t cls = TD_UNCLASSIFIED;
  if (!dib_hit && !buf_hit) {
    if (td_occ_entry != 0 && in_occ()) {
      if (alt_window_pending(rawip)) {
        cls = TD_LATE;
      } else if (td_occ_probe == TRIG_NONE) {
        cls = TD_NOT_PROBED;
      } else if (td_occ_probe == TRIG_CONF || td_occ_probe == TRIG_CAP) {
        cls = TD_DROPPED;
      } else if (td_occ_probe == TRIG_RESIDENT) {
        cls = TD_RESIDENT_AT_PROBE;
      } else {
        cls = staged_evicted ? TD_STAGED_EVICTED : TD_WALKED_GONE;
      }
    } else if (fill_store.peek_window(rawip)) {
      cls = TD_SIDE_DOOR;
    } else if (trace_stall_enable && stall.covers(rawip)) {
      cls = TD_NOT_STORED;
    } else if (!trace_stall_enable) {
      cls = TD_UNCOVERED;
    } else if (td_seen_windows.count(wtag) == 0) {
      cls = TD_COLD;
    } else if (const uint32_t n = stall.costly_count(wtag); stall.past_depth_now() || (n == 0 && stall.costly_past_depth(wtag))) {
      cls = TD_PAST_DEPTH;
    } else {
      cls = (n == 0) ? TD_NEVER_COSTLY : (n == 1) ? TD_COSTLY_ONCE : TD_COSTLY_REPEAT;
    }
    ++sim_stats.td_miss[cls];
    if (in_recovery) {
      ++sim_stats.td_miss_rec[cls];
    }
    if (!td_in_run) {
      td_run_head = cls; // the first miss after a hit: the stretch's head
    }
  }
  td_in_run = !dib_hit && !buf_hit;
  instr.trig_class = cls;
  instr.trig_head = td_in_run ? td_run_head : TD_UNCLASSIFIED;
  if (at_entry) {
    ++sim_stats.td_arrivals;
    if (dib_hit) {
      ++sim_stats.td_arr_resident;
    } else if (buf_hit) {
      ++sim_stats.td_arr_staged;
    } else {
      ++sim_stats.td_arr_miss[cls];
    }
    if (td_occ_probe == TRIG_LAUNCHED || td_occ_probe == TRIG_DEDUP) {
      const uint64_t now = static_cast<uint64_t>(current_time.time_since_epoch() / clock_period);
      const uint64_t lead = (now > instr.trig_probe_cycle) ? now - instr.trig_probe_cycle : 0;
      const std::size_t b = (lead <= 8) ? 0 : (lead <= 16) ? 1 : (lead <= 32) ? 2 : (lead <= 64) ? 3 : (lead <= 128) ? 4 : 5;
      ++((dib_hit || buf_hit) ? sim_stats.td_lead_served : sim_stats.td_lead_missed)[b];
    }
  }
}

// trace_chain_depth: after a trace hit, follow its successor pointers and walk
// each successor the store holds, up to chain_depth hops (a cycle ends the chain)
void O3_CPU::chain_follow(uint64_t entry, uint64_t probe_id)
{
  if (chain_depth <= 0 || fill_mode != fill_mode_type::CHAIN || (warmup && !trace_warmup_train)) {
    return;
  }
  std::array<uint64_t, 16> seen{};
  std::size_t n_seen = 0;
  seen[n_seen++] = entry;
  uint64_t cur = entry;
  for (int hop = 1; hop <= chain_depth; ++hop) {
    ++sim_stats.chain_hops;
    const uint64_t next = fill_store.next_of(cur);
    if (next == 0 || std::find(seen.begin(), seen.begin() + static_cast<long>(n_seen), next) != seen.begin() + static_cast<long>(n_seen)) {
      return;
    }
    const auto* ips = fill_store.lookup_entry(next);
    if (ips == nullptr || ips->empty()) {
      return; // the successor is not stored: the chain ends (the cursor keeps probing the FTQ)
    }
    ++sim_stats.chain_found;
    chain_hop_probing = static_cast<uint8_t>(std::min(hop, 4));
    do_chain_trigger(*ips, probe_id);
    chain_hop_probing = 0;
    if (n_seen < seen.size()) {
      seen[n_seen++] = next;
    }
    cur = next;
  }
}

// every costly stretch the segmenter judged in its last call trains the
// prediction its first instruction followed
void O3_CPU::vpred_train_from_stall()
{
  // trace_chain_depth: each costly stretch becomes the successor of the previous one
  if (chain_depth > 0 && fill_mode == fill_mode_type::CHAIN) {
    for (const auto& ev : stall.costly_events()) {
      if (chain_prev_valid && ev.start_id > chain_prev_end_id && (chain_window == 0 || ev.start_id - chain_prev_end_id - 1 <= chain_window)
          && ev.first_pc != chain_prev_first && fill_store.set_next(chain_prev_first, ev.first_pc)) {
        ++sim_stats.chain_links;
      }
      chain_prev_valid = true;
      chain_prev_first = ev.first_pc;
      chain_prev_end_id = ev.end_id;
    }
  }
  if (!vp_enable) {
    return;
  }
  for (const auto& ev : stall.costly_events()) {
    vpred.train(ev.ctx, ev.first_pc, sim_stats);
  }
}

// A branch just mispredicted at prediction.  The instruction supply past it is
// the correct path, which a real FTQ would not hold until resolution: walks the
// deep probe launched from there (and the windows they staged) used knowledge
// the hardware does not have.  Legacy (trace_probe_squash 0) keeps them and only
// flags them, so their later buffer hits are counted as oracle benefit; mode 1
// squashes them and resets the cursor, which stays frozen until resolution.
// Lines those walks already brought into the L1I stay (a real FTQ would have
// fetched wrong-path lines instead): counted, not undone.
void O3_CPU::chain_mispredict_shadow(uint64_t branch_id)
{
  chain_shadow_branch = branch_id;
  ++sim_stats.chain_shadow_mispredicts;
  if (chain_probe_squash == 1) {
    chain_probe_cursor = 0;
    alt_walks.erase(std::remove_if(std::begin(alt_walks), std::end(alt_walks),
                                   [this, branch_id](const auto& w) {
                                     if (w.ucp || w.probe_id <= branch_id) {
                                       return false;
                                     }
                                     ++sim_stats.chain_shadow_walks;
                                     sim_stats.chain_shadow_lines += w.lines_issued;
                                     return true;
                                   }),
                    std::end(alt_walks));
    uop_buffer.erase(std::remove_if(std::begin(uop_buffer), std::end(uop_buffer),
                                    [this, branch_id](const auto& e) {
                                      if (e.served || e.probe_id <= branch_id) {
                                        return false;
                                      }
                                      ++sim_stats.chain_shadow_windows;
                                      return true;
                                    }),
                     std::end(uop_buffer));
    return;
  }
  for (auto& w : alt_walks) {
    if (!w.ucp && !w.oracle && w.probe_id > branch_id) {
      w.oracle = true;
      ++sim_stats.chain_shadow_walks;
      sim_stats.chain_shadow_lines += w.lines_issued;
    }
  }
  for (auto& e : uop_buffer) {
    if (!e.served && !e.oracle && e.probe_id > branch_id) {
      e.oracle = true;
      ++sim_stats.chain_shadow_windows;
    }
  }
}

// ===== trace_wrong_path >= 2: a synthetic wrong path past a mispredicted branch =====
// A trace holds only the correct path, so the wrong path is generated the way the
// UCP_ISCA24 artifact does it: from the branch's predicted continuation, follow
// BTB targets (side-effect-free probe; returns from a copy of the RAS), take
// conditional directions from the main predictor (lookup only, untrained -- its
// history does not see wrong-path branches), and step +4 otherwise (exact on ARM,
// an approximation on x86 where instruction lengths are unknown).  The predictor
// side (wp_generate) runs ahead of fetch by up to the FTQ lead; fetch (wp_fetch)
// follows it from the branch until resolution (wp_resolve).  The wrong path does
// not fetch lines or fill the u-op cache (as in the artifact).
void O3_CPU::wp_begin(const ooo_model_instr& branch)
{
  wp_windows_fetched.clear();
  wp_branch_id = branch.instr_id;
  wp_active = true;
  wp_fetching = false;
  wp_decoders_busy = false;
  wp_ftq.clear();
  wp_issue_idx = 0;
  wp_consumed = 0;
  wp_lines_requested.clear();
  wp_lines_ready.clear();
  // the predicted continuation: the predicted target if predicted taken, else the fall-through
  wp_next = (branch.branch_prediction && branch.pred_btb_target != 0) ? branch.pred_btb_target : branch.ip.to<uint64_t>() + WP_STEP;
  wp_gen_stopped = false;
  if (wp_exact && !(branch.branch_prediction && branch.pred_btb_target != 0)) {
    // ucp2: the fall-through is the branch's learned successor; none = never seen falling through
    if (const auto it = code_next.find(branch.ip.to<uint64_t>()); it != code_next.end()) {
      wp_next = it->second;
    } else {
      wp_gen_stopped = true;
      ++sim_stats.wp_stops;
    }
  }
  wp_ras = ucp::btb_ras_snapshot ? ucp::btb_ras_snapshot() : std::vector<uint64_t>{};
  ++sim_stats.wp_shadows;
}

void O3_CPU::wp_generate()
{
  // how far the predictor runs ahead of fetch: the deep-probe lead when the cursor
  // models the FTQ, otherwise one fetch group (the predictor is at fetch)
  const auto fetch_width = static_cast<std::size_t>(champsim::to_underlying(FETCH_WIDTH));
  const std::size_t cursor_lead = (fill_mode == fill_mode_type::CHAIN && chain_probe_ahead > 0) ? static_cast<std::size_t>(chain_probe_ahead) : 0;
  std::size_t lead = std::max(fetch_width, cursor_lead);
  if (wrong_path_level >= 3) {
    // the wrong path fills the frontend queue behind the branch as the correct path would:
    // the IFETCH slots the branch's older instructions leave free, plus the cursor lead
    const std::size_t ifetch_free = (IFETCH_BUFFER_SIZE > std::size(IFETCH_BUFFER)) ? IFETCH_BUFFER_SIZE - std::size(IFETCH_BUFFER) : 0;
    lead = std::max(fetch_width, ifetch_free + cursor_lead);
  }
  // decoupled front end (fetch_buffer_blocks): the wrong path shares the one FTQ with
  // the correct-path blocks still queued ahead of the branch, and the predictor
  // covers as many blocks per cycle on it as on the correct path
  const bool ftq_model = ((fetch_buffer_blocks > 0 || ftq_shared_wp) && cursor_lead > 0);
  int gen_blocks = 1;
  if (ftq_model) {
    const std::size_t queued = std::min(cursor_lead, chain_probe_cursor);
    lead = std::max(fetch_width, cursor_lead - queued);
    gen_blocks = std::max(1, chain_probe_blocks);
  }
  if (wp_exact) {
    // ucp2: walk real instructions -- the BTB identifies branches at their exact
    // addresses, the predictor gives directions (history not updated), anything
    // else continues to its learned successor; one fetch block per cycle
    int block_br = 0;
    int blocks_done = 0;
    for (champsim::bandwidth gen_bw{champsim::bandwidth::maximum_type{static_cast<long>(FETCH_WIDTH) * gen_blocks}};
         gen_bw.has_remaining() && wp_ftq.size() < lead && !wp_gen_stopped; gen_bw.consume()) {
      const uint64_t addr = wp_next;
      wp_entry e{addr};
      ++sim_stats.wp_addresses;
      if (ftq_fdip) {
        fdip_note(addr, true); // the wrong-path block enters the FTQ: FDIP prefetches it like any other
      }
      if (rp_mode != 0) {
        rp_lookup(addr); // ...and so does the resteer prefetcher's table lookup
      }
      if (ftq_fdip && fdip_predecode != 0) {
        // ...and FDIP-triggered pre-decode, one window at a time
        const uint64_t win = addr >> dib_window_bits;
        if (win != wp_pd_window && !wp_pd_ips.empty()) {
          fdip_predecode_window(wp_pd_ips, true);
          wp_pd_ips.clear();
        }
        wp_pd_window = win;
        wp_pd_ips.push_back(addr);
      }
      if (ucp_enable && ucp_btb_banks > 0) {
        ucp_note_demand_bank(addr); // the predictor's BTB access holds the bank, as on the correct path
      }
      const auto seq_it = code_next.find(addr);
      uint64_t next = (seq_it != code_next.end()) ? seq_it->second : 0; // 0 = code never seen executing
      if (ucp::btb_probe) {
        if (const auto pb = ucp::btb_probe(addr); pb.has_value()) {
          e.branch = true;
          e.taken = true;
          uint64_t tgt = pb->target;
          if (pb->conditional) {
            e.taken = impl_predict_branch(champsim::address{addr}, champsim::address{tgt}, false, BRANCH_CONDITIONAL);
            if (ucp_enable && ucp::bp_last_h2p) {
              wp_ucp_trigger(addr, e.taken, tgt); // UCP fires on a hard-to-predict wrong-path branch like on any other
            }
          }
          if (rtt_wrong_path && (rtt_mode == 2 || rtt_mode == 3 || rtt_mode == 5) && fill_mode == fill_mode_type::CHAIN && (!warmup || trace_warmup_train)) {
            rtt_on_wp_predict(addr, pb->conditional, e.taken, pb->target, next, pb->conditional && ucp::bp_last_h2p, pb->branch_type);
          }
          if (pb->branch_type == BRANCH_RETURN) {
            tgt = 0;
            if (!wp_ras.empty()) {
              tgt = wp_ras.back();
              wp_ras.pop_back();
            }
          } else if ((pb->branch_type == BRANCH_DIRECT_CALL || pb->branch_type == BRANCH_INDIRECT_CALL) && next != 0) {
            wp_ras.push_back(next); // return to the call's real fall-through
          }
          if (e.taken) {
            next = tgt; // 0 = unknown target
          }
        }
      }
      wp_ftq.push_back(e);
      // SUCP probes the trace store with wrong-path PCs as the cursor does on the correct path
      if (fill_mode == fill_mode_type::CHAIN && (!warmup || trace_warmup_train) && sucp_trace_walks()) {
        if (const auto* ips = fill_store.lookup_entry(addr); ips != nullptr && !ips->empty()) {
          ++sim_stats.wp_store_hits;
          do_chain_trigger(*ips, 0, /*wrong_path=*/true);
        }
      }
      if (next == 0) {
        wp_gen_stopped = true; // unseen code or unknown target: the wrong path ends here
        ++sim_stats.wp_stops;
        break;
      }
      wp_next = next;
      if (e.branch && (e.taken || ++block_br == 2)) {
        // fetch-block rule: a block ends at a taken branch or holds two branches
        if (++blocks_done >= gen_blocks) {
          break;
        }
        block_br = 0;
      }
    }
    return;
  }
  for (champsim::bandwidth gen_bw{FETCH_WIDTH}; gen_bw.has_remaining() && wp_ftq.size() < lead; gen_bw.consume()) {
    const uint64_t addr = wp_next;
    wp_ftq.push_back({addr});
    ++sim_stats.wp_addresses;
    if (ucp_enable && ucp_btb_banks > 0) {
      ucp_note_demand_bank(addr); // the predictor's BTB access holds the bank, as on the correct path
    }
    uint64_t next = addr + WP_STEP;
    if (ucp::btb_probe) {
      if (const auto pb = ucp::btb_probe(addr); pb.has_value()) {
        bool taken = true;
        uint64_t tgt = pb->target;
        if (pb->conditional) {
          taken = impl_predict_branch(champsim::address{addr}, champsim::address{tgt}, false, BRANCH_CONDITIONAL);
          if (wrong_path_level >= 3 && ucp_enable && ucp::bp_last_h2p) {
            wp_ucp_trigger(addr, taken, tgt); // UCP fires on a hard-to-predict wrong-path branch like on any other
          }
        }
        if (pb->branch_type == BRANCH_RETURN) {
          tgt = 0;
          if (!wp_ras.empty()) {
            tgt = wp_ras.back();
            wp_ras.pop_back();
          }
        } else if (pb->branch_type == BRANCH_DIRECT_CALL || pb->branch_type == BRANCH_INDIRECT_CALL) {
          wp_ras.push_back(addr + WP_STEP);
        }
        if (taken && tgt != 0) {
          next = tgt;
        }
      }
    }
    // level 3: SUCP probes the trace store with wrong-path PCs as the cursor does on the correct path
    if (wrong_path_level >= 3 && fill_mode == fill_mode_type::CHAIN && (!warmup || trace_warmup_train) && sucp_trace_walks()) {
      if (const auto* ips = fill_store.lookup_entry(addr); ips != nullptr && !ips->empty()) {
        ++sim_stats.wp_store_hits;
        do_chain_trigger(*ips, 0, /*wrong_path=*/true);
      }
    }
    wp_next = next;
  }
}

// fetch follows the wrong path one fetch group per cycle: when a group misses the
// u-op cache (and the staging buffer), the decoders are in build mode this cycle
// and walks may not use them (the artifact's allow_pref_to_use_decoders_wp)
void O3_CPU::wp_fetch()
{
  wp_decoders_busy = false;
  wp_decoded_now = 0;
  ++sim_stats.wp_fetch_cycles;
  if (wrong_path_level >= 3) {
    wp_fetch_lines();
    return;
  }
  for (champsim::bandwidth fetch_bw{FETCH_WIDTH}; fetch_bw.has_remaining() && !wp_ftq.empty(); fetch_bw.consume()) {
    const uint64_t addr = wp_ftq.front().addr;
    wp_ftq.pop_front();
    ++sim_stats.wp_fetched;
    if (!uop_resident(addr)) {
      ++sim_stats.wp_uop_misses;
      wp_decoders_busy = true;
      ++wp_decoded_now;
    }
  }
  if (wp_decoders_busy) {
    ++sim_stats.wp_decoder_busy_cycles;
  }
}

bool O3_CPU::uop_resident(uint64_t ip) const
{
  const uint64_t wtag = ip >> dib_window_bits;
  return DIB.probe(champsim::address{ip}) || std::any_of(std::begin(uop_buffer), std::end(uop_buffer), [wtag](const auto& e) { return e.tag == wtag; });
}

// level 3: the wrong path is fetched like the correct path.
//  1. tag check + line request: up to a fetch group of wrong-path FTQ entries per
//     cycle are checked against the u-op cache; a miss requests its line from the
//     L1I ahead of fetch (as fetch_instruction does along IFETCH_BUFFER), with the
//     L1I bandwidth this cycle's correct-path fetch left unused.
//  2. fetch/decode in order: a u-op-cache hit streams, a miss decodes (decode
//     width) once its line has arrived -- fetch waits on a line still in flight.
//     Wrong-path instructions occupy decode/dispatch/ROB slots behind the branch
//     until the flush, so fetch stops when the backend is full; older correct-path
//     instructions retiring makes room.  (Loads/stores and registers are not
//     limited: the trace does not say what the wrong path contains.)
void O3_CPU::wp_fetch_lines()
{
  const auto fetch_width = static_cast<std::size_t>(champsim::to_underlying(FETCH_WIDTH));
  long l1i_budget = static_cast<long>(champsim::to_underlying(L1I_BANDWIDTH)) - l1i_demand_issued;
  for (std::size_t n = 0; n < fetch_width && wp_issue_idx < wp_ftq.size(); ++n) {
    auto& e = wp_ftq[wp_issue_idx];
    if (!e.checked) {
      e.checked = true;
      if (wp_uop_fill) {
        // ucp2: a real lookup, like demand (a hit updates recency)
        const uint64_t wtag = e.addr >> dib_window_bits;
        e.miss = !DIB.check_hit(champsim::address{e.addr})
                 && std::none_of(std::begin(uop_buffer), std::end(uop_buffer), [wtag](const auto& b) { return b.tag == wtag; });
      } else {
        e.miss = !uop_resident(e.addr);
      }
    }
    if (e.miss) {
      const uint64_t blk = champsim::block_number{champsim::address{e.addr}}.to<uint64_t>();
      if (wp_lines_requested.count(blk) == 0) {
        if (l1i_budget <= 0) {
          break; // retry next cycle
        }
        CacheBus::request_type line_pkt;
        line_pkt.v_address = champsim::address{e.addr};
        line_pkt.ip = champsim::address{e.addr};
        line_pkt.instr_id = 0; // no dependents: the response only marks the line ready
        if (!L1I_bus.issue_read(line_pkt)) {
          break;
        }
        wp_lines_requested.insert(blk);
        --l1i_budget;
        ++sim_stats.wp_l1i_lines;
      }
    }
    ++wp_issue_idx;
  }

  const auto backend_size = static_cast<long>(DECODE_BUFFER_SIZE + DISPATCH_BUFFER_SIZE + ROB_SIZE);
  const auto occupied = static_cast<long>(std::size(IFETCH_BUFFER) + std::size(DIB_HIT_BUFFER) + std::size(DECODE_BUFFER) + std::size(DISPATCH_BUFFER)
                                          + std::size(ROB) + wp_consumed);
  long room = backend_size - occupied;
  const auto decode_width = static_cast<std::size_t>(champsim::to_underlying(DECODE_WIDTH));
  std::size_t fetched = 0;
  std::size_t decoded = 0;
  int block_br = 0;
  bool line_stall = false;
  while (fetched < fetch_width && room > 0 && wp_issue_idx > 0) {
    const wp_entry e = wp_ftq.front();
    if (e.miss) {
      if (wp_lines_ready.count(champsim::block_number{champsim::address{e.addr}}.to<uint64_t>()) == 0) {
        line_stall = true;
        break;
      }
      if (decoded >= decode_width) {
        break;
      }
      ++decoded;
      ++sim_stats.wp_uop_misses;
      if (wp_uop_fill) {
        DIB.fill(champsim::address{e.addr}, e.taken, e.branch); // ucp2: the decoded wrong path fills the u-op cache (pollution)
        ++sim_stats.wp_uop_fills;
      }
    }
    if (rtt_seg_reconv) {
      wp_windows_fetched.insert(e.addr >> dib_window_bits);
    }
    wp_ftq.pop_front();
    --wp_issue_idx;
    ++wp_consumed;
    --room;
    ++fetched;
    ++sim_stats.wp_fetched;
    if (wp_exact && e.branch && (e.taken || ++block_br == 2)) {
      break; // fetch-block rule on the wrong path too
    }
  }
  wp_decoders_busy = (decoded > 0);
  wp_decoded_now = static_cast<uint64_t>(decoded);
  if (wp_decoders_busy) {
    ++sim_stats.wp_decoder_busy_cycles;
  }
  if (line_stall) {
    ++sim_stats.wp_line_stall_cycles;
  }
  if (room <= 0) {
    ++sim_stats.wp_full_cycles;
  }
}

// resolution: the wrong path leaves the FTQ, and walks it launched that are still
// in flight are dropped (lines already requested still arrive; staged windows stay)
void O3_CPU::wp_resolve()
{
  if (rtt_seg_reconv) {
    last_wp_windows.swap(wp_windows_fetched); // the recovery path is fetched next: reconvergence is checked against these
    last_wp_branch = wp_branch_id;
  }
  wp_windows_fetched.clear();
  wp_active = false;
  wp_fetching = false;
  wp_decoders_busy = false;
  wp_decoded_now = 0;
  wp_ftq.clear();
  wp_ras.clear();
  wp_issue_idx = 0;
  wp_consumed = 0;
  wp_lines_requested.clear();
  wp_lines_ready.clear();
  wp_gen_stopped = false;
  wp_pd_ips.clear();
  alt_walks.erase(std::remove_if(std::begin(alt_walks), std::end(alt_walks),
                                 [this](const auto& w) {
                                   if (!w.wrong_path) {
                                     return false;
                                   }
                                   ++sim_stats.wp_walks_flushed;
                                   return true;
                                 }),
                  std::end(alt_walks));
}

// level 3: a wrong-path branch the main predictor marks hard-to-predict launches a
// UCP alternate-path walk like a correct-path one, without training the Alt-BP
// (it trains on the demand stream only)
void O3_CPU::wp_ucp_trigger(uint64_t pc, bool predicted_taken, uint64_t target)
{
  if (ucp_altbp == nullptr || !ucp::btb_ras_snapshot || (warmup && !trace_warmup_train)) {
    return;
  }
  const uint64_t start = predicted_taken ? pc + static_cast<uint64_t>(ucp_step) : target;
  if (start == 0) {
    return;
  }
  auto ips = ucp_generate_alt_path(pc, BRANCH_CONDITIONAL, predicted_taken, start);
  if (!ips.empty()) {
    ++sim_stats.wp_ucp_triggers;
    do_ucp_trigger(pc, ips, /*wrong_path=*/true);
  }
}

// ===== UCP port (Singh et al., ISCA'24): alternate-path u-op cache prefetching =====
// Trigger: a hard-to-predict conditional at prediction time (classification from
// TAGE-SC-L provenance, published via ucp_hooks.h).  The walker follows the
// NOT-predicted direction -- targets from the real BTB, conditional directions
// from the dedicated 8KB Alt-BP, returns from an Alt-RAS snapshot -- and the
// resulting windows are prefetched into the u-op cache through the existing
// alt-walk machinery (prefetch-class installs, bytes through the real L1I).

void O3_CPU::do_ucp_branch(const ooo_model_instr& arch_instr, uint64_t btb_target, bool always_taken, bool h2p)
{
  if (!ucp::btb_probe || !ucp::btb_ras_snapshot) {
    return; // configured modules do not provide the UCP hooks (needs basic_btb + tage_sc_l)
  }
  if (ucp_altbp == nullptr) {
    ucp_altbp = new TAGE_PREDICTOR_8KB();
  }
  if (ucp_altind_enable && ucp_altind == nullptr) {
    ucp_altind = new alt_ittage();
  }
  const uint64_t pc = arch_instr.ip.to<uint64_t>();
  const uint64_t target = arch_instr.branch_target.to<uint64_t>();
  if (ucp_altind != nullptr) {
    // train along the demand stream, like the main ITTAGE (indirect tables train
    // internally only for indirect branches; history advances for every branch)
    ucp_altind->update_brindirect(pc, arch_instr.branch, arch_instr.branch_taken, target);
    ucp_altind->fetch_history_update(pc, arch_instr.branch, arch_instr.branch_taken, target);
  }

  if (arch_instr.branch == BRANCH_CONDITIONAL) {
    // Alt-BP protocol: predict-before-update keeps its internal indices consistent
    // and its history in sync with the demand stream (artifact order)
    (void)ucp_altbp->GetPrediction(pc);

    ++sim_stats.ucp_cond_seen;
    const bool miss = (arch_instr.branch_taken != arch_instr.branch_prediction);
    if (miss) {
      ++sim_stats.ucp_cond_misses;
    }
    if (h2p && !always_taken) {
      ++sim_stats.ucp_h2p_marked;
      if (miss) {
        ++sim_stats.ucp_h2p_marked_misses;
      }
      // walk the NOT-predicted direction: predicted-taken -> fall-through,
      // predicted-not-taken -> the BTB target (0 = unknown, cannot start)
      const uint64_t start = arch_instr.branch_prediction ? pc + static_cast<uint64_t>(ucp_step) : btb_target;
      if (start != 0) {
        auto ips = ucp_generate_alt_path(pc, arch_instr.branch, arch_instr.branch_prediction, start);
        if (!ips.empty()) {
          ucp_trigger_branch = arch_instr.instr_id;
          do_ucp_trigger(pc, ips);
          ucp_trigger_branch = 0;
        }
      }
    }
    ucp_altbp->UpdatePredictor(pc, arch_instr.branch, arch_instr.branch_taken, target, miss);
  } else {
    ucp_altbp->TrackOtherInst(pc, arch_instr.branch, arch_instr.branch_taken, target);
  }
}

std::vector<uint64_t> O3_CPU::ucp_generate_alt_path(uint64_t pc, uint8_t branch_type, bool prediction, uint64_t start)
{
  // checkpoint the Alt-BP: its GHR diverges with the walked (alternate) directions
  // during the walk and is restored at speculative_end (paper IV-C)
  ucp_altbp->speculative_begin(pc, branch_type, !prediction, start);
  if (ucp_altind != nullptr) {
    ucp_altind->speculative_begin();
  }
  auto alt_ras = ucp::btb_ras_snapshot(); // Alt-RAS: copy of the main RAS at trigger time

  std::vector<uint64_t> ips;
  uint64_t m_ip = start;
  int end_counter = 0; // Table-I weighted stop counter (paper IV-E)
  int run = 0;         // straight-line steps since the last BTB-known branch
  bool done = false;
  while (!done) {
    const auto pb = ucp::btb_probe(m_ip);
    bool pred = false;
    uint64_t tgt = 0;
    uint8_t bt = NOT_BRANCH;
    if (pb.has_value()) {
      bt = pb->branch_type;
      tgt = pb->target;
      if (pb->conditional) {
        pred = ucp_altbp->GetPrediction(m_ip);
        end_counter += ucp_altbp->is_h2p(m_ip); // Table-I weight from the Alt-BP's own provenance
      } else {
        pred = true;
        end_counter += 1;
      }
      if (bt == BRANCH_INDIRECT || bt == BRANCH_INDIRECT_CALL) {
        if (ucp_altind != nullptr) {
          tgt = ucp_altind->predict_brindirect(m_ip); // 12.95KB flavor: continue on a predicted target
          if (tgt == 0) {
            ++sim_stats.ucp_stop_ind; // Alt-Ind had no prediction
            done = true;
          } else {
            ++sim_stats.ucp_ind_walked;
            if (bt == BRANCH_INDIRECT_CALL) {
              alt_ras.push_back(m_ip + static_cast<uint64_t>(ucp_step));
            }
          }
        } else {
          ++sim_stats.ucp_stop_ind; // no-Alt-Ind flavor: target unknown, stop after this instruction
          done = true;
        }
      }
      if (bt == BRANCH_RETURN) {
        if (alt_ras.empty()) {
          tgt = 0; // falls out as a BTB-miss stop below
        } else {
          tgt = alt_ras.back();
          alt_ras.pop_back();
        }
      }
      if (bt == BRANCH_DIRECT_CALL) {
        alt_ras.push_back(m_ip + static_cast<uint64_t>(ucp_step));
      }
    } else {
      end_counter += 1; // artifact counts every stepped instruction toward the threshold
    }
    ips.push_back(m_ip);
    if (done) {
      break;
    }
    if (pb.has_value()) {
      // speculative Alt-BP/Alt-Ind updates along the alternate path (undone at speculative_end)
      if (pb->conditional) {
        ucp_altbp->UpdatePredictor(m_ip, bt, pred, tgt, false);
      } else {
        ucp_altbp->TrackOtherInst(m_ip, bt, pred, tgt);
      }
      if (ucp_altind != nullptr) {
        ucp_altind->update_brindirect(m_ip, bt, pred, tgt);
        ucp_altind->fetch_history_update(m_ip, bt, pred, tgt);
      }
      run = 0;
    } else if (++run >= ucp_max_ip_check) {
      ++sim_stats.ucp_stop_maxip;
      break;
    }
    if (pb.has_value() && pred && tgt == 0) {
      ++sim_stats.ucp_stop_btbmiss; // predicted taken with no known target
      break;
    }
    m_ip = (pb.has_value() && pred && tgt != 0) ? tgt : m_ip + static_cast<uint64_t>(ucp_step);
    if (end_counter >= ucp_threshold) {
      ++sim_stats.ucp_stop_sat;
      break;
    }
    if (ips.size() >= 4096) {
      break; // safety net against degenerate paths
    }
  }
  ucp_altbp->speculative_end();
  if (ucp_altind != nullptr) {
    ucp_altind->speculative_end();
  }
  return ips;
}

void O3_CPU::do_ucp_trigger(uint64_t h2p_ip, const std::vector<uint64_t>& ips, bool wrong_path)
{
  if (ucp_artifact & 1) {
    ucp_stop_previous(); // artifact stop_previous_alternate_path: one alternate path at a time
  } else if (std::any_of(std::begin(alt_walks), std::end(alt_walks), [h2p_ip](const auto& w) { return w.entry == h2p_ip; })) {
    return; // this H2P's alternate path is already in flight
  }
  // group the path's instructions into aligned windows, in path order, DEDUPED by
  // window tag: the artifact's uop-MSHR fetches/installs each line once per path
  // (loop iterations re-hitting a window must not re-install it -- pure churn)
  std::vector<std::vector<uint64_t>> groups;
  std::set<uint64_t> seen_tags;
  for (uint64_t raw : ips) {
    const uint64_t tag = raw >> dib_window_bits;
    if (!seen_tags.insert(tag).second) {
      if (!groups.empty() && (groups.back().front() >> dib_window_bits) == tag) {
        groups.back().push_back(raw); // still filling the window's first visit
      }
      continue;
    }
    groups.push_back({raw});
  }
  // alt-FTQ tag check (paper IV-D): prefetch only windows absent from the u-op cache
  groups.erase(std::remove_if(std::begin(groups), std::end(groups),
                              [this](const auto& g) { return DIB.probe(champsim::address{g.front()}); }),
               std::end(groups));
  const auto ucp_mshr_find = [this](uint64_t ip) {
    const uint64_t blk = champsim::block_number{champsim::address{ip}}.to<uint64_t>();
    return std::find_if(std::begin(ucp_mshr), std::end(ucp_mshr), [blk](const auto& m) { return m.blk == blk; });
  };
  if (ucp_artifact & 2) {
    // line-granular uop-MSHR merge: a line an earlier path requested is not requested again; if that
    // line already arrived (pref_done), the artifact never decodes the instructions merged into it
    const auto before = groups.size();
    groups.erase(std::remove_if(std::begin(groups), std::end(groups),
                                [&](const auto& g) {
                                  const auto m = ucp_mshr_find(g.front());
                                  return m != std::end(ucp_mshr) && m->filled;
                                }),
                 std::end(groups));
    sim_stats.ucp_art_merged += before - groups.size();
  } else if (ucp_artifact & 1) {
    // a window an earlier path already requested joins that request (no second request)
    const auto before = groups.size();
    groups.erase(std::remove_if(std::begin(groups), std::end(groups), [this](const auto& g) { return ucp_window_pending(g.front()); }), std::end(groups));
    sim_stats.ucp_art_merged += before - groups.size();
  }
  if (groups.empty()) {
    return; // fully resident: nothing to prefetch
  }
  ++sim_stats.ucp_paths;
  if (ucp_artifact == 0 && alt_walks.size() >= alt_walk_max) {
    ++sim_stats.alt_drops;
    return;
  }
  alt_walk w;
  w.entry = h2p_ip;
  w.ucp = true;
  w.wrong_path = wrong_path;
  w.ucp_branch = wrong_path ? 0 : ucp_trigger_branch;
  if ((ucp_artifact & 2) && w.ucp_branch != 0) {
    ucp_unresolved.insert(w.ucp_branch);
  }
  if (wrong_path) {
    ++sim_stats.wp_walks;
  }
  w.ready = current_time + alt_walk_delay_cycles * clock_period;
  w.last_progress = current_time;
  w.probe_time = current_time;
  w.windows = std::move(groups);
  if (ucp_artifact != 0) {
    w.done.assign(w.windows.size(), 0); // windows install in line-arrival order
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
      if (ucp_artifact & 2) {
        if (const auto m = ucp_mshr_find(win.front()); m != std::end(ucp_mshr)) {
          w.pf_wait.emplace_back(m->issued, blk); // in flight for an earlier path: decodes when it arrives
          continue;
        }
      }
      w.lines_pending.push_back(win.front());
    }
    // all line addresses are known at trigger: issue up to two this cycle (artifact prefetch mode: the
    // uop-MSHR queue issues them, one per cycle, from drain_alt_walks)
    for (int n = 0; n < 2 && (ucp_artifact & 2) == 0 && !w.lines_pending.empty() && walk_issue_available(); ++n) {
      CacheBus::request_type line_pkt;
      line_pkt.v_address = champsim::address{w.lines_pending.front()};
      line_pkt.ip = champsim::address{w.lines_pending.front()};
      line_pkt.instr_id = 0;
      line_pkt.no_fill = alt_walk_nofill && !w.ucp; // SUCP walk bytes allocate in no cache level
      line_pkt.walker = (walk_l2_mshrs > 0 && !w.ucp);
      if (line_pkt.walker) {
        line_pkt.no_fill = true; // handed to the walk, not filled into the L1I (L2/LLC still allocate)
      }
      if (!L1I_bus.issue_read(line_pkt)) {
        break;
      }
      w.lines_pending.pop_front();
      ++sim_stats.alt_lines_issued;
      ++w.lines_issued;
      if (w.wrong_path) {
        ++sim_stats.wp_lines;
      }
      walk_issue_note();
    }
  }
  alt_walks.push_back(std::move(w));
  ++sim_stats.alt_triggers;
}

// CHAIN staging buffer: FIFO of pre-decoded windows awaiting demand.  A refresh
// of a resident tag replaces it; capacity eviction discards the oldest window
// (counted as overshoot if it was never demanded -- hits erase their entry).
void O3_CPU::uop_buffer_push(const std::vector<uint64_t>& window_ips, champsim::chrono::clock::time_point probe_time, uint64_t origin, uint64_t probe_id,
                             bool oracle, bool wrong_path, bool recovery, uint8_t vp, uint8_t chain_hop, uint8_t pos)
{
  const uint64_t tag = window_ips.front() >> dib_window_bits;
  if (auto it = std::find_if(std::begin(uop_buffer), std::end(uop_buffer), [tag](const auto& e) { return e.tag == tag; });
      it != std::end(uop_buffer)) {
    it->ips = window_ips; // refresh
    it->probe_time = probe_time;
    it->origin = origin;
    // a window staged legitimately stays legitimate: squash/oracle only if both stagings were past a mispredict
    it->probe_id = (it->probe_id == 0 || probe_id == 0) ? 0 : std::min(it->probe_id, probe_id);
    it->oracle = it->oracle && oracle;
    it->wrong_path = it->wrong_path && wrong_path;
    return;
  }
  while (uop_buffer.size() >= static_cast<std::size_t>(uop_buffer_windows)) {
    if (uop_buffer.front().served) {
      uop_buffer.pop_front(); // mode 0: served at least once, so not an unused eviction
      continue;
    }
    if (chain_launch_conf > 0) {
      fill_store.conf_miss(uop_buffer.front().origin); // staged, never demanded: the launching trace loses confidence
    }
    uop_buf_evict_log.emplace_back(uop_buffer.front().tag, current_time);
    uop_buf_evict_map[uop_buffer.front().tag] = current_time;
    while (uop_buf_evict_log.size() > UOP_BUF_EVICT_LOG) {
      const auto& [old_tag, old_time] = uop_buf_evict_log.front();
      if (auto mit = uop_buf_evict_map.find(old_tag); mit != std::end(uop_buf_evict_map) && mit->second == old_time) {
        uop_buf_evict_map.erase(mit); // only drop the map entry if no later eviction of the same tag superseded it
      }
      uop_buf_evict_log.pop_front();
    }
    if (trigger_diag && !warmup) {
      ++(uop_buffer.front().late ? sim_stats.td_unused_late : sim_stats.td_unused_ontime);
      ++sim_stats.td_pos_unused[std::min<std::size_t>(uop_buffer.front().pos, 4)];
    }
    uop_buffer.pop_front();
    ++sim_stats.chain_buf_evict_unused; // anything still resident was never demanded
  }
  // trace_trigger_diag: demand already missed this window after the walk launched -> the staging is late
  bool late = false;
  if (trigger_diag) {
    auto lm = td_last_miss.find(tag);
    late = (lm != std::end(td_last_miss) && lm->second >= probe_time);
    if (!warmup) {
      ++sim_stats.td_staged;
      if (late) {
        ++sim_stats.td_staged_late;
      }
    }
  }
  uop_buffer.push_back({tag, window_ips, probe_time, origin, false, probe_id, oracle, wrong_path, recovery, vp, chain_hop, late, pos});
  if (oracle) {
    ++sim_stats.chain_shadow_windows;
  }
  if (wrong_path) {
    ++sim_stats.wp_staged;
  }
}

// wait-eligible variant: the walk holding the window, or nullptr -- background
// walks (wait-cap expired: their bytes proved far) are not waited on.
// Wait-mode-2 gate: only worth stalling fetch if the pending window's leading
// line has already arrived (install is imminent -- a couple of cycles for the
// install slot).  Stalling on bytes still in flight measured slower than letting
// the warm, pipelined slow path absorb the miss (wait=1: -34% misses, -0.18% IPC).
// Optimistic about preceding windows' install order; the wait cap bounds the error.
bool O3_CPU::walk_wait_worthwhile(const alt_walk* holder, uint64_t rawip) const
{
  if (alt_walk_wait_mode != 2 || !alt_walk_l1i) {
    return true; // mode 1 (or byte-free walks): always wait, legacy behavior
  }
  const uint64_t wtag = rawip >> dib_window_bits;
  for (std::size_t i = holder->idx; i < holder->windows.size(); ++i) {
    if ((holder->windows[i].front() >> dib_window_bits) == wtag) {
      return holder->lines_ready.count(champsim::block_number{champsim::address{holder->windows[i].front()}}.to<uint64_t>()) != 0;
    }
  }
  return false;
}

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
      line_pkt.no_fill = alt_walk_nofill && !w.ucp; // SUCP walk bytes allocate in no cache level
      line_pkt.walker = (walk_l2_mshrs > 0 && !w.ucp);
      if (line_pkt.walker) {
        line_pkt.no_fill = true; // handed to the walk, not filled into the L1I (L2/LLC still allocate)
      }
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
  if (cold_diag && !warmup && last_redirect_id != 0 && instr.instr_id > last_redirect_id && instr.instr_id - last_redirect_id <= 64) {
    const uint64_t line = champsim::block_number{instr.ip}.to<uint64_t>();
    if (instr.instr_id == last_redirect_id + 1) {
      redirect_target_line = line; // the first instruction of the recovery path
    }
    if (!hit && redirect_target_line != 0 && !l1i->has_virtual_block(instr.ip)) {
      const std::size_t s = seen_win.count(instr.ip.to<uint64_t>() >> dib_window_bits) != 0 ? 0 : 1;
      const int64_t dist = static_cast<int64_t>(line) - static_cast<int64_t>(redirect_target_line);
      const std::size_t b = dist < 0 ? 5 : dist == 0 ? 0 : dist == 1 ? 1 : dist <= 3 ? 2 : dist <= 7 ? 3 : 4;
      ++sim_stats.cold_dist[s][b];
      ++sim_stats.cold_class[s][std::min<std::size_t>(rec_class, 5)];
    }
  }
  if (!hit && uop_ideal_split != 0 && !warmup) {
    // diagnostic oracle: serve the selected kinds of u-op misses as hits
    const bool near_redirect = last_redirect_id != 0 && instr.instr_id > last_redirect_id
                               && instr.instr_id - last_redirect_id <= static_cast<uint64_t>(uop_ideal_split_window);
    const bool resident = l1i->has_virtual_block(instr.ip);
    const unsigned kind = (near_redirect ? 2u : 0u) + (resident ? 0u : 1u);
    // bit 16: only windows looked up before (warmup included) -- the reach of any history-based scheme
    const bool seen_ok = (uop_ideal_split & 16) == 0 || seen_win.count(instr.ip.to<uint64_t>() >> dib_window_bits) != 0;
    if (((uop_ideal_split >> kind) & 1) && seen_ok) {
      hit = true;
      ++sim_stats.uop_split_hits[kind];
    }
  }
  const bool dib_hit = hit;     // trace_trigger_diag: resident before any trace fill
  bool buf_hit = false;         // ...served by the CHAIN staging buffer
  bool staged_evicted = false;  // ...staged, then evicted from the buffer before this demand
  // ALT: on a demand miss, probe the trace store and launch the walk FIRST (dedup
  // and walk-cap inside do_alt_trigger), then wait-on-pending.  Trigger-before-wait
  // means even the walk's own triggering miss is served as a delayed hit: the
  // frontend never switches to build mode for a covered miss (no switch stall per
  // walk).  Uncovered or walk-capped misses fall through to the normal miss path.
  // The wait leaves the entry un-checked (re-checks next cycle, L1I fetch held);
  // placed before any counter so re-checks are side-effect-free.
  if (fill_mode == fill_mode_type::ALT && !hit && (!warmup || trace_warmup_train)) {
    do_alt_trigger(instr.ip.to<uint64_t>());
    // wait-cap: wait on the fill buffer, not on DRAM.  An instruction stalls at
    // most alt_walk_wait_cap cycles on a pending window (enough to cover an
    // L1I-hit fill); if the cap expires the bytes proved far, so the WHOLE WALK
    // is demoted to a background prefetcher -- no instruction waits on it again,
    // demand proceeds through build mode (its fetch MSHR-merges with the walk's
    // line requests) while the walk keeps installing ahead.  Cap 0 = unbounded.
    if (alt_walk_wait_mode != 0) {
      if (alt_walk* holder = alt_pending_walk(instr.ip.to<uint64_t>()); holder != nullptr && walk_wait_worthwhile(holder, instr.ip.to<uint64_t>())) {
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
  if (fill_mode == fill_mode_type::HEAD && !hit && (!warmup || trace_warmup_train)) {
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
    if (!hit && alt_walk_wait_mode != 0) {
      if (alt_walk* holder = alt_pending_walk(rawip); holder != nullptr && walk_wait_worthwhile(holder, rawip)) {
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
  if (fill_mode == fill_mode_type::CHAIN && !hit && (!warmup || trace_warmup_train)) {
    const uint64_t rawip = instr.ip.to<uint64_t>();
    const uint64_t wtag = rawip >> dib_window_bits;
    auto bit = std::find_if(std::begin(uop_buffer), std::end(uop_buffer), [wtag](const auto& e) { return e.tag == wtag; });
    if (chain_parallel && fill_store.lookup_window(rawip) != nullptr) {
      ++sim_stats.uop_trace_fill_hits; // ceiling: the stored trace's window is delivered the moment demand needs it
      hit = true;
    } else if (bit != std::end(uop_buffer)) {
      ++sim_stats.chain_buf_hits;
      hit = true;
      buf_hit = true;
      if (bit->served) {
        ++sim_stats.chain_buf_rehits; // mode 0: a later demand hit served by the buffer again
      } else {
        if (rtt_l2_keep && l2_emissary_ways > 0 && l2_bound) {
          emissary_mark(champsim::address{rawip}); // a recovery-path line demand used: keep it in the L2
        }
        if (trigger_diag && !warmup) {
          ++sim_stats.td_pos_used[std::min<std::size_t>(bit->pos, 4)];
        }
        if (bit->oracle) {
          ++sim_stats.chain_shadow_hits; // served by a window probed past a mispredicted branch
        }
        if (bit->wrong_path) {
          ++sim_stats.wp_buffer_hits; // a wrong-path prefetch the correct path used
        }
        if (bit->recovery) {
          ++sim_stats.rk_buffer_hits; // staged by a recovery-key walk
        }
        if (bit->chain_hop != 0) {
          ++sim_stats.chain_bhits_hop[bit->chain_hop - 1]; // staged by a chained successor walk
        }
        if (bit->vp != 0) {
          ++sim_stats.vp_buffer_hits; // staged by a value-predictor walk
          if (bit->vp == 2) {
            ++sim_stats.vp_buffer_hits_mp;
          }
        }
        const auto slack = (current_time - bit->probe_time) / clock_period;
        const std::size_t bucket = (slack <= 4) ? 0 : (slack <= 8) ? 1 : (slack <= 16) ? 2 : (slack <= 32) ? 3 : (slack <= 64) ? 4 : 5;
        ++sim_stats.chain_slack[bucket];
        if (chain_launch_conf > 0) {
          fill_store.conf_hit(bit->origin); // staged window demanded: the launching trace gains confidence
        }
      }
      if (chain_buffer_promote == 0) {
        bit->served = true; // no u-op-cache fill: the window stays in the buffer until FIFO eviction
      } else {
        // 1: demand-class fill at MRU; 2: prefetch-class fill at LRU (promoted on its first u-op-cache hit)
        DIB.install(bit->ips, /*prefetch=*/chain_buffer_promote == 2);
        uop_buffer.erase(bit);
      }
    } else {
      // buffer miss: was this window staged and then evicted unused?  Then a
      // larger FIFO would have served it -- count it once and record how long
      // after the eviction the demand arrived (the residency the FIFO lacked).
      if (auto mit = uop_buf_evict_map.find(wtag); mit != std::end(uop_buf_evict_map)) {
        ++sim_stats.chain_evicted_then_missed;
        staged_evicted = true;
        const auto age = (current_time - mit->second) / clock_period;
        const std::size_t ab = (age <= 256) ? 0 : (age <= 1024) ? 1 : (age <= 4096) ? 2 : (age <= 16384) ? 3 : 4;
        ++sim_stats.chain_evicted_age[ab];
        uop_buf_evict_map.erase(mit);
      }
      if (alt_walk_wait_mode != 0) {
        if (alt_walk* holder = alt_pending_walk(rawip); holder != nullptr && walk_wait_worthwhile(holder, rawip)) {
          if (alt_walk_wait_cap == 0 || instr.dib_wait_cycles < alt_walk_wait_cap) {
            ++instr.dib_wait_cycles;
            ++sim_stats.alt_wait_cycles;
            return; // walk in flight: hit-under-fill
          }
          holder->background = true;
        }
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
    // CHAIN shadow classification of each miss against the CURRENT store contents
    // (not builder coverage): was a walk already in flight (timeliness loss), is
    // the miss IP itself a stored trace ENTRY (front-door existed), or does it
    // fall mid-trace in a stored window (side-door: the trigger gap that window-
    // index probing would convert)?  Measurement only -- no behavior change.
    if (fill_mode == fill_mode_type::CHAIN) {
      if (alt_window_pending(rawip)) {
        ++sim_stats.chain_miss_inflight;
        late_window_classify(rawip);
      } else if (fill_store.peek_entry(rawip)) {
        ++sim_stats.chain_miss_entry;
      } else if (fill_store.peek_window(rawip)) {
        ++sim_stats.chain_miss_midtrace;
      }
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

  if (trigger_diag && fill_mode == fill_mode_type::CHAIN) {
    if (!warmup) {
      trigger_diag_check(instr, dib_hit, buf_hit, staged_evicted);
    }
    td_seen_windows.insert(instr.ip.to<uint64_t>() >> dib_window_bits); // after classifying: a first fetch is cold
    if (!hit) {
      td_last_miss[instr.ip.to<uint64_t>() >> dib_window_bits] = current_time;
    }
  }

  // stall-triggered segmenter: feed the fetch stream (hit closes/commits a costly
  // build-mode stretch; miss extends the current one). See inc/trace_stall.h.  When
  // it commits a new trace and trace-fill is on, push it into the trace cache -- so
  // trace_builder=stall makes the stall segmenter the fill source.
  if (trace_stall_enable && (!warmup || trace_warmup_train)) {
    const std::size_t before = stall.get_traces().size();
    uint64_t mp_key = 0;
    if (instr.pred_wrong) {
      // trace_stall_mispredict_open 2: key = (branch, where it really went) -- the
      // key the predict-time probe forms from the alternate of its prediction
      const uint64_t bip = instr.ip.to<uint64_t>();
      const uint64_t restart = instr.branch_taken ? instr.branch_target.to<uint64_t>() : bip + 4;
      mp_key = (restart != 0) ? ((bip ^ (restart << 1)) | (uint64_t{1} << 63)) : 0;
    }
    stall.on_dib(instr.ip.to<uint64_t>(), hit, sim_stats, instr.instr_id, (fill_mode != fill_mode_type::OFF) ? &fill_store : nullptr, instr.pred_wrong,
                 mp_key, static_cast<uint64_t>(current_time.time_since_epoch() / clock_period), instr.vp_ctx);
    feed_stall_traces(before);
    vpred_train_from_stall();
  }

  instr.dib_checked = true;
  instr.uop_miss = !hit;
  if (rtt_mode != 0 && (!warmup || trace_warmup_train)) {
    rtt_capture_step(instr, dib_hit); // a hit on a window our own walk staged does not close a recording
  }
  if (rtt_diag || oracle_recovery_seen || (uop_ideal_split & 16) != 0 || cold_diag || rtt_cold_mode >= 2) {
    seen_win.insert(instr.ip.to<uint64_t>() >> dib_window_bits);
  }
  if (stall_taxonomy) {
    instr.tax_first_touch = tax_seen.insert(instr.ip.to<uint64_t>() >> dib_window_bits).second;
  }
  if (miss_profile && !warmup && !hit) {
    ++miss_sites[instr.ip.to<uint64_t>() >> miss_window_bits].misses;
  }

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

// Mispredict-keyed trigger (trace_stall_mispredict): fired at branch-misprediction
// resolution, when the resolved (branch, restart) pair is architecturally known.
// Probes the trace store under a hashed event key -- a hit launches a standard
// chain walk for the recovery path recorded after past occurrences of this exact
// mispredict, with the full redirect+refetch latency as lead (FTQ probing has ~zero
// lead here: the queue is empty after the flush).  Then arms the stall segmenter to
// capture the upcoming restart stretch under the same key (no ROB-drain gate).
// Bit 63 separates event keys from instruction-PC keys in the shared store.
void O3_CPU::do_event_trigger(const ooo_model_instr& branch)
{
  // mode 4 probes the regular store (no event traces to capture)
  if (stall_mispredict_mode == 0 || stall_mispredict_mode >= 4 || warmup || fill_mode != fill_mode_type::CHAIN) {
    return;
  }
  const uint64_t bip = branch.ip.to<uint64_t>();
  const uint64_t restart = branch.branch_taken ? branch.branch_target.to<uint64_t>() : bip + 4;
  if (restart == 0) {
    return;
  }
  const uint64_t key = (bip ^ (restart << 1)) | (uint64_t{1} << 63);
  if (stall_mispredict_mode == 1) {
    // mode 1: probe at resolution.  Measured negative (-0.08pp qualcomm): with a
    // 1-cycle redirect the lead is structurally absent and walk traffic contends
    // with recovery demand fetch.  Kept for the ablation table.
    ++sim_stats.event_probes;
    if (const auto* ips = fill_store.lookup_entry(key); ips != nullptr && !ips->empty()) {
      ++sim_stats.event_probe_hits;
      do_chain_trigger(*ips);
    }
  }
  // both modes capture identically: the recovery trace is keyed by the resolved
  // (branch, restart) pair -- which is exactly hash(branch, alternate-of-the-
  // failed-prediction), the key mode 2 recomputes at predict time
  if (trace_stall_enable) {
    stall.arm_event(key, restart);
  }
}

long O3_CPU::fetch_instruction()
{
  long progress{0};

  // Fetch a single cache line
  auto fetch_ready = [head = (ftq_lookup_window > 0)](const ooo_model_instr& x) {
    if (head) {
      // decoupled front end: FDIP for blocks predicted to miss (before their lookup), demand for lookup misses
      return !x.fetch_issued && (x.dib_checked ? !x.fetch_completed : x.pre_miss);
    }
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

  l1i_demand_issued = progress; // wrong-path fetch (trace_wrong_path 3) gets the rest of this cycle's L1I bandwidth
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

  const bool head = (ftq_lookup_window > 0);
  auto fetch_complete_and_ready = [time = current_time, head](const auto& x) {
    return x.fetch_completed && x.ready_time <= time && (!head || x.dib_checked); // FDIP bytes wait for the lookup
  };

  champsim::bandwidth available_fetch_bandwidth{
      std::min(FETCH_WIDTH, std::min(champsim::bandwidth::maximum_type{static_cast<long>(DIB_HIT_BUFFER_SIZE - std::size(DIB_HIT_BUFFER))},
                                     champsim::bandwidth::maximum_type{static_cast<long>(DECODE_BUFFER_SIZE - std::size(DECODE_BUFFER))}))};

  auto fetched_check_end = std::find_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER),
                                        [head](const ooo_model_instr& x) { return !x.fetch_completed || (head && !x.dib_checked); });
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
  demand_decoded_now = static_cast<uint64_t>(available_decode_bandwidth.amount_consumed()); // walks drain later this cycle (check_dib)

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
        this->do_flush_fill(db_entry);    // FLUSH mode: warm the recovery path
        this->do_event_trigger(db_entry); // mispredict-keyed probe + capture arm
        this->in_recovery = true;         // frontend now refilling the recovery path
        this->last_redirect_id = db_entry.instr_id;
        this->rec_class = mispredict_class(db_entry);
        ++this->sim_stats.rec_mispredicts[this->rec_class];
        this->chain_probe_cursor = 0;     // CHAIN deep probe: FTQ flush -- lead rebuilds
        this->chain_shadow_branch = 0;    // resolved: the supply past this point is the real path again
        this->chain_cursor_blocked = false;
        this->chain_peek_left = 0;
        if (this->wp_active) {
          this->wp_resolve(); // flush the wrong path
        }
        this->rtt_on_resolve(db_entry);
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
  DIB.fill(instr.ip, taken_end, instr.is_branch, /*prefetch=*/false, instr.retain_costly);
}

void O3_CPU::feed_stall_traces(std::size_t before)
{
  if (fill_mode == fill_mode_type::OFF) {
    return;
  }
  // one on_dib / advance can commit several traces (entry-keyed + mispredict-keyed,
  // or several held stretches finalized at once)
  const auto insert_trace = [this](const trace_stall::trace& t) {
    if (fill_mode == fill_mode_type::CHAIN) {
      // enforce the metadata format at insert (see chain_encode)
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
  };
  const std::size_t after = stall.get_traces().size();
  for (std::size_t ti = before; ti < after; ++ti) {
    insert_trace(stall.get_traces()[ti]);
  }
  for (const auto& t : stall.readmits()) {
    insert_trace(t); // a re-captured stretch the store had evicted (trace_stall_readmit)
  }
  // refresh accumulated costs so cost-aware eviction sees current values
  // (a no-op with identical value right after a fresh insert; the real work
  // is the re-capture case, where only the stored cost changes)
  for (const auto& [entry, cost] : stall.touches()) {
    fill_store.update_cost(entry, cost);
  }
  if (stall.touched_event()) {
    fill_store.update_cost(stall.touch_event_entry(), stall.touch_event_cost());
  }
}

void O3_CPU::advance_stall_attrib()
{
  if (!trace_stall_enable || (warmup && !trace_warmup_train) || stall.attrib() != 1) {
    return;
  }
  // the oldest instruction not yet dispatched; queues are in program order
  uint64_t oldest = std::numeric_limits<uint64_t>::max();
  for (const auto* q : {&DISPATCH_BUFFER, &DECODE_BUFFER, &IFETCH_BUFFER, &input_queue}) {
    if (!q->empty()) {
      oldest = q->front().instr_id;
      break;
    }
  }
  const std::size_t before = stall.get_traces().size();
  stall.advance(oldest, sim_stats, (fill_mode != fill_mode_type::OFF) ? &fill_store : nullptr);
  feed_stall_traces(before);
  vpred_train_from_stall();
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
    if (stall_taxonomy && DISPATCH_BUFFER.front().tax_starve_upper > 0) {
      const auto& d = DISPATCH_BUFFER.front();
      const uint32_t lat = d.tax_fetch_lat;
      const std::size_t level = (lat == 0) ? 4 : (lat <= 8) ? 0 : (lat <= 30) ? 1 : (lat <= 110) ? 2 : 3;
      const std::size_t idx = (d.tax_recovery ? 10 : 0) + (d.tax_first_touch ? 5 : 0) + level;
      sim_stats.tax_tight[idx] += d.tax_starve_tight;
      sim_stats.tax_upper[idx] += d.tax_starve_upper;
    }
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
  // frontend-lost dispatch SLOTS (top-down style, the middle bound): dispatch
  // bandwidth left unused in BUILD mode while the backend could have accepted
  // more (ROB has room) and the front end had nothing ready -- counts partial
  // shortfall cycles the zero-dispatch starve bound misses.
  if (fetch_mode == fetch_mode_type::BUILD && available_dispatch_bandwidth.has_remaining() && std::size(ROB) != ROB_SIZE
      && (std::empty(DISPATCH_BUFFER) || DISPATCH_BUFFER.front().ready_time > current_time)) {
    const auto lost = static_cast<uint64_t>(available_dispatch_bandwidth.amount_remaining());
    if (in_recovery) {
      sim_stats.fe_lost_slots_recovery += lost;
    } else {
      sim_stats.fe_lost_slots_steady += lost;
    }
  }
  if (available_dispatch_bandwidth.amount_consumed() == 0 && fetch_mode == fetch_mode_type::BUILD
      && (std::empty(DISPATCH_BUFFER) || DISPATCH_BUFFER.front().ready_time > current_time)) {
    // EMISSARY marking / resteer-prefetcher recording: an empty ROB waiting on a u-op-cache miss
    // whose L1I fetch missed (decode starvation with an empty issue queue)
    if (std::empty(ROB) && (l2_emissary_ways > 0 || rp_mode != 0 || fdip_predecode == 2)) {
      ooo_model_instr* oldest = nullptr;
      for (auto* q : {&DISPATCH_BUFFER, &DECODE_BUFFER, &IFETCH_BUFFER, &input_queue}) {
        if (!q->empty()) {
          oldest = &q->front();
          break;
        }
      }
      if (fdip_predecode == 2 && oldest != nullptr && oldest->dib_checked && oldest->uop_miss) {
        cwt_mark(oldest->ip.to<uint64_t>() >> dib_window_bits); // this window's u-op miss left the ROB empty
      }
      if (oldest != nullptr && oldest->dib_checked && oldest->uop_miss && oldest->fetch_issued) {
        const bool l1i_missed = oldest->fetch_completed ? (oldest->tax_fetch_lat > STALL_L1I_MISS_CYCLES)
                                                        : (current_time - oldest->fetch_issue_time > STALL_L1I_MISS_CYCLES * clock_period);
        if (l1i_missed && l2_emissary_ways > 0 && l2_bound && !rtt_l2_keep && !oldest->emissary_done) {
          oldest->emissary_done = true;
          if (l2_emissary_rand <= 1 || emissary_rng() % l2_emissary_rand == 0) {
            if (oldest->fetch_completed) {
              emissary_mark(oldest->ip);
            } else {
              oldest->emissary_pending = true;
            }
          }
        }
        if (l1i_missed && rp_mode == 1 && oldest->rp_key != 0 && !oldest->rp_recorded) {
          oldest->rp_recorded = true;
          if (rp_entry* e = rp_find(oldest->rp_key, true); e != nullptr) {
            rp_add_target(*e, champsim::block_number{oldest->ip}.to<uint64_t>());
            ++sim_stats.rp_records;
          }
        }
        if (rp_mode == 2 && oldest->rp_key != 0) {
          rp_path_starved(oldest->rp_key);
        }
      }
    }
    // stall taxonomy: charge the cycle to the instruction dispatch waits for
    if (stall_taxonomy && !warmup && std::size(ROB) != ROB_SIZE) {
      ooo_model_instr* oldest = nullptr;
      for (auto* q : {&DISPATCH_BUFFER, &DECODE_BUFFER, &IFETCH_BUFFER, &input_queue}) {
        if (!q->empty()) {
          oldest = &q->front();
          break;
        }
      }
      const bool tight = std::empty(ROB);
      if (oldest == nullptr || !oldest->dib_checked) {
        ++sim_stats.tax_other[tight ? 0 : 1];
      } else if (!oldest->uop_miss) {
        ++sim_stats.tax_other[tight ? 2 : 3];
      } else {
        if (oldest->tax_starve_upper == 0) {
          oldest->tax_recovery = in_recovery;
        }
        ++oldest->tax_starve_upper;
        if (tight) {
          ++oldest->tax_starve_tight;
        }
      }
    }
    if (std::size(ROB) != ROB_SIZE) {
      if (in_recovery) {
        ++sim_stats.fe_stall_recovery;
      } else {
        ++sim_stats.fe_stall_steady;
      }
    }
    if (std::empty(ROB)) {
      ++rob_empty_total;
      if (in_recovery) {
        ++sim_stats.rob_idle_recovery;
        ++sim_stats.rec_idle_cycles[rec_class];
      } else {
        ++sim_stats.rob_idle_steady;
      }
      // uop_retain: the window dispatch waits for missed the u-op cache and starved the backend
      if (DIB.retain_cap() > 0) {
        for (auto* q : {&DISPATCH_BUFFER, &DECODE_BUFFER, &IFETCH_BUFFER, &input_queue}) {
          if (!q->empty()) {
            if (q->front().uop_miss && !DIB.mark_costly(q->front().ip)) {
              q->front().retain_costly = true; // not built yet: marked at its fill
            }
            break;
          }
        }
      }
      // miss-cost profile: charge the cycle to the window dispatch waits for
      if (miss_profile && !warmup) {
        const ooo_model_instr* oldest = nullptr;
        for (const auto* q : {&DISPATCH_BUFFER, &DECODE_BUFFER, &IFETCH_BUFFER, &input_queue}) {
          if (!q->empty()) {
            oldest = &q->front();
            break;
          }
        }
        if (oldest != nullptr && oldest->uop_miss) {
          ++miss_sites[oldest->ip.to<uint64_t>() >> miss_window_bits].cost;
        } else {
          ++miss_prof_other;
        }
      }
    }
    // trace_trigger_diag: charge the starve cycle to the class of the miss dispatch waits for
    if (trigger_diag && fill_mode == fill_mode_type::CHAIN && !warmup && std::size(ROB) != ROB_SIZE) {
      const ooo_model_instr* oldest = nullptr;
      for (const auto* q : {&DISPATCH_BUFFER, &DECODE_BUFFER, &IFETCH_BUFFER, &input_queue}) {
        if (!q->empty()) {
          oldest = &q->front();
          break;
        }
      }
      const bool charged = (oldest != nullptr && oldest->uop_miss);
      if (charged) {
        ++sim_stats.td_cost_upper[oldest->trig_class];
        ++sim_stats.td_head_upper[oldest->trig_head];
      } else {
        ++sim_stats.td_cost_upper_other;
      }
      if (std::empty(ROB)) {
        if (charged) {
          ++sim_stats.td_cost_tight[oldest->trig_class];
          ++sim_stats.td_head_tight[oldest->trig_head];
        } else {
          ++sim_stats.td_cost_tight_other;
        }
      }
    }
    // stall-segmenter trigger: a build-mode dispatch-starve cycle where the ROB has
    // drained to at/below stall_rob_threshold marks the in-flight stretch costly.
    // threshold 0 = fully empty (the tight condition); a higher value broadens the
    // trigger to partial (near-empty) stalls, capturing recurring stretches the
    // fully-empty trigger misses.
    if (trace_stall_enable && (!warmup || trace_warmup_train) && std::size(ROB) <= stall_rob_threshold) {
      // the oldest instruction still in the front end is the one dispatch waits for
      uint64_t starve_id = 0;
      for (const auto* q : {&DISPATCH_BUFFER, &DECODE_BUFFER, &IFETCH_BUFFER, &input_queue}) {
        if (!q->empty()) {
          starve_id = q->front().instr_id;
          break;
        }
      }
      if (!stall.note_stall(starve_id)) {
        ++sim_stats.stall_marks_dropped; // tight-stall cycle after the stretch closed (attribution race)
      }
    }
  }
  if (rtt_mode != 0 && rtt_store_filter != 0 && (!rtt_open.empty() || !rtt_wait.empty())) {
    rtt_rob_empty();
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

  if ((ucp_artifact & 2) && instr.is_branch) {
    ucp_unresolved.erase(instr.instr_id); // uop-MSHR entries of its alternate path may now be freed
  }
  if (instr.rtt_key != 0) {
    // rtt_conf_min: was this low-confidence prediction actually wrong?
    if (instr.pred_wrong) {
      rtt_conf_train(instr.rtt_key, true);
      ++sim_stats.rtt_conf_up;
    } else {
      rtt_conf_train(instr.rtt_key, false);
      ++sim_stats.rtt_conf_down;
    }
  }
  if (rtt_cancel && instr.rtt_walked && !instr.pred_wrong) {
    rtt_cancel_walk(instr.instr_id); // predicted right: its recovery path will not be fetched
  }
  if (instr.branch_mispredicted) {
    fetch_resume_time = current_time + BRANCH_MISPREDICT_PENALTY;
    do_flush_fill(instr);    // FLUSH mode: warm the recovery path
    do_event_trigger(instr); // mispredict-keyed probe + capture arm
    in_recovery = true;      // frontend now refilling the recovery path
    last_redirect_id = instr.instr_id;
    rec_class = mispredict_class(instr);
    ++sim_stats.rec_mispredicts[rec_class];
    chain_probe_cursor = 0;  // CHAIN deep probe: FTQ flush -- lead rebuilds
    chain_shadow_branch = 0; // resolved: the supply past this point is the real path again
    chain_cursor_blocked = false;
    chain_peek_left = 0;
    if (wp_active) {
      wp_resolve(); // flush the wrong path
    }
    rtt_on_resolve(instr);
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
    if ((fill_mode == fill_mode_type::ALT || fill_mode == fill_mode_type::HEAD || fill_mode == fill_mode_type::CHAIN || ucp_enable) && alt_walk_l1i
        && !alt_walks.empty()) {
      const uint64_t blk = champsim::block_number{l1i_entry.v_address}.to<uint64_t>();
      for (auto& w : alt_walks) {
        w.lines_ready.insert(blk);
      }
    }
    if (wp_active && wrong_path_level >= 3) {
      wp_lines_ready.insert(champsim::block_number{l1i_entry.v_address}.to<uint64_t>()); // wrong-path fetch may now decode it
    }

    bool line_counted = false; // demand-fetch diagnostic: once per returned line
    while (fetch_bw.has_remaining() && !l1i_entry.instr_depend_on_me.empty()) {
      auto fetched = std::find_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), ooo_model_instr::matches_id(l1i_entry.instr_depend_on_me.front()));
      if (fetched != std::end(IFETCH_BUFFER) && champsim::block_number{fetched->ip} == champsim::block_number{l1i_entry.v_address} && fetched->fetch_issued) {
        fetched->fetch_completed = true;
        fetched->tax_fetch_lat = static_cast<uint32_t>((current_time - fetched->fetch_issue_time) / clock_period);
        if (fetched->emissary_pending) {
          fetched->emissary_pending = false;
          emissary_mark(fetched->ip); // the line is in the L1I now
        }
        if (rp_mode == 2 && fetched->rp_key != 0 && fetched->tax_fetch_lat > STALL_L1I_MISS_CYCLES) {
          rp_path_line(fetched->rp_key, champsim::block_number{fetched->ip}.to<uint64_t>()); // an L1I-missed line on a resteer path
        }
        // a demand fetch the u-op-cache miss sent to L1I and that missed there:
        // the latency a line prefetch hides (how many, and how long they waited)
        if (!line_counted && !warmup && current_time - fetched->fetch_issue_time > STALL_L1I_MISS_CYCLES * clock_period) {
          const auto wait = static_cast<uint64_t>((current_time - fetched->fetch_issue_time) / clock_period);
          ++(in_recovery ? sim_stats.fetch_l1i_miss_recovery : sim_stats.fetch_l1i_miss_steady);
          (in_recovery ? sim_stats.fetch_l1i_wait_recovery : sim_stats.fetch_l1i_wait_steady) += wait;
          line_counted = true;
        }
        // L1I-miss admission gate: a fetch that took longer than an L1I hit round
        // trip missed L1I; mark the segmenter's in-flight stretch (loose
        // attribution, same style as the ROB-drain note_stall signal).
        if (trace_stall_enable && (!warmup || trace_warmup_train) && current_time - fetched->fetch_issue_time > STALL_L1I_MISS_CYCLES * clock_period) {
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

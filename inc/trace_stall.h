#ifndef TRACE_STALL_H
#define TRACE_STALL_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <deque>
#include <vector>

#include "core_stats.h"
#include "trace_coverage.h"
#include "trace_store.h"

// Stall-triggered segmenter.  Instead of segmenting hot code, it records the code
// that runs during a *costly* build-mode stretch -- one in which the ROB drained
// empty (a genuine backend-idle stall).  A stretch is buffered from the first
// u-op-cache miss that entered build mode until the next hit, and is committed
// only if the ROB went empty during it (marked via note_rob_empty()).  Keyed by
// the stretch-start IP.  This targets exactly the tight backend-idle loss -- both
// post-misprediction refills and steady-state long build-mode stretches.  Capture
// is forward/streaming (tap the fetch stream); no backward walk.  Reuses the shared
// coverage_meter, so covers()/coverage are measured the same way as the stager.
class trace_stall
{
public:
  struct trace {
    uint64_t entry = 0;
    std::vector<uint64_t> ips;
    uint64_t occurrences = 0; // how many times this stretch (start IP) was captured
    uint64_t cost = 0;        // ROB-stall cycles observed across all captures of this stretch
  };

  static constexpr std::size_t DEFAULT_MAX_UOPS = 64;

  trace_stall() : max_uops(env_max_uops()) {}

  // occurrence filter: only capture a stretch once its start IP has been seen
  // >= min_occ times (1 = no filtering).  Prunes one-shot stretches (which can't
  // be prefetched anyway) so the store holds only the recurring, valuable ones.
  void configure(int min_occ) { threshold_ = (min_occ < 1) ? 1 : static_cast<uint64_t>(min_occ); }

  // max trace length in u-ops (a build-mode stretch longer than this is truncated).
  void set_max_uops(int d)
  {
    if (d > 0) {
      max_uops = static_cast<std::size_t>(d);
    }
  }

  // window granularity for the starve-offset measurement (log2 window bytes)
  void set_window_bits(unsigned b) { window_bits_ = b; }

  // costly-mark attribution (trace_stall_attrib):
  //   0 = a ROB-empty starve cycle marks whichever stretch is open at the u-op-cache
  //       lookup (legacy; the lookup runs far ahead of dispatch, so the mark often
  //       lands on a later stretch than the one that starved dispatch);
  //   1 = dispatch-side: the mark goes to the stretch containing the instruction
  //       dispatch is waiting for, and a closed stretch is committed or discarded
  //       only once dispatch has passed it.  Models a recorder at dispatch fed by a
  //       1-bit u-op-cache-miss tag per instruction.
  void set_attrib(int mode) { attrib_ = mode; }
  [[nodiscard]] int attrib() const { return attrib_; }

  // re-admission of evicted traces (trace_stall_readmit).  The segmenter's history
  // of captured stretches is unbounded, so a re-captured stretch used to count as
  // "already stored" even after the trace store had evicted it -- and was never
  // re-inserted (hardware has no such history: it looks the entry up in the store).
  //   0 = legacy (never re-inserted; such re-captures are only counted as lost)
  //   1 = strict: an evicted stretch re-qualifies like a new candidate (min_occ fresh sightings)
  //   2 = immediate: re-inserted on its next costly occurrence
  void set_readmit(int mode) { readmit_ = mode; }
  [[nodiscard]] int readmit() const { return readmit_; }

  // trace_stall_split: a miss run longer than the trace depth is not truncated --
  // the full stretch closes and recording continues in a new stretch (a
  // continuation), until a u-op-cache hit ends the run.  A continuation inherits
  // the costly verdict (and cost) of the segment before it, so a costly run is
  // stored whole as consecutive traces; consecutive costly stretches are what
  // trace chaining links, so the segments come out chained.
  void set_split(bool on) { split_ = on; }
  [[nodiscard]] bool split() const { return split_; }

  // trace_run_format: every trace is a run of sequential u-op-cache windows.  A
  // miss stretch is cut where the next instruction's window is neither the
  // current one nor the next one (a jump, or a taken branch leaving the window),
  // and where a run reaches max_windows; the rest of the stretch continues as a
  // new trace that inherits the costly verdict.  The trace depth still bounds the
  // stretch as a whole.  Consecutive runs are consecutive costly stretches, so
  // trace chaining links them (and the last run to the next costly trace).
  // mode 2 = head run only: the stretch stops recording at its first cut, but its
  // instruction range still runs to the closing hit, so a stall anywhere in the
  // stretch makes the head costly; one trace per stretch, and the successor
  // pointer always links stretch to stretch
  void set_run_format(int mode, int max_windows)
  {
    run_ = (mode != 0);
    run_mode_ = mode;
    run_max_windows_ = std::max(1, max_windows);
  }
  [[nodiscard]] int run_format() const { return run_mode_; }
  [[nodiscard]] int run_max_windows() const { return run_max_windows_; }
  // stretches to (re-)insert into the store after this on_dib / advance call
  [[nodiscard]] const std::vector<trace>& readmits() const { return readmits_; }

  // resolved-knob accessors (startup self-report)
  [[nodiscard]] uint64_t min_occ() const { return threshold_; }
  [[nodiscard]] std::size_t max_trace_uops() const { return max_uops; }
  [[nodiscard]] bool l1i_gated() const { return l1i_gate_; }

  // L1I admission gate: when enabled, a costly stretch commits only if an L1I
  // miss was observed during it (note_l1i_miss).  Selects the stretches where a
  // zero-latency u-op replay saves the most: byte fetch AND decode.
  void set_l1i_gate(bool g) { l1i_gate_ = g; }

  // mispredict-keyed capture (trace_stall_mispredict): called at a branch-
  // misprediction flush.  Arms a one-shot tag: if a FRESH stretch begins at
  // `target` with a u-op-cache miss, it commits under `key` at the next hit
  // WITHOUT the ROB-drain gate -- recovery stretches starve dispatch without
  // draining the ROB, so the costly gate would filter out exactly these events.
  // The in-flight stretch is left alone: the stream is trace-driven (no wrong
  // path ever enters the buffer), and killing it at every mispredict starves
  // the steady-state capture path.  If a stretch spans the flush, this arm is
  // simply consumed without tagging.
  void arm_event(uint64_t key, uint64_t target)
  {
    armed_ = true;
    armed_key_ = key;
    armed_target_ = target;
  }

  // an instruction fetch belonging to the fetch stream completed slower than an
  // L1I hit while this stretch was recording (loose attribution, like note_stall)
  void note_l1i_miss()
  {
    if (recording) {
      l1i_missed_ = true;
    }
  }

  // per instruction at the u-op-cache check (fetch stage).  hit = final DIB hit.
  // store: the trace store the commits feed (nullptr when trace-fill is off) --
  // consulted only to see whether a re-captured stretch is still resident
  // mispredicted: this instruction is a branch the predictor got wrong
  // (trace_stall_mispredict_open: it opens a stretch like a u-op-cache miss does)
  // mp_key: the stretch's store key under trace_stall_mispredict_open 2 (branch + the
  // direction it really went; probed when the branch is predicted the other way)
  // cycle: the current cycle (trace_stall_trigger_lead keys stretches by time)
  // ctx: an opaque context tag carried by the stretch its instruction opens and
  // reported back with its costly verdict (trace_vpred: the last prediction before it)
  void on_dib(uint64_t ip, bool hit, cpu_stats& stats, uint64_t instr_id = 0, const trace_store* store = nullptr, bool mispredicted = false,
              uint64_t mp_key = 0, uint64_t cycle = 0, uint64_t ctx = 0)
  {
    touches_.clear();
    costly_.clear();
    cur_ctx_ = ctx;
    if (mispredicted) {
      mp_seen_ = true; // the next stretch to open follows a misprediction
    }
    readmits_.clear();
    store_ = store;
    touched_event_ = false;
    note_fetch(ip, cycle);
    if (armed_) {
      if (!hit && !recording && ip == armed_target_) {
        event_tag_ = armed_key_; // the restart stretch begins with a miss: tag it
      }
      armed_ = false; // one-shot: consumed by the first post-flush DIB check
    }
    cov.observe(ip, stats.stall_dynamic_uops, stats.stall_dynamic_uops_covered, stats.stall_unique_ips_seen);
    if (mp_open_ != 0 && mispredicted) {
      // a mispredicted branch opens a stretch, as a u-op-cache miss does: the open
      // one closes first, and the new stretch starts at the branch.  What follows
      // it is the path fetch takes after resolution -- the correct path
      if (recording) {
        close_stretch(stats);
      }
      open_stretch(ip, instr_id, (mp_open_ == 2 && mp_key != 0) ? mp_key : lead_key(ip, cycle, stats));
      forced_ = true;
      ++stats.stall_mp_opened;
      return;
    }
    if (hit) {
      if (recording && forced_) {
        // a misprediction-opened stretch records the path after the branch, hits
        // included, up to the trace depth (resident windows cost nothing to walk)
        ++stretch_len_;
        buf.push_back(ip);
        last_ip_ = ip;
        if (stretch_len_ >= max_uops) {
          close_stretch(stats);
        }
        return;
      }
      if (recording) {
        close_stretch(stats);
      }
      return;
    }
    // u-op-cache miss -> build mode: keep buffering the stretch
    if (!recording) {
      open_stretch(ip, instr_id, lead_key(ip, cycle, stats));
    } else if (run_ && !forced_) {
      // trace_run_format: grow the run while windows stay sequential, else cut
      if (run_frozen_ || run_total_ >= max_uops) {
        ++stretch_len_; // past the cut (head-only) or the trace depth: the stretch goes on unrecorded
      } else {
        const uint64_t w = ip >> window_bits_;
        const uint64_t lw = last_ip_ >> window_bits_;
        const bool jump = (w != lw) && (w != lw + 1);
        const bool full = (w == lw + 1) && (run_windows_ >= run_max_windows_);
        if ((jump || full) && run_mode_ == 2) {
          // head run only: keep the stretch open (its costly verdict covers the rest)
          // but record nothing more
          run_frozen_ = true;
          ++stretch_len_;
          ++(jump ? stats.run_cut_jump : stats.run_cut_full);
        } else if (jump || full) {
          const uint64_t total = run_total_;
          close_stretch(stats);
          open_stretch(ip, instr_id, lead_key(ip, cycle, stats));
          cont_ = true;
          run_total_ = total + 1;
          ++(jump ? stats.run_cut_jump : stats.run_cut_full);
        } else {
          if (w == lw + 1) {
            ++run_windows_;
          }
          ++stretch_len_;
          buf.push_back(ip);
          last_ip_ = ip;
          ++run_total_;
        }
      }
    } else if (split_ && !forced_ && buf.size() >= max_uops) {
      // trace_stall_split: the stretch is full and the miss run goes on -- close
      // it and continue the run in a new stretch
      close_stretch(stats);
      open_stretch(ip, instr_id, lead_key(ip, cycle, stats));
      cont_ = true;
      ++stats.stall_split_segments;
    } else {
      ++stretch_len_;
      if (buf.size() < max_uops) {
        buf.push_back(ip);
      } else if (win_hist_ && (tail_wins_.empty() || tail_wins_.back() != (ip >> window_bits_))) {
        tail_wins_.push_back(ip >> window_bits_); // past the depth: unrecorded, remembered for the window history
      }
      last_ip_ = ip;
      if (forced_ && stretch_len_ >= max_uops) {
        close_stretch(stats);
      }
    }
  }

  void set_mispredict_open(int mode) { mp_open_ = mode; }

  // window history (trace_trigger_diag): per window, how many costly stretches held
  // it in their recorded part, and whether one held it past the depth cap
  void set_window_history(bool on) { win_hist_ = on; }
  void set_filter_cap(std::size_t n) { filter_cap_ = n; }
  [[nodiscard]] std::size_t filter_cap() const { return filter_cap_; }
  [[nodiscard]] uint32_t costly_count(uint64_t wtag) const
  {
    auto it = costly_win_.find(wtag);
    return (it == costly_win_.end()) ? 0 : it->second;
  }
  [[nodiscard]] bool costly_past_depth(uint64_t wtag) const { return costly_tail_win_.count(wtag) != 0; }
  // the open stretch is past the depth cap: a miss now extends it unrecorded
  [[nodiscard]] bool past_depth_now() const { return recording && !forced_ && !split_ && !run_ && stretch_len_ >= max_uops; }
  [[nodiscard]] int mispredict_open() const { return mp_open_; }

  // trace_stall_trigger_lead: store each stretch under the pc that reached the
  // u-op-cache check at least this many cycles before the stretch opened, instead
  // of under its own first pc.  The cursor then meets the key that much earlier
  // than the stretch (plus the key's own FTQ residence), so the walk's lead is set
  // by design rather than by FTQ occupancy.  After a misprediction the rule lands
  // on the branch or just before it (fetch waited on resolution), i.e. before the
  // point where the cursor stops.  0 = keyed by the stretch's own pc (legacy).
  void set_trigger_lead(uint64_t cycles) { lead_ = cycles; }
  [[nodiscard]] uint64_t trigger_lead() const { return lead_; }

  // a costly-enough backend stall was observed in build mode (ROB at/below the
  // configured threshold): mark the in-flight stretch as costly, so it will be
  // committed at the next hit.  Called once per stalled cycle, so the count is
  // the stretch's cost in ROB-stall cycles.  Returns false when no stretch is
  // open -- the close-to-drain attribution race (the stretch already committed
  // before its pipeline bubble finished draining the ROB); the caller counts
  // these dropped marks.
  // starve_id: sequence number of the instruction dispatch is waiting for (the
  // oldest in the front end); the stretch's first one is kept for the
  // starve-offset measurement.
  bool note_stall(uint64_t starve_id)
  {
    if (attrib_ == 1) {
      // charge the stretch that contains the stalled instruction (stretch
      // instructions are consecutive misses, so each covers an id range); none
      // contains it when that instruction hit in the u-op cache or has not been
      // looked up yet -- not a u-op-supply stall
      if (recording && starve_id >= start_id_ && starve_id - start_id_ < stretch_len_) {
        if (!costly) {
          first_starve_id_ = starve_id;
        }
        costly = true;
        ++stretch_cost;
        return true;
      }
      for (auto& s : awaiting_) {
        if (starve_id >= s.start_id && starve_id - s.start_id < s.len) {
          if (!s.costly) {
            s.first_starve_id = starve_id;
          }
          s.costly = true;
          ++s.cost;
          return true;
        }
      }
      return false;
    }
    if (recording) {
      if (!costly) {
        first_starve_id_ = starve_id;
      }
      costly = true;
      ++stretch_cost;
      return true;
    }
    return false;
  }

  // dispatch-side attribution: dispatch's oldest not-yet-dispatched instruction
  // (UINT64_MAX when the front end is empty).  Every held stretch whose last
  // instruction is older than it can no longer be charged: commit or discard it.
  void advance(uint64_t oldest_undispatched_id, cpu_stats& stats, const trace_store* store = nullptr)
  {
    touches_.clear();
    costly_.clear();
    readmits_.clear();
    store_ = store;
    while (!awaiting_.empty() && awaiting_.front().start_id + awaiting_.front().len <= oldest_undispatched_id) {
      finalize(awaiting_.front(), stats);
      awaiting_.pop_front();
    }
  }

  [[nodiscard]] bool covers(uint64_t ip) const { return cov.covers(ip); }
  [[nodiscard]] const std::vector<trace>& get_traces() const { return traces; }

  // stretches this on_dib / advance call committed or re-captured, as (entry,
  // accumulated cost) -- feed hook: lets the fill store refresh a stored trace's
  // cost on re-capture.  advance() can finalize several held stretches at once.
  [[nodiscard]] const std::vector<std::pair<uint64_t, uint64_t>>& touches() const { return touches_; }

  // stretches that received a costly verdict in the last on_dib / advance call, as
  // (context tag of their first instruction, first pc) -- whether or not they are
  // then stored (occurrence filter, L1I gate)
  struct costly_event {
    uint64_t ctx = 0;      // context tag of its first instruction
    uint64_t first_pc = 0; // its first instruction
    uint64_t start_id = 0;
    uint64_t end_id = 0;   // its last instruction
  };
  [[nodiscard]] const std::vector<costly_event>& costly_events() const { return costly_; }

  // same hook for a re-captured event-keyed stretch (both can fire in one on_dib)
  [[nodiscard]] bool touched_event() const { return touched_event_; }
  [[nodiscard]] uint64_t touch_event_entry() const { return touch_event_entry_; }
  [[nodiscard]] uint64_t touch_event_cost() const { return touch_event_cost_; }

  // end-of-run Pareto analysis: cumulative dynamic weight (occurrences x length)
  // captured by the top-N traces, ranked by occurrence, by dynamic weight, and by
  // cost (ROB-stall cycles) -- the cost ranking answers "how small can a store be
  // if it keeps only the traces whose stretches actually hurt?".
  void finalize_buckets(cpu_stats& stats) const
  {
    const std::size_t n = traces.size();
    std::vector<uint64_t> w(n);
    uint64_t total = 0, total_cost = 0;
    for (std::size_t i = 0; i < n; ++i) {
      w[i] = traces[i].occurrences * traces[i].ips.size();
      total += w[i];
      total_cost += traces[i].cost;
    }
    stats.stall_total_dynweight = total;
    stats.stall_total_cost = total_cost;

    static constexpr std::array<std::size_t, 7> BKT = {16, 32, 64, 128, 256, 512, 1024};
    std::vector<std::size_t> idx(n);
    std::iota(idx.begin(), idx.end(), std::size_t{0});

    // cumulative dyn-weight and stall-cost at each bucket, plus avg occurrence and
    // avg length of the top-N traces (written into the supplied output arrays).
    auto cumulative = [&](const std::vector<std::size_t>& order, std::array<uint64_t, 7>& cumcost, std::array<double, 7>& avgocc,
                          std::array<double, 7>& avglen) {
      std::array<uint64_t, 7> out{};
      uint64_t run_w = 0, run_occ = 0, run_len = 0, run_cost = 0;
      std::size_t bi = 0;
      auto record = [&](std::size_t cnt) {
        out[bi] = run_w;
        cumcost[bi] = run_cost;
        const double c = cnt ? static_cast<double>(cnt) : 1.0;
        avgocc[bi] = static_cast<double>(run_occ) / c;
        avglen[bi] = static_cast<double>(run_len) / c;
        ++bi;
      };
      for (std::size_t k = 0; k < order.size(); ++k) {
        run_w += w[order[k]];
        run_occ += traces[order[k]].occurrences;
        run_len += traces[order[k]].ips.size();
        run_cost += traces[order[k]].cost;
        while (bi < BKT.size() && (k + 1) >= BKT[bi]) {
          record(k + 1);
        }
      }
      while (bi < BKT.size()) { // fewer traces than the bucket: all of them
        record(order.size());
      }
      return out;
    };

    std::sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) { return traces[a].occurrences > traces[b].occurrences; });
    const auto occ = cumulative(idx, stats.stall_occ_cumcost, stats.stall_occ_avgocc, stats.stall_occ_avglen);
    std::sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) { return w[a] > w[b]; });
    const auto cvg = cumulative(idx, stats.stall_cov_cumcost, stats.stall_cov_avgocc, stats.stall_cov_avglen);
    std::sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) { return traces[a].cost > traces[b].cost; });
    stats.stall_cost_cumw = cumulative(idx, stats.stall_cost_cum, stats.stall_cost_avgocc, stats.stall_cost_avglen);
    // past 1024: how much stall cost the costliest 2K / 4K / 8K traces hold (sizing a smaller store)
    {
      static constexpr std::array<std::size_t, 3> BIG = {2048, 4096, 8192};
      uint64_t run_cost = 0;
      std::size_t bi = 0;
      for (std::size_t k = 0; k < idx.size() && bi < BIG.size(); ++k) {
        run_cost += traces[idx[k]].cost;
        while (bi < BIG.size() && (k + 1) >= BIG[bi]) {
          stats.stall_cost_cum_big[bi++] = run_cost;
        }
      }
      while (bi < BIG.size()) {
        stats.stall_cost_cum_big[bi++] = total_cost; // fewer traces than the bucket: all of them
      }
    }

    stats.stall_occ_top16 = occ[0], stats.stall_occ_top32 = occ[1], stats.stall_occ_top64 = occ[2], stats.stall_occ_top128 = occ[3];
    stats.stall_occ_top256 = occ[4], stats.stall_occ_top512 = occ[5], stats.stall_occ_top1024 = occ[6];
    stats.stall_cov_top16 = cvg[0], stats.stall_cov_top32 = cvg[1], stats.stall_cov_top64 = cvg[2], stats.stall_cov_top128 = cvg[3];
    stats.stall_cov_top256 = cvg[4], stats.stall_cov_top512 = cvg[5], stats.stall_cov_top1024 = cvg[6];
  }

private:
  struct closed_stretch {
    uint64_t start_ip = 0;
    uint64_t start_id = 0;
    uint64_t len = 0;            // instructions in the stretch (not capped by max_uops)
    std::vector<uint64_t> ips;   // recorded part (capped at max_uops)
    bool costly = false;
    uint64_t cost = 0;           // ROB-stall cycles charged to it
    bool l1i_missed = false;
    uint64_t first_starve_id = 0;
    bool forced = false;         // opened by a misprediction (trace_stall_mispredict_open)
    uint64_t ctx = 0;            // context tag of its first instruction (see on_dib)
    uint64_t end_ip = 0;         // its last instruction
    bool after_mp = false;       // a misprediction came after the previous stretch opened and before this one
    bool cont = false;           // trace_stall_split: continues the stretch before it (same miss run)
    std::vector<uint64_t> tail_wins; // window history: windows past the depth cap
  };
  static constexpr std::size_t MAX_AWAITING = 64; // stretches between lookup and dispatch (front end holds a few hundred instructions)

  // succession statistic (no timing effect): for consecutive costly stretches A -> B
  // (program order -- finalize runs in order), where B starts relative to A's last
  // instruction and how many instructions lie between them, split by whether a
  // misprediction came in between.  Sizes a "next trace = fall-through of this one"
  // chaining rule against a stored successor pointer.
  void note_succession(const closed_stretch& s, cpu_stats& stats)
  {
    const uint64_t first = s.ips.empty() ? s.start_ip : s.ips.front();
    if (prev_valid_ && s.start_id > prev_end_id_) {
      const auto d = static_cast<int64_t>(first - prev_end_ip_);
      const std::size_t a = (d > 0 && d <= 16) ? 0 : (d > 0 && d <= 256) ? 1 : (d > -4096 && d <= 4096) ? 2 : 3;
      const uint64_t gap = s.start_id - prev_end_id_ - 1;
      const std::size_t g = (gap == 0) ? 0 : (gap <= 8) ? 1 : (gap <= 64) ? 2 : (gap <= 256) ? 3 : 4;
      ++(s.after_mp ? stats.succ_addr_mp : stats.succ_addr_steady)[a];
      ++(s.after_mp ? stats.succ_gap_mp : stats.succ_gap_steady)[g];
    }
    prev_valid_ = true;
    prev_end_id_ = s.start_id + s.len - 1;
    prev_end_ip_ = s.end_ip;
  }

  // trace_stall_trigger_lead: remember when each pc reached the u-op-cache check;
  // the front is kept at the newest entry at least lead_ cycles old
  void note_fetch(uint64_t ip, uint64_t cycle)
  {
    if (lead_ == 0) {
      return;
    }
    lead_ring_.emplace_back(cycle, ip);
    while (lead_ring_.size() > 1 && lead_ring_[1].first + lead_ <= cycle) {
      lead_ring_.pop_front();
    }
    if (lead_ring_.size() > LEAD_RING_MAX) {
      lead_ring_.pop_front(); // dense fetch over a long lead: the front is younger than lead_ (counted as self-keyed)
    }
  }

  // the key a stretch opening now is stored under: the pc checked at least lead_
  // cycles ago, or the stretch's own pc when none is that old (or the knob is off)
  uint64_t lead_key(uint64_t ip, uint64_t cycle, cpu_stats& stats) const
  {
    if (lead_ == 0) {
      return ip;
    }
    if (lead_ring_.empty() || lead_ring_.front().first + lead_ > cycle) {
      ++stats.stall_lead_self;
      return ip;
    }
    ++stats.stall_lead_keyed;
    return lead_ring_.front().second;
  }

  // key: the store key (the first instruction, or an event key for mp_open 2)
  void open_stretch(uint64_t ip, uint64_t instr_id, uint64_t key = 0)
  {
    recording = true;
    start_ctx_ = cur_ctx_;
    costly = false;
    forced_ = false;
    cont_ = false;
    run_windows_ = 1;
    run_total_ = 1;
    run_frozen_ = false;
    start_ip = (key != 0) ? key : ip;
    start_id_ = instr_id;
    stretch_len_ = 1;
    buf.clear();
    buf.push_back(ip);
    tail_wins_.clear();
    last_ip_ = ip;
    start_after_mp_ = mp_seen_;
    mp_seen_ = false;
  }

  // the stretch ends (a u-op-cache hit, or a misprediction-opened stretch reached
  // the trace depth / a new misprediction): hand it to the costly verdict
  void close_stretch(cpu_stats& stats)
  {
    if (event_tag_ != 0) {
      commit_event(stats); // mispredict-keyed: no ROB-drain gate (see arm_event)
    }
    closed_stretch s{start_ip, start_id_, stretch_len_, buf, costly, stretch_cost, l1i_missed_, first_starve_id_, forced_, start_ctx_, last_ip_,
                     start_after_mp_, cont_};
    s.tail_wins = std::move(tail_wins_);
    tail_wins_.clear();
    if (attrib_ == 1) {
      // dispatch-side: dispatch has not reached this stretch yet, so its costly
      // verdict is still open -- hold it until dispatch passes its last instruction
      awaiting_.push_back(std::move(s));
      while (awaiting_.size() > MAX_AWAITING) {
        finalize(awaiting_.front(), stats);
        awaiting_.pop_front();
      }
    } else {
      finalize(s, stats);
    }
    reset();
  }

  // the costly verdict on a stretch is final: commit (subject to the L1I gate and
  // the occurrence filter) or discard
  void finalize(const closed_stretch& in, cpu_stats& stats)
  {
    // stretches finalize in program order, so the previous call is the segment a
    // continuation follows (trace_stall_split)
    const bool inherit = (split_ || run_) && in.cont && !in.costly && prev_final_costly_;
    closed_stretch inherited;
    if (inherit) {
      inherited = in;
      inherited.costly = true;
      inherited.cost = std::max(in.cost, prev_final_cost_);
      ++stats.stall_split_inherited;
    }
    const closed_stretch& s = inherit ? inherited : in;
    prev_final_costly_ = s.costly;
    prev_final_cost_ = s.cost;
    if (!s.costly) {
      return;
    }
    if (win_hist_) {
      uint64_t last = ~uint64_t{0};
      for (uint64_t v : s.ips) {
        if ((v >> window_bits_) != last) {
          last = v >> window_bits_;
          ++costly_win_[last];
        }
      }
      costly_tail_win_.insert(std::begin(s.tail_wins), std::end(s.tail_wins));
    }
    costly_.push_back({s.ctx, s.ips.empty() ? s.start_ip : s.ips.front(), s.start_id, s.start_id + s.len - 1});
    note_succession(s, stats);
    if (!inherit) {
      measure_starve_offset(s, stats);
    }
    if (!l1i_gate_ || s.l1i_missed) {
      commit(s, stats);
    } else {
      ++stats.stall_gated_l1i; // costly but no L1I miss observed: gated out
    }
  }

  // where the first starving instruction of the closing (costly) stretch sits,
  // located by instruction sequence number (stretch instructions are consecutive
  // misses): window offset 0,1,2,3,4-7,8+ inside the recorded part; [6] inside the
  // stretch but past the recorded depth; [7] older than the stretch's first
  // instruction (an earlier stretch's instructions starved dispatch); [8] at or
  // after the closing hit.  Measurement only.
  void measure_starve_offset(const closed_stretch& s, cpu_stats& stats) const
  {
    std::size_t bucket = 0;
    if (s.first_starve_id < s.start_id) {
      bucket = 7;
    } else if (const uint64_t off = s.first_starve_id - s.start_id; off >= s.len) {
      bucket = 8;
    } else if (off >= s.ips.size()) {
      bucket = 6;
    } else {
      int windows = 0;
      for (std::size_t k = 1; k <= off; ++k) {
        if ((s.ips[k] >> window_bits_) != (s.ips[k - 1] >> window_bits_)) {
          ++windows;
        }
      }
      bucket = (windows <= 3) ? static_cast<std::size_t>(windows) : (windows <= 7) ? 4 : 5;
    }
    ++stats.stall_starve_offset[bucket];
  }

  void commit(const closed_stretch& s, cpu_stats& stats)
  {
    if (s.ips.empty()) {
      return;
    }
    if (auto it = start_index.find(s.start_ip); it != start_index.end()) {
      ++traces[it->second].occurrences; // re-capture of an already-stored stretch
      traces[it->second].cost += s.cost;
      ++stats.stall_dedup;
      if (lead_ != 0 && traces[it->second].ips.front() != s.ips.front()) {
        ++stats.stall_lead_conflicts; // the trigger now precedes a stretch other than the stored one
      }
      touches_.emplace_back(s.start_ip, traces[it->second].cost); // expose the refreshed cost to the fill feed
      if (store_ != nullptr && !store_->contains(s.start_ip)) {
        // the store evicted it: hardware would see an unknown stretch here
        ++stats.stall_lost_recaptures;
        bool readmit_now = (readmit_ == 2);
        if (readmit_ == 1) {
          auto& rp = readmit_pending_[s.start_ip];
          rp.touch = ++filter_tick_;
          readmit_now = (++rp.count >= threshold_);
          if (!readmit_now) {
            bound_filter(stats);
          }
        }
        if (readmit_now) {
          readmit_pending_.erase(s.start_ip);
          trace tr;
          tr.entry = s.start_ip;
          tr.ips = s.ips; // the current occurrence, as a recorder would capture it
          tr.occurrences = traces[it->second].occurrences;
          tr.cost = traces[it->second].cost;
          readmits_.push_back(std::move(tr));
          ++stats.stall_readmits;
        }
      }
      return;
    }
    // candidate stretch: accumulate occurrences until it clears the filter threshold
    auto [pit, inserted] = pending.try_emplace(s.start_ip);
    if (inserted) {
      ++stats.stall_traces_candidates; // first sighting of this stretch-start (unfiltered total)
    }
    pit->second.cost += s.cost;
    pit->second.touch = ++filter_tick_;
    if (++pit->second.count < threshold_) {
      if (inserted) {
        bound_filter(stats); // a new candidate may push the least recent one out (pit is the most recent)
      }
      return; // filtered out for now -- not recurred enough to capture
    }
    trace tr;
    tr.entry = s.start_ip;
    tr.ips = s.ips; // IPs from the occurrence that clears the threshold
    tr.occurrences = pit->second.count;
    tr.cost = pit->second.cost;
    for (uint64_t v : tr.ips) {
      cov.cover(v, stats.stall_unique_ips_covered);
    }
    stats.stall_stored_uops += tr.ips.size();
    ++stats.stall_traces;
    if (s.forced) {
      ++stats.stall_mp_traces;
    }
    if (s.cont) {
      ++stats.stall_split_traces;
    }
    if ((s.start_ip >> 63) == 0 && s.start_ip != tr.ips.front()) {
      // keyed by an earlier pc: the entry also names the stretch's first pc --
      // its distance from the key sizes that field
      ++stats.stall_lead_traces;
      const uint64_t d = (s.start_ip > tr.ips.front()) ? s.start_ip - tr.ips.front() : tr.ips.front() - s.start_ip;
      ++stats.stall_lead_dist[(d <= (uint64_t{1} << 12)) ? 0 : (d <= (uint64_t{1} << 16)) ? 1 : (d <= (uint64_t{1} << 20)) ? 2 : 3];
    }
    touches_.emplace_back(s.start_ip, tr.cost);
    start_index.emplace(s.start_ip, traces.size());
    traces.push_back(std::move(tr));
    pending.erase(pit);
  }

  // commit the in-flight stretch under its mispredict-event key.  Same depth cap,
  // same occurrence filter, no ROB gate.  Cost floor of one cycle per u-op keeps
  // event traces competitive under cost-eviction (their stretches rarely trip the
  // ROB-drain condition, so raw stretch_cost is usually zero).
  void commit_event(cpu_stats& stats)
  {
    if (buf.empty()) {
      return;
    }
    const uint64_t ev_cost = std::max<uint64_t>(stretch_cost, buf.size());
    if (auto it = start_index.find(event_tag_); it != start_index.end()) {
      ++traces[it->second].occurrences;
      traces[it->second].cost += ev_cost;
      ++stats.stall_event_dedup;
      touched_event_ = true;
      touch_event_entry_ = event_tag_;
      touch_event_cost_ = traces[it->second].cost;
      return;
    }
    auto [pit, inserted] = pending.try_emplace(event_tag_);
    if (inserted) {
      ++stats.stall_event_candidates;
    }
    pit->second.cost += ev_cost;
    if (++pit->second.count < threshold_) {
      return;
    }
    trace tr;
    tr.entry = event_tag_;
    tr.ips = buf;
    tr.occurrences = pit->second.count;
    tr.cost = pit->second.cost;
    ++stats.stall_event_traces;
    touched_event_ = true;
    touch_event_entry_ = event_tag_;
    touch_event_cost_ = tr.cost;
    start_index.emplace(event_tag_, traces.size());
    traces.push_back(std::move(tr));
    pending.erase(pit);
  }

  void reset()
  {
    recording = false;
    costly = false;
    l1i_missed_ = false;
    stretch_cost = 0;
    event_tag_ = 0;
    forced_ = false;
    buf.clear();
  }

  static std::size_t env_max_uops()
  {
    if (const char* e = std::getenv("PROMETHEUS_STALL_DEPTH"); e != nullptr) {
      if (const long v = std::atol(e); v > 0) {
        return static_cast<std::size_t>(v);
      }
    }
    return DEFAULT_MAX_UOPS;
  }

  struct pend {
    uint64_t count = 0; // occurrences so far (below threshold)
    uint64_t cost = 0;  // ROB-stall cycles accrued while still filtered
    uint64_t touch = 0; // trace_stall_filter_entries: last sighting (LRU)
  };

  // trace_stall_filter_entries: hardware keeps candidates (stretch starts seen fewer
  // than min_occ times, or evicted traces re-qualifying) in a bounded table; on
  // overflow the least recently seen candidate is forgotten
  void bound_filter(cpu_stats& stats)
  {
    if (filter_cap_ == 0) {
      return;
    }
    while (pending.size() + readmit_pending_.size() > filter_cap_) {
      auto vp = pending.end();
      auto vr = readmit_pending_.end();
      uint64_t oldest = ~uint64_t{0};
      for (auto it = pending.begin(); it != pending.end(); ++it) {
        if (it->second.touch < oldest) {
          oldest = it->second.touch;
          vp = it;
          vr = readmit_pending_.end();
        }
      }
      for (auto it = readmit_pending_.begin(); it != readmit_pending_.end(); ++it) {
        if (it->second.touch < oldest) {
          oldest = it->second.touch;
          vr = it;
          vp = pending.end();
        }
      }
      if (vp != pending.end()) {
        pending.erase(vp);
      } else if (vr != readmit_pending_.end()) {
        readmit_pending_.erase(vr);
      } else {
        break;
      }
      ++stats.stall_filter_evictions;
    }
  }

  std::size_t max_uops;
  bool recording = false;
  bool costly = false;
  bool l1i_gate_ = false;    // admission gate: require an L1I miss in the stretch
  bool l1i_missed_ = false;  // an L1I miss was observed during the in-flight stretch
  std::vector<std::pair<uint64_t, uint64_t>> touches_; // (entry, cost) committed or re-captured by the last on_dib / advance
  std::vector<costly_event> costly_;                   // stretches judged costly in the last call, program order
  uint64_t cur_ctx_ = 0;   // context tag of the instruction being looked up
  uint64_t last_ip_ = 0;   // last instruction of the open stretch
  bool mp_seen_ = false;   // a misprediction since the last stretch opened
  bool split_ = false;     // trace_stall_split
  bool run_ = false;       // trace_run_format
  int run_mode_ = 0;       // 1 = every run stored, 2 = head run only
  bool run_frozen_ = false; // head-only: the open stretch has been cut, record nothing more
  int run_max_windows_ = 8;
  int run_windows_ = 0;    // windows in the open run
  uint64_t run_total_ = 0; // instructions recorded in the stretch so far, across its runs
  bool cont_ = false;      // the open stretch continues the previous one
  bool prev_final_costly_ = false; // verdict of the last finalized stretch (inherited by a continuation)
  uint64_t prev_final_cost_ = 0;
  bool start_after_mp_ = false;
  bool prev_valid_ = false; // previous costly stretch (succession statistic)
  uint64_t prev_end_id_ = 0;
  uint64_t prev_end_ip_ = 0;
  uint64_t start_ctx_ = 0; // ...of the open stretch's first instruction
  int attrib_ = 0;                                      // costly-mark attribution (set_attrib)
  int readmit_ = 0;                                     // re-admission of evicted traces (set_readmit)
  const trace_store* store_ = nullptr;                  // valid only during the current on_dib / advance call
  std::vector<trace> readmits_;                         // evicted stretches to re-insert after the current call
  std::unordered_map<uint64_t, pend> readmit_pending_; // readmit 1: fresh sightings since eviction (count, touch)
  std::size_t filter_cap_ = 0;                        // trace_stall_filter_entries (0 = unbounded)
  uint64_t filter_tick_ = 0;
  std::deque<closed_stretch> awaiting_;                 // attrib 1: closed at the lookup, not yet passed by dispatch
  int mp_open_ = 0;            // trace_stall_mispredict_open: mispredicted branches open stretches
  uint64_t lead_ = 0;          // trace_stall_trigger_lead: key stretches by the pc checked this many cycles earlier
  static constexpr std::size_t LEAD_RING_MAX = 16384;
  std::deque<std::pair<uint64_t, uint64_t>> lead_ring_; // (cycle, pc) at the u-op-cache check, oldest first
  bool forced_ = false;        // the open stretch was opened by a misprediction (records hits too, to the depth)
  bool touched_event_ = false; // this on_dib committed or re-captured an event-keyed stretch
  uint64_t touch_event_entry_ = 0;
  uint64_t touch_event_cost_ = 0;
  bool armed_ = false;         // one-shot mispredict tag pending (arm_event)
  uint64_t armed_key_ = 0;
  uint64_t armed_target_ = 0;
  uint64_t event_tag_ = 0;     // nonzero: in-flight stretch commits under this event key
  uint64_t start_ip = 0;
  uint64_t stretch_cost = 0; // ROB-stall cycles in the in-flight stretch
  uint64_t first_starve_id_ = 0; // sequence number of the instruction dispatch waited for at the stretch's first costly cycle
  uint64_t start_id_ = 0;        // sequence number of the stretch's first instruction
  uint64_t stretch_len_ = 0;     // instructions in the stretch so far (not capped by max_uops)
  bool win_hist_ = false;                             // window history on (trace_trigger_diag)
  std::unordered_map<uint64_t, uint32_t> costly_win_; // window -> costly stretches holding it (recorded part)
  std::unordered_set<uint64_t> costly_tail_win_;      // windows a costly stretch held past the depth cap
  std::vector<uint64_t> tail_wins_;                   // the open stretch's windows past the depth cap
  unsigned window_bits_ = 5;
  std::vector<uint64_t> buf;
  uint64_t threshold_ = 1;                               // min occurrences to capture (occurrence filter)
  std::vector<trace> traces;
  std::unordered_map<uint64_t, std::size_t> start_index; // start IP -> index in traces (captured)
  std::unordered_map<uint64_t, pend> pending;            // start IP -> occurrences+cost (below threshold)
  coverage_meter cov;
};

#endif

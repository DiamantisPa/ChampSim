#ifndef TRACE_STALL_H
#define TRACE_STALL_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "core_stats.h"
#include "trace_coverage.h"

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

  // L1I admission gate: when enabled, a costly stretch commits only if an L1I
  // miss was observed during it (note_l1i_miss).  Selects the stretches where a
  // zero-latency u-op replay saves the most: byte fetch AND decode.
  void set_l1i_gate(bool g) { l1i_gate_ = g; }

  // an instruction fetch belonging to the fetch stream completed slower than an
  // L1I hit while this stretch was recording (loose attribution, like note_stall)
  void note_l1i_miss()
  {
    if (recording) {
      l1i_missed_ = true;
    }
  }

  // per instruction at the u-op-cache check (fetch stage).  hit = final DIB hit.
  void on_dib(uint64_t ip, bool hit, cpu_stats& stats)
  {
    touched_ = false;
    cov.observe(ip, stats.stall_dynamic_uops, stats.stall_dynamic_uops_covered, stats.stall_unique_ips_seen);
    if (hit) {
      if (recording) {
        if (costly && (!l1i_gate_ || l1i_missed_)) {
          commit(stats);
        } else if (costly) {
          ++stats.stall_gated_l1i; // costly but no L1I miss observed: gated out
        }
        reset();
      }
      return;
    }
    // u-op-cache miss -> build mode: keep buffering the stretch
    if (!recording) {
      recording = true;
      costly = false;
      start_ip = ip;
      buf.clear();
      buf.push_back(ip);
    } else if (buf.size() < max_uops) {
      buf.push_back(ip);
    }
  }

  // a costly-enough backend stall was observed in build mode (ROB at/below the
  // configured threshold): mark the in-flight stretch as costly, so it will be
  // committed at the next hit.  Called once per stalled cycle, so the count is
  // the stretch's cost in ROB-stall cycles.
  void note_stall()
  {
    if (recording) {
      costly = true;
      ++stretch_cost;
    }
  }

  [[nodiscard]] bool covers(uint64_t ip) const { return cov.covers(ip); }
  [[nodiscard]] const std::vector<trace>& get_traces() const { return traces; }

  // did this on_dib call commit or re-capture a stretch? (feed hook: lets the
  // fill store refresh a stored trace's cost on re-capture)
  [[nodiscard]] bool touched() const { return touched_; }
  [[nodiscard]] uint64_t touch_entry() const { return touch_entry_; }
  [[nodiscard]] uint64_t touch_cost() const { return touch_cost_; }

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

    stats.stall_occ_top16 = occ[0], stats.stall_occ_top32 = occ[1], stats.stall_occ_top64 = occ[2], stats.stall_occ_top128 = occ[3];
    stats.stall_occ_top256 = occ[4], stats.stall_occ_top512 = occ[5], stats.stall_occ_top1024 = occ[6];
    stats.stall_cov_top16 = cvg[0], stats.stall_cov_top32 = cvg[1], stats.stall_cov_top64 = cvg[2], stats.stall_cov_top128 = cvg[3];
    stats.stall_cov_top256 = cvg[4], stats.stall_cov_top512 = cvg[5], stats.stall_cov_top1024 = cvg[6];
  }

private:
  void commit(cpu_stats& stats)
  {
    if (buf.empty()) {
      return;
    }
    if (auto it = start_index.find(start_ip); it != start_index.end()) {
      ++traces[it->second].occurrences; // re-capture of an already-stored stretch
      traces[it->second].cost += stretch_cost;
      ++stats.stall_dedup;
      touched_ = true; // expose the refreshed cost to the fill feed
      touch_entry_ = start_ip;
      touch_cost_ = traces[it->second].cost;
      return;
    }
    // candidate stretch: accumulate occurrences until it clears the filter threshold
    auto [pit, inserted] = pending.try_emplace(start_ip);
    if (inserted) {
      ++stats.stall_traces_candidates; // first sighting of this stretch-start (unfiltered total)
    }
    pit->second.cost += stretch_cost;
    if (++pit->second.count < threshold_) {
      return; // filtered out for now -- not recurred enough to capture
    }
    trace tr;
    tr.entry = start_ip;
    tr.ips = buf; // IPs from the occurrence that clears the threshold
    tr.occurrences = pit->second.count;
    tr.cost = pit->second.cost;
    for (uint64_t v : tr.ips) {
      cov.cover(v, stats.stall_unique_ips_covered);
    }
    stats.stall_stored_uops += tr.ips.size();
    ++stats.stall_traces;
    touched_ = true;
    touch_entry_ = start_ip;
    touch_cost_ = tr.cost;
    start_index.emplace(start_ip, traces.size());
    traces.push_back(std::move(tr));
    pending.erase(pit);
  }

  void reset()
  {
    recording = false;
    costly = false;
    l1i_missed_ = false;
    stretch_cost = 0;
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
  };

  std::size_t max_uops;
  bool recording = false;
  bool costly = false;
  bool l1i_gate_ = false;    // admission gate: require an L1I miss in the stretch
  bool l1i_missed_ = false;  // an L1I miss was observed during the in-flight stretch
  bool touched_ = false;     // this on_dib committed or re-captured a stretch
  uint64_t touch_entry_ = 0;
  uint64_t touch_cost_ = 0;
  uint64_t start_ip = 0;
  uint64_t stretch_cost = 0; // ROB-stall cycles in the in-flight stretch
  std::vector<uint64_t> buf;
  uint64_t threshold_ = 1;                               // min occurrences to capture (occurrence filter)
  std::vector<trace> traces;
  std::unordered_map<uint64_t, std::size_t> start_index; // start IP -> index in traces (captured)
  std::unordered_map<uint64_t, pend> pending;            // start IP -> occurrences+cost (below threshold)
  coverage_meter cov;
};

#endif

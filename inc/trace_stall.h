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
  };

  static constexpr std::size_t DEFAULT_MAX_UOPS = 64;

  trace_stall() : max_uops(env_max_uops()) {}

  // per instruction at the u-op-cache check (fetch stage).  hit = final DIB hit.
  void on_dib(uint64_t ip, bool hit, cpu_stats& stats)
  {
    cov.observe(ip, stats.stall_dynamic_uops, stats.stall_dynamic_uops_covered, stats.stall_unique_ips_seen);
    if (hit) {
      if (recording) {
        if (costly) {
          commit(stats);
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

  // the ROB was observed fully empty in build mode (the tight backend-idle event):
  // mark the in-flight stretch as costly, so it will be committed at the next hit.
  void note_rob_empty()
  {
    if (recording) {
      costly = true;
    }
  }

  [[nodiscard]] bool covers(uint64_t ip) const { return cov.covers(ip); }
  [[nodiscard]] const std::vector<trace>& get_traces() const { return traces; }

  // end-of-run Pareto analysis: cumulative dynamic weight (occurrences x length)
  // captured by the top-N traces, ranked by occurrence and by dynamic weight.
  void finalize_buckets(cpu_stats& stats) const
  {
    const std::size_t n = traces.size();
    std::vector<uint64_t> w(n);
    uint64_t total = 0;
    for (std::size_t i = 0; i < n; ++i) {
      w[i] = traces[i].occurrences * traces[i].ips.size();
      total += w[i];
    }
    stats.stall_total_dynweight = total;

    static constexpr std::array<std::size_t, 7> BKT = {16, 32, 64, 128, 256, 512, 1024};
    std::vector<std::size_t> idx(n);
    std::iota(idx.begin(), idx.end(), std::size_t{0});

    // cumulative dyn-weight at each bucket, plus avg occurrence and avg length of
    // the top-N traces (written into the supplied avgocc/avglen arrays).
    auto cumulative = [&](const std::vector<std::size_t>& order, std::array<double, 7>& avgocc, std::array<double, 7>& avglen) {
      std::array<uint64_t, 7> out{};
      uint64_t run_w = 0, run_occ = 0, run_len = 0;
      std::size_t bi = 0;
      auto record = [&](std::size_t cnt) {
        out[bi] = run_w;
        const double c = cnt ? static_cast<double>(cnt) : 1.0;
        avgocc[bi] = static_cast<double>(run_occ) / c;
        avglen[bi] = static_cast<double>(run_len) / c;
        ++bi;
      };
      for (std::size_t k = 0; k < order.size(); ++k) {
        run_w += w[order[k]];
        run_occ += traces[order[k]].occurrences;
        run_len += traces[order[k]].ips.size();
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
    const auto occ = cumulative(idx, stats.stall_occ_avgocc, stats.stall_occ_avglen);
    std::sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) { return w[a] > w[b]; });
    const auto cvg = cumulative(idx, stats.stall_cov_avgocc, stats.stall_cov_avglen);

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
      ++traces[it->second].occurrences; // re-capture of an existing stretch
      ++stats.stall_dedup;
      return;
    }
    trace tr;
    tr.entry = start_ip;
    tr.ips = buf;
    tr.occurrences = 1;
    for (uint64_t v : tr.ips) {
      cov.cover(v, stats.stall_unique_ips_covered);
    }
    stats.stall_stored_uops += tr.ips.size();
    ++stats.stall_traces;
    start_index.emplace(start_ip, traces.size());
    traces.push_back(std::move(tr));
  }

  void reset()
  {
    recording = false;
    costly = false;
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

  std::size_t max_uops;
  bool recording = false;
  bool costly = false;
  uint64_t start_ip = 0;
  std::vector<uint64_t> buf;
  std::vector<trace> traces;
  std::unordered_map<uint64_t, std::size_t> start_index; // start IP -> index in traces
  coverage_meter cov;
};

#endif

#ifndef TRACE_COVERAGE_H
#define TRACE_COVERAGE_H

#include <cstdint>
#include <unordered_map>
#include <unordered_set>

// Shared coverage accounting for the trace builders (segmenter / recorder /
// stager).  It tracks, per instruction IP: the dynamic execution count, whether
// it was ever covered by a stored trace, and -- if not -- the best-known reason
// it was lost.  From these it produces, at end of run:
//
//   * dyn_covered_final  -- dynamic u-ops whose IP is in the FINAL covered set
//        (time-AGNOSTIC).  Subtracting the time-aware covered count gives the
//        BUILD-LATENCY loss; the rest is never-covered.
//   * the never-covered loss partitioned by reason:
//        no_trigger  -- IP never appeared in any trace window (straight-line /
//                       one-shot code; the algorithm's ceiling)
//        overflow    -- IP only in windows whose target/call fell out (window
//                       too small)
//        bad_layout / short -- dropped by those rules
//
// By construction: dyn_covered_final + no_trigger + overflow + bad_layout +
// short == total dynamic u-ops (a useful self-check).
struct coverage_meter {
  enum reason : uint8_t { NO_TRIGGER = 0, OVERFLOW = 1, SHORT = 2, BAD_LAYOUT = 3 };

  std::unordered_map<uint64_t, uint64_t> dyn_count; // ip -> dynamic executions
  std::unordered_map<uint64_t, uint8_t> lost_reason; // ip -> best-known loss reason
  std::unordered_set<uint64_t> seen;
  std::unordered_set<uint64_t> covered;

  // per u-op: bump the dynamic count and the three running coverage counters
  void observe(uint64_t ip, uint64_t& dyn_total, uint64_t& dyn_covered, uint64_t& uniq_seen)
  {
    ++dyn_total;
    ++dyn_count[ip];
    if (covered.count(ip) > 0) {
      ++dyn_covered;
    }
    if (seen.insert(ip).second) {
      ++uniq_seen;
    }
  }

  // an IP entered a stored trace
  void cover(uint64_t ip, uint64_t& uniq_covered)
  {
    if (covered.insert(ip).second) {
      ++uniq_covered;
    }
  }

  // an IP was in a dropped window; keep the highest-priority reason
  void mark(uint64_t ip, reason r)
  {
    auto& s = lost_reason[ip];
    if (s < static_cast<uint8_t>(r)) {
      s = static_cast<uint8_t>(r);
    }
  }

  [[nodiscard]] bool covers(uint64_t ip) const { return covered.count(ip) > 0; }

  // end-of-run partition; the five outputs sum to the total dynamic u-ops
  void finalize(uint64_t& covered_final, uint64_t& no_trig, uint64_t& ov, uint64_t& bad_layout, uint64_t& short_drop) const
  {
    covered_final = no_trig = ov = bad_layout = short_drop = 0;
    for (const auto& [ip, cnt] : dyn_count) {
      if (covered.count(ip) > 0) {
        covered_final += cnt;
        continue;
      }
      const auto it = lost_reason.find(ip);
      const uint8_t r = (it == lost_reason.end()) ? NO_TRIGGER : it->second;
      switch (r) {
      case OVERFLOW:
        ov += cnt;
        break;
      case SHORT:
        short_drop += cnt;
        break;
      case BAD_LAYOUT:
        bad_layout += cnt;
        break;
      default:
        no_trig += cnt;
        break;
      }
    }
  }
};

#endif

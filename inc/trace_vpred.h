#ifndef TRACE_VPRED_H
#define TRACE_VPRED_H

// trace_vpred: a value predictor whose value is a trace (trace_vpred knob).
//
// Organized like TAGE (branch/tage_sc_l): a base table indexed by the predicting
// branch alone plus tagged tables with geometrically longer global histories.
// The longest matching table provides the prediction; a wrong prediction
// allocates in a longer table; usefulness counters decide replacement.
//
// Context: every branch prediction, in program order -- the branch pc, what was
// predicted for it (a direction, or a target for other branches) and the global
// history before it.  Value: the entry pc of the trace expected to start next
// with a costly u-op-cache miss (a pointer into the regular trace store; hardware
// would keep the store slot, about 13 bits).
//
// Training: every costly stretch (u-op-cache miss that starved dispatch with
// the ROB empty -- the stall segmenter's verdict) trains the record of the last
// prediction before its first instruction.  After a misprediction that is the
// mispredicted branch with its WRONG predicted direction, so the next time the
// predictor makes the same mistake the trace that follows it is named while the
// branch is still on its way to resolve.  In steady state it is the branch just
// before the costly block (one prediction early).  A prediction no costly
// stretch follows ages its provider's confidence.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "core_stats.h"

class trace_vpred
{
public:
  static constexpr int MAX_TABLES = 9;          // base + up to 8 tagged
  static constexpr std::size_t RING = 4096;     // predictions awaiting their outcome (power of two)
  static constexpr uint64_t AGE_PERIOD = 1 << 18; // allocations between usefulness halvings

  // tagged = tagged tables above the base table; entries per table (power of two)
  void configure(int tagged, int entries, int min_hist, int max_hist, int tag_bits, int conf_thr)
  {
    n_tables_ = std::clamp(tagged, 0, MAX_TABLES - 1) + 1;
    std::size_t e = 1;
    while (e < static_cast<std::size_t>(std::max(entries, 1))) {
      e <<= 1;
    }
    entries_ = e;
    tag_bits_ = std::clamp(tag_bits, 1, 16);
    conf_thr_ = std::clamp(conf_thr, 0, CONF_MAX);
    hist_[0] = 0;
    const int lo = std::clamp(min_hist, 1, 64);
    const int hi = std::clamp(max_hist, lo, 64);
    for (int t = 1; t < n_tables_; ++t) {
      // geometric series from lo to hi over the tagged tables
      const double f = (n_tables_ > 2) ? static_cast<double>(t - 1) / (n_tables_ - 2) : 1.0;
      hist_[t] = static_cast<int>(std::lround(lo * std::pow(static_cast<double>(hi) / lo, f)));
    }
    tables_.assign(static_cast<std::size_t>(n_tables_), std::vector<entry>(entries_));
  }

  [[nodiscard]] int tables() const { return n_tables_; }
  [[nodiscard]] int hist(int t) const { return hist_[static_cast<std::size_t>(t)]; }
  [[nodiscard]] std::size_t entries() const { return entries_; }
  [[nodiscard]] uint64_t last_seq() const { return seq_; }
  // tag + 13-bit store pointer + confidence + usefulness, per entry
  [[nodiscard]] uint64_t storage_bits() const { return static_cast<uint64_t>(n_tables_) * entries_ * (tag_bits_ + 13 + 2 + 2); }

  // one branch prediction (program order).  Returns the trace entry pc to walk,
  // 0 = none.  mispredicted is simulator knowledge, used for statistics only.
  uint64_t predict(uint64_t pc, uint64_t sig, uint64_t ghr, bool mispredicted, cpu_stats& stats)
  {
    ++seq_;
    record& r = ring_[seq_ & (RING - 1)];
    if (r.seq != 0 && !r.resolved) {
      resolve_none(r, stats); // no costly stretch followed that prediction
    }
    r = record{};
    r.seq = seq_;
    r.mispredicted = mispredicted;
    r.resolved = false;
    const uint64_t base = mix(pc ^ mix(sig + 0x9e3779b97f4a7c15ULL));
    for (int t = 0; t < n_tables_; ++t) {
      const int h = hist_[static_cast<std::size_t>(t)];
      const uint64_t hmask = (h >= 64) ? ~uint64_t{0} : ((uint64_t{1} << h) - 1);
      const uint64_t k = mix(base ^ mix((ghr & hmask) + (static_cast<uint64_t>(t) << 56) + static_cast<uint64_t>(h)));
      r.idx[static_cast<std::size_t>(t)] = static_cast<uint32_t>(k & (entries_ - 1));
      r.tag[static_cast<std::size_t>(t)] = static_cast<uint16_t>((k >> 40) & ((uint64_t{1} << tag_bits_) - 1));
    }
    ++stats.vp_lookups;
    for (int t = n_tables_ - 1; t >= 0; --t) {
      const entry& e = at(r, t);
      if (e.valid && e.tag == r.tag[static_cast<std::size_t>(t)]) {
        r.provider = t;
        r.value = e.value;
        break;
      }
    }
    if (r.provider < 0) {
      return 0;
    }
    ++stats.vp_matches;
    if (at(r, r.provider).conf < conf_thr_) {
      return 0;
    }
    ++stats.vp_fires;
    return r.value;
  }

  // a costly stretch began at `entry_pc`; `seq` is the last prediction before its
  // first instruction (0 = none yet).  Only the first costly stretch after a
  // prediction trains it.
  void train(uint64_t seq, uint64_t entry_pc, cpu_stats& stats)
  {
    if (seq == 0 || entry_pc == 0) {
      return;
    }
    record& r = ring_[seq & (RING - 1)];
    if (r.seq != seq || r.resolved) {
      return;
    }
    r.resolved = true;
    ++stats.vp_train;
    if (r.mispredicted) {
      ++stats.vp_train_mp;
    }
    if (r.provider >= 0 && provider_alive(r)) {
      entry& e = at(r, r.provider);
      if (e.value == entry_pc) {
        ++stats.vp_correct;
        if (r.mispredicted) {
          ++stats.vp_correct_mp;
        }
        e.conf = static_cast<uint8_t>(std::min<int>(e.conf + 1, CONF_MAX));
        e.u = static_cast<uint8_t>(std::min<int>(e.u + 1, U_MAX));
        return;
      }
      ++stats.vp_wrong;
      if (e.conf > 0) {
        --e.conf;
      } else {
        e.value = entry_pc; // low confidence: take the new value in place
      }
      allocate(r, r.provider + 1, entry_pc, stats);
      return;
    }
    ++stats.vp_noprov;
    allocate(r, 0, entry_pc, stats);
  }

private:
  static constexpr int CONF_MAX = 3;
  static constexpr int U_MAX = 3;

  struct entry {
    uint64_t value = 0; // trace entry pc
    uint16_t tag = 0;
    uint8_t conf = 0;
    uint8_t u = 0;
    bool valid = false;
  };
  struct record {
    uint64_t seq = 0;
    int provider = -1;
    std::array<uint32_t, MAX_TABLES> idx{};
    std::array<uint16_t, MAX_TABLES> tag{};
    uint64_t value = 0;
    bool mispredicted = false;
    bool resolved = true;
  };

  static uint64_t mix(uint64_t x)
  {
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33;
    return x;
  }

  entry& at(const record& r, int t) { return tables_[static_cast<std::size_t>(t)][r.idx[static_cast<std::size_t>(t)]]; }

  // the provider entry may have been replaced since the prediction
  bool provider_alive(const record& r)
  {
    const entry& e = at(r, r.provider);
    return e.valid && e.tag == r.tag[static_cast<std::size_t>(r.provider)];
  }

  // no costly stretch followed the prediction: the provider's confidence ages
  void resolve_none(record& r, cpu_stats& stats)
  {
    r.resolved = true;
    if (r.provider >= 0 && provider_alive(r)) {
      ++stats.vp_none;
      entry& e = at(r, r.provider);
      if (e.conf > 0) {
        --e.conf;
      }
    }
  }

  // TAGE-style: one new entry in the shortest table at or above `start` whose slot
  // is free or no longer useful; otherwise age those slots' usefulness
  void allocate(const record& r, int start, uint64_t value, cpu_stats& stats)
  {
    for (int t = start; t < n_tables_; ++t) {
      entry& e = at(r, t);
      if (!e.valid || e.u == 0) {
        e = entry{value, r.tag[static_cast<std::size_t>(t)], 0, 0, true};
        ++stats.vp_allocs;
        if (++n_alloc_ % AGE_PERIOD == 0) {
          for (auto& tab : tables_) {
            for (auto& x : tab) {
              x.u = static_cast<uint8_t>(x.u >> 1);
            }
          }
        }
        return;
      }
    }
    ++stats.vp_alloc_fail;
    for (int t = start; t < n_tables_; ++t) {
      entry& e = at(r, t);
      if (e.u > 0) {
        --e.u;
      }
    }
  }

  int n_tables_ = 1;
  std::size_t entries_ = 512;
  int tag_bits_ = 12;
  int conf_thr_ = 0;
  std::array<int, MAX_TABLES> hist_{};
  std::vector<std::vector<entry>> tables_;
  std::array<record, RING> ring_{};
  uint64_t seq_ = 0;
  uint64_t n_alloc_ = 0;
};

#endif

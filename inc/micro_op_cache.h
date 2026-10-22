#ifndef MICRO_OP_CACHE_H
#define MICRO_OP_CACHE_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <unordered_set>
#include <vector>

#include "address.h"

// Micro-op cache (decoded-instruction buffer) modeled after the UCP_ISCA24
// artifact (its inc/micro_op_cache.h). It is set-associative and operates at
// "window" granularity: an entry covers one aligned window (window_bits wide,
// e.g. 32B). The window size implicitly enforces two of the paper's three
// entry-termination conditions:
//   * the u-op-count limit  (one window holds window/4 u-ops), and
//   * L1I-line crossing      (a new window is always a new entry).
// The third condition -- termination on a taken branch, plus a 2-branch-per-
// entry cap -- is modeled explicitly here, exactly as in UCP's Insert(): a
// terminated entry is *deactivated* so the next access to that window must
// re-allocate a (fragmenting) way, which reproduces the capacity pressure of a
// real u-op-cache build rule. Lookup is tag-only (matches UCP's Lookup, which
// ignores the active flag), so termination manifests as fragmentation/eviction
// pressure rather than as an immediate miss for the same window.
class micro_op_cache
{
public:
  static constexpr int MAX_BRANCHES_PER_ENTRY = 2;

  // "ideal" DIB knob modes:
  static constexpr int IDEAL_OFF = 0;       // normal finite set-associative cache
  static constexpr int IDEAL_ORACLE = 1;    // every lookup hits (paper's IDEAL_UOP_CACHE upper bound)
  static constexpr int IDEAL_COLD_MISS = 2; // infinite capacity: first access to a window misses, then always hits

  struct entry {
    uint64_t tag = 0;
    bool valid = false;
    bool active = false;             // an entry deactivated by a termination condition
    bool terminated_by_taken = false;
    bool prefetched = false;         // installed by a trace-fill walk, not yet demand-hit
    bool costly = false;             // uop_retain: a miss on this window starved the backend
    int num_br = 0;
    uint64_t last_used = 0;          // LRU timestamp
  };

  micro_op_cache(std::size_t sets, std::size_t ways, champsim::data::bits window_bits, int ideal = IDEAL_OFF)
      : NUM_SET(sets), NUM_WAY(ways), window_shift(window_bits), ideal_(ideal), block(sets * ways)
  {
  }

  // stream-mode lookup: hit on any matching window tag (tag-only, as in UCP).
  // was_prefetched (optional out) reports a first demand hit on a walk-installed
  // window (the walk-accuracy signal); the hit also promotes it to MRU as usual.
  // diagnostic oracle (PROMETHEUS_UOP_ORACLE): these windows always hit -- prices
  // the u-op-cache misses of a chosen set of windows (e.g. the costliest N)
  // install = true (PROMETHEUS_ORACLE_INSTALL): an oracle window missing from the
  // cache is filled the moment demand needs it -- a perfect, on-time prefetch that
  // still takes capacity -- instead of hitting for free outside the cache
  void set_oracle_windows(std::unordered_set<uint64_t> windows, bool install = false)
  {
    oracle_ = std::move(windows);
    oracle_install_ = install;
  }
  [[nodiscard]] std::size_t oracle_windows() const { return oracle_.size(); }
  [[nodiscard]] bool oracle_install() const { return oracle_install_; }

  // uop_retain: cost-aware replacement.  Lines whose miss starved the backend are
  // marked costly; replacement evicts the LRU non-costly line while the set holds
  // at most `ways` costly lines, else the LRU costly line.  0 = plain LRU.
  void set_retain(std::size_t ways) { retain_cap_ = ways; }
  void set_ideal(int ideal) { ideal_ = ideal; } // PROMETHEUS_DIB_IDEAL: runtime override of the "ideal" knob
  // PROMETHEUS_UOP_INCLUSIVE: the u-op cache is inclusive of the L1I (as measured on Intel/AMD parts) --
  // a window whose L1I line is gone is invalid.  Checked lazily at lookup/probe.
  void set_line_present(std::function<bool(champsim::address)> f) { line_present_ = std::move(f); }
  [[nodiscard]] uint64_t inclusive_invalidations() const { return inclusive_invalidations_; }
  // PROMETHEUS_UOP_INSERT_FILTER: only these windows allocate normally; others bypass (mode 0) or insert at LRU (mode 1)
  void set_insert_filter(std::unordered_set<uint64_t> windows, int mode)
  {
    insert_filter_ = std::move(windows);
    insert_mode_ = mode;
  }
  [[nodiscard]] std::size_t retain_cap() const { return retain_cap_; }
  // mark a resident window costly; false if it is not resident (the caller marks it at its fill)
  bool mark_costly(champsim::address ip)
  {
    if (block.empty() || ideal_ != IDEAL_OFF) {
      return true;
    }
    auto t = tag_of(ip);
    auto [set_begin, set_end] = get_set(ip);
    bool found = false;
    for (auto it = set_begin; it != set_end; ++it) {
      if (it->valid && it->tag == t) {
        if (!it->costly) {
          it->costly = true;
          ++retain_marks_;
        }
        found = true;
      }
    }
    return found;
  }
  [[nodiscard]] uint64_t retain_marks() const { return retain_marks_; }
  [[nodiscard]] uint64_t retain_diverted() const { return retain_diverted_; }
  [[nodiscard]] uint64_t retain_costly_evicted() const { return retain_costly_evicted_; }

  bool check_hit(champsim::address ip, bool* was_prefetched = nullptr)
  {
    if (ideal_ == IDEAL_ORACLE) // every lookup hits (perfect-DIB upper bound)
      return true;
    const bool oracle_hit = !oracle_.empty() && oracle_.count(tag_of(ip)) != 0;
    if (oracle_hit && !oracle_install_)
      return true;
    if (ideal_ == IDEAL_COLD_MISS) // infinite capacity: hit iff this window was decoded before
      return seen_.count(tag_of(ip)) != 0;
    if (block.empty()) // disabled (no u-op cache: sets==0 or ways==0)
      return oracle_hit;
    auto t = tag_of(ip);
    auto [set_begin, set_end] = get_set(ip);
    for (auto it = set_begin; it != set_end; ++it) {
      if (it->valid && it->tag == t && line_present_ && !line_present_(ip)) {
        it->valid = false; // inclusion: the L1I evicted this window's line
        ++inclusive_invalidations_;
        continue;
      }
      if (it->valid && it->tag == t) {
        it->last_used = ++lru_clock;
        if (it->prefetched) {
          it->prefetched = false;
          if (was_prefetched != nullptr) {
            *was_prefetched = true;
          }
        }
        return true;
      }
    }
    if (oracle_hit) {
      fill(ip, false, false); // install oracle: on-time fill at MRU, then hit
      return true;
    }
    return false;
  }

  // build-mode fill: replicates UCP Insert() entry-termination semantics.
  // prefetch=true (trace-fill walks): a NEW allocation is inserted at LRU (most
  // evictable) and flagged, so unused prefetches die first and a demand hit
  // promotes them (check_hit) -- models a prefetch/staging insertion policy
  // instead of letting speculative installs displace hot entries at MRU.
  void fill(champsim::address ip, bool taken_end, bool is_branch, bool prefetch = false, bool costly = false)
  {
    if (ideal_ == IDEAL_ORACLE) // nothing to store: every lookup already hits
      return;
    if (ideal_ == IDEAL_COLD_MISS) { // infinite capacity, never evict: mark this window decoded
      seen_.insert(tag_of(ip));
      return;
    }
    if (block.empty()) // disabled (no u-op cache)
      return;
    auto t = tag_of(ip);
    auto [set_begin, set_end] = get_set(ip);

    // Pass 1: deactivate matching-tag ways that just hit a termination
    // condition, or reactivate ones re-seen as a non-terminating continuation.
    for (auto it = set_begin; it != set_end; ++it) {
      if (it->valid && it->tag == t && it->active && (it->terminated_by_taken || it->num_br == MAX_BRANCHES_PER_ENTRY)) {
        it->active = false;
      }
      if (it->valid && it->tag == t && !it->active && it->num_br < MAX_BRANCHES_PER_ENTRY && !taken_end) {
        it->active = true;
      }
    }

    // Pass 2: accumulate into the first active matching-tag way.
    bool tag_found = false;
    for (auto it = set_begin; it != set_end; ++it) {
      if (it->valid && it->tag == t && it->active) {
        it->terminated_by_taken = taken_end;
        if (is_branch && it->num_br < MAX_BRANCHES_PER_ENTRY) {
          ++it->num_br;
        }
        if (!prefetch) { // a walk refresh must not promote (nor demote) a resident entry
          it->last_used = ++lru_clock;
          it->prefetched = false;
        }
        if (costly && !it->costly) {
          it->costly = true;
          ++retain_marks_;
        }
        tag_found = true;
        break;
      }
    }

    // Miss: allocate the LRU way for a fresh entry. When a terminated entry's
    // successor lands here, this is the window fragmentation the rule models.
    const bool low_priority = !insert_filter_.empty() && !prefetch && insert_filter_.count(t) == 0;
    if (!tag_found && low_priority && insert_mode_ == 0) {
      return; // profile-guided insertion: not a costly-trace window, do not allocate
    }
    if (!tag_found) {
      auto lru_less = [](const entry& a, const entry& b) { return a.last_used < b.last_used; };
      auto victim = std::min_element(set_begin, set_end, lru_less);
      if (retain_cap_ > 0) {
        const auto n_costly = static_cast<std::size_t>(std::count_if(set_begin, set_end, [](const entry& e) { return e.valid && e.costly; }));
        auto pick = set_end;
        for (auto it = set_begin; it != set_end; ++it) {
          // over quota: the costly lines compete among themselves; else spare them
          const bool eligible = (n_costly > retain_cap_) ? (it->valid && it->costly) : !(it->valid && it->costly);
          if (eligible && (pick == set_end || it->last_used < pick->last_used)) {
            pick = it;
          }
        }
        if (pick != set_end) {
          if (pick != victim) {
            ++retain_diverted_; // plain LRU would have evicted a costly line here
          }
          victim = pick;
        }
      }
      if (victim->valid && victim->prefetched) {
        ++pf_evicted_unused_; // walk-installed window evicted without ever being demand-hit
      }
      if (victim->valid && victim->costly) {
        ++retain_costly_evicted_;
      }
      victim->tag = t;
      victim->valid = true;
      victim->active = true;
      victim->terminated_by_taken = false;
      victim->num_br = 0;
      victim->prefetched = prefetch;
      victim->costly = costly;
      if (costly) {
        ++retain_marks_;
      }
      if (prefetch || low_priority) { // insert at LRU: oldest surviving timestamp in the set, minus one
        uint64_t oldest = std::numeric_limits<uint64_t>::max();
        for (auto it = set_begin; it != set_end; ++it) {
          if (it->valid && it != victim && it->last_used < oldest) {
            oldest = it->last_used;
          }
        }
        victim->last_used = (oldest == std::numeric_limits<uint64_t>::max() || oldest == 0) ? 0 : oldest - 1;
      } else {
        victim->last_used = ++lru_clock;
      }
    }
  }

  // cumulative count of walk-installed windows evicted with the prefetched bit
  // still set (never demand-hit); mirrored into stats each cycle so the phase
  // subtraction yields per-phase deltas.  With alt_useful_hits this gives walk
  // accuracy over completed window lifetimes: useful / (useful + evicted-unused).
  [[nodiscard]] uint64_t pf_evicted_unused() const { return pf_evicted_unused_; }

  // presence check without touching LRU state (used by the alt-trigger walk to
  // skip walks whose target windows are already resident).
  [[nodiscard]] bool probe(champsim::address ip) const
  {
    if (ideal_ == IDEAL_ORACLE)
      return true;
    if (!oracle_install_ && !oracle_.empty() && oracle_.count(tag_of(ip)) != 0)
      return true;
    if (ideal_ == IDEAL_COLD_MISS)
      return seen_.count(tag_of(ip)) != 0;
    if (block.empty())
      return false;
    const uint64_t t = tag_of(ip);
    const auto set_idx = static_cast<std::size_t>(t % NUM_SET);
    for (std::size_t w = 0; w < NUM_WAY; ++w) {
      const auto& e = block[set_idx * NUM_WAY + w];
      if (e.valid && e.tag == t) {
        return !line_present_ || line_present_(ip);
      }
    }
    return false;
  }

  // trace-fill (option a): pre-install a trace's windows so later fetches hit.
  // Marks each distinct aligned window present (a build with no termination).
  // prefetch=true uses the LRU-insertion policy (see fill).
  // Returns the number of distinct windows touched.
  std::size_t install(const std::vector<uint64_t>& ips, bool prefetch = false, bool costly = false)
  {
    // oracle: nothing to install (all hits); off with no cache: nothing to do.
    // cold-miss falls through and marks windows decoded via fill() -> seen_.
    if (ideal_ == IDEAL_ORACLE || (ideal_ == IDEAL_OFF && block.empty())) {
      return 0;
    }
    std::size_t windows = 0;
    uint64_t last = 0;
    bool have_last = false;
    for (uint64_t raw : ips) {
      const champsim::address ip{raw};
      const uint64_t t = tag_of(ip);
      if (have_last && t == last) {
        continue; // same window as the previous u-op
      }
      have_last = true;
      last = t;
      fill(ip, false, false, prefetch, costly); // insert/refresh -> window present
      ++windows;
    }
    return windows;
  }

private:
  std::size_t NUM_SET;
  std::size_t NUM_WAY;
  champsim::data::bits window_shift;
  int ideal_;                          // IDEAL_OFF / IDEAL_ORACLE / IDEAL_COLD_MISS
  std::unordered_set<uint64_t> seen_;  // windows ever decoded (IDEAL_COLD_MISS only)
  std::unordered_set<uint64_t> oracle_; // windows that always hit (diagnostic)
  bool oracle_install_ = false;         // ...by being filled on demand (take capacity) instead of hitting outside the cache
  std::vector<entry> block;
  uint64_t lru_clock = 0;
  uint64_t pf_evicted_unused_ = 0; // see pf_evicted_unused()
  std::size_t retain_cap_ = 0;         // uop_retain: costly lines a set may protect (0 = off)
  std::unordered_set<uint64_t> insert_filter_; // PROMETHEUS_UOP_INSERT_FILTER windows (empty = off)
  int insert_mode_ = 0;                 // 0 = bypass others, 1 = insert others at LRU
  std::function<bool(champsim::address)> line_present_; // PROMETHEUS_UOP_INCLUSIVE (empty = non-inclusive)
  uint64_t inclusive_invalidations_ = 0;
  uint64_t retain_marks_ = 0;          // lines marked costly
  uint64_t retain_diverted_ = 0;       // evictions moved off a costly LRU line
  uint64_t retain_costly_evicted_ = 0; // costly lines evicted anyway

  uint64_t tag_of(champsim::address ip) const { return ip.slice_upper(window_shift).to<uint64_t>(); }

  std::pair<std::vector<entry>::iterator, std::vector<entry>::iterator> get_set(champsim::address ip)
  {
    auto set_idx = static_cast<std::size_t>(tag_of(ip) % NUM_SET);
    auto set_begin = std::next(std::begin(block), static_cast<std::ptrdiff_t>(set_idx * NUM_WAY));
    return {set_begin, std::next(set_begin, static_cast<std::ptrdiff_t>(NUM_WAY))};
  }
};

#endif

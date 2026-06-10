#ifndef MICRO_OP_CACHE_H
#define MICRO_OP_CACHE_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
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

  struct entry {
    uint64_t tag = 0;
    bool valid = false;
    bool active = false;             // an entry deactivated by a termination condition
    bool terminated_by_taken = false;
    int num_br = 0;
    uint64_t last_used = 0;          // LRU timestamp
  };

  micro_op_cache(std::size_t sets, std::size_t ways, champsim::data::bits window_bits)
      : NUM_SET(sets), NUM_WAY(ways), window_shift(window_bits), block(sets * ways)
  {
  }

  // stream-mode lookup: hit on any matching window tag (tag-only, as in UCP).
  bool check_hit(champsim::address ip)
  {
    if (block.empty()) // disabled (no u-op cache: sets==0 or ways==0)
      return false;
    auto t = tag_of(ip);
    auto [set_begin, set_end] = get_set(ip);
    for (auto it = set_begin; it != set_end; ++it) {
      if (it->valid && it->tag == t) {
        it->last_used = ++lru_clock;
        return true;
      }
    }
    return false;
  }

  // build-mode fill: replicates UCP Insert() entry-termination semantics.
  void fill(champsim::address ip, bool taken_end, bool is_branch)
  {
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
        it->last_used = ++lru_clock;
        tag_found = true;
        break;
      }
    }

    // Miss: allocate the LRU way for a fresh entry. When a terminated entry's
    // successor lands here, this is the window fragmentation the rule models.
    if (!tag_found) {
      auto victim = std::min_element(set_begin, set_end, [](const entry& a, const entry& b) { return a.last_used < b.last_used; });
      victim->tag = t;
      victim->valid = true;
      victim->active = true;
      victim->terminated_by_taken = false;
      victim->num_br = 0;
      victim->last_used = ++lru_clock;
    }
  }

private:
  std::size_t NUM_SET;
  std::size_t NUM_WAY;
  champsim::data::bits window_shift;
  std::vector<entry> block;
  uint64_t lru_clock = 0;

  uint64_t tag_of(champsim::address ip) const { return ip.slice_upper(window_shift).to<uint64_t>(); }

  std::pair<std::vector<entry>::iterator, std::vector<entry>::iterator> get_set(champsim::address ip)
  {
    auto set_idx = static_cast<std::size_t>(tag_of(ip) % NUM_SET);
    auto set_begin = std::next(std::begin(block), static_cast<std::ptrdiff_t>(set_idx * NUM_WAY));
    return {set_begin, std::next(set_begin, static_cast<std::ptrdiff_t>(NUM_WAY))};
  }
};

#endif

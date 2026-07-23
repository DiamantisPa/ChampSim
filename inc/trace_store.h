#ifndef TRACE_STORE_H
#define TRACE_STORE_H

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <unordered_map>
#include <vector>

// Bounded trace cache for the trace-fill experiments.  Finalized traces (from
// the stager) are inserted here keyed by entry PC; it also (optionally) builds a
// secondary index over EVERY 32B window a trace covers, so a miss anywhere in a
// trace's footprint can install it ("window" fill mode).  Capacity (number of
// traces, LRU) is the PROMETHEUS_TRACE_STORE knob.
class trace_store
{
public:
  static constexpr std::size_t DEFAULT_CAPACITY = 256;

  trace_store() : trace_store(env_capacity()) {}
  explicit trace_store(std::size_t cap) : capacity(cap < 1 ? 1 : cap) {}

  // window_bits = lg2(u-op-cache window size); window_indexed enables the
  // per-window secondary index used by the "window" fill mode.
  void configure(unsigned window_bits_, bool window_indexed_)
  {
    window_bits = window_bits_;
    window_indexed = window_indexed_;
  }

  // capacity in traces; must be called before any insert (env still overrides
  // the JSON knob: precedence env > JSON > default, like the other knobs).
  void set_capacity(std::size_t cap) { capacity = (cap < 1) ? 1 : cap; }

  // organization: ways == 0 -> fully-associative pool (capacity = sets if
  // sets > 0, else whatever set_capacity gave); ways > 0 -> sets x ways
  // set-associative, indexed by entry-PC word address, victim chosen within
  // the entry's set only.  Must be called before any insert.
  void set_geometry(std::size_t sets_, unsigned ways_)
  {
    num_ways = ways_;
    if (num_ways == 0) {
      if (sets_ > 0) {
        capacity = sets_;
      }
      return;
    }
    num_sets = (sets_ < 1) ? 1 : sets_;
    capacity = num_sets * num_ways;
    entries.assign(capacity, {});
    entry_index.clear();
    window_index.clear();
    live = 0;
  }

  // replacement policy: false = LRU (default), true = evict the minimum
  // stall-cost trace (LRU tie-break) -- keep what hurts most, not what's recent.
  void set_cost_policy(bool cost_evict_) { cost_evict = cost_evict_; }

  [[nodiscard]] std::size_t cap() const { return capacity; }
  [[nodiscard]] std::size_t size() const { return (num_ways > 0) ? live : entries.size(); }
  [[nodiscard]] std::size_t sets() const { return (num_ways > 0) ? num_sets : 0; }
  [[nodiscard]] unsigned ways() const { return num_ways; }
  [[nodiscard]] bool cost_policy() const { return cost_evict; }
  [[nodiscard]] uint64_t evictions() const { return n_evict; }
  [[nodiscard]] uint64_t conflict_evictions() const { return n_conflict_evict; }
  [[nodiscard]] bool hash_mode() const { return xor_hash; }

  // set-index hash: false = plain modulo (legacy first-cut), true = xor-fold
  void set_hash(bool xor_hash_) { xor_hash = xor_hash_; }

  // cost aging for cost_evict: halve every resident cost each `period`
  // insertions (0 = off).  Without aging, accumulated costs fossilize stale
  // traces (measured -3.5pp); periodic halving keeps the ranking recent.
  void set_cost_decay(uint64_t period) { decay_period = period; }
  [[nodiscard]] uint64_t cost_decay() const { return decay_period; }

  // model a finite saturating cost counter: N bits -> values clamp at 2^N-1
  // (0 = unbounded, the idealized default).  Eviction picks the minimum cost,
  // so saturation at the top should be harmless -- this knob proves it.
  void set_cost_bits(unsigned bits) { cost_cap = (bits == 0 || bits >= 64) ? 0 : ((uint64_t{1} << bits) - 1); }
  [[nodiscard]] unsigned cost_bits() const
  {
    if (cost_cap == 0) {
      return 0;
    }
    unsigned b = 0;
    for (uint64_t v = cost_cap; v != 0; v >>= 1) {
      ++b;
    }
    return b;
  }

  void insert(uint64_t entry, const std::vector<uint64_t>& ips, uint64_t cost = 0)
  {
    cost = clamp_cost(cost);
    if (decay_period > 0 && ++n_insert % decay_period == 0) {
      for (auto& e : entries) {
        e.cost >>= 1;
      }
    }
    if (auto it = entry_index.find(entry); it != entry_index.end()) { // refresh
      const std::size_t slot = it->second;
      if (window_indexed) {
        remove_windows(slot);
      }
      entries[slot].ips = ips;
      entries[slot].lru = ++tick;
      entries[slot].cost = cost;
      if (window_indexed) {
        add_windows(slot);
      }
      return;
    }

    std::size_t slot = 0;
    if (num_ways > 0) { // set-associative: free way in the entry's set, else per-set victim
      const std::size_t base = set_of(entry) * num_ways;
      slot = base;
      bool found_free = false;
      for (std::size_t i = base; i < base + num_ways; ++i) {
        if (!entries[i].valid) {
          slot = i;
          found_free = true;
          break;
        }
      }
      if (!found_free) {
        slot = victim(base, base + num_ways);
        ++n_evict;
        if (live < capacity) {
          ++n_conflict_evict; // evicted from a full set while the store had free slots elsewhere = set skew
        }
        entry_index.erase(entries[slot].entry);
        if (window_indexed) {
          remove_windows(slot);
        }
      } else {
        ++live;
      }
      entries[slot] = {entry, ips, ++tick, cost, {}, true};
    } else if (entries.size() < capacity) {
      slot = entries.size();
      entries.push_back({entry, ips, ++tick, cost, {}, true});
    } else { // evict per policy (LRU or min-cost)
      slot = victim(0, entries.size());
      ++n_evict;
      entry_index.erase(entries[slot].entry);
      if (window_indexed) {
        remove_windows(slot);
      }
      entries[slot] = {entry, ips, ++tick, cost, {}, true};
    }
    entry_index.emplace(entry, slot);
    if (window_indexed) {
      add_windows(slot);
    }
  }

  // refresh a resident trace's accumulated stall cost (on re-capture)
  void update_cost(uint64_t entry, uint64_t cost)
  {
    if (auto it = entry_index.find(entry); it != entry_index.end()) {
      entries[it->second].cost = clamp_cost(cost);
    }
  }

  // lookup by trace entry PC (used by "miss" and "every" fill modes)
  const std::vector<uint64_t>* lookup_entry(uint64_t ip)
  {
    auto it = entry_index.find(ip);
    if (it == entry_index.end()) {
      return nullptr;
    }
    entries[it->second].lru = ++tick;
    return &entries[it->second].ips;
  }

  // lookup by the window containing ip (used by "window" fill mode)
  const std::vector<uint64_t>* lookup_window(uint64_t ip)
  {
    if (!window_indexed) {
      return nullptr;
    }
    auto it = window_index.find(ip >> window_bits);
    if (it == window_index.end()) {
      return nullptr;
    }
    entries[it->second].lru = ++tick;
    return &entries[it->second].ips;
  }

private:
  struct ent {
    uint64_t entry = 0;
    std::vector<uint64_t> ips;
    uint64_t lru = 0;
    uint64_t cost = 0;             // accumulated ROB-stall cycles of the trace (cost-aware eviction)
    std::vector<uint64_t> windows; // distinct window keys (only if window_indexed)
    bool valid = false;            // slot occupancy (pre-sized set-associative array)
  };

  static std::size_t env_capacity()
  {
    if (const char* e = std::getenv("PROMETHEUS_TRACE_STORE"); e != nullptr) {
      if (const long v = std::atol(e); v > 0) {
        return static_cast<std::size_t>(v);
      }
    }
    return DEFAULT_CAPACITY;
  }

  std::size_t victim(std::size_t begin, std::size_t end) const
  {
    std::size_t v = begin;
    for (std::size_t i = begin + 1; i < end; ++i) {
      if (cost_evict) {
        if (entries[i].cost < entries[v].cost || (entries[i].cost == entries[v].cost && entries[i].lru < entries[v].lru)) {
          v = i;
        }
      } else if (entries[i].lru < entries[v].lru) {
        v = i;
      }
    }
    return v;
  }

  // set index: entry PCs are word-aligned, so index on the word address.
  // xor_hash folds higher PC bits into the index (hot code clusters entry PCs
  // so low bits alone skew badly: measured 100% of set-assoc evictions occur
  // while the store is globally underfull under the plain modulo index).
  [[nodiscard]] std::size_t set_of(uint64_t entry) const
  {
    const uint64_t h = entry >> 2;
    if (xor_hash) {
      return static_cast<std::size_t>((h ^ (h >> 10) ^ (h >> 20)) % num_sets);
    }
    return static_cast<std::size_t>(h % num_sets);
  }

  void add_windows(std::size_t slot)
  {
    auto& e = entries[slot];
    e.windows.clear();
    uint64_t last = 0;
    bool have = false;
    for (uint64_t ip : e.ips) {
      const uint64_t w = ip >> window_bits;
      if (have && w == last) {
        continue;
      }
      have = true;
      last = w;
      e.windows.push_back(w);
      window_index[w] = slot; // last trace covering this window wins
    }
  }

  void remove_windows(std::size_t slot)
  {
    for (uint64_t w : entries[slot].windows) {
      if (auto it = window_index.find(w); it != window_index.end() && it->second == slot) {
        window_index.erase(it);
      }
    }
    entries[slot].windows.clear();
  }

  std::size_t capacity;
  std::size_t num_sets = 1;  // only meaningful when num_ways > 0
  unsigned num_ways = 0;     // 0 = fully associative (global victim scan)
  std::size_t live = 0;      // valid-entry count in set-associative mode
  unsigned window_bits = 0;
  bool window_indexed = false;
  [[nodiscard]] uint64_t clamp_cost(uint64_t c) const { return (cost_cap > 0 && c > cost_cap) ? cost_cap : c; }

  bool cost_evict = false;
  bool xor_hash = false;
  uint64_t cost_cap = 0;     // saturating cost ceiling (0 = unbounded idealized counter)
  uint64_t decay_period = 0; // cost halving interval in insertions (0 = off)
  uint64_t n_insert = 0;
  uint64_t tick = 0;
  uint64_t n_evict = 0;          // total evictions (any mode)
  uint64_t n_conflict_evict = 0; // set-assoc evictions while the store was globally underfull (set skew)
  std::vector<ent> entries;
  std::unordered_map<uint64_t, std::size_t> entry_index;
  std::unordered_map<uint64_t, std::size_t> window_index;
};

#endif

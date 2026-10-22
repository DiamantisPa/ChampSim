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

  // path-fragmentation diagnostic: of the distinct windows inserted with NEW
  // traces, how many were already covered by a DIFFERENT resident trace?  High
  // sharing = the same code regions captured repeatedly under different paths.
  [[nodiscard]] uint64_t windows_added() const { return n_win_added; }
  [[nodiscard]] uint64_t windows_shared() const { return n_win_shared; }
  [[nodiscard]] bool hash_mode() const { return xor_hash; }

  // set-index hash: false = plain modulo (legacy first-cut), true = xor-fold
  void set_hash(bool xor_hash_) { xor_hash = xor_hash_; }

  // partial tags (set-associative mode only): 0 = full entry-PC match (default);
  // N > 0 = ways compare an N-bit hashed tag of the entry PC.  A probe whose
  // partial tag matches a different trace launches that trace's window deltas
  // from the PROBING PC's window (what hardware storing only deltas does), and
  // an insert whose partial tag matches a different trace overwrites it.
  void set_tag_bits(unsigned bits) { tag_bits_ = (bits >= 64) ? 0 : bits; }
  [[nodiscard]] unsigned tag_bits() const { return tag_bits_; }
  [[nodiscard]] uint64_t lookups() const { return n_lookups; }
  [[nodiscard]] uint64_t alias_hits() const { return n_alias_hits; }
  [[nodiscard]] uint64_t alias_overwrites() const { return n_alias_overwrites; }
  // side-effect-free residency check by entry PC (partial-tag aware)
  [[nodiscard]] bool contains(uint64_t pc) const { return find_slot(pc) != NPOS; }

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
    if (const std::size_t found = find_slot(entry); found != NPOS) { // refresh (or partial-tag alias)
      const std::size_t slot = found;
      if (window_indexed) {
        remove_windows(slot);
      }
      if (entries[slot].entry != entry) {
        ++n_alias_overwrites; // a different trace with the same partial tag is replaced
        entry_index.erase(entries[slot].entry);
        entries[slot].entry = entry;
        entry_index[entry] = slot;
        entries[slot].conf = CONF_INIT;
      }
      entries[slot].ips = ips;
      entries[slot].lru = ++tick;
      entries[slot].cost = cost;
      if (entries[slot].conf == 0) {
        entries[slot].conf = 1; // re-captured: retry a suppressed launch
      }
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
      count_overlap(slot); // BEFORE add_windows claims these windows for this trace
      add_windows(slot);
    }
  }

  // refresh a resident trace's accumulated stall cost (on re-capture).  A
  // re-capture is fresh evidence the stretch still stalls the ROB, so a
  // launch-suppressed trace (conf 0) gets one retry (conf 1).
  void update_cost(uint64_t entry, uint64_t cost)
  {
    if (const std::size_t slot = find_slot(entry); slot != NPOS) {
      entries[slot].cost = clamp_cost(cost);
      if (entries[slot].conf == 0) {
        entries[slot].conf = 1;
      }
    }
  }

  // launch confidence (trace_launch_conf): 2-bit saturating counter per trace,
  // fed back from the staging buffer -- +1 when a staged window is demanded,
  // -1 when it is evicted unused.  conf 0 = do not launch this trace's walk.
  static constexpr uint8_t CONF_INIT = 2;
  static constexpr uint8_t CONF_MAX = 3;
  void conf_hit(uint64_t entry)
  {
    if (const std::size_t slot = find_slot(entry); slot != NPOS && entries[slot].conf < CONF_MAX) {
      ++entries[slot].conf;
    }
  }
  void conf_miss(uint64_t entry)
  {
    if (const std::size_t slot = find_slot(entry); slot != NPOS && entries[slot].conf > 0) {
      --entries[slot].conf;
    }
  }
  [[nodiscard]] int conf_of(uint64_t entry) const
  {
    const std::size_t slot = find_slot(entry);
    return (slot == NPOS) ? -1 : static_cast<int>(entries[slot].conf);
  }
  // census of the stored windows: total (per trace, deduplicated within a trace), distinct across traces, and those
  // held by two or more traces
  void window_census(unsigned bits, uint64_t& total, uint64_t& distinct, uint64_t& shared) const
  {
    std::unordered_map<uint64_t, uint32_t> count;
    total = 0;
    for (const auto& e : entries) {
      if (!e.valid) {
        continue;
      }
      std::unordered_set<uint64_t> mine;
      for (uint64_t ip : e.ips) {
        mine.insert(ip >> bits);
      }
      total += mine.size();
      for (uint64_t w : mine) {
        ++count[w];
      }
    }
    distinct = count.size();
    shared = 0;
    for (const auto& [w, n] : count) {
      shared += (n >= 2) ? 1 : 0;
    }
  }
  [[nodiscard]] int64_t cost_of(uint64_t entry) const
  {
    const std::size_t slot = find_slot(entry);
    return (slot == NPOS) ? -1 : static_cast<int64_t>(entries[slot].cost);
  }
  // conf-aware eviction: a conf-0 trace is the first-choice victim, ahead of
  // the LRU / min-cost compare
  void set_conf_evict(bool on) { conf_evict = on; }
  [[nodiscard]] bool conf_policy() const { return conf_evict; }

  // trace_chain_depth: the successor pointer of `entry`'s trace (side-effect free)
  [[nodiscard]] uint64_t next_of(uint64_t entry) const
  {
    const std::size_t slot = find_slot(entry);
    return (slot != NPOS) ? entries[slot].next : 0;
  }
  // record that `next` followed `entry`; false when `entry` is not stored
  bool set_next(uint64_t entry, uint64_t next)
  {
    const std::size_t slot = find_slot(entry);
    if (slot == NPOS) {
      return false;
    }
    entries[slot].next = next;
    return true;
  }

  // lookup by trace entry PC (used by "miss" and "every" fill modes)
  const std::vector<uint64_t>* lookup_entry(uint64_t ip)
  {
    ++n_lookups;
    const std::size_t slot = find_slot(ip);
    if (slot == NPOS) {
      return nullptr;
    }
    entries[slot].lru = ++tick;
    if (entries[slot].entry == ip) {
      return &entries[slot].ips;
    }
    // partial-tag alias: the stored deltas are applied from the probing PC's
    // window; the result is valid until the next lookup_entry call
    ++n_alias_hits;
    if (((ip >> 63) != 0) || ((entries[slot].entry >> 63) != 0)) {
      // a hashed key (mispredict event / recovery key) is not an address: its entry
      // carries its own start, so a false match walks the stored path as recorded
      return &entries[slot].ips;
    }
    const uint64_t shift = ((ip >> window_bits) - (entries[slot].entry >> window_bits)) << window_bits;
    alias_ips_.clear();
    for (uint64_t v : entries[slot].ips) {
      alias_ips_.push_back(v + shift);
    }
    return &alias_ips_;
  }

  // measurement-only peeks: no LRU update, no side effects.  Used by the chain
  // miss-classification shadow instrumentation (mid-trace trigger-gap sizing).
  [[nodiscard]] bool peek_entry(uint64_t ip) const { return entry_index.count(ip) != 0; }
  [[nodiscard]] bool peek_window(uint64_t ip) const { return window_indexed && window_index.count(ip >> window_bits) != 0; }
  // the stored uops of the trace whose entry is exactly ip (trace_trigger_diag)
  [[nodiscard]] const std::vector<uint64_t>* peek_ips(uint64_t ip) const
  {
    auto it = entry_index.find(ip);
    return (it == entry_index.end()) ? nullptr : &entries[it->second].ips;
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
    uint8_t conf = CONF_INIT;      // launch confidence (trace_launch_conf); fresh traces start weakly confident
    uint64_t next = 0;             // trace_chain_depth: entry pc of the costly trace that followed this one last time (0 = none)
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
      if (conf_evict) {
        const bool dead_i = entries[i].conf == 0;
        const bool dead_v = entries[v].conf == 0;
        if (dead_i != dead_v) {
          if (dead_i) {
            v = i;
          }
          continue; // a conf-0 trace always loses to a live one
        }
      }
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
  static constexpr std::size_t NPOS = static_cast<std::size_t>(-1);

  // N-bit hashed partial tag of an entry PC: xor-fold of the address bits the set
  // index does not consume -- the word address above the index bits plus the two
  // byte-offset bits (x86 traces start entries at any byte; the index drops them)
  [[nodiscard]] uint64_t ptag(uint64_t pc) const
  {
    unsigned idx_bits = 0;
    while ((std::size_t{1} << idx_bits) < num_sets) {
      ++idx_bits;
    }
    uint64_t src = (((pc >> 2) >> idx_bits) << 2) | (pc & 3);
    const uint64_t mask = (uint64_t{1} << tag_bits_) - 1;
    uint64_t t = 0;
    while (src != 0) {
      t ^= src & mask;
      src >>= tag_bits_;
    }
    return t;
  }

  // the slot holding `pc`'s trace: exact entry-PC match, or (partial tags) the way
  // in pc's set with an equal partial tag; NPOS if none.  Inserts replace partial-
  // tag matches, so a set never holds two traces with the same partial tag.
  [[nodiscard]] std::size_t find_slot(uint64_t pc) const
  {
    if (tag_bits_ == 0 || num_ways == 0) {
      auto it = entry_index.find(pc);
      return (it == entry_index.end()) ? NPOS : it->second;
    }
    const std::size_t base = set_of(pc) * num_ways;
    const uint64_t t = ptag(pc);
    for (std::size_t i = base; i < base + num_ways; ++i) {
      if (entries[i].valid && ptag(entries[i].entry) == t) {
        return i;
      }
    }
    return NPOS;
  }

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

  // fragmentation diagnostic (new inserts only): count this trace's distinct
  // windows and how many already belong to another resident trace
  void count_overlap(std::size_t slot)
  {
    uint64_t last = 0;
    bool have = false;
    for (uint64_t ip : entries[slot].ips) {
      const uint64_t w = ip >> window_bits;
      if (have && w == last) {
        continue;
      }
      have = true;
      last = w;
      ++n_win_added;
      if (window_index.count(w) != 0) {
        ++n_win_shared;
      }
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
  bool conf_evict = false;   // trace_launch_conf mode 2: conf-0 traces evicted first
  bool xor_hash = false;
  uint64_t cost_cap = 0;     // saturating cost ceiling (0 = unbounded idealized counter)
  uint64_t decay_period = 0; // cost halving interval in insertions (0 = off)
  uint64_t n_insert = 0;
  uint64_t tick = 0;
  uint64_t n_evict = 0;          // total evictions (any mode)
  uint64_t n_conflict_evict = 0; // set-assoc evictions while the store was globally underfull (set skew)
  uint64_t n_win_added = 0;      // distinct windows inserted with new traces
  uint64_t n_win_shared = 0;     // of those, already covered by a different resident trace (fragmentation)
  std::vector<ent> entries;
  std::unordered_map<uint64_t, std::size_t> entry_index;
  std::unordered_map<uint64_t, std::size_t> window_index;
  unsigned tag_bits_ = 0;         // partial-tag width (0 = full match)
  uint64_t n_lookups = 0;         // entry probes
  uint64_t n_alias_hits = 0;      // probes that matched a DIFFERENT trace's partial tag
  uint64_t n_alias_overwrites = 0; // inserts that replaced a different trace with the same partial tag
  std::vector<uint64_t> alias_ips_; // rebased manifest returned on an alias hit
};

#endif

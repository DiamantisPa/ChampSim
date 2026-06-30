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

  [[nodiscard]] std::size_t cap() const { return capacity; }
  [[nodiscard]] std::size_t size() const { return entries.size(); }

  void insert(uint64_t entry, const std::vector<uint64_t>& ips)
  {
    if (auto it = entry_index.find(entry); it != entry_index.end()) { // refresh
      const std::size_t slot = it->second;
      if (window_indexed) {
        remove_windows(slot);
      }
      entries[slot].ips = ips;
      entries[slot].lru = ++tick;
      if (window_indexed) {
        add_windows(slot);
      }
      return;
    }

    std::size_t slot = 0;
    if (entries.size() < capacity) {
      slot = entries.size();
      entries.push_back({entry, ips, ++tick, {}});
    } else { // evict LRU
      slot = lru_victim();
      entry_index.erase(entries[slot].entry);
      if (window_indexed) {
        remove_windows(slot);
      }
      entries[slot] = {entry, ips, ++tick, {}};
    }
    entry_index.emplace(entry, slot);
    if (window_indexed) {
      add_windows(slot);
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
    std::vector<uint64_t> windows; // distinct window keys (only if window_indexed)
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

  std::size_t lru_victim() const
  {
    std::size_t v = 0;
    for (std::size_t i = 1; i < entries.size(); ++i) {
      if (entries[i].lru < entries[v].lru) {
        v = i;
      }
    }
    return v;
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
  unsigned window_bits = 0;
  bool window_indexed = false;
  uint64_t tick = 0;
  std::vector<ent> entries;
  std::unordered_map<uint64_t, std::size_t> entry_index;
  std::unordered_map<uint64_t, std::size_t> window_index;
};

#endif

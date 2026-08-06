#ifndef PREFETCHER_DJOLT_H
#define PREFETCHER_DJOLT_H

#include <cstdint>
#include <array>
#include <vector>
#include <algorithm>
#include <utility>
#include <memory>
#include <numeric>
#include <cassert>
#include <iostream>

#include "modules.h"
#include "instruction.h"

struct djolt : public champsim::modules::prefetcher {
  using prefetcher::prefetcher;

  // =========================================================================
  // D-JOLT parameters
  // =========================================================================
  static constexpr size_t UpperBitPtrBits   = 5;
  static constexpr size_t UpperBitMask      = 0xffffffffff000000ULL;
  static constexpr size_t SignatureBits     = 23;

  static constexpr size_t LongRangePrefetcherDistance   = 15;
  static constexpr size_t LongRangePrefetcher_N_Sets    = 2048;
  static constexpr size_t LongRangePrefetcher_N_Ways    = 4;
  static constexpr size_t LongRangePrefetcher_N_Vectors = 2;
  static constexpr size_t LongRangePrefetcher_VectorSize= 8;
  static constexpr size_t LongRangePrefetcher_TagBits   = 12;

  static constexpr size_t ShortRangePrefetcherDistance   = 4;
  static constexpr size_t ShortRangePrefetcher_N_Sets    = 1024;
  static constexpr size_t ShortRangePrefetcher_N_Ways    = 4;
  static constexpr size_t ShortRangePrefetcher_N_Vectors = 2;
  static constexpr size_t ShortRangePrefetcher_VectorSize= 8;
  static constexpr size_t ShortRangePrefetcher_TagBits   = 13;

  static constexpr size_t ExtraMissTable_N_Sets    = 256;
  static constexpr size_t ExtraMissTable_N_Ways    = 4;
  static constexpr size_t ExtraMissTable_N_Vectors = 2;
  static constexpr size_t ExtraMissTable_VectorSize= 8;
  static constexpr size_t ExtraMissTable_TagBits   = 15;

  // =========================================================================
  // Nested types (originally in anonymous namespace)
  // =========================================================================

  struct UpperBitPtr {
    size_t ptr = 0;
    bool operator==(const UpperBitPtr& rhs) const noexcept { return ptr == rhs.ptr; }
    bool operator!=(const UpperBitPtr& rhs) const noexcept { return ptr != rhs.ptr; }
  };

  struct CompressedLineAddress {
    UpperBitPtr upper_part = {};
    uint64_t    lower_part = 0;
    bool isValid() const noexcept { return upper_part.ptr != 0; }
  };

  class UpperBitTable {
    struct Entry { bool valid = false; uint64_t upper_bits = 0; };
    std::array<Entry, (1ull << UpperBitPtrBits) - 1> table = {};
  public:
    std::pair<bool, CompressedLineAddress> compress(uint64_t full_address) {
      const uint64_t upper_bits = full_address & UpperBitMask;
      const uint64_t lower_bits = (full_address & ~UpperBitMask) >> LOG2_BLOCK_SIZE;

      const auto exists_pos = std::find_if(table.begin(), table.end(),
        [upper_bits](const Entry& e) noexcept { return e.valid && upper_bits == e.upper_bits; });
      const bool entry_exists = exists_pos != table.end();

      if (entry_exists) {
        return { true, { { static_cast<size_t>(exists_pos - table.begin()) + 1 }, lower_bits } };
      } else {
        const auto invalid_pos = std::find_if(table.begin(), table.end(),
          [](const Entry& e) noexcept { return !e.valid; });
        const bool invalid_entry_found = invalid_pos != table.end();

        if (invalid_entry_found) {
          (*invalid_pos) = { true, upper_bits };
          return { true, { { static_cast<size_t>(invalid_pos - table.begin()) + 1 }, lower_bits } };
        } else {
          return { false, {} };
        }
      }
    }

    uint64_t decompress(CompressedLineAddress cla) const {
      return table.at(cla.upper_part.ptr - 1).upper_bits + (cla.lower_part << LOG2_BLOCK_SIZE);
    }
  };

  // -------------------------------------------------------------------------
  // Utility: LRU helpers
  // -------------------------------------------------------------------------
  template<size_t N>
  static std::array<size_t, N> make_initialized_lru_order() {
    std::array<size_t, N> ret;
    std::iota(ret.begin(), ret.end(), 0);
    return ret;
  }

  template<class T>
  static void update_lru_order(T& lru_order, size_t touch_pos) noexcept {
    assert(touch_pos < lru_order.size());
    for (auto& e : lru_order) {
      if (e < lru_order.at(touch_pos)) { ++e; }
    }
    lru_order.at(touch_pos) = 0;
  }

  // -------------------------------------------------------------------------
  // Siggen_FifoRetCnt<HistLen>
  // -------------------------------------------------------------------------
  template<size_t HistLen>
  class Siggen_FifoRetCnt {
    std::array<uint32_t, HistLen> ghist = {};
    size_t   head         = 0;
    size_t   return_count = 0;

    uint32_t makeSig() const noexcept {
      uint32_t sig = 0;
      for (size_t i = head; i < head + HistLen; ++i) {
        const uint32_t pc = ghist.at(i % HistLen);
        sig = (sig << (SignatureBits - 5)) | (sig >> 5);
        sig ^= pc ^ (pc >> 2);
        sig &= ((1ull << SignatureBits) - 1);
      }
      sig ^= static_cast<uint32_t>(return_count * 0xabcd);
      return sig & ((1ull << SignatureBits) - 1);
    }
  public:
    uint32_t onReturnInstruction(uint64_t, uint64_t) {
      ++return_count;
      return makeSig();
    }
    uint32_t onCallInstruction(uint64_t ip, uint64_t) {
      return_count = 0;
      ghist.at(head) = static_cast<uint32_t>(ip);
      head = (head + 1) % HistLen;
      return makeSig();
    }
  };

  // -------------------------------------------------------------------------
  // MissInfo<VectorSize>
  // -------------------------------------------------------------------------
  template<size_t VectorSize>
  class MissInfo {
    CompressedLineAddress base_address = {};
    std::array<bool, VectorSize> bit_vector = {};
  public:
    bool add(CompressedLineAddress address) noexcept {
      if (base_address.isValid()) {
        if (base_address.upper_part.ptr != address.upper_part.ptr) return false;
        const int64_t diff = static_cast<int64_t>(address.lower_part) - static_cast<int64_t>(base_address.lower_part);
        if (diff < 0) return false;
        if (diff == 0) return true;
        if (static_cast<size_t>(diff - 1) < bit_vector.size()) {
          bit_vector.at(static_cast<size_t>(diff - 1)) = true;
          return true;
        }
        return false;
      } else {
        base_address = address;
        return true;
      }
    }
    bool isValid() const noexcept { return base_address.isValid(); }
    std::vector<CompressedLineAddress> getAddresses() const {
      assert(isValid());
      std::vector<CompressedLineAddress> ret;
      ret.push_back(base_address);
      for (size_t i = 0; i < bit_vector.size(); ++i) {
        if (bit_vector.at(i)) {
          CompressedLineAddress tmp = base_address;
          tmp.lower_part += (i + 1);
          ret.push_back(tmp);
        }
      }
      return ret;
    }
  };

  // -------------------------------------------------------------------------
  // MissTableEntry<N_Vectors, VectorSize>
  // -------------------------------------------------------------------------
  template<size_t N_Vectors, size_t VectorSize>
  class MissTableEntry {
    std::array<MissInfo<VectorSize>, N_Vectors> elems = {};
  public:
    bool insert_but_do_not_evict(CompressedLineAddress address) {
      for (size_t i = 0; i < N_Vectors; ++i) {
        if (elems.at(i).add(address)) return true;
      }
      return false;
    }
    std::vector<MissInfo<VectorSize>> getValidEntries() const {
      std::vector<MissInfo<VectorSize>> ret;
      for (const auto& e : elems)
        if (e.isValid()) ret.push_back(e);
      return ret;
    }
  };

  // -------------------------------------------------------------------------
  // FullyAssociativeLRUTable
  // -------------------------------------------------------------------------
  template<size_t N_Ways, class T, class U, class Hasher>
  class FullyAssociativeLRUTable {
    struct Entry {
      size_t tag  = 0;
      U      value = {};
      bool   valid = false;
    };
    std::array<Entry, N_Ways> table = {};
    std::array<size_t, N_Ways> lru_order = make_initialized_lru_order<N_Ways>();

    size_t find_index_of(const T& key) const {
      const size_t tag = Hasher{}(key);
      return static_cast<size_t>(std::find_if(table.begin(), table.end(),
        [tag](const Entry& entry) noexcept { return entry.valid && entry.tag == tag; })
        - table.begin());
    }
  public:
    const U& operator[](const T& key) const {
      assert(contains(key));
      return table.at(find_index_of(key)).value;
    }
    U& operator[](const T& key) {
      assert(contains(key));
      return table.at(find_index_of(key)).value;
    }
    void touch(const T& key) {
      assert(contains(key));
      update_lru_order(lru_order, find_index_of(key));
    }
    void insert(const T& key, const U& elem) {
      if (contains(key)) {
        const size_t index = find_index_of(key);
        table.at(index).value = elem;
        touch(key);
      } else {
        const size_t victim_index = static_cast<size_t>(
          std::max_element(lru_order.begin(), lru_order.end()) - lru_order.begin());
        table.at(victim_index).tag   = Hasher{}(key);
        table.at(victim_index).value = elem;
        table.at(victim_index).valid = true;
        touch(key);
      }
    }
    bool contains(const T& key) const { return find_index_of(key) != N_Ways; }
  };

  // -------------------------------------------------------------------------
  // FullyAssociativeLRUSet
  // -------------------------------------------------------------------------
  template<size_t N_Ways, class T>
  class FullyAssociativeLRUSet {
    std::array<T, N_Ways> table = {};
    std::array<size_t, N_Ways> lru_order = make_initialized_lru_order<N_Ways>();
  public:
    T&       at(size_t i)       { return table.at(i); }
    const T& at(size_t i) const { return table.at(i); }
    void     touch(size_t i)    { update_lru_order(lru_order, i); }
    size_t   find_lru_index()   {
      return static_cast<size_t>(
        std::max_element(lru_order.begin(), lru_order.end()) - lru_order.begin());
    }
  };

  // -------------------------------------------------------------------------
  // SetAssociativeLRUTable
  // -------------------------------------------------------------------------
  template<size_t N_Sets, size_t N_Ways, class T, class U, class Hasher>
  class SetAssociativeLRUTable {
    struct HasherForIndex { size_t operator()(const T& key) const { return Hasher{}(key) % N_Sets; } };
    struct HasherForTag   { size_t operator()(const T& key) const { return Hasher{}(key) / N_Sets; } };
    using Entry = FullyAssociativeLRUTable<N_Ways, T, U, HasherForTag>;
    std::array<Entry, N_Sets> table = {};
  public:
    const U& operator[](const T& key) const { return table.at(HasherForIndex{}(key))[key]; }
    U&       operator[](const T& key)       { return table.at(HasherForIndex{}(key))[key]; }
    void     touch(const T& key)            { table.at(HasherForIndex{}(key)).touch(key); }
    void     insert(const T& key, const U& elem) { table.at(HasherForIndex{}(key)).insert(key, elem); }
    bool     contains(const T& key) const   { return table.at(HasherForIndex{}(key)).contains(key); }
  };

  // -------------------------------------------------------------------------
  // WindowBasedStreamPrefetcher (uses djolt's prefetch_line via pointer back)
  // -------------------------------------------------------------------------
  class WindowBasedStreamPrefetcher {
    static constexpr size_t TrainingThreshold  = 3;
    static constexpr size_t WindowSize         = 2;
    static constexpr size_t Distance           = 2;
    static constexpr size_t Degree             = 2;
    static constexpr size_t TrainingTableSize  = 16;
    static constexpr size_t MonitoringTableSize= 16;

    struct TrainingStreamEntry {
      bool     valid              = false;
      uint64_t start_line_address = 0;
      size_t   count              = 0;
    };
    struct MonitoringStreamEntry {
      bool     valid              = false;
      uint64_t start_line_address = 0;
    };

    FullyAssociativeLRUSet<TrainingTableSize,  TrainingStreamEntry>  training_table  = {};
    FullyAssociativeLRUSet<MonitoringTableSize, MonitoringStreamEntry> monitoring_table = {};

    static bool is_in_range(uint64_t line_address, uint64_t start_line_address,
                            size_t range_size) noexcept {
      return start_line_address <= line_address && line_address < start_line_address + range_size;
    }

    void allocate_training_stream(uint64_t line_address) {
      const size_t lru_index = training_table.find_lru_index();
      training_table.at(lru_index) = TrainingStreamEntry{ true, line_address, 0 };
      training_table.touch(lru_index);
    }

    bool update_training_stream_and_prefetch(uint64_t line_address, djolt* owner) {
      for (size_t i = 0; i < TrainingTableSize; ++i) {
        TrainingStreamEntry& stream = training_table.at(i);
        if (!stream.valid) continue;
        if (!is_in_range(line_address, stream.start_line_address, WindowSize)) continue;
        ++stream.count;
        if (stream.count >= TrainingThreshold) {
          prefetch_initial_stream(line_address, stream, owner);
          allocate_monitoring_stream(MonitoringStreamEntry{ true, line_address });
          stream.valid = false;
        } else {
          training_table.touch(i);
        }
        return true;
      }
      return false;
    }

    void allocate_monitoring_stream(MonitoringStreamEntry entry) {
      const size_t lru_index = monitoring_table.find_lru_index();
      monitoring_table.at(lru_index) = std::move(entry);
      monitoring_table.touch(lru_index);
    }

    bool update_monitoring_stream_and_prefetch(uint64_t line_address, djolt* owner) {
      for (size_t i = 0; i < MonitoringTableSize; ++i) {
        MonitoringStreamEntry& stream = monitoring_table.at(i);
        if (!stream.valid) continue;
        if (!is_in_range(line_address, stream.start_line_address, Distance)) continue;
        for (size_t j = 0; j < Degree; ++j) {
          const uint64_t pf_addr = (stream.start_line_address + Distance) << LOG2_BLOCK_SIZE;
          owner->prefetch_line(champsim::address{pf_addr}, true, 0);
          ++stream.start_line_address;
        }
        monitoring_table.touch(i);
        return true;
      }
      return false;
    }

    void prefetch_initial_stream(uint64_t line_address, const TrainingStreamEntry& /*stream*/,
                                 djolt* owner) {
      for (size_t i = 1; i < Distance; ++i) {
        const uint64_t pf_addr = (line_address + i) << LOG2_BLOCK_SIZE;
        owner->prefetch_line(champsim::address{pf_addr}, true, 0);
      }
    }

  public:
    void cache_operate(uint64_t address, uint8_t cache_hit, uint8_t prefetch_hit, djolt* owner) {
      const bool virtual_miss = cache_hit == 0 || prefetch_hit == 1;
      const uint64_t line_address = address >> LOG2_BLOCK_SIZE;

      const bool already_monitored_stream =
        update_monitoring_stream_and_prefetch(line_address, owner);

      if (virtual_miss) {
        const bool already_training_stream =
          update_training_stream_and_prefetch(line_address, owner);

        if (!already_monitored_stream && !already_training_stream) {
          allocate_training_stream(line_address);
        }
      }
    }
  };

  // -------------------------------------------------------------------------
  // SignatureQueue<N>
  // -------------------------------------------------------------------------
  template<size_t N>
  class SignatureQueue {
    std::array<uint32_t, N> queue = {};
    size_t head = 0;
  public:
    uint32_t back() const noexcept { return queue.at((head + 1) % N); }
    void insert(uint32_t x) noexcept { queue.at((head + 1) % N) = x; head = (head + 1) % N; }
  };

  // =========================================================================
  // State — all D_JOLT_PREFETCHER members become members of djolt
  // =========================================================================
  struct SigHasher { size_t operator()(const uint32_t& x) const noexcept { return x; } };

  Siggen_FifoRetCnt<4>  siggen_1 = {};
  Siggen_FifoRetCnt<7>  siggen_2 = {};

  SignatureQueue<ShortRangePrefetcherDistance> sig_history_1 = {};
  SignatureQueue<LongRangePrefetcherDistance>  sig_history_2 = {};

  SetAssociativeLRUTable<
    ShortRangePrefetcher_N_Sets, ShortRangePrefetcher_N_Ways,
    uint32_t, MissTableEntry<ShortRangePrefetcher_N_Vectors, ShortRangePrefetcher_VectorSize>,
    SigHasher> miss_table_1 = {};

  SetAssociativeLRUTable<
    LongRangePrefetcher_N_Sets, LongRangePrefetcher_N_Ways,
    uint32_t, MissTableEntry<LongRangePrefetcher_N_Vectors, LongRangePrefetcher_VectorSize>,
    SigHasher> miss_table_2 = {};

  SetAssociativeLRUTable<
    ExtraMissTable_N_Sets, ExtraMissTable_N_Ways,
    uint32_t, MissTableEntry<ExtraMissTable_N_Vectors, ExtraMissTable_VectorSize>,
    SigHasher> extra_miss_table = {};

  WindowBasedStreamPrefetcher stream_prefetcher = {};

  UpperBitTable upper_bit_table = {};

  // =========================================================================
  // Required prefetcher interface
  // =========================================================================
  void     prefetcher_initialize();
  void     prefetcher_branch_operate(champsim::address ip, uint8_t branch_type,
                                     champsim::address branch_target);
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip,
                                    uint8_t cache_hit, bool useful_prefetch,
                                    access_type type, uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way,
                                 uint8_t prefetch, champsim::address evicted_addr,
                                 uint32_t metadata_in);
  void     prefetcher_cycle_operate();
  void     prefetcher_final_stats();

private:
  // =========================================================================
  // Private helpers (D_JOLT_PREFETCHER methods)
  // =========================================================================
  template<class Table>
  void prefetch_with_sig(const Table& table, uint32_t sig);

  template<class Table>
  void learn_with_sig(Table& table, uint32_t sig, CompressedLineAddress c_address);
};

#endif // PREFETCHER_DJOLT_H

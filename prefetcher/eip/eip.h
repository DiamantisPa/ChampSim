////////////////////////////////////////////////////////////////////////
//
//  Code submitted for the First Instruction Prefetching Championship
//
//  Authors: Alberto Ros (aros@ditec.um.es)
//           Alexandra Jimborean (alexandra.jimborean@um.es)
//
//  Paper #30: The Entangling Instruction Prefetcher
//
//  Converted to ChampSim module-based API by Georgios Vavouliotis
//
////////////////////////////////////////////////////////////////////////

#ifndef PREFETCHER_EIP_H
#define PREFETCHER_EIP_H

#include <cassert>
#include <cstdint>
#include <algorithm>

#include "modules.h"

// LINE AND MERGE BASIC BLOCK SIZE
#define L1I_MERGE_BBSIZE_BITS 7
#define L1I_MERGE_BBSIZE_MAX_VALUE ((1 << L1I_MERGE_BBSIZE_BITS) - 1)

// TIME AND OVERFLOWS
#define L1I_TIME_DIFF_BITS 20
#define L1I_TIME_DIFF_OVERFLOW ((uint64_t)1 << L1I_TIME_DIFF_BITS)
#define L1I_TIME_DIFF_MASK (L1I_TIME_DIFF_OVERFLOW - 1)

#define L1I_TIME_BITS 12
#define L1I_TIME_OVERFLOW ((uint64_t)1 << L1I_TIME_BITS)
#define L1I_TIME_MASK (L1I_TIME_OVERFLOW - 1)

// ENTANGLED COMPRESSION FORMAT
#define L1I_ENTANGLED_MAX_FORMATS 7

// HISTORY TABLE
#define L1I_HIST_TABLE_ENTRIES 1072
#define L1I_HIST_TABLE_MASK (L1I_HIST_TABLE_ENTRIES - 1)
#define L1I_BB_MERGE_ENTRIES 4
#define L1I_HIST_TAG_BITS 58
#define L1I_HIST_TAG_MASK (((uint64_t)1 << L1I_HIST_TAG_BITS) - 1)

// TIMING TABLE PARAMETERS
// L1I cache geometry (hardcoded to match baseline_golden_cove.json)
#define L1I_SET 64
#define L1I_WAY 8
#define L1I_PQ_SIZE 32
#define L1I_MSHR_SIZE 8

#define L1I_SET_BITS 6
#define L1I_TIMING_MSHR_SIZE (L1I_PQ_SIZE + L1I_MSHR_SIZE + 2)
#define L1I_TIMING_MSHR_TAG_BITS 42
#define L1I_TIMING_MSHR_TAG_MASK (((uint64_t)1 << L1I_HIST_TAG_BITS) - 1)
#define L1I_TIMING_CACHE_TAG_BITS (L1I_TIMING_MSHR_TAG_BITS - L1I_SET_BITS)
#define L1I_TIMING_CACHE_TAG_MASK (((uint64_t)1 << L1I_HIST_TAG_BITS) - 1)

// ENTANGLED TABLE PARAMETERS
#define L1I_ENTANGLED_TABLE_INDEX_BITS 8
#define L1I_ENTANGLED_TABLE_SETS (1 << L1I_ENTANGLED_TABLE_INDEX_BITS)
#define L1I_ENTANGLED_TABLE_WAYS 34
#define L1I_MAX_ENTANGLED_PER_LINE L1I_ENTANGLED_NUM_FORMATS
#define L1I_TAG_BITS (42 - L1I_ENTANGLED_TABLE_INDEX_BITS)
#define L1I_TAG_MASK (((uint64_t)1 << L1I_TAG_BITS) - 1)
#define L1I_CONFIDENCE_COUNTER_BITS 2
#define L1I_CONFIDENCE_COUNTER_MAX_VALUE ((1 << L1I_CONFIDENCE_COUNTER_BITS) - 1)
#define L1I_CONFIDENCE_COUNTER_THRESHOLD 1

#define L1I_TRIES_AVAIL_ENTANGLED 6
#define L1I_TRIES_AVAIL_ENTANGLED_NOT_PRESENT 2

// EXTRA PREFETCH QUEUE
#define L1I_XPQ_ENTRIES 32
#define L1I_XPQ_MASK (L1I_XPQ_ENTRIES - 1)

#define L1I_ENTANGLED_NUM_FORMATS 6

struct eip : public champsim::modules::prefetcher {
  using prefetcher::prefetcher;

  // ----------------------------------------------------------------
  // Nested types
  // ----------------------------------------------------------------
  struct l1i_hist_entry {
    uint64_t tag;       // L1I_HIST_TAG_BITS bits
    uint64_t time_diff; // L1I_TIME_DIFF_BITS bits
    uint32_t bb_size;   // L1I_MERGE_BBSIZE_BITS bits
  };

  struct l1i_timing_mshr_entry {
    bool valid;
    uint64_t tag;            // L1I_TIMING_MSHR_TAG_BITS bits
    uint64_t bere_line_addr; // 58 bits
    uint64_t timestamp;      // L1I_TIME_BITS bits
    bool accessed;
  };

  struct l1i_timing_cache_entry {
    bool valid;
    uint64_t tag;            // L1I_TIMING_CACHE_TAG_BITS bits
    uint64_t bere_line_addr; // 58 bits
    bool accessed;
  };

  struct l1i_entangled_entry {
    uint64_t tag;                                        // L1I_TAG_BITS bits
    uint32_t format;                                     // 3 bits
    uint64_t entangled_addr[L1I_MAX_ENTANGLED_PER_LINE]; // compressed
    uint32_t entangled_conf[L1I_MAX_ENTANGLED_PER_LINE]; // L1I_CONFIDENCE_COUNTER_BITS bits
    uint32_t bb_size;                                    // L1I_MERGE_BBSIZE_BITS bits
  };

  struct l1i_xpq_entry {
    uint64_t line_addr;      // 58 bits
    uint64_t entangled_addr; // 58 bits
    uint32_t bb_size;        // L1I_MERGE_BBSIZE_BITS bits
  };

  // ----------------------------------------------------------------
  // Member state (replaces NUM_CPUS-indexed globals)
  // ----------------------------------------------------------------

  // Current cycle counter (incremented in prefetcher_cycle_operate)
  uint64_t current_cycle = 0;

  // Basic block tracking
  uint64_t l1i_last_basic_block = 0;
  uint32_t l1i_consecutive_count = 0;
  uint32_t l1i_basic_block_merge_diff = 0;

  // History table
  l1i_hist_entry l1i_hist_table[L1I_HIST_TABLE_ENTRIES];
  uint64_t l1i_hist_table_head = 0;
  uint64_t l1i_hist_table_head_time = 0;

  // Timing tables
  l1i_timing_mshr_entry  l1i_timing_mshr_table[L1I_TIMING_MSHR_SIZE];
  l1i_timing_cache_entry l1i_timing_cache_table[L1I_SET][L1I_WAY];

  // Entangled table
  uint32_t L1I_ENTANGLED_FORMATS[L1I_ENTANGLED_MAX_FORMATS] = {58, 28, 18, 13, 10, 8, 6};
  l1i_entangled_entry l1i_entangled_table[L1I_ENTANGLED_TABLE_SETS][L1I_ENTANGLED_TABLE_WAYS];
  uint32_t l1i_entangled_fifo[L1I_ENTANGLED_TABLE_SETS];

  // Extra prefetch queue
  l1i_xpq_entry l1i_xpq[L1I_XPQ_ENTRIES];
  uint64_t l1i_xpq_head = 0;

  // ----------------------------------------------------------------
  // Interface methods
  // ----------------------------------------------------------------
  void prefetcher_initialize();
  void prefetcher_branch_operate(champsim::address ip, uint8_t branch_type, champsim::address branch_target);
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
  void prefetcher_cycle_operate();
  void prefetcher_final_stats();

private:
  // ----------------------------------------------------------------
  // Private helper methods
  // ----------------------------------------------------------------

  // Latency helper
  uint64_t l1i_get_latency(uint64_t cycle, uint64_t cycle_prev);

  // History table helpers
  void     l1i_init_hist_table();
  uint64_t l1i_find_hist_entry(uint64_t line_addr);
  void     l1i_add_hist_table(uint64_t line_addr);
  void     l1i_add_bb_size_hist_table(uint64_t line_addr, uint32_t bb_size);
  uint32_t l1i_find_bb_merge_hist_table(uint64_t line_addr);
  uint64_t l1i_get_bere_hist_table(uint64_t line_addr, uint64_t latency, uint32_t skip = 0);

  // Timing table helpers
  void     l1i_init_timing_tables();
  uint64_t l1i_find_timing_mshr_entry(uint64_t line_addr);
  uint64_t l1i_find_timing_cache_entry(uint64_t line_addr);
  uint32_t l1i_get_invalid_timing_mshr_entry();
  uint32_t l1i_get_invalid_timing_cache_entry(uint64_t line_addr);
  void     l1i_add_timing_entry(uint64_t line_addr, uint64_t bere_line_addr);
  void     l1i_invalid_timing_mshr_entry(uint64_t line_addr);
  void     l1i_move_timing_entry(uint64_t line_addr);
  bool     l1i_invalid_timing_cache_entry(uint64_t line_addr, uint64_t &bere_line_addr);
  void     l1i_access_timing_entry(uint64_t line_addr);
  bool     l1i_is_accessed_timing_entry(uint64_t line_addr);
  bool     l1i_completed_request(uint64_t line_addr);
  bool     l1i_ongoing_request(uint64_t line_addr);
  bool     l1i_ongoing_accessed_request(uint64_t line_addr);
  uint64_t l1i_get_latency_timing_mshr(uint64_t line_addr);

  // Entangled table helpers
  void     l1i_init_entangled_table();
  uint32_t l1i_get_format_entangled(uint64_t line_addr, uint64_t entangled_addr);
  uint64_t l1i_extend_format_entangled(uint64_t line_addr, uint64_t entangled_addr, uint32_t format);
  uint64_t l1i_compress_format_entangled(uint64_t entangled_addr, uint32_t format);
  uint32_t l1i_get_way_entangled_table(uint64_t line_addr);
  void     l1i_add_entangled_table(uint64_t line_addr, uint64_t entangled_addr);
  bool     l1i_avail_entangled_table(uint64_t line_addr, uint64_t entangled_addr, bool insert_not_present);
  void     l1i_add_bbsize_table(uint64_t line_addr, uint32_t bb_size);
  uint64_t l1i_get_entangled_addr_entangled_table(uint64_t line_addr, uint32_t index_k);
  uint32_t l1i_get_bbsize_entangled_table(uint64_t line_addr);
  void     l1i_update_confidence_entangled_table(uint64_t line_addr, uint64_t entangled_addr, bool accessed);

  // Extra prefetch queue helpers
  void     l1i_init_xpq();
  void     l1i_add_xpq(uint64_t line_addr, uint64_t entangled_addr, uint32_t bb_size);
  bool     l1i_empty_xpq();
  uint64_t l1i_get_xpq(uint64_t &entangled_addr);
};

#endif // PREFETCHER_EIP_H

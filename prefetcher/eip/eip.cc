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

#include "eip.h"

#include <algorithm>
#include <iostream>

#include "cache.h"

// ============================================================
// INTERFACE
// ============================================================

void eip::prefetcher_initialize()
{
  std::cout << "CPU EIP prefetcher" << std::endl;

  l1i_last_basic_block = 0;
  l1i_consecutive_count = 0;
  l1i_basic_block_merge_diff = 0;
  current_cycle = 0;

  l1i_init_hist_table();
  l1i_init_timing_tables();
  l1i_init_entangled_table();
  l1i_init_xpq();
}

void eip::prefetcher_branch_operate(champsim::address ip, uint8_t branch_type, champsim::address branch_target)
{
  // EIP does not use branch information
}

uint32_t eip::prefetcher_cache_operate(champsim::address addr, champsim::address ip_unused, uint8_t cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in)
{
  uint64_t v_addr = addr.to<uint64_t>();
  uint64_t line_addr = v_addr >> LOG2_BLOCK_SIZE;

  bool consecutive = false;
  if (l1i_last_basic_block + l1i_consecutive_count == line_addr) {
    // Same cache line — already being tracked, nothing to do
    return metadata_in;
  } else if (l1i_last_basic_block + l1i_consecutive_count + 1 == line_addr) {
    // Consecutive cache line
    l1i_consecutive_count++;
    consecutive = true;
  }

  // Queue basic block prefetches
  uint32_t bb_size = l1i_get_bbsize_entangled_table(line_addr);
  if (bb_size > 0) {
    l1i_add_xpq(line_addr + 1, 0, bb_size);
  }

  // Queue entangled and basic block of entangled prefetches
  for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
    uint64_t entangled_addr = l1i_get_entangled_addr_entangled_table(line_addr, k);
    if (entangled_addr && (entangled_addr != line_addr)) {
      uint32_t ebb_size = l1i_get_bbsize_entangled_table(entangled_addr);
      l1i_add_xpq(entangled_addr, line_addr, ebb_size + 1);
    }
  }

  if (!consecutive) {
    // New basic block found
    uint32_t max_bb_size = l1i_get_bbsize_entangled_table(l1i_last_basic_block);

    // Check for merging bb opportunities
    if (l1i_consecutive_count) {
      if (l1i_basic_block_merge_diff > 0) {
        l1i_add_bbsize_table(l1i_last_basic_block - l1i_basic_block_merge_diff,
                             l1i_consecutive_count + l1i_basic_block_merge_diff);
        l1i_add_bb_size_hist_table(l1i_last_basic_block - l1i_basic_block_merge_diff,
                                   l1i_consecutive_count + l1i_basic_block_merge_diff);
      } else {
        l1i_add_bbsize_table(l1i_last_basic_block, std::max(max_bb_size, l1i_consecutive_count));
        l1i_add_bb_size_hist_table(l1i_last_basic_block, std::max(max_bb_size, l1i_consecutive_count));
      }
    }
  }

  if (!consecutive) {
    l1i_consecutive_count = 0;
    l1i_last_basic_block = line_addr;
  }

  if (!consecutive) {
    l1i_basic_block_merge_diff = l1i_find_bb_merge_hist_table(l1i_last_basic_block);
  }

  // Add the request in the history buffer
  if (!consecutive && l1i_basic_block_merge_diff == 0) {
    if (l1i_find_hist_entry(line_addr) == L1I_HIST_TABLE_ENTRIES) {
      l1i_add_hist_table(line_addr);
    } else {
      if (!cache_hit && !l1i_ongoing_accessed_request(line_addr)) {
        l1i_add_hist_table(line_addr);
      }
    }
  }

  // Add miss in the latency table
  if (!cache_hit && !l1i_ongoing_request(line_addr)) {
    l1i_add_timing_entry(line_addr, 0);
    l1i_access_timing_entry(line_addr);
  } else {
    l1i_access_timing_entry(line_addr);
  }

  // Do prefetches — try until PQ full (prefetch_line returns false) or queue empty
  while (!l1i_empty_xpq()) {
    uint64_t entangled_addr = 0;
    uint64_t pf_line_addr = l1i_get_xpq(entangled_addr);
    uint64_t pf_addr = pf_line_addr << LOG2_BLOCK_SIZE;
    if (!l1i_ongoing_request(pf_line_addr)) {
      if (!prefetch_line(champsim::address{pf_addr}, true, 0)) break;
      l1i_add_timing_entry(pf_line_addr, entangled_addr);
    }
  }

  return metadata_in;
}

void eip::prefetcher_cycle_operate()
{
  current_cycle++;

  // Do prefetches
  while (!l1i_empty_xpq()) {
    uint64_t entangled_addr = 0;
    uint64_t pf_line_addr = l1i_get_xpq(entangled_addr);
    uint64_t pf_addr = pf_line_addr << LOG2_BLOCK_SIZE;
    if (!l1i_ongoing_request(pf_line_addr)) {
      if (!prefetch_line(champsim::address{pf_addr}, true, 0)) break;
      l1i_add_timing_entry(pf_line_addr, entangled_addr);
    }
  }
}

uint32_t eip::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr_arg, uint32_t metadata_in)
{
  uint64_t v_addr = addr.to<uint64_t>();
  uint64_t evicted_v_addr = evicted_addr_arg.to<uint64_t>();
  uint64_t line_addr = v_addr >> LOG2_BLOCK_SIZE;
  uint64_t evicted_line_addr = evicted_v_addr >> LOG2_BLOCK_SIZE;

  // Line is being evicted from cache
  if (evicted_v_addr) {
    uint64_t bere_line_addr = 0;
    bool accessed = l1i_invalid_timing_cache_entry(evicted_line_addr, bere_line_addr);
    if (bere_line_addr != 0) {
      l1i_update_confidence_entangled_table(bere_line_addr, evicted_line_addr, accessed);
    }
  }

  uint64_t latency = l1i_get_latency_timing_mshr(line_addr);

  l1i_move_timing_entry(line_addr);

  // Get and update entangled
  if (latency) {
    bool inserted = false;
    for (uint32_t i = 0; i < L1I_TRIES_AVAIL_ENTANGLED; i++) {
      uint64_t bere = l1i_get_bere_hist_table(line_addr, latency, i);
      if (bere && line_addr != bere) {
        if (l1i_avail_entangled_table(bere, line_addr, false)) {
          l1i_add_entangled_table(bere, line_addr);
          inserted = true;
          break;
        }
      }
    }
    if (!inserted) {
      for (uint32_t i = 0; i < L1I_TRIES_AVAIL_ENTANGLED_NOT_PRESENT; i++) {
        uint64_t bere = l1i_get_bere_hist_table(line_addr, latency, i);
        if (bere && line_addr != bere) {
          if (l1i_avail_entangled_table(bere, line_addr, true)) {
            l1i_add_entangled_table(bere, line_addr);
            inserted = true;
            break;
          }
        }
      }
    }
    if (!inserted) {
      uint64_t bere = l1i_get_bere_hist_table(line_addr, latency);
      if (bere && line_addr != bere) {
        l1i_add_entangled_table(bere, line_addr);
      }
    }
  }

  return metadata_in;
}

void eip::prefetcher_final_stats()
{
  std::cout << "CPU EIP prefetcher final stats" << std::endl;
}

// ============================================================
// LATENCY HELPER
// ============================================================

uint64_t eip::l1i_get_latency(uint64_t cycle, uint64_t cycle_prev)
{
  uint64_t cycle_masked = cycle & L1I_TIME_MASK;
  uint64_t cycle_prev_masked = cycle_prev & L1I_TIME_MASK;
  if (cycle_prev_masked > cycle_masked) {
    return (cycle_masked + L1I_TIME_OVERFLOW) - cycle_prev_masked;
  }
  return cycle_masked - cycle_prev_masked;
}

// ============================================================
// HISTORY TABLE
// ============================================================

void eip::l1i_init_hist_table()
{
  l1i_hist_table_head = 0;
  l1i_hist_table_head_time = current_cycle;
  for (uint32_t i = 0; i < L1I_HIST_TABLE_ENTRIES; i++) {
    l1i_hist_table[i].tag = 0;
    l1i_hist_table[i].time_diff = 0;
    l1i_hist_table[i].bb_size = 0;
  }
}

uint64_t eip::l1i_find_hist_entry(uint64_t line_addr)
{
  uint64_t tag = line_addr & L1I_HIST_TAG_MASK;
  for (uint32_t count = 0, i = (l1i_hist_table_head + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES;
       count < L1I_HIST_TABLE_ENTRIES;
       count++, i = (i + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES) {
    if (l1i_hist_table[i].tag == tag) return i;
  }
  return L1I_HIST_TABLE_ENTRIES;
}

void eip::l1i_add_hist_table(uint64_t line_addr)
{
  // Insert empty addresses in hist to avoid timediff overflows
  while (current_cycle - l1i_hist_table_head_time >= L1I_TIME_DIFF_OVERFLOW) {
    l1i_hist_table[l1i_hist_table_head].tag = 0;
    l1i_hist_table[l1i_hist_table_head].time_diff = L1I_TIME_DIFF_MASK;
    l1i_hist_table[l1i_hist_table_head].bb_size = 0;
    l1i_hist_table_head = (l1i_hist_table_head + 1) % L1I_HIST_TABLE_ENTRIES;
    l1i_hist_table_head_time += L1I_TIME_DIFF_MASK;
  }

  l1i_hist_table[l1i_hist_table_head].tag = line_addr & L1I_HIST_TAG_MASK;
  l1i_hist_table[l1i_hist_table_head].time_diff = (current_cycle - l1i_hist_table_head_time) & L1I_TIME_DIFF_MASK;
  l1i_hist_table[l1i_hist_table_head].bb_size = 0;
  l1i_hist_table_head = (l1i_hist_table_head + 1) % L1I_HIST_TABLE_ENTRIES;
  l1i_hist_table_head_time = current_cycle;
}

void eip::l1i_add_bb_size_hist_table(uint64_t line_addr, uint32_t bb_size)
{
  uint64_t index = l1i_find_hist_entry(line_addr);
  if (index < L1I_HIST_TABLE_ENTRIES)
    l1i_hist_table[index].bb_size = bb_size & L1I_MERGE_BBSIZE_MAX_VALUE;
}

uint32_t eip::l1i_find_bb_merge_hist_table(uint64_t line_addr)
{
  uint64_t tag = line_addr & L1I_HIST_TAG_MASK;
  for (uint32_t count = 0, i = (l1i_hist_table_head + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES;
       count < L1I_HIST_TABLE_ENTRIES;
       count++, i = (i + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES) {
    if (count >= L1I_BB_MERGE_ENTRIES) {
      return 0;
    }
    if (tag > l1i_hist_table[i].tag
        && (tag - l1i_hist_table[i].tag) <= l1i_hist_table[i].bb_size) {
      return static_cast<uint32_t>(tag - l1i_hist_table[i].tag);
    }
  }
  assert(false);
  return 0;
}

uint64_t eip::l1i_get_bere_hist_table(uint64_t line_addr, uint64_t latency, uint32_t skip)
{
  uint64_t tag = line_addr & L1I_HIST_TAG_MASK;
  assert(tag);
  uint32_t first = (l1i_hist_table_head + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES;
  uint64_t time_i = l1i_hist_table_head_time;
  uint64_t req_time = 0;
  uint32_t num_skipped = 0;
  for (uint32_t count = 0, i = first;
       count < L1I_HIST_TABLE_ENTRIES;
       count++, i = (i + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES) {
    if (req_time == 0
        && l1i_hist_table[i].tag == tag
        && time_i + latency >= current_cycle) {
      req_time = time_i;
    } else if (req_time) {
      if (l1i_hist_table[i].tag == tag) {
        return 0;
      }
      if (time_i + latency <= req_time && l1i_hist_table[i].tag) {
        if (skip == num_skipped) {
          return l1i_hist_table[i].tag;
        } else {
          num_skipped++;
        }
      }
    }
    time_i -= l1i_hist_table[i].time_diff;
  }
  return 0;
}

// ============================================================
// TIMING TABLES
// ============================================================

void eip::l1i_init_timing_tables()
{
  for (uint32_t i = 0; i < L1I_TIMING_MSHR_SIZE; i++) {
    l1i_timing_mshr_table[i].valid = false;
  }
  for (uint32_t i = 0; i < L1I_SET; i++) {
    for (uint32_t j = 0; j < L1I_WAY; j++) {
      l1i_timing_cache_table[i][j].valid = false;
    }
  }
}

uint64_t eip::l1i_find_timing_mshr_entry(uint64_t line_addr)
{
  for (uint32_t i = 0; i < L1I_TIMING_MSHR_SIZE; i++) {
    if (l1i_timing_mshr_table[i].tag == (line_addr & L1I_TIMING_MSHR_TAG_MASK)
        && l1i_timing_mshr_table[i].valid) return i;
  }
  return L1I_TIMING_MSHR_SIZE;
}

uint64_t eip::l1i_find_timing_cache_entry(uint64_t line_addr)
{
  uint64_t i = line_addr % L1I_SET;
  for (uint32_t j = 0; j < L1I_WAY; j++) {
    if (l1i_timing_cache_table[i][j].tag == ((line_addr >> L1I_SET_BITS) & L1I_TIMING_CACHE_TAG_MASK)
        && l1i_timing_cache_table[i][j].valid) return j;
  }
  return L1I_WAY;
}

uint32_t eip::l1i_get_invalid_timing_mshr_entry()
{
  for (uint32_t i = 0; i < L1I_TIMING_MSHR_SIZE; i++) {
    if (!l1i_timing_mshr_table[i].valid) return i;
  }
  return L1I_TIMING_MSHR_SIZE; // full; caller must check
}

uint32_t eip::l1i_get_invalid_timing_cache_entry(uint64_t line_addr)
{
  uint32_t i = line_addr % L1I_SET;
  for (uint32_t j = 0; j < L1I_WAY; j++) {
    if (!l1i_timing_cache_table[i][j].valid) return j;
  }
  return L1I_WAY; // full; caller must check
}

void eip::l1i_add_timing_entry(uint64_t line_addr, uint64_t bere_line_addr)
{
  if (l1i_find_timing_mshr_entry(line_addr) < L1I_TIMING_MSHR_SIZE) return;
  if (l1i_find_timing_cache_entry(line_addr) < L1I_WAY) return;

  uint32_t i = l1i_get_invalid_timing_mshr_entry();
  if (i >= L1I_TIMING_MSHR_SIZE) return; // MSHR full, drop timing entry
  l1i_timing_mshr_table[i].valid = true;
  l1i_timing_mshr_table[i].tag = line_addr & L1I_TIMING_MSHR_TAG_MASK;
  l1i_timing_mshr_table[i].bere_line_addr = bere_line_addr;
  l1i_timing_mshr_table[i].timestamp = current_cycle & L1I_TIME_MASK;
  l1i_timing_mshr_table[i].accessed = false;
}

void eip::l1i_invalid_timing_mshr_entry(uint64_t line_addr)
{
  uint32_t index = static_cast<uint32_t>(l1i_find_timing_mshr_entry(line_addr));
  assert(index < L1I_TIMING_MSHR_SIZE);
  l1i_timing_mshr_table[index].valid = false;
}

void eip::l1i_move_timing_entry(uint64_t line_addr)
{
  uint32_t index_mshr = static_cast<uint32_t>(l1i_find_timing_mshr_entry(line_addr));
  if (index_mshr == L1I_TIMING_MSHR_SIZE) {
    // Not in MSHR — add directly to cache table as accessed
    uint32_t set = line_addr % L1I_SET;
    uint32_t index_cache = l1i_get_invalid_timing_cache_entry(line_addr);
    if (index_cache >= L1I_WAY) return; // timing cache set full, skip
    l1i_timing_cache_table[set][index_cache].valid = true;
    l1i_timing_cache_table[set][index_cache].tag = (line_addr >> L1I_SET_BITS) & L1I_TIMING_CACHE_TAG_MASK;
    l1i_timing_cache_table[set][index_cache].accessed = true;
    return;
  }
  uint64_t set = line_addr % L1I_SET;
  uint64_t index_cache = l1i_get_invalid_timing_cache_entry(line_addr);
  if (index_cache >= L1I_WAY) {
    // timing cache set full; discard timing info, just free MSHR slot
    l1i_invalid_timing_mshr_entry(line_addr);
    return;
  }
  l1i_timing_cache_table[set][index_cache].valid = true;
  l1i_timing_cache_table[set][index_cache].tag = (line_addr >> L1I_SET_BITS) & L1I_TIMING_CACHE_TAG_MASK;
  l1i_timing_cache_table[set][index_cache].bere_line_addr = l1i_timing_mshr_table[index_mshr].bere_line_addr;
  l1i_timing_cache_table[set][index_cache].accessed = l1i_timing_mshr_table[index_mshr].accessed;
  l1i_invalid_timing_mshr_entry(line_addr);
}

bool eip::l1i_invalid_timing_cache_entry(uint64_t line_addr, uint64_t &bere_line_addr)
{
  uint32_t set = line_addr % L1I_SET;
  uint32_t way = static_cast<uint32_t>(l1i_find_timing_cache_entry(line_addr));
  if (way >= L1I_WAY) { bere_line_addr = 0; return false; } // not tracked
  l1i_timing_cache_table[set][way].valid = false;
  bere_line_addr = l1i_timing_cache_table[set][way].bere_line_addr;
  return l1i_timing_cache_table[set][way].accessed;
}

void eip::l1i_access_timing_entry(uint64_t line_addr)
{
  uint32_t index = static_cast<uint32_t>(l1i_find_timing_mshr_entry(line_addr));
  if (index < L1I_TIMING_MSHR_SIZE) {
    if (!l1i_timing_mshr_table[index].accessed) {
      l1i_timing_mshr_table[index].accessed = true;
    }
    return;
  }
  uint32_t set = line_addr % L1I_SET;
  uint32_t way = static_cast<uint32_t>(l1i_find_timing_cache_entry(line_addr));
  if (way < L1I_WAY) {
    l1i_timing_cache_table[set][way].accessed = true;
  }
}

bool eip::l1i_is_accessed_timing_entry(uint64_t line_addr)
{
  uint32_t index = static_cast<uint32_t>(l1i_find_timing_mshr_entry(line_addr));
  if (index < L1I_TIMING_MSHR_SIZE) {
    return l1i_timing_mshr_table[index].accessed;
  }
  uint32_t set = line_addr % L1I_SET;
  uint32_t way = static_cast<uint32_t>(l1i_find_timing_cache_entry(line_addr));
  if (way < L1I_WAY) {
    return l1i_timing_cache_table[set][way].accessed;
  }
  return false;
}

bool eip::l1i_completed_request(uint64_t line_addr)
{
  return l1i_find_timing_cache_entry(line_addr) < L1I_WAY;
}

bool eip::l1i_ongoing_request(uint64_t line_addr)
{
  return l1i_find_timing_mshr_entry(line_addr) < L1I_TIMING_MSHR_SIZE;
}

bool eip::l1i_ongoing_accessed_request(uint64_t line_addr)
{
  uint32_t index = static_cast<uint32_t>(l1i_find_timing_mshr_entry(line_addr));
  if (index == L1I_TIMING_MSHR_SIZE) return false;
  return l1i_timing_mshr_table[index].accessed;
}

uint64_t eip::l1i_get_latency_timing_mshr(uint64_t line_addr)
{
  uint32_t index = static_cast<uint32_t>(l1i_find_timing_mshr_entry(line_addr));
  if (index == L1I_TIMING_MSHR_SIZE) return 0;
  if (!l1i_timing_mshr_table[index].accessed) return 0;
  return l1i_get_latency(current_cycle, l1i_timing_mshr_table[index].timestamp);
}

// ============================================================
// ENTANGLED TABLE
// ============================================================

uint32_t eip::l1i_get_format_entangled(uint64_t line_addr, uint64_t entangled_addr)
{
  for (uint32_t i = L1I_ENTANGLED_NUM_FORMATS; i != 0; i--) {
    if ((line_addr >> L1I_ENTANGLED_FORMATS[i - 1]) == (entangled_addr >> L1I_ENTANGLED_FORMATS[i - 1])) {
      return i;
    }
  }
  assert(false);
  return 0;
}

uint64_t eip::l1i_extend_format_entangled(uint64_t line_addr, uint64_t entangled_addr, uint32_t format)
{
  return ((line_addr >> L1I_ENTANGLED_FORMATS[format - 1]) << L1I_ENTANGLED_FORMATS[format - 1])
       | (entangled_addr & (((uint64_t)1 << L1I_ENTANGLED_FORMATS[format - 1]) - 1));
}

uint64_t eip::l1i_compress_format_entangled(uint64_t entangled_addr, uint32_t format)
{
  return entangled_addr & (((uint64_t)1 << L1I_ENTANGLED_FORMATS[format - 1]) - 1);
}

void eip::l1i_init_entangled_table()
{
  for (uint32_t i = 0; i < L1I_ENTANGLED_TABLE_SETS; i++) {
    for (uint32_t j = 0; j < L1I_ENTANGLED_TABLE_WAYS; j++) {
      l1i_entangled_table[i][j].tag = 0;
      l1i_entangled_table[i][j].format = 1;
      for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
        l1i_entangled_table[i][j].entangled_addr[k] = 0;
        l1i_entangled_table[i][j].entangled_conf[k] = 0;
      }
      l1i_entangled_table[i][j].bb_size = 0;
    }
    l1i_entangled_fifo[i] = 0;
  }
}

uint32_t eip::l1i_get_way_entangled_table(uint64_t line_addr)
{
  uint64_t tag = (line_addr >> L1I_ENTANGLED_TABLE_INDEX_BITS) & L1I_TAG_MASK;
  uint32_t set = line_addr % L1I_ENTANGLED_TABLE_SETS;
  for (int i = 0; i < L1I_ENTANGLED_TABLE_WAYS; i++) {
    if (l1i_entangled_table[set][i].tag == tag) {
      return static_cast<uint32_t>(i);
    }
  }
  return L1I_ENTANGLED_TABLE_WAYS;
}

void eip::l1i_add_entangled_table(uint64_t line_addr, uint64_t entangled_addr)
{
  uint64_t tag = (line_addr >> L1I_ENTANGLED_TABLE_INDEX_BITS) & L1I_TAG_MASK;
  uint32_t set = line_addr % L1I_ENTANGLED_TABLE_SETS;
  uint32_t way = l1i_get_way_entangled_table(line_addr);
  if (way == L1I_ENTANGLED_TABLE_WAYS) {
    way = l1i_entangled_fifo[set];
    l1i_entangled_table[set][way].tag = tag;
    l1i_entangled_table[set][way].format = 1;
    for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
      l1i_entangled_table[set][way].entangled_addr[k] = 0;
      l1i_entangled_table[set][way].entangled_conf[k] = 0;
    }
    l1i_entangled_table[set][way].bb_size = 0;
    l1i_entangled_fifo[set] = (l1i_entangled_fifo[set] + 1) % L1I_ENTANGLED_TABLE_WAYS;
  }
  for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
    if (l1i_entangled_table[set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD
        && l1i_extend_format_entangled(line_addr, l1i_entangled_table[set][way].entangled_addr[k], l1i_entangled_table[set][way].format) == entangled_addr) {
      l1i_entangled_table[set][way].entangled_conf[k] = L1I_CONFIDENCE_COUNTER_MAX_VALUE;
      return;
    }
  }

  // Adding a new entangled entry
  uint32_t format_new = l1i_get_format_entangled(line_addr, entangled_addr);

  // Check for evictions
  while (true) {
    uint32_t min_format = format_new;
    uint32_t num_valid = 1;
    uint32_t min_value = L1I_CONFIDENCE_COUNTER_MAX_VALUE + 1;
    uint32_t min_pos = 0;
    for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
      if (l1i_entangled_table[set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD) {
        num_valid++;
        uint32_t format_k = l1i_get_format_entangled(
          line_addr,
          l1i_extend_format_entangled(line_addr, l1i_entangled_table[set][way].entangled_addr[k], l1i_entangled_table[set][way].format));
        if (format_k < min_format) {
          min_format = format_k;
        }
        if (l1i_entangled_table[set][way].entangled_conf[k] < min_value) {
          min_value = l1i_entangled_table[set][way].entangled_conf[k];
          min_pos = k;
        }
      }
    }
    if (num_valid > min_format) {
      l1i_entangled_table[set][way].entangled_conf[min_pos] = 0;
    } else {
      // Reformat
      for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
        if (l1i_entangled_table[set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD) {
          l1i_entangled_table[set][way].entangled_addr[k] = l1i_compress_format_entangled(
            l1i_extend_format_entangled(line_addr, l1i_entangled_table[set][way].entangled_addr[k], l1i_entangled_table[set][way].format),
            min_format);
        }
      }
      l1i_entangled_table[set][way].format = min_format;
      break;
    }
  }
  for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
    if (l1i_entangled_table[set][way].entangled_conf[k] < L1I_CONFIDENCE_COUNTER_THRESHOLD) {
      l1i_entangled_table[set][way].entangled_addr[k] = l1i_compress_format_entangled(entangled_addr, l1i_entangled_table[set][way].format);
      l1i_entangled_table[set][way].entangled_conf[k] = L1I_CONFIDENCE_COUNTER_MAX_VALUE;
      return;
    }
  }
}

bool eip::l1i_avail_entangled_table(uint64_t line_addr, uint64_t entangled_addr, bool insert_not_present)
{
  uint32_t set = line_addr % L1I_ENTANGLED_TABLE_SETS;
  uint32_t way = l1i_get_way_entangled_table(line_addr);
  if (way == L1I_ENTANGLED_TABLE_WAYS) return insert_not_present;
  for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
    if (l1i_entangled_table[set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD
        && l1i_extend_format_entangled(line_addr, l1i_entangled_table[set][way].entangled_addr[k], l1i_entangled_table[set][way].format) == entangled_addr) {
      return true;
    }
  }
  uint32_t min_format = l1i_get_format_entangled(line_addr, entangled_addr);
  uint32_t num_valid = 1;
  for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
    if (l1i_entangled_table[set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD) {
      num_valid++;
      uint32_t format_k = l1i_get_format_entangled(
        line_addr,
        l1i_extend_format_entangled(line_addr, l1i_entangled_table[set][way].entangled_addr[k], l1i_entangled_table[set][way].format));
      if (format_k < min_format) {
        min_format = format_k;
      }
    }
  }
  return (num_valid <= min_format);
}

void eip::l1i_add_bbsize_table(uint64_t line_addr, uint32_t bb_size)
{
  uint64_t tag = (line_addr >> L1I_ENTANGLED_TABLE_INDEX_BITS) & L1I_TAG_MASK;
  uint32_t set = line_addr % L1I_ENTANGLED_TABLE_SETS;
  uint32_t way = l1i_get_way_entangled_table(line_addr);
  if (way == L1I_ENTANGLED_TABLE_WAYS) {
    way = l1i_entangled_fifo[set];
    l1i_entangled_table[set][way].tag = tag;
    l1i_entangled_table[set][way].format = 1;
    for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
      l1i_entangled_table[set][way].entangled_addr[k] = 0;
      l1i_entangled_table[set][way].entangled_conf[k] = 0;
    }
    l1i_entangled_table[set][way].bb_size = 0;
    l1i_entangled_fifo[set] = (l1i_entangled_fifo[set] + 1) % L1I_ENTANGLED_TABLE_WAYS;
  }
  if (bb_size > l1i_entangled_table[set][way].bb_size) {
    l1i_entangled_table[set][way].bb_size = bb_size & L1I_MERGE_BBSIZE_MAX_VALUE;
  }
}

uint64_t eip::l1i_get_entangled_addr_entangled_table(uint64_t line_addr, uint32_t index_k)
{
  uint32_t set = line_addr % L1I_ENTANGLED_TABLE_SETS;
  uint32_t way = l1i_get_way_entangled_table(line_addr);
  if (way < L1I_ENTANGLED_TABLE_WAYS) {
    if (l1i_entangled_table[set][way].entangled_conf[index_k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD) {
      return l1i_extend_format_entangled(line_addr, l1i_entangled_table[set][way].entangled_addr[index_k], l1i_entangled_table[set][way].format);
    }
  }
  return 0;
}

uint32_t eip::l1i_get_bbsize_entangled_table(uint64_t line_addr)
{
  uint32_t set = line_addr % L1I_ENTANGLED_TABLE_SETS;
  uint32_t way = l1i_get_way_entangled_table(line_addr);
  if (way < L1I_ENTANGLED_TABLE_WAYS) {
    return l1i_entangled_table[set][way].bb_size;
  }
  return 0;
}

void eip::l1i_update_confidence_entangled_table(uint64_t line_addr, uint64_t entangled_addr, bool accessed)
{
  uint32_t set = line_addr % L1I_ENTANGLED_TABLE_SETS;
  uint32_t way = l1i_get_way_entangled_table(line_addr);
  if (way < L1I_ENTANGLED_TABLE_WAYS) {
    for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
      if (l1i_entangled_table[set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD
          && l1i_extend_format_entangled(line_addr, l1i_entangled_table[set][way].entangled_addr[k], l1i_entangled_table[set][way].format) == entangled_addr) {
        if (accessed && l1i_entangled_table[set][way].entangled_conf[k] < L1I_CONFIDENCE_COUNTER_MAX_VALUE) {
          l1i_entangled_table[set][way].entangled_conf[k]++;
        }
        if (!accessed && l1i_entangled_table[set][way].entangled_conf[k] > 0) {
          l1i_entangled_table[set][way].entangled_conf[k]--;
        }
      }
    }
  }
}

// ============================================================
// EXTRA PREFETCH QUEUE
// ============================================================

void eip::l1i_init_xpq()
{
  l1i_xpq_head = 0;
  for (uint32_t i = 0; i < L1I_XPQ_ENTRIES; i++) {
    l1i_xpq[i].line_addr = 0;
    l1i_xpq[i].entangled_addr = 0;
    l1i_xpq[i].bb_size = 0;
  }
}

void eip::l1i_add_xpq(uint64_t line_addr, uint64_t entangled_addr, uint32_t bb_size)
{
  assert(bb_size > 0);

  // Merge if possible
  uint32_t first = (l1i_xpq_head + L1I_XPQ_MASK) % L1I_XPQ_ENTRIES;
  for (uint32_t count = 0, i = first;
       count < L1I_XPQ_ENTRIES;
       count++, i = (i + L1I_XPQ_MASK) % L1I_XPQ_ENTRIES) {
    if (l1i_xpq[l1i_xpq_head].bb_size
        && line_addr == l1i_xpq[i].line_addr) {
      if (l1i_xpq[l1i_xpq_head].bb_size < bb_size) {
        l1i_xpq[l1i_xpq_head].bb_size = bb_size;
        return;
      }
    }
  }

  l1i_xpq[l1i_xpq_head].line_addr = line_addr;
  l1i_xpq[l1i_xpq_head].entangled_addr = entangled_addr;
  l1i_xpq[l1i_xpq_head].bb_size = bb_size;
  l1i_xpq_head = (l1i_xpq_head + 1) % L1I_XPQ_ENTRIES;
}

bool eip::l1i_empty_xpq()
{
  return l1i_xpq[(l1i_xpq_head + L1I_XPQ_MASK) % L1I_XPQ_ENTRIES].bb_size == 0;
}

uint64_t eip::l1i_get_xpq(uint64_t &entangled_addr)
{
  assert(!l1i_empty_xpq());

  // find tail
  uint32_t tail;
  for (tail = l1i_xpq_head;
       tail != (l1i_xpq_head + L1I_XPQ_MASK) % L1I_XPQ_ENTRIES;
       tail = (tail + 1) % L1I_XPQ_ENTRIES) {
    if (l1i_xpq[tail].bb_size) {
      break;
    }
  }

  uint64_t pf_addr = l1i_xpq[tail].line_addr;
  entangled_addr = l1i_xpq[tail].entangled_addr;

  l1i_xpq[tail].bb_size--;
  if (l1i_xpq[tail].bb_size == 0) {
    return pf_addr;
  }
  l1i_xpq[tail].line_addr++;
  l1i_xpq[tail].entangled_addr = 0;
  return pf_addr;
}

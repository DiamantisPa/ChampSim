#ifndef PREFETCHER_EP4KE_H
#define PREFETCHER_EP4KE_H

#include <cstdint>

#include "address.h"
#include "modules.h"

// EP-4Ke: ISCA'21 cost-effective Entangling Instruction Prefetcher (UCP artifact version)
struct ep4ke : public champsim::modules::prefetcher {
  using prefetcher::prefetcher;

  uint64_t core_cycle_ = 0; // maintained via prefetcher_cycle_operate (module API has no cycle source)

  void prefetcher_initialize();
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in);
  void prefetcher_cycle_operate();
  void prefetcher_final_stats();
};

#endif

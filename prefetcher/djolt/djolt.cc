// D-JOLT instruction prefetcher
// Converted from IPC1 championship format to ChampSim module API

#include "djolt.h"
#include "cache.h"

// ---------------------------------------------------------------------------
// prefetch_with_sig: issue prefetches for all addresses recorded under sig
// ---------------------------------------------------------------------------
template<class Table>
void djolt::prefetch_with_sig(const Table& table, uint32_t sig)
{
  if (table.contains(sig)) {
    for (const auto& v : table[sig].getValidEntries()) {
      for (const auto& address : v.getAddresses()) {
        const uint64_t pf_addr = upper_bit_table.decompress(address);
        prefetch_line(champsim::address{pf_addr}, true, 0);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// learn_with_sig: record a miss address under the given signature
// ---------------------------------------------------------------------------
template<class Table>
void djolt::learn_with_sig(Table& table, uint32_t sig, CompressedLineAddress c_address)
{
  if (!table.contains(sig)) {
    table.insert(sig, {});
  } else {
    table.touch(sig);
  }
  const bool success = table[sig].insert_but_do_not_evict(c_address);
  if (!success) {
    if (!extra_miss_table.contains(sig)) {
      extra_miss_table.insert(sig, {});
    } else {
      extra_miss_table.touch(sig);
    }
    extra_miss_table[sig].insert_but_do_not_evict(c_address);
  } else if (extra_miss_table.contains(sig)) {
    extra_miss_table.touch(sig);
  }
}

// ---------------------------------------------------------------------------
// prefetcher_initialize
// ---------------------------------------------------------------------------
void djolt::prefetcher_initialize()
{
  std::cout << "L1I D-JOLT instruction prefetcher has been constructed!" << std::endl;
}

// ---------------------------------------------------------------------------
// prefetcher_branch_operate
// ---------------------------------------------------------------------------
void djolt::prefetcher_branch_operate(champsim::address ip, uint8_t branch_type,
                                      champsim::address branch_target)
{
  uint64_t ip_val         = ip.to<uint64_t>();
  uint64_t branch_tgt     = branch_target.to<uint64_t>();

  uint32_t sig_1, sig_2;

  if (branch_type == BRANCH_DIRECT_CALL || branch_type == BRANCH_INDIRECT_CALL) {
    sig_1 = siggen_1.onCallInstruction(ip_val, branch_tgt);
    sig_2 = siggen_2.onCallInstruction(ip_val, branch_tgt);
  } else if (branch_type == BRANCH_RETURN) {
    sig_1 = siggen_1.onReturnInstruction(ip_val, branch_tgt);
    sig_2 = siggen_2.onReturnInstruction(ip_val, branch_tgt);
  } else {
    return;
  }

  assert(sig_1 < (1ull << ExtraMissTable_TagBits) * ExtraMissTable_N_Sets);
  assert(sig_2 < (1ull << ExtraMissTable_TagBits) * ExtraMissTable_N_Sets);
  assert(sig_1 < (1ull << ShortRangePrefetcher_TagBits) * ShortRangePrefetcher_N_Sets);
  assert(sig_2 < (1ull << LongRangePrefetcher_TagBits)  * LongRangePrefetcher_N_Sets);

  sig_history_1.insert(sig_1);
  sig_history_2.insert(sig_2);

  prefetch_with_sig(miss_table_1,    sig_1);
  prefetch_with_sig(extra_miss_table, sig_1);
  prefetch_with_sig(miss_table_2,    sig_2);
  prefetch_with_sig(extra_miss_table, sig_2);
}

// ---------------------------------------------------------------------------
// prefetcher_cache_operate
// ---------------------------------------------------------------------------
uint32_t djolt::prefetcher_cache_operate(champsim::address addr_param, champsim::address /*ip*/,
                                         uint8_t cache_hit, bool useful_prefetch,
                                         access_type /*type*/, uint32_t metadata_in)
{
  uint64_t v_addr = addr_param.to<uint64_t>();

  // Convert useful_prefetch back to the prefetch_hit uint8_t convention
  const uint8_t prefetch_hit = useful_prefetch ? 1 : 0;

  const bool miss         = cache_hit == 0;

  stream_prefetcher.cache_operate(v_addr, cache_hit, prefetch_hit, this);

  if (miss) {
    const auto compress_result = upper_bit_table.compress(v_addr);
    const bool compress_success  = compress_result.first;
    const CompressedLineAddress c_address = compress_result.second;

    if (!compress_success) { return metadata_in; }

    learn_with_sig(miss_table_1, sig_history_1.back(), c_address);
    learn_with_sig(miss_table_2, sig_history_2.back(), c_address);
  }

  return metadata_in;
}

// ---------------------------------------------------------------------------
// prefetcher_cache_fill
// ---------------------------------------------------------------------------
uint32_t djolt::prefetcher_cache_fill(champsim::address /*addr*/, long /*set*/, long /*way*/,
                                      uint8_t /*prefetch*/, champsim::address /*evicted_addr*/,
                                      uint32_t metadata_in)
{
  return metadata_in;
}

// ---------------------------------------------------------------------------
// prefetcher_cycle_operate
// ---------------------------------------------------------------------------
void djolt::prefetcher_cycle_operate()
{
}

// ---------------------------------------------------------------------------
// prefetcher_final_stats
// ---------------------------------------------------------------------------
void djolt::prefetcher_final_stats()
{
}

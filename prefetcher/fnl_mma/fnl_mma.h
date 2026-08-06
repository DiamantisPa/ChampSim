#ifndef PREFETCHER_FNL_MMA_H
#define PREFETCHER_FNL_MMA_H

#include <cstdint>
#include <cstdio>
#include <iostream>

#include "modules.h"

struct fnl_mma : public champsim::modules::prefetcher {
  using prefetcher::prefetcher;

  // -----------------------------------------------------------------------
  // Compile-time constants
  // -----------------------------------------------------------------------
  static constexpr int DISTAHEAD       = 10;
  static constexpr int NSHIFT          = 10;
  static constexpr int LOGMULTSIZE     = 0;
  static constexpr int MMA_FILT_SIZE   = 24;
  static constexpr int DISTAHEADMAX    = 80;
  static constexpr int MAXFNL          = 5;
  static constexpr int PERIODRESET     = 8192;
  static constexpr int FNL_NBENTRIES   = (1 << (16 + LOGMULTSIZE));
  static constexpr int NBWAYISHADOW    = 3;
  static constexpr int SIZESHADOWICACHE= 64 * NBWAYISHADOW;
  static constexpr int NBWAYFILTERFNL  = 4;
  static constexpr int SIZEWAYFILTERFNL= 32;
  static constexpr int SIZEFILTERFNL   = SIZEWAYFILTERFNL * NBWAYFILTERFNL;
  static constexpr int NBWAYPRED       = 8;
  static constexpr int LOGTAGNEXTMISS  = 12;
  static constexpr int LOGWAYNEXTMISS  = (10 + LOGMULTSIZE);
  static constexpr int SIZEWAYNEXTMISS = (1 << LOGWAYNEXTMISS);

  // -----------------------------------------------------------------------
  // PredictMiss — inner class; pointers set at runtime to the shared arrays
  // -----------------------------------------------------------------------
  class PredictMiss {
  public:
    uint64_t* Ntag;
    uint64_t* NBlock;
    int8_t*   U;
    int*      NbMiss;
    int       distahead;

    // Pointer arguments supplied by the outer class in prefetcher_initialize()
    void init(uint64_t* ntag, uint64_t* nblock, int* nbmiss, int8_t* u, int x);
    uint64_t AheadPredict(uint64_t Addr);
    void     LinkAhead(uint64_t Block, uint64_t PrevAddr, uint8_t Hit);

  private:
    // Needs access to MYRANDOM() from owner — forward reference resolved
    // via a function pointer set by the outer class.
    uint64_t (*myrandom_fn)() = nullptr;

  public:
    void set_rng(uint64_t (*fn)()) { myrandom_fn = fn; }
  };

  // -----------------------------------------------------------------------
  // Shared MMA table arrays (pointed to by both AHEAD and AHEADphist)
  // -----------------------------------------------------------------------
  uint64_t GNtag   [NBWAYPRED * SIZEWAYNEXTMISS] = {};
  uint64_t GNblock [NBWAYPRED * SIZEWAYNEXTMISS] = {};
  int      GNbMiss [NBWAYPRED * SIZEWAYNEXTMISS] = {};
  int8_t   GU      [NBWAYPRED * SIZEWAYNEXTMISS] = {};

  // -----------------------------------------------------------------------
  // Global state (was file-scope in original)
  // -----------------------------------------------------------------------
  uint64_t PREVPRED [MMA_FILT_SIZE]                    = {};
  uint64_t PREVADDR [DISTAHEADMAX + 1]                 = {};
  uint64_t PREFCAND [DISTAHEADMAX + 1]                 = {};
  uint64_t PrefetchCandidate                           = 0;

  int  WorthPF [FNL_NBENTRIES] = {};
  int  Touched [FNL_NBENTRIES] = {};
  int  ptReset                 = 0;

  uint64_t ShadowICache [64][NBWAYISHADOW] = {};
  uint64_t JUSTNLPREFETCH [SIZEWAYFILTERFNL][NBWAYFILTERFNL] = {};

  // PredictMiss instances
  PredictMiss AHEAD;
  PredictMiss AHEADphist;

  // -----------------------------------------------------------------------
  // Private helper functions
  // -----------------------------------------------------------------------
private:
  uint64_t MYRANDOM();
  void     JustFnl(uint64_t Block);
  bool     WasNotJustFnl(uint64_t Block);
  bool     IsInIShadow(uint64_t Block, bool Insert);

  // Static trampoline so PredictMiss can call back into the instance.
  // We store a raw pointer to the owning fnl_mma and use a static wrapper.
  static fnl_mma* s_instance;
  static uint64_t s_myrandom_trampoline();

public:
  // -----------------------------------------------------------------------
  // Required prefetcher interface
  // -----------------------------------------------------------------------
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
};

#endif // PREFETCHER_FNL_MMA_H

#include "fnl_mma.h"

#include <cassert>
#include <cstdio>
#include <iostream>

#include "cache.h"

// ---------------------------------------------------------------------------
// Static trampoline state
// ---------------------------------------------------------------------------
fnl_mma* fnl_mma::s_instance = nullptr;

uint64_t fnl_mma::s_myrandom_trampoline()
{
  assert(s_instance != nullptr);
  return s_instance->MYRANDOM();
}

// ---------------------------------------------------------------------------
// MYRANDOM — pseudo-RNG, static seed kept per instance via a member
// ---------------------------------------------------------------------------
uint64_t fnl_mma::MYRANDOM()
{
  static uint64_t RANDSEED = 0x3f79a17b4ULL;
  uint64_t X = (RANDSEED >> 7) * 0x9745931ULL;
  if (X == RANDSEED)
    X++;
  RANDSEED = X;
  return (RANDSEED & 127);
}

// ---------------------------------------------------------------------------
// JustFnl — mark block as just fetched/prefetched
// ---------------------------------------------------------------------------
void fnl_mma::JustFnl(uint64_t Block)
{
  uint64_t set = Block & (SIZEWAYFILTERFNL - 1);
  uint64_t tag = (Block / SIZEWAYFILTERFNL) & ((1 << 15) - 1);
  for (int i = NBWAYFILTERFNL - 1; i > 0; i--)
    JUSTNLPREFETCH[set][i] = JUSTNLPREFETCH[set][i - 1];
  JUSTNLPREFETCH[set][0] = tag;
}

// ---------------------------------------------------------------------------
// WasNotJustFnl — check if predecessor block was recently fetched
// ---------------------------------------------------------------------------
bool fnl_mma::WasNotJustFnl(uint64_t Block)
{
  uint64_t prev = Block - 1;
  int      set  = static_cast<int>(prev & (SIZEWAYFILTERFNL - 1));
  uint64_t tag  = (prev / SIZEWAYFILTERFNL) & ((1 << 15) - 1);
  for (int i = 0; i < NBWAYFILTERFNL; i++)
    if (JUSTNLPREFETCH[set][i] == tag)
      return false;
  return true;
}

// ---------------------------------------------------------------------------
// IsInIShadow — shadow I-cache lookup/insert
// ---------------------------------------------------------------------------
bool fnl_mma::IsInIShadow(uint64_t Block, bool Insert)
{
  int      Hit = -1;
  int      set = static_cast<int>(Block & 63);
  uint64_t tag = (Block >> 6) & ((1 << 15) - 1);
  for (int i = 0; i < NBWAYISHADOW; i++)
    if (tag == ShadowICache[set][i]) {
      Hit = i;
      break;
    }
  if (Insert) {
    int Max = (Hit != -1) ? Hit : NBWAYISHADOW - 1;
    for (int i = Max; i > 0; i--)
      ShadowICache[set][i] = ShadowICache[set][i - 1];
    ShadowICache[set][0] = tag;
  }
  return (Hit != -1);
}

// ---------------------------------------------------------------------------
// PredictMiss::init
// ---------------------------------------------------------------------------
void fnl_mma::PredictMiss::init(uint64_t* ntag, uint64_t* nblock,
                                 int* nbmiss, int8_t* u, int x)
{
  Ntag   = ntag;
  NBlock = nblock;
  NbMiss = nbmiss;
  U      = u;
  for (int i = 0; i < NBWAYPRED * SIZEWAYNEXTMISS; i++) {
    U[i]      = 0;
    Ntag[i]   = 0;
    NBlock[i] = 0;
    NbMiss[i] = 0;
  }
  distahead = x;
}

// ---------------------------------------------------------------------------
// PredictMiss::AheadPredict
// ---------------------------------------------------------------------------
uint64_t fnl_mma::PredictMiss::AheadPredict(uint64_t Addr)
{
  // Note: PrefetchCandidate is a member of the outer fnl_mma, but
  // PredictMiss stores a pointer to fnl_mma::PrefetchCandidate via
  // the trampoline. For simplicity we declare a local and mirror it
  // back via the outer class through the global instance pointer.
  fnl_mma::s_instance->PrefetchCandidate = 0;

  int index[NBWAYPRED];
  int A = static_cast<int>(Addr & (SIZEWAYNEXTMISS - 1));
  int B = static_cast<int>((Addr >> LOGWAYNEXTMISS) & (SIZEWAYNEXTMISS - 1));
  for (int i = 0; i < NBWAYPRED; i++) {
    index[i] = (A ^ B) + (i << LOGWAYNEXTMISS);
    A = (A >> 7) + ((A & 127) << (LOGWAYNEXTMISS - 7));
  }

  uint64_t tag  = (Addr >> LOGWAYNEXTMISS) & ((1 << LOGTAGNEXTMISS) - 1);
  int      NHIT = -1;
  for (int i = 0; i < NBWAYPRED; i++) {
    if (Ntag[index[i]] == tag) {
      NHIT = i;
      fnl_mma::s_instance->PrefetchCandidate = NBlock[index[NHIT]];
      break;
    }
  }
  if (NHIT == -1)
    return 0;

  uint64_t X = 0;
  if (U[index[NHIT]] > 0)
    X = NBlock[index[NHIT]];
  return X;
}

// ---------------------------------------------------------------------------
// PredictMiss::LinkAhead
// ---------------------------------------------------------------------------
void fnl_mma::PredictMiss::LinkAhead(uint64_t Block, uint64_t PrevAddr,
                                      uint8_t Hit)
{
  if (Hit)
    return;

  int index[NBWAYPRED];
  int A = static_cast<int>(PrevAddr & (SIZEWAYNEXTMISS - 1));
  int B = static_cast<int>((PrevAddr >> LOGWAYNEXTMISS) & (SIZEWAYNEXTMISS - 1));
  for (int i = 0; i < NBWAYPRED; i++) {
    index[i] = (A ^ B) + (i << LOGWAYNEXTMISS);
    A = (A >> 7) + ((A & 127) << (LOGWAYNEXTMISS - 7));
  }

  uint64_t tag  = (PrevAddr >> LOGWAYNEXTMISS) & ((1 << LOGTAGNEXTMISS) - 1);
  int      NHIT = -1;
  for (int i = 0; i < NBWAYPRED; i++) {
    if (Ntag[index[i]] == tag) {
      NbMiss[index[i]]++;
      NHIT = i;
      if (NBlock[index[i]] == Block) {
        U[index[i]] = 1;
      } else {
        NBlock[index[i]] = Block;
        U[index[i]]      = 0;
      }
      return;
    }
  }

  // Try to allocate a new entry
  int X = static_cast<int>(myrandom_fn() % NBWAYPRED);
  for (int i = 0; i < NBWAYPRED; i++) {
    if (U[index[X]] == 0) {
      NHIT = X;
      break;
    }
    X = (X + 1) % NBWAYPRED;
  }
  if (NHIT == -1) {
    if ((myrandom_fn() & 15) == 0)
      U[index[X]] = 0;
  }
  if (NHIT != -1) {
    NBlock[index[NHIT]] = Block;
    Ntag[index[NHIT]]   = tag;
    NbMiss[index[NHIT]] = 0;
    U[index[X]]         = 0;
  }
}

// ---------------------------------------------------------------------------
// prefetcher_initialize
// ---------------------------------------------------------------------------
void fnl_mma::prefetcher_initialize()
{
  std::cout << "CPU 0 L1I FNL-MMA prefetcher" << std::endl;

  // Register the static trampoline so PredictMiss can call back
  s_instance = this;
  AHEAD.set_rng(s_myrandom_trampoline);
  AHEADphist.set_rng(s_myrandom_trampoline);

  AHEAD.init(GNtag, GNblock, GNbMiss, GU, DISTAHEAD);
  // AHEADphist shares the same backing arrays (they are zeroed by AHEAD.init)
  AHEADphist.Ntag      = GNtag;
  AHEADphist.NBlock    = GNblock;
  AHEADphist.NbMiss    = GNbMiss;
  AHEADphist.U         = GU;
  AHEADphist.distahead = DISTAHEAD;
}

// ---------------------------------------------------------------------------
// prefetcher_branch_operate
// ---------------------------------------------------------------------------
void fnl_mma::prefetcher_branch_operate(champsim::address ip,
                                         uint8_t branch_type,
                                         champsim::address branch_target)
{
  // Original implementation was empty
  (void)ip;
  (void)branch_type;
  (void)branch_target;
}

// ---------------------------------------------------------------------------
// prefetcher_cache_operate
// ---------------------------------------------------------------------------
uint32_t fnl_mma::prefetcher_cache_operate(champsim::address addr,
                                            champsim::address ip,
                                            uint8_t cache_hit,
                                            bool useful_prefetch,
                                            access_type type,
                                            uint32_t metadata_in)
{
  uint64_t v_addr = addr.to<uint64_t>();
  uint64_t Block  = v_addr >> LOG2_BLOCK_SIZE;
  int      index  = static_cast<int>(Block & (FNL_NBENTRIES - 1));
  bool     ShadowMiss = (!IsInIShadow(Block, true));
  uint64_t AheadPredictedBlock = 0;

  if (ShadowMiss) {
    // ---- FNL prefetcher ----
    int previndex = (index - 1) & (FNL_NBENTRIES - 1);
    Touched[index] = 1;
    if (Touched[previndex]) {
      if ((cache_hit == 0) || (WorthPF[previndex]))
        WorthPF[previndex] = 3;
    }

    for (int i = ptReset; i < ptReset + (FNL_NBENTRIES / PERIODRESET); i++) {
      if (Touched[i])
        if (WorthPF[i] > 0)
          WorthPF[i]--;
      Touched[i] = 0;
    }
    ptReset += (FNL_NBENTRIES / PERIODRESET);
    ptReset  &= (FNL_NBENTRIES - 1);

    // Next-line prefetch
    if (WorthPF[index] > 0) {
      bool NotJustAHEAD = true;
      for (int i = MMA_FILT_SIZE - 1; i >= 0; i--)
        if (PREVPRED[i] == Block) {
          NotJustAHEAD = false;
          break;
        }
      if (NotJustAHEAD) {
        for (int i = 1; i <= MAXFNL; i++) {
          uint64_t pf_Block = Block + i;
          if ((WasNotJustFnl(Block)) || (i == MAXFNL))
            prefetch_line(champsim::address{pf_Block << LOG2_BLOCK_SIZE}, true, 0);
          if (WorthPF[(index + i) & (FNL_NBENTRIES - 1)] == 0)
            break;
        }
      }
      // FITERFNLON defined in original
      JustFnl(Block);
    }

    // ---- AHEAD predictor ----
    AheadPredictedBlock =
      AHEADphist.AheadPredict((v_addr >> 2) ^ (PREVADDR[NSHIFT - 1] << 1));
    if (AheadPredictedBlock != 0) {
      bool NotJustMMA = true;
      for (int i = MMA_FILT_SIZE - 1; i >= 0; i--) {
        if (PREVPRED[i] == AheadPredictedBlock) {
          NotJustMMA = false;
          break;
        }
      }
      if (!NotJustMMA)
        AheadPredictedBlock = 0;
      if (NotJustMMA) {
        int idx = static_cast<int>(AheadPredictedBlock & (FNL_NBENTRIES - 1));
        prefetch_line(champsim::address{AheadPredictedBlock << LOG2_BLOCK_SIZE}, true, 0);
        if (WorthPF[idx] > 0) {
          for (int i = 1; i <= MAXFNL; i++) {
            uint64_t pf_Block = AheadPredictedBlock + i;
            if ((WasNotJustFnl(AheadPredictedBlock)) || (i == MAXFNL))
              prefetch_line(champsim::address{pf_Block << LOG2_BLOCK_SIZE}, true, 0);
            if (WorthPF[(idx + i) & (FNL_NBENTRIES - 1)] == 0)
              break;
          }
          JustFnl(AheadPredictedBlock);
        }
      }
    }

    // First access resulted in a miss
    if (AheadPredictedBlock == 0) {
      AheadPredictedBlock = AHEAD.AheadPredict(v_addr >> 2);
      if (AheadPredictedBlock != 0) {
        bool NotJustMMA = true;
        for (int i = MMA_FILT_SIZE - 1; i >= 0; i--) {
          if (PREVPRED[i] == AheadPredictedBlock) {
            NotJustMMA = false;
            break;
          }
        }
        if (!NotJustMMA)
          AheadPredictedBlock = 0;
        if (NotJustMMA) {
          int idx = static_cast<int>(AheadPredictedBlock & (FNL_NBENTRIES - 1));
          prefetch_line(champsim::address{AheadPredictedBlock << LOG2_BLOCK_SIZE}, true, 0);
          if (WorthPF[idx] > 0) {
            for (int i = 1; i <= MAXFNL; i++) {
              uint64_t pf_Block = AheadPredictedBlock + i;
              if ((WasNotJustFnl(AheadPredictedBlock)) || (i == MAXFNL))
                prefetch_line(champsim::address{pf_Block << LOG2_BLOCK_SIZE}, true, 0);
              if (WorthPF[(idx + i) & (FNL_NBENTRIES - 1)] == 0)
                break;
            }
            JustFnl(AheadPredictedBlock);
          }
        }
      }
    }

    // Link ahead
    if ((Block != (PREVADDR[0] >> 4) + 1) || (MAXFNL == 0)) {
      AHEAD.LinkAhead(Block, PREVADDR[AHEAD.distahead], cache_hit);
      if ((PREFCAND[AHEAD.distahead] != 0) &&
          (PREFCAND[AHEAD.distahead] != Block))
        AHEADphist.LinkAhead(
          Block,
          PREVADDR[AHEADphist.distahead] ^ (PREVADDR[AHEADphist.distahead + NSHIFT] << 1),
          cache_hit);
    }

    for (int i = DISTAHEADMAX; i > 0; i--)
      PREVADDR[i] = PREVADDR[i - 1];
    PREVADDR[0] = v_addr >> 2;
    for (int i = DISTAHEADMAX; i > 0; i--)
      PREFCAND[i] = PREFCAND[i - 1];
    PREFCAND[0] = PrefetchCandidate;

    if (AheadPredictedBlock != 0) {
      for (int i = MMA_FILT_SIZE - 1; i >= 1; i--)
        PREVPRED[i] = PREVPRED[i - 1];
      PREVPRED[0] = AheadPredictedBlock;
    }
  }

  return metadata_in;
}

// ---------------------------------------------------------------------------
// prefetcher_cache_fill
// ---------------------------------------------------------------------------
uint32_t fnl_mma::prefetcher_cache_fill(champsim::address addr, long set,
                                         long way, uint8_t prefetch,
                                         champsim::address evicted_addr,
                                         uint32_t metadata_in)
{
  return metadata_in;
}

// ---------------------------------------------------------------------------
// prefetcher_cycle_operate
// ---------------------------------------------------------------------------
void fnl_mma::prefetcher_cycle_operate()
{
  // Original implementation was empty
}

// ---------------------------------------------------------------------------
// prefetcher_final_stats
// ---------------------------------------------------------------------------
void fnl_mma::prefetcher_final_stats()
{
  printf("I-Shadow cache %d bytes\n",
         (SIZESHADOWICACHE * (15 + 2)) / 8);
  printf("Touched + WorthPF tables %d bytes \n",
         (FNL_NBENTRIES * 3) / 8);
  printf("MMA filter %d bytes \n",
         (MMA_FILT_SIZE * 58) / 8);
  printf("FNL filter %d bytes \n",
         (SIZEFILTERFNL * (15 + 2)) / 8);
  printf("TOTAL PREFETCHER STORAGE SIZE: %d bytes\n",
         (71 * NBWAYPRED * SIZEWAYNEXTMISS / 8) +
         ((SIZESHADOWICACHE * (15 + 2)) / 8) +
         ((FNL_NBENTRIES * 3) / 8) +
         ((MMA_FILT_SIZE * 58) / 8) +
         ((SIZEFILTERFNL * (15 + 2) / 8)));
  std::cout << "CPU 0 L1I FNL-MMA prefetcher final stats" << std::endl;
}

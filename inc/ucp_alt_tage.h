// Alt-BP for the UCP port: 8KB TAGE-SC-L (class TAGE_PREDICTOR_8KB), taken verbatim
// from the UCP_ISCA24 artifact (inc/alt_tage_8KB.h).  Self-contained (all state in
// members, incl. its own bimodal_miss_hist and the Table-I stop weights in is_h2p()).
// speculative_begin()/speculative_end() checkpoint/restore for alternate-path walks.
// Included ONLY by src/ooo_cpu.cc; the trailing #undefs keep its config macros local.
#ifndef UCP_ALT_TAGE_H
#define UCP_ALT_TAGE_H
#ifndef H2P_T
#define H2P_T 500
#endif
#ifndef _PREDICTOR_8KB_H_
#define _PREDICTOR_8KB_H_

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "instruction.h"

class TAGE_PREDICTOR_8KB
{
  enum prediction_source { ALT_BANK, HIT_BANK, BIMODAL, LOOP, SAT_PRED, NONE };
  uint8_t bimodal_miss_hist = 0;

public:
  prediction_source last_pred_source, spec_last_pred_source;
  int8_t last_pointer_for_pred, spec_last_pointer_for_pred = 0;
  int8_t last_pointer_for_pred_sat = 0;

  bool spec_state = false;
#define BORNTICK 1024
  // for the allocation policy

  // To get the predictor storage budget on stderr  uncomment the next line
#define PRINTSIZE
#include <vector>

#define SC // Enables the statiscal corrctor + 5.7 % MPKI without SC

#define GSC           // global history in SC  + 0.4% //if no GSC, no local, no loop: +3.2 %
#define IMLI          // 0.2 %
#define LOCALH        // + 0.9 % without local
#define LOOPPREDICTOR // loop predictor enable //0.4 % mispred reduction

// The statistical corrector components
// The two BIAS tables in the SC component
// We play with confidence here
#define LOGBIAS 7
  int8_t Bias[(1 << LOGBIAS)];
  int8_t spec_Bias[(1 << LOGBIAS)];

#define INDBIAS (((((PC ^ (PC >> 2)) << 1) ^ (LowConf & (LongestMatchPred != alttaken))) << 1) + pred_inter) & ((1 << LOGBIAS) - 1)
  int8_t BiasSK[(1 << LOGBIAS)];
  int8_t spec_BiasSK[(1 << LOGBIAS)];
#define INDBIASSK (((((PC ^ (PC >> (LOGBIAS - 2))) << 1) ^ (HighConf)) << 1) + pred_inter) & ((1 << LOGBIAS) - 1)

  int8_t BiasBank[(1 << LOGBIAS)];
  int8_t spec_BiasBank[(1 << LOGBIAS)];
#define INDBIASBANK (pred_inter + (((HitBank + 1) / 4) << 4) + (HighConf << 1) + (LowConf << 2) + ((AltBank != 0) << 3)) & ((1 << LOGBIAS) - 1)

  long long IMLIcount; // use to monitor the iteration number
  long long spec_IMLIcount;
#ifdef IMLI

#define LOGINB 7
#define INB 1
  int Im[INB] = {8};
  int spec_Im[INB] = {8};
  int8_t IGEHLA[INB][(1 << LOGINB)] = {{0}};
  int8_t spec_IGEHLA[INB][(1 << LOGINB)] = {{0}};

  int8_t* IGEHL[INB];
  int8_t* spec_IGEHL[INB];

#endif

#define LOGBWNB 7
#define BWNB 2

  int BWm[BWNB] = {16, 8};
  int8_t BWGEHLA[BWNB][(1 << LOGBWNB)] = {{0}};
  int spec_BWm[BWNB] = {16, 8};
  int8_t spec_BWGEHLA[BWNB][(1 << LOGBWNB)] = {{0}};

  int8_t* BWGEHL[BWNB];
  long long BWHIST;

  int8_t* spec_BWGEHL[BWNB];
  long long spec_BWHIST;

  // global branch GEHL

#define LOGGNB 7
#define GNB 2
  long long GHIST;
  int Gm[GNB] = {6, 3};
  int8_t GGEHLA[GNB][(1 << LOGGNB)] = {{0}};
  int8_t* GGEHL[GNB];

  long long spec_GHIST;
  int spec_Gm[GNB] = {6, 3};
  int8_t spec_GGEHLA[GNB][(1 << LOGGNB)] = {{0}};
  int8_t* spec_GGEHL[GNB];

  // large local history

#define LOGLNB 7
#define LNB 2

  int Lm[LNB] = {6, 3};
  int8_t LGEHLA[LNB][(1 << LOGLNB)] = {{0}};
  int8_t* LGEHL[LNB];

  int spec_Lm[LNB] = {6, 3};
  int8_t spec_LGEHLA[LNB][(1 << LOGLNB)] = {{0}};
  int8_t* spec_LGEHL[LNB];

#define LOGLOCAL 6
#define NLOCAL (1 << LOGLOCAL)
#define INDLOCAL ((PC ^ (PC >> 2)) & (NLOCAL - 1))
  long long L_shist[NLOCAL];
  long long spec_L_shist[NLOCAL];

  // update threshold for the statistical corrector
#define VARTHRES
// more than one update threshold
#define WIDTHRES 12
#define WIDTHRESP 8
#ifdef VARTHRES
#define LOGSIZEUP 6 // not worth increasing:  0-> 6 0.05 MPKI
#else
#define LOGSIZEUP 0
#endif
#define LOGSIZEUPS (LOGSIZEUP / 2)
  int updatethreshold;
  int Pupdatethreshold[(1 << LOGSIZEUP)]; // size is fixed by LOGSIZEUP

  int spec_updatethreshold;
  int spec_Pupdatethreshold[(1 << LOGSIZEUP)]; // size is fixed by LOGSIZEUP
#define INDUPD (PC ^ (PC >> 2)) & ((1 << LOGSIZEUP) - 1)
#define INDUPDS ((PC ^ (PC >> 2)) & ((1 << (LOGSIZEUPS)) - 1))

  int8_t WB[(1 << LOGSIZEUPS)];
  int8_t WG[(1 << LOGSIZEUPS)];
  int8_t WL[(1 << LOGSIZEUPS)];
  int8_t WI[(1 << LOGSIZEUPS)];
  int8_t WBW[(1 << LOGSIZEUPS)];

  int8_t spec_WB[(1 << LOGSIZEUPS)];
  int8_t spec_WG[(1 << LOGSIZEUPS)];
  int8_t spec_WL[(1 << LOGSIZEUPS)];
  int8_t spec_WI[(1 << LOGSIZEUPS)];
  int8_t spec_WBW[(1 << LOGSIZEUPS)];

#define EWIDTH 6

  // The two counters used to choose between TAGE ang SC on High Conf TAGE/Low Conf SC
  int8_t FirstH, SecondH;
  int8_t spec_FirstH, spec_SecondH;
#define CONFWIDTH 7 // for the counters in the choser

  int LSUM;
  int spec_LSUM;

  // utility class for index computation
  // this is the cyclic shift register for folding
  // a long global history into a smaller number of bits; see P. Michaud's PPM-like predictor at CBP-1
#define HISTBUFFERLENGTH 4096 // we use a 4K entries history buffer to store the branch history
  class folded_history
  {
  public:
    unsigned comp;
    int CLENGTH;
    int OLENGTH;
    int OUTPOINT;

    folded_history() {}

    void init(int original_length, int compressed_length, int N)
    {
      comp = 0;
      OLENGTH = original_length;
      CLENGTH = compressed_length;
      OUTPOINT = OLENGTH % CLENGTH;
    }

    void update(uint8_t* h, int PT)
    {
      comp = (comp << 1) ^ h[PT & (HISTBUFFERLENGTH - 1)];

      comp ^= h[(PT + OLENGTH) & (HISTBUFFERLENGTH - 1)] << OUTPOINT;
      comp ^= (comp >> CLENGTH);
      comp = (comp) & ((1 << CLENGTH) - 1);
    }
  };

  class bentry // TAGE bimodal table entry
  {
  public:
    int8_t hyst;
    int8_t pred;

    bentry()
    {
      pred = 0;

      hyst = 1;
    }
  };
  class gentry // TAGE global table entry
  {
  public:
    int8_t ctr;
    uint tag;
    int8_t u;

    gentry()
    {
      ctr = 0;
      u = 0;
      tag = 0;
    }
  };

#define POWER
  // use geometric history length
#define NHIST 30 // in practice 15 different lengths
#define BORN 11  // tables below BORN shared NBBANK[0] banks, ..
  int NBBANK[2] = {9, 17};
  int spec_NBBANK[2] = {9, 17};

  bool NOSKIP[NHIST + 1]; // management of partial associativity
  bool spec_NOSKIP[NHIST + 1];

#define BORNINFASSOC 7 // 2 -way assoc for the lengths between the two borns: 0.6 %
#define BORNSUPASSOC 21

#define MINHIST 4
#define MAXHIST 1000

#define LOGG 7 /* logsize of a bank in TAGE tables */

#define TBITS 8
  bool LowConf;
  bool HighConf;
  bool AltConf;
  bool MedConf;
  bool alttaken;  // alternate  TAGEprediction
  bool tage_pred; // TAGE prediction
  bool LongestMatchPred;
  int HitBank; // longest matching bank
  int AltBank; // alternate matching bank
  int Seed;    // for the pseudo-random number generator
  int8_t BIM;
  bool pred_inter;

  bool spec_LowConf;
  bool spec_HighConf;
  bool spec_AltConf;
  bool spec_MedConf;
  bool spec_alttaken;  // alternate  TAGEprediction
  bool spec_tage_pred; // TAGE prediction
  bool spec_LongestMatchPred;
  int spec_HitBank; // longest matching bank
  int spec_AltBank; // alternate matching bank
  int spec_Seed;    // for the pseudo-random number generator
  int8_t spec_BIM;
  bool spec_pred_inter;

#define NNN 1       // number of extra entries allocated on a TAGE misprediction: 0.4 % better if allocation of 2 elements instead of 1
#define HYSTSHIFT 2 // bimodal hysteresis shared by 4 entries
#define LOGB 12     // log of number of entries in bimodal predictor
#define PERCWIDTH 6 // Statistical corrector maximum counter width: 5 bits would  be sufficient

#define PHISTWIDTH 27 // width of the path history used in TAGE
#define UWIDTH 2      // u counter width on TAGE (2 bits- -> 1 bit about 1.5 %)
#define CWIDTH 3      // predictor counter width on the TAGE tagged tables

  // the counter(s) to chose between longest match and alternate prediction on TAGE when weak counters
#define LOGSIZEUSEALT 3
#define ALTWIDTH 5
#define SIZEUSEALT (1 << (LOGSIZEUSEALT))
#define INDUSEALT ((((HitBank - 1) / 8) << 1) + AltConf)
  // #define INDUSEALT 0
  int8_t use_alt_on_na[SIZEUSEALT];
  int TICK; // for the reset of the u counter
  uint8_t ghist[HISTBUFFERLENGTH];
  int ptghist;
  long long phist;                   // path history
  folded_history ch_i[NHIST + 1];    // utility for computing TAGE indices
  folded_history ch_t[2][NHIST + 1]; // utility for computing TAGE tags
  bentry* btable;                    // bimodal TAGE table
  gentry* gtable[NHIST + 1];         // tagged TAGE tables
  int m[NHIST + 1];
  int TB[NHIST + 1];
  int logg[NHIST + 1];
  int GI[NHIST + 1];    // indexes to the different tables are computed only once
  uint GTAG[NHIST + 1]; // tags for the different tables are computed only once
  int BI;               // index of the bimodal table
  bool pred_taken;      // prediction

  int8_t spec_use_alt_on_na[SIZEUSEALT];
  int spec_TICK; // for the reset of the u counter
  uint8_t spec_ghist[HISTBUFFERLENGTH];
  int spec_ptghist;
  long long spec_phist;                   // path history
  folded_history spec_ch_i[NHIST + 1];    // utility for computing TAGE indices
  folded_history spec_ch_t[2][NHIST + 1]; // utility for computing TAGE tags
  bentry* spec_btable;                    // bimodal TAGE table
  gentry* spec_gtable[NHIST + 1];         // tagged TAGE tables
  int spec_m[NHIST + 1];
  int spec_TB[NHIST + 1];
  int spec_logg[NHIST + 1];
  int spec_GI[NHIST + 1];    // indexes to the different tables are computed only once
  uint spec_GTAG[NHIST + 1]; // tags for the different tables are computed only once
  int spec_BI;               // index of the bimodal table
  bool spec_pred_taken;      // prediction

#ifdef LOOPPREDICTOR
  // parameters of the loop predictor
#define LOGL 3
#define WIDTHNBITERLOOP 10 // we predict only loops with less than 1K iterations
#define LOOPTAG 10         // tag width in the loop predictor

  class lentry // loop predictor entry
  {
  public:
    uint16_t NbIter;      // 10 bits
    uint8_t confid;       // 4bits
    uint16_t CurrentIter; // 10 bits

    uint16_t TAG; // 10 bits
    uint8_t age;  // 4 bits
    bool dir;     // 1 bit

    // 39 bits per entry
    lentry()
    {
      confid = 0;
      CurrentIter = 0;
      NbIter = 0;
      TAG = 0;
      age = 0;
      dir = false;
    }
  };

  lentry* ltable; // loop predictor table
  bool predloop;  // loop predictor prediction
  int LIB;
  int LI;
  int LHIT;        // hitting way in the loop predictor
  int LTAG;        // tag on the loop predictor
  bool LVALID;     // validity of the loop predictor prediction
  int8_t WITHLOOP; // counter to monitor whether or not loop prediction is beneficial

  lentry* spec_ltable; // loop predictor table
  bool spec_predloop;  // loop predictor prediction
  int spec_LIB;
  int spec_LI;
  int spec_LHIT;        // hitting way in the loop predictor
  int spec_LTAG;        // tag on the loop predictor
  bool spec_LVALID;     // validity of the loop predictor prediction
  int8_t spec_WITHLOOP; // counter to monitor whether or not loop prediction is beneficial

#endif

  int predictorsize()
  {
    int STORAGESIZE = 0;
    int inter = 0;

    STORAGESIZE += NBBANK[1] * (1 << (logg[BORN])) * (CWIDTH + UWIDTH + TB[BORN]);
    STORAGESIZE += NBBANK[0] * (1 << (logg[1])) * (CWIDTH + UWIDTH + TB[1]);
    STORAGESIZE += (SIZEUSEALT)*ALTWIDTH;
    STORAGESIZE += (1 << LOGB) + (1 << (LOGB - HYSTSHIFT));
    STORAGESIZE += m[NHIST];
    STORAGESIZE += PHISTWIDTH;
    STORAGESIZE += 10; // the TICK counter

    fprintf(stderr, " (TAGE %d) ", STORAGESIZE);
#ifdef SC
#ifdef LOOPPREDICTOR

    inter = (1 << LOGL) * (2 * WIDTHNBITERLOOP + LOOPTAG + 4 + 4 + 1);
    fprintf(stderr, " (LOOP %d) ", inter);
    STORAGESIZE += inter;

#endif

    inter = WIDTHRESP * (1 << LOGSIZEUP); // the update threshold counters
    inter += WIDTHRES;
    inter += EWIDTH * (1 << LOGSIZEUPS); // the extra weight of the partial sums
    inter += (PERCWIDTH)*3 * (1 << LOGBIAS);
#ifdef GSC
    inter += GNB * (1 << LOGGNB) * (PERCWIDTH);
    inter += Gm[0];                      // the global  history
    inter += EWIDTH * (1 << LOGSIZEUPS); // the extra weight of the partial sums
    inter += BWNB * (1 << LOGBWNB) * PERCWIDTH;
    inter += EWIDTH * (1 << LOGSIZEUPS); // the extra weight of the partial sums
    inter += BWm[0];
#endif
#ifdef LOCALH
    inter += LNB * (1 << LOGLNB) * (PERCWIDTH);
    inter += NLOCAL * Lm[0];             // the local history
    inter += EWIDTH * (1 << LOGSIZEUPS); // the extra weight of the partial sums
#endif

#ifdef IMLI
    inter += (1 << LOGINB) * PERCWIDTH;
    inter += Im[0];
    inter += EWIDTH * (1 << LOGSIZEUPS); // the extra weight of the partial sums

#endif
    inter += 2 * CONFWIDTH; // the 2 counters in the choser
    STORAGESIZE += inter;

    fprintf(stderr, " (SC %d) ", inter);
#endif
#ifdef PRINTSIZE

    fprintf(stderr, " (TOTAL %d KB) ", STORAGESIZE / 8192);
    fprintf(stdout, " (TOTAL %d KB)\n ", STORAGESIZE / 8192);
#endif
    return (STORAGESIZE);
  }

public:
  TAGE_PREDICTOR_8KB(void)
  {

    reinit();
#ifdef PRINTSIZE
    predictorsize();
#endif
  }

  void reinit()
  {
    std::cout << "Alternate TAGE_SC_L branch predictor! " << std::endl;
    m[1] = MINHIST;
    m[NHIST / 2] = MAXHIST;
    for (int i = 2; i <= NHIST / 2; i++) {
      m[i] = (int)(((double)MINHIST * pow((double)(MAXHIST) / (double)MINHIST, (double)(i - 1) / (double)(((NHIST / 2) - 1)))) + 0.5);
    }

    for (int i = 1; i <= NHIST; i++) {
      NOSKIP[i] = ((i - 1) & 1) || ((i >= BORNINFASSOC) & (i < BORNSUPASSOC));
    }

    for (int i = NHIST; i > 1; i--) {
      m[i] = m[(i + 1) / 2];
    }
    for (int i = 1; i <= NHIST; i++) {
      TB[i] = TBITS + 4 * (i >= BORN);
      logg[i] = LOGG;
    }

#ifdef LOOPPREDICTOR
    ltable = new lentry[1 << (LOGL)];
    spec_ltable = new lentry[1 << (LOGL)];
#endif

    gtable[1] = new gentry[NBBANK[0] * (1 << LOGG)];
    spec_gtable[1] = new gentry[NBBANK[0] * (1 << LOGG)];
    gtable[BORN] = new gentry[NBBANK[1] * (1 << LOGG)];
    spec_gtable[BORN] = new gentry[NBBANK[1] * (1 << LOGG)];
    for (int i = BORN + 1; i <= NHIST; i++) {
      gtable[i] = gtable[BORN];
      spec_gtable[i] = spec_gtable[BORN];
    }
    for (int i = 2; i <= BORN - 1; i++) {
      gtable[i] = gtable[1];
      spec_gtable[i] = spec_gtable[1];
    }
    btable = new bentry[1 << LOGB];
    spec_btable = new bentry[1 << LOGB];
    for (int i = 1; i <= NHIST; i++) {

      ch_i[i].init(m[i], 17 + (2 * ((i - 1) / 2) % 4), i - 1);
      ch_t[0][i].init(ch_i[i].OLENGTH, 13, i);
      ch_t[1][i].init(ch_i[i].OLENGTH, 11, i + 2);
    }
#ifdef LOOPPREDICTOR
    LVALID = false;
    WITHLOOP = -1;
#endif
    Seed = 0;

    TICK = 0;
    phist = 0;
    Seed = 0;

    for (int i = 0; i < HISTBUFFERLENGTH; i++)
      ghist[0] = 0;
    ptghist = 0;

    updatethreshold = 35 << 3;

    for (int i = 0; i < (1 << LOGSIZEUP); i++)
      Pupdatethreshold[i] = 0;
    for (int i = 0; i < GNB; i++)
      GGEHL[i] = &GGEHLA[i][0];
    for (int i = 0; i < LNB; i++)
      LGEHL[i] = &LGEHLA[i][0];

    for (int i = 0; i < GNB; i++)
      for (int j = 0; j < ((1 << LOGGNB) - 1); j++) {
        if (!(j & 1)) {
          GGEHL[i][j] = -1;
        }
      }
    for (int i = 0; i < LNB; i++)
      for (int j = 0; j < ((1 << LOGLNB) - 1); j++) {
        if (!(j & 1)) {
          LGEHL[i][j] = -1;
        }
      }

    for (int i = 0; i < BWNB; i++)
      BWGEHL[i] = &BWGEHLA[i][0];
    for (int i = 0; i < BWNB; i++)
      for (int j = 0; j < ((1 << LOGBWNB) - 1); j++) {
        if (!(j & 1)) {
          BWGEHL[i][j] = -1;
        }
      }

#ifdef IMLI

    for (int i = 0; i < INB; i++)
      IGEHL[i] = &IGEHLA[i][0];
    for (int i = 0; i < INB; i++)
      for (int j = 0; j < ((1 << LOGINB) - 1); j++) {
        if (!(j & 1)) {
          IGEHL[i][j] = -1;
        }
      }

#endif

    for (int i = 0; i < (1 << LOGB); i++) {
      btable[i].pred = 0;
      btable[i].hyst = 1;
    }

    for (int j = 0; j < (1 << LOGBIAS); j++) {
      switch (j & 3) {
      case 0:
        BiasSK[j] = -8;
        break;
      case 1:
        BiasSK[j] = 7;
        break;
      case 2:
        BiasSK[j] = -32;

        break;
      case 3:
        BiasSK[j] = 31;
        break;
      }
    }
    for (int j = 0; j < (1 << LOGBIAS); j++) {
      switch (j & 3) {
      case 0:
        Bias[j] = -32;

        break;
      case 1:
        Bias[j] = 31;
        break;
      case 2:
        Bias[j] = -1;
        break;
      case 3:
        Bias[j] = 0;
        break;
      }
    }
    for (int j = 0; j < (1 << LOGBIAS); j++) {
      switch (j & 3) {
      case 0:
        BiasBank[j] = -32;

        break;
      case 1:
        BiasBank[j] = 31;
        break;
      case 2:
        BiasBank[j] = -1;
        break;
      case 3:
        BiasBank[j] = 0;
        break;
      }
    }
    for (int i = 0; i < SIZEUSEALT; i++) {
      use_alt_on_na[i] = 0;
    }
    for (int i = 0; i < (1 << LOGSIZEUPS); i++) {
      WB[i] = 4;
      WG[i] = 7;
      WL[i] = 7;
      WI[i] = 7;
      WBW[i] = 7;
    }
    TICK = 0;
    for (int i = 0; i < NLOCAL; i++) {
      L_shist[i] = 0;
    }

    GHIST = 0;
    ptghist = 0;
    phist = 0;
  }
  // index function for the bimodal table

  int bindex(uint64_t PC) { return ((PC ^ (PC >> LOGB)) & ((1 << (LOGB)) - 1)); }

  // the index functions for the tagged tables uses path history as in the OGEHL predictor
  // F serves to mix path history: not very important impact

  int F(long long A, int size, int bank)
  {
    int A1, A2;
    A = A & ((1 << size) - 1);
    A1 = (A & ((1 << logg[bank]) - 1));
    A2 = (A >> logg[bank]);
    if (bank < logg[bank])
      A2 = ((A2 << bank) & ((1 << logg[bank]) - 1)) + (A2 >> (logg[bank] - bank));
    A = A1 ^ A2;
    if (bank < logg[bank])
      A = ((A << bank) & ((1 << logg[bank]) - 1)) + (A >> (logg[bank] - bank));
    return (A);
  }

  // gindex computes a full hash of PC, ghist and phist
  int gindex(unsigned int PC, int bank, long long hist, folded_history* ch_i)
  {
    int index;
    int M = (m[bank] > PHISTWIDTH) ? PHISTWIDTH : m[bank];
    index = PC ^ (PC >> (abs(logg[bank] - bank) + 1)) ^ ch_i[bank].comp ^ F(hist, M, bank);
    return ((index ^ (index >> logg[bank]) ^ (index >> 2 * logg[bank])) & ((1 << (logg[bank])) - 1));
  }

  //  tag computation
  uint16_t gtag(unsigned int PC, int bank, folded_history* ch0, folded_history* ch1)
  {
    int tag = (ch_i[bank - 1].comp << 2) ^ PC ^ (PC >> 2) ^ (ch_i[bank].comp);
    int M = (m[bank] > PHISTWIDTH) ? PHISTWIDTH : m[bank];
    tag = (tag >> 1) ^ ((tag & 1) << 10) ^ F(phist, M, bank);
    tag ^= ch0[bank].comp ^ (ch1[bank].comp << 1);

    return ((tag ^ (tag >> TB[bank])) & ((1 << (TB[bank])) - 1));
  }

  // up-down saturating counter
  void ctrupdate(int8_t& ctr, bool taken, int nbits)
  {
    if (taken) {
      if (ctr < ((1 << (nbits - 1)) - 1))
        ctr++;
    } else {
      if (ctr > -(1 << (nbits - 1)))
        ctr--;
    }
  }

  bool getbim()
  {
    BIM = (btable[BI].pred << 1) + (btable[BI >> HYSTSHIFT].hyst);
    HighConf = (BIM == 0) || (BIM == 3);
    LowConf = !HighConf;
    AltConf = HighConf;
    MedConf = false;
    last_pointer_for_pred = BIM;
    return (btable[BI].pred > 0);
  }

  void baseupdate(bool Taken)
  {
    int inter = BIM;
    if (Taken) {
      if (inter < 3)
        inter += 1;
    } else if (inter > 0)
      inter--;
    btable[BI].pred = inter >> 1;
    btable[BI >> HYSTSHIFT].hyst = (inter & 1);
  };
  // just a simple pseudo random number generator: use available information

  int MYRANDOM()
  {
    Seed++;
    Seed ^= phist;
    Seed = (Seed >> 21) + (Seed << 11);
    Seed ^= ptghist;
    Seed = (Seed >> 10) + (Seed << 22);
    Seed ^= GTAG[BORN + 2];
    return (Seed);
  };
  //  TAGE PREDICTION: same code at fetch or retire time but the index and tags must recomputed
  void Tagepred(uint64_t PC)
  {
    last_pred_source = NONE;
    HitBank = 0;
    AltBank = 0;
    for (int i = 1; i <= NHIST; i += 2) {
      GI[i] = gindex(PC, i, phist, ch_i);
      GTAG[i] = gtag(PC, i, ch_t[0], ch_t[1]);
      GTAG[i + 1] = GTAG[i];
      GI[i + 1] = GI[i] ^ (GTAG[i] & ((1 << LOGG) - 1));
    }

    uint T = (PC ^ (phist & ((1 << m[BORN]) - 1))) % NBBANK[1];
    for (int i = BORN; i <= NHIST; i++)
      if (NOSKIP[i]) {
        GI[i] += (T << LOGG);
        T++;
        T = T % NBBANK[1];
      }
    T = (PC ^ (phist & ((1 << m[1]) - 1))) % NBBANK[0];
    for (int i = 1; i <= BORN - 1; i++)
      if (NOSKIP[i]) {
        GI[i] += (T << LOGG);
        T++;
        T = T % NBBANK[0];
      }

    // just do not forget most address are aligned on 4 bytes
    BI = (PC ^ (PC >> 2)) & ((1 << LOGB) - 1);

    {
      alttaken = getbim();
      tage_pred = alttaken;
      LongestMatchPred = alttaken;
      last_pred_source = BIMODAL;
    }

    // Look for the bank with longest matching history
    for (int i = NHIST; i > 0; i--) {
      if (NOSKIP[i])
        if (gtable[i][GI[i]].tag == GTAG[i]) {
          HitBank = i;
          LongestMatchPred = (gtable[HitBank][GI[HitBank]].ctr >= 0);
          break;
        }
    }

    // Look for the alternate bank
    for (int i = HitBank - 1; i > 0; i--) {
      if (NOSKIP[i])
        if (gtable[i][GI[i]].tag == GTAG[i]) {

          AltBank = i;
          break;
        }
    }
    // computes the prediction and the alternate prediction

    if (HitBank > 0) {
      if (AltBank > 0) {
        alttaken = (gtable[AltBank][GI[AltBank]].ctr >= 0);
        AltConf = (abs(2 * gtable[AltBank][GI[AltBank]].ctr + 1) > 1);
        last_pred_source = ALT_BANK;
        last_pointer_for_pred = gtable[AltBank][GI[AltBank]].ctr;
      }

      else {
        alttaken = getbim();
        last_pred_source = BIMODAL;
      }
      // if the entry is recognized as a newly allocated entry and
      // USE_ALT_ON_NA is positive  use the alternate prediction
      bool Huse_alt_on_na = (use_alt_on_na[INDUSEALT] >= 0);
      if ((!Huse_alt_on_na) || (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) > 1)) {
        tage_pred = LongestMatchPred;
        last_pred_source = HIT_BANK;
        last_pointer_for_pred = gtable[HitBank][GI[HitBank]].ctr;
      } else
        tage_pred = alttaken;
      HighConf = (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) >= (1 << CWIDTH) - 1);
      LowConf = (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) == 1);
      MedConf = (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) == 5);
    }
  }
  int THRES;
  int spec_THRES;
  // compute the prediction
  bool GetPrediction(uint64_t PC)
  {
    // computes the TAGE table addresses and the partial tags
    Tagepred(PC);
    pred_taken = tage_pred;
#ifndef SC
    return (tage_pred);
#endif
#ifdef LOOPPREDICTOR
    predloop = getloop(PC); // loop prediction
    pred_taken = ((WITHLOOP >= 0) && (LVALID)) ? predloop : pred_taken;
    if ((WITHLOOP >= 0) && (LVALID)) {
      last_pred_source = LOOP;
    }
#endif
    pred_inter = pred_taken;
    // Compute the SC prediction
    LSUM = 0;
    // integrate BIAS prediction
    int8_t ctr = Bias[INDBIAS];
    LSUM += (2 * ctr + 1);
    ctr = BiasSK[INDBIASSK];
    LSUM += (2 * ctr + 1);
    ctr = BiasBank[INDBIASBANK];
    LSUM += (2 * ctr + 1);
#ifdef VARTHRES
    LSUM = (1 + (WB[INDUPDS] >= 0)) * LSUM;
#endif
    // integrate the GEHL predictions

#ifdef GSC
    LSUM += Gpredict(PC, GHIST, Gm, GGEHL, GNB, LOGGNB, WG);
    LSUM += Gpredict(PC, BWHIST, BWm, BWGEHL, BWNB, LOGBWNB, WBW);
#endif
#ifdef LOCALH
    LSUM += Gpredict(PC, L_shist[INDLOCAL], Lm, LGEHL, LNB, LOGLNB, WL);
#endif
#ifdef IMLI
    LSUM += Gpredict(PC, IMLIcount, Im, IGEHL, INB, LOGINB, WI);

#endif
    bool SCPRED = (LSUM >= 0);
    THRES = Pupdatethreshold[INDUPD] + (updatethreshold >> 3);

#ifdef VARTHRES
    +6
        * ((WB[INDUPDS] >= 0)
#ifdef LOCALH
           + (WL[INDUPDS] >= 0)
#endif
#ifdef GSC
           + (WG[INDUPDS] >= 0) + (WBW[INDUPDS] >= 0)
#endif
#ifdef IMLI
           + (WI[INDUPDS] >= 0)
#endif
        )
#endif
        ;

    auto prev_pred_src = last_pred_source;
    auto prev_pointer = last_pointer_for_pred;
    bool use_prev_src = false;
    // Minimal benefit in trying to exploit high confidence on TAGE
    if (pred_inter != SCPRED) {
      // Choser uses TAGE confidence and |LSUM|
      pred_taken = SCPRED;
      last_pred_source = SAT_PRED;
      last_pointer_for_pred_sat = abs(LSUM);
      if (HighConf) {
        if ((abs(LSUM) < THRES / 4)) {
          pred_taken = pred_inter;
        }

        else if ((abs(LSUM) < THRES / 2)) {
          pred_taken = (SecondH < 0) ? SCPRED : pred_inter;
          if ((SecondH >= 0)) {
            use_prev_src = true;
          }
        }
      }

      if (MedConf)
        if ((abs(LSUM) < THRES / 4)) {
          pred_taken = (FirstH < 0) ? SCPRED : pred_inter;
          if ((FirstH >= 0)) {
            use_prev_src = true;
          }
        }
    }

    if (use_prev_src) {
      last_pred_source = prev_pred_src;
      last_pointer_for_pred = prev_pointer;
    }

    return pred_taken;
  }

  void HistoryUpdate(uint64_t PC, uint8_t opType, bool taken, uint64_t target, long long& X, int& Y, folded_history* H, folded_history* G, folded_history* J)
  {

    int brtype = 0;

    switch (opType) {
    case BRANCH_INDIRECT:
    case BRANCH_INDIRECT_CALL:
    case BRANCH_RETURN:
    case BRANCH_OTHER:
      brtype = 2;
      break;
    case BRANCH_DIRECT_JUMP:
    case BRANCH_CONDITIONAL:
    case BRANCH_DIRECT_CALL:
      brtype = 0;
      break;
    default:
      exit(1);
    }
    switch (opType) {
    case BRANCH_CONDITIONAL:
    case BRANCH_OTHER:
      brtype += 1;
      break;
    }

    // special treatment for indirect  branchs;
    int maxt = 2;
    if (brtype & 1)
      maxt = 2;
    else if (brtype & 2)
      maxt = 3;
#ifdef IMLI
    if (brtype & 1) {
      if (target < PC)

      {
        // This branch corresponds to a loop
        if (!taken) {
          // exit of the "loop"
          IMLIcount = 0;
        }
        if (taken) {

          if (IMLIcount < ((1 << Im[0]) - 1))
            IMLIcount++;
        }
      }
    }
#endif

    if (brtype & 1) {
      BWHIST = (BWHIST << 1) + ((target < PC) & taken);
      GHIST = (GHIST << 1) + (taken);
      L_shist[INDLOCAL] = (L_shist[INDLOCAL] << 1) + taken;
    }

    int T = ((PC ^ (PC >> 2))) ^ taken;
    int PATH = PC ^ (PC >> 2) ^ (PC >> 4);
    if ((brtype == 3) & taken) {
      T = (T ^ (target >> 2));
      PATH = PATH ^ (target >> 2) ^ (target >> 4);
    }

    for (int t = 0; t < maxt; t++) {
      bool DIR = (T & 1);
      T >>= 1;
      int PATHBIT = (PATH & 127);
      PATH >>= 1;
      // update  history
      Y--;
      ghist[Y & (HISTBUFFERLENGTH - 1)] = DIR;
      X = (X << 1) ^ PATHBIT;
      for (int i = 1; i <= NHIST; i++) {

        H[i].update(ghist, Y);
        G[i].update(ghist, Y);
        J[i].update(ghist, Y);
      }
    }

    // END UPDATE  HISTORIES
  }

  // TAGE_PREDICTOR_8KB UPDATE

  void UpdatePredictor(uint64_t PC, uint8_t opType, bool resolveDir, uint64_t branchTarget, bool br_miss)
  {

    // cout << PC << " " << int(opType) << " " << resolveDir << " " << branchTarget << endl;

    if (!spec_state) {
      if (last_pred_source == BIMODAL) {
        bimodal_miss_hist = (bimodal_miss_hist << 1);
        if (br_miss) {
          bimodal_miss_hist++;
        }
      }
    }

#ifdef SC
#ifdef LOOPPREDICTOR
    if (LVALID) {
      if (pred_taken != predloop)
        ctrupdate(WITHLOOP, (predloop == resolveDir), 7);
    }
    loopupdate(PC, resolveDir, (pred_taken != resolveDir));
#endif
    bool SCPRED = (LSUM >= 0);
    if (pred_inter != SCPRED) {
      if ((abs(LSUM) < THRES))
        if ((HighConf)) {

          if ((abs(LSUM) < THRES / 2))
            if ((abs(LSUM) >= THRES / 4))
              ctrupdate(SecondH, (pred_inter == resolveDir), CONFWIDTH);
        }
      if ((MedConf))
        if ((abs(LSUM) < THRES / 4)) {

          ctrupdate(FirstH, (pred_inter == resolveDir), 7);
        }
    }

    if ((SCPRED != resolveDir) || ((abs(LSUM) < THRES))) {
      {
        if (SCPRED != resolveDir) {
          Pupdatethreshold[INDUPD] += 1;
          updatethreshold += 1;
        }

        else {
          Pupdatethreshold[INDUPD] -= 1;
          updatethreshold -= 1;
        }

        if (Pupdatethreshold[INDUPD] >= (1 << (WIDTHRESP - 1)))
          Pupdatethreshold[INDUPD] = (1 << (WIDTHRESP - 1)) - 1;
        // Pupdatethreshold[INDUPD] could be negative
        if (Pupdatethreshold[INDUPD] < -(1 << (WIDTHRESP - 1)))
          Pupdatethreshold[INDUPD] = -(1 << (WIDTHRESP - 1));
        if (updatethreshold >= (1 << (WIDTHRES - 1)))
          updatethreshold = (1 << (WIDTHRES - 1)) - 1;
        // updatethreshold could be negative
        if (updatethreshold < -(1 << (WIDTHRES - 1)))
          updatethreshold = -(1 << (WIDTHRES - 1));
      }
#ifdef VARTHRES
      {
        int XSUM = LSUM - ((WB[INDUPDS] >= 0) * ((2 * Bias[INDBIAS] + 1) + (2 * BiasSK[INDBIASSK] + 1) + (2 * BiasBank[INDBIASBANK] + 1)));
        if ((XSUM + ((2 * Bias[INDBIAS] + 1) + (2 * BiasSK[INDBIASSK] + 1) + (2 * BiasBank[INDBIASBANK] + 1)) >= 0) != (XSUM >= 0))
          ctrupdate(WB[INDUPDS], (((2 * Bias[INDBIAS] + 1) + (2 * BiasSK[INDBIASSK] + 1) + (2 * BiasBank[INDBIASBANK] + 1) >= 0) == resolveDir), EWIDTH);
      }
#endif
      ctrupdate(Bias[INDBIAS], resolveDir, PERCWIDTH);
      ctrupdate(BiasSK[INDBIASSK], resolveDir, PERCWIDTH);
      ctrupdate(BiasBank[INDBIASBANK], resolveDir, PERCWIDTH);
      Gupdate(PC, resolveDir, GHIST, Gm, GGEHL, GNB, LOGGNB, WG);
      Gupdate(PC, resolveDir, BWHIST, BWm, BWGEHL, BWNB, LOGBWNB, WBW);
#ifdef LOCALH
      Gupdate(PC, resolveDir, L_shist[INDLOCAL], Lm, LGEHL, LNB, LOGLNB, WL);
#endif
#ifdef IMLI

      Gupdate(PC, resolveDir, IMLIcount, Im, IGEHL, INB, LOGINB, WI);
#endif
    }
#endif

    // TAGE UPDATE
    bool ALLOC = ((tage_pred != resolveDir) & (HitBank < NHIST));
    if (pred_taken == resolveDir)
      if ((MYRANDOM() & 31) != 0)
        ALLOC = false;
    // do not allocate too often if the overall prediction is correct
    if (HitBank > 0) {
      // Manage the selection between longest matching and alternate matching
      // for "pseudo"-newly allocated longest matching entry
      // this is extremely important for TAGE only (0.166 MPKI), not that important when the overall predictor is implemented (0.006 MPKI)
      bool PseudoNewAlloc = (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) <= 1);
      // an entry is considered as newly allocated if its prediction counter is weak
      if (PseudoNewAlloc) {
        if (LongestMatchPred == resolveDir)
          ALLOC = false;
        // if it was delivering the correct prediction, no need to allocate a new entry
        // even if the overall prediction was false
        if (LongestMatchPred != alttaken) {
          ctrupdate(use_alt_on_na[INDUSEALT], (alttaken == resolveDir), ALTWIDTH);
        }
      }
    }

    if (ALLOC) {
      int T = NNN;

      int A = 1;
      if ((MYRANDOM() & 127) < 32)
        A = 2;
      int Penalty = 0;
      int TruePen = 0;
      int NA = 0;
      int DEP = ((((HitBank - 1 + 2 * A) & 0xffe)) ^ (MYRANDOM() & 1));

      for (int I = DEP; I < NHIST; I += 2) {

        int i = I + 1;
        bool Done = false;
        if (NOSKIP[i]) {
          // std::cout << "PC " << PC << " entring the loop u " << int(gtable[i][GI[i]].u) << std::endl;
          if ((gtable[i][GI[i]].u == 0)) {
            {

#define DIPINSP
#ifdef DIPINSP

              gtable[i][GI[i]].u = ((MYRANDOM() & 31) == 0);
              // std::cout << "PC " << PC << " updated if " << int(gtable[i][GI[i]].u) << std::endl;
              //  protect randomly from fast replacement
#endif
              gtable[i][GI[i]].tag = GTAG[i];
              gtable[i][GI[i]].ctr = (resolveDir) ? 0 : -1;
              NA++;
              if (T <= 0) {
                break;
              }
              I += 2;
              T -= 1;
            }
          } else {
#ifdef DIPINSP
            if ((gtable[i][GI[i]].u == 1) & (abs(2 * gtable[i][GI[i]].ctr + 1) == 1)) {
              if ((MYRANDOM() & 7) == 0) {
                gtable[i][GI[i]].u = 0;
                // std::cout << "PC " << PC << " saved else " << int(gtable[i][GI[i]].u) << std::endl;
              }
            } else
#endif
              TruePen++;
            Penalty++;
          }
          // std::cout << "PC " << PC << " just before !Done " << int(gtable[i][GI[i]].u) << " i " << i << std::endl;
          if (!Done) {
            i = (I ^ 1) + 1;
            // std::cout << "PC " << PC << " !Done "
            //           << " i " << i << std::endl;
            if (NOSKIP[i] && (i < (NHIST + 1))) {
              // std::cout << "PC at segfault " << PC << std::endl;
              if ((int(gtable[i][GI[i]].u) == 0)) {
#ifdef DIPINSP
                gtable[i][GI[i]].u = ((MYRANDOM() & 31) == 0);
#endif
                gtable[i][GI[i]].tag = GTAG[i];
                gtable[i][GI[i]].ctr = (resolveDir) ? 0 : -1;
                NA++;
                if (T <= 0) {
                  break;
                }
                I += 2;
                T -= 1;
              }

              else {
#ifdef DIPINSP
                if ((gtable[i][GI[i]].u == 1) & (abs(2 * gtable[i][GI[i]].ctr + 1) == 1)) {
                  if ((MYRANDOM() & 7) == 0)
                    gtable[i][GI[i]].u = 0;
                } else
#endif
                  TruePen++;
                Penalty++;
              }
            }
          }
        }
      }

      TICK += (TruePen + Penalty - 5 * NA);
      // just the best formula for the Championship:
      // In practice when one out of two entries are useful
      if (TICK < 0)
        TICK = 0;
      if (TICK >= BORNTICK) {

        for (int i = 1; i <= BORN; i += BORN - 1)
          for (int j = 0; j <= NBBANK[i / BORN] * (1 << logg[i]) - 1; j++) {

            //                gtable[i][j].u >>= 1;
            /*this is not realistic: in a real processor:    gtable[i][j].u >>= 1;  */
            if (gtable[i][j].u > 0)
              gtable[i][j].u--;
          }
        TICK = 0;
      }
    }

    // update predictions
    if (HitBank > 0) {
      if (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) == 1)
        if (LongestMatchPred != resolveDir) { // acts as a protection
          if (AltBank > 0) {
            if (abs(2 * gtable[AltBank][GI[AltBank]].ctr + 1) == 1)
              gtable[AltBank][GI[AltBank]].u = 0;
            // just mute from protected to unprotected
            ctrupdate(gtable[AltBank][GI[AltBank]].ctr, resolveDir, CWIDTH);
            if (abs(2 * gtable[AltBank][GI[AltBank]].ctr + 1) == 1)
              gtable[AltBank][GI[AltBank]].u = 0;
          }
          if (AltBank == 0)
            baseupdate(resolveDir);
        }
      if (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) == 1)
        gtable[HitBank][GI[HitBank]].u = 0;
      // just mute from protected to unprotected
      ctrupdate(gtable[HitBank][GI[HitBank]].ctr, resolveDir, CWIDTH);
      // sign changes: no way it can have been useful
      if (abs(2 * gtable[HitBank][GI[HitBank]].ctr + 1) == 1)
        gtable[HitBank][GI[HitBank]].u = 0;
      if (alttaken == resolveDir)
        if (AltBank > 0)
          if (abs(2 * gtable[AltBank][GI[AltBank]].ctr + 1) == 7)
            if (gtable[HitBank][GI[HitBank]].u == 1) {
              if (LongestMatchPred == resolveDir) {
                gtable[HitBank][GI[HitBank]].u = 0;
              }
            }

    } else
      baseupdate(resolveDir);
    if (LongestMatchPred != alttaken)
      if (LongestMatchPred == resolveDir) {

        if (gtable[HitBank][GI[HitBank]].u < (1 << UWIDTH) - 1)
          gtable[HitBank][GI[HitBank]].u++;
      }
    // END TAGE UPDATE

    HistoryUpdate(PC, opType, resolveDir, branchTarget, phist, ptghist, ch_i, ch_t[0], ch_t[1]);
    // END TAGE_PREDICTOR_8KB UPDATE
  }

#define GINDEX                                                                                                                                           \
  (((long long)PC) ^ bhist ^ (bhist >> (8 - i)) ^ (bhist >> (16 - 2 * i)) ^ (bhist >> (24 - 3 * i)) ^ (bhist >> (32 - 3 * i)) ^ (bhist >> (40 - 4 * i))) \
      & ((1 << logs) - 1)
  int Gpredict(uint64_t PC, long long BHIST, int* length, int8_t** tab, int NBR, int logs, int8_t* W)
  {
    int PERCSUM = 0;
    for (int i = 0; i < NBR; i++) {
      long long bhist = BHIST & ((long long)((1 << length[i]) - 1));
      long long index = GINDEX;
      int8_t ctr = tab[i][index];
      PERCSUM += (2 * ctr + 1);
    }
#ifdef VARTHRES
    PERCSUM = (1 + (W[INDUPDS] >= 0)) * PERCSUM;
#endif
    return ((PERCSUM));
  }
  void Gupdate(uint64_t PC, bool taken, long long BHIST, int* length, int8_t** tab, int NBR, int logs, int8_t* W)
  {

    int PERCSUM = 0;
    for (int i = 0; i < NBR; i++) {
      long long bhist = BHIST & ((long long)((1 << length[i]) - 1));
      long long index = GINDEX;
      PERCSUM += (2 * tab[i][index] + 1);
      ctrupdate(tab[i][index], taken, PERCWIDTH);
    }
#ifdef VARTHRES
    {
      int XSUM = LSUM - ((W[INDUPDS] >= 0)) * PERCSUM;
      if ((XSUM + PERCSUM >= 0) != (XSUM >= 0))
        ctrupdate(W[INDUPDS], ((PERCSUM >= 0) == taken), EWIDTH);
    }
#endif
  }

  void TrackOtherInst(uint64_t PC, uint8_t opType, bool taken, uint64_t branchTarget)
  {

    HistoryUpdate(PC, opType, taken, branchTarget, phist, ptghist, ch_i, ch_t[0], ch_t[1]);
  }
#ifdef LOOPPREDICTOR
  int lindex(uint64_t PC) { return (((PC ^ (PC >> 2)) & ((1 << (LOGL - 2)) - 1)) << 2); }

  // loop prediction: only used if high confidence
  // skewed associative 4-way
  // At fetch time: speculative
#define CONFLOOP 15
  bool getloop(uint64_t PC)
  {
    LHIT = -1;
    LI = lindex(PC);
    LIB = ((PC >> (LOGL - 2)) & ((1 << (LOGL - 2)) - 1));
    LTAG = (PC >> (LOGL - 2)) & ((1 << 2 * LOOPTAG) - 1);
    LTAG ^= (LTAG >> LOOPTAG);
    LTAG = (LTAG & ((1 << LOOPTAG) - 1));
    for (int i = 0; i < 4; i++) {
      int index = (LI ^ ((LIB >> i) << 2)) + i;
      if (ltable[index].TAG == LTAG) {
        LHIT = i;
        LVALID = ((ltable[index].confid == CONFLOOP) || (ltable[index].confid * ltable[index].NbIter > 128));
        if (ltable[index].CurrentIter + 1 == ltable[index].NbIter)
          return (!(ltable[index].dir));
        return ((ltable[index].dir));
      }
    }

    LVALID = false;
    return (false);
  }

  void loopupdate(uint64_t PC, bool Taken, bool ALLOC)
  {
    if (LHIT >= 0) {
      int index = (LI ^ ((LIB >> LHIT) << 2)) + LHIT;
      // already a hit
      if (LVALID) {
        if (Taken != predloop) {
          // free the entry
          ltable[index].NbIter = 0;
          ltable[index].age = 0;
          ltable[index].confid = 0;
          ltable[index].CurrentIter = 0;
          return;
        } else if ((predloop != tage_pred) || ((MYRANDOM() & 7) == 0))
          if (ltable[index].age < CONFLOOP)
            ltable[index].age++;
      }

      ltable[index].CurrentIter++;
      ltable[index].CurrentIter &= ((1 << WIDTHNBITERLOOP) - 1);
      // loop with more than 2** WIDTHNBITERLOOP iterations are not treated correctly; but who cares :-)
      if (ltable[index].CurrentIter > ltable[index].NbIter) {
        ltable[index].confid = 0;
        ltable[index].NbIter = 0;
        // treat like the 1st encounter of the loop
      }
      if (Taken != ltable[index].dir) {
        if (ltable[index].CurrentIter == ltable[index].NbIter) {
          if (ltable[index].confid < CONFLOOP)
            ltable[index].confid++;
          if (ltable[index].NbIter < 3)
          // just do not predict when the loop count is 1 or 2
          {
            // free the entry
            ltable[index].dir = Taken;
            ltable[index].NbIter = 0;
            ltable[index].age = 0;
            ltable[index].confid = 0;
          }
        } else {
          if (ltable[index].NbIter == 0) {
            // first complete nest;
            ltable[index].confid = 0;
            ltable[index].NbIter = ltable[index].CurrentIter;
          } else {
            // not the same number of iterations as last time: free the entry
            ltable[index].NbIter = 0;
            ltable[index].confid = 0;
          }
        }
        ltable[index].CurrentIter = 0;
      }

    } else if (ALLOC) {
      uint64_t X = MYRANDOM() & 3;
      if ((MYRANDOM() & 3) == 0)
        for (int i = 0; i < 4; i++) {
          int LHIT = (X + i) & 3;
          int index = (LI ^ ((LIB >> LHIT) << 2)) + LHIT;
          if (ltable[index].age == 0) {
            ltable[index].dir = !Taken;
            // most of mispredictions are on last iterations
            ltable[index].TAG = LTAG;
            ltable[index].NbIter = 0;
            ltable[index].age = 7;
            ltable[index].confid = 0;
            ltable[index].CurrentIter = 0;
            break;
          } else
            ltable[index].age--;
          break;
        }
    }
  }
#endif

  bool is_h2p_branch(uint64_t ip)
  {
    if ((last_pred_source == HIT_BANK && (last_pointer_for_pred > -4 && last_pointer_for_pred < 3)) || (last_pred_source == ALT_BANK)
        || ((last_pred_source == BIMODAL && (last_pointer_for_pred != 0 && last_pointer_for_pred != 3)))) {
      return true;
    }
    return false;
  }

  int is_h2p(uint64_t ip)
  {

#ifdef H2P_TAGE_STYLE_SEZNEC
    if (last_pred_source == HIT_BANK) {
      // saturated so mostly correct prediction
      if (last_pointer_for_pred == -4 || last_pointer_for_pred == 3) {
        return 0;
      }
      if (last_pointer_for_pred == -3 || last_pointer_for_pred == 2) {
        return 3;
      }
      if (last_pointer_for_pred == -2 || last_pointer_for_pred == 1) {
        return 4;
      }
      if (last_pointer_for_pred == -1 || last_pointer_for_pred == 0) {
        return 6;
      }
    }
    if (last_pred_source == BIMODAL) {
      if (bimodal_miss_hist) {
        if (last_pointer_for_pred == 0 || last_pointer_for_pred == 3) {
          return 6;
        } else {
          return 2;
        }
      } else {
        if (last_pointer_for_pred == 0 || last_pointer_for_pred == 3) {
          return 0;
        } else {
          return 2;
        }
      }
    }
    return H2P_T;
#else

    // cout << __func__ << " last_pred_source " << last_pred_source << " last_pointer_for_pred " << int(last_pointer_for_pred) << endl;
    if (last_pred_source == HIT_BANK) {
      // saturated so mostly correct prediction
      if (last_pointer_for_pred == -4 || last_pointer_for_pred == 3) {
        return 1;
      }
      if (last_pointer_for_pred == -3 || last_pointer_for_pred == 2) {
        return 3;
      }
      if (last_pointer_for_pred == -2 || last_pointer_for_pred == 1) {
        return 4;
      }
      if (last_pointer_for_pred == -1 || last_pointer_for_pred == 0) {
        return 6;
      }
    }
    if (last_pred_source == ALT_BANK) {
      if (last_pointer_for_pred == 3 || last_pointer_for_pred == -4) {
        return 5;
      } else {
        return 7;
      }
    }
    if (last_pred_source == BIMODAL) {
      if (bimodal_miss_hist) {
        if (last_pointer_for_pred == 0 || last_pointer_for_pred == 3) {
          return 6;
        } else {
          return 2;
        }
      } else {
        if (last_pointer_for_pred == 0 || last_pointer_for_pred == 3) {
          return 1;
        } else {
          return 2;
        }
      }
    }
    if (last_pred_source == SAT_PRED) {
      if (last_pointer_for_pred_sat >= 0 && last_pointer_for_pred_sat <= 31)
        return 10;
      if (last_pointer_for_pred_sat >= 32 && last_pointer_for_pred_sat <= 63)
        return 8;
      if (last_pointer_for_pred_sat >= 64 && last_pointer_for_pred_sat <= 127)
        return 6;
      if (last_pointer_for_pred_sat >= 128 && last_pointer_for_pred_sat <= 255)
        return 3;
    }
    if (last_pred_source == LOOP) {
      return 1;
    }
#endif
    return 1;
  }

  void speculative_begin(uint64_t PC, uint8_t opType, bool resolveDir, uint64_t branchTarget)
  {
    // cout << __func__ << " " << PC << endl;

    spec_last_pred_source = last_pred_source;
    spec_last_pointer_for_pred = last_pointer_for_pred;

    spec_state = true;
    spec_IMLIcount = IMLIcount;

    for (int i = 0; i < (1 << LOGBIAS); i++) {
      spec_Bias[i] = Bias[i];
    }
    for (int i = 0; i < (1 << LOGBIAS); i++) {
      spec_BiasSK[i] = BiasSK[i];
    }
    for (int i = 0; i < (1 << LOGBIAS); i++) {
      spec_BiasBank[i] = BiasBank[i];
    }

    for (int i = 0; i < INB; i++) {
      spec_Im[i] = Im[i];
      spec_IGEHL[i] = IGEHL[i];
      for (int j = 0; j < (1 << LOGINB); j++) {
        spec_IGEHLA[i][j] = IGEHLA[i][j];
      }
    }

    for (int i = 0; i < GNB; i++) {
      spec_Gm[i] = Gm[i];
      spec_GGEHL[i] = GGEHL[i];
      for (int j = 0; j < (1 << LOGGNB); j++) {
        spec_GGEHLA[i][j] = GGEHLA[i][j];
      }
    }

    for (int i = 0; i < LNB; i++) {
      spec_Lm[i] = Lm[i];
      spec_LGEHL[i] = LGEHL[i];
      for (int j = 0; j < (1 << LOGLNB); j++) {
        spec_LGEHLA[i][j] = LGEHLA[i][j];
      }
    }

    for (int i = 0; i < NLOCAL; i++) {
      spec_L_shist[i] = L_shist[i];
    }

    spec_updatethreshold = updatethreshold;

    for (int i = 0; i < (1 << LOGSIZEUP); i++) {
      spec_Pupdatethreshold[i] = Pupdatethreshold[i];
    }

    for (int i = 0; i < (1 << LOGSIZEUPS); i++) {
      spec_WG[i] = WG[i];
      spec_WL[i] = WL[i];
      spec_WI[i] = WI[i];
      spec_WB[i] = WB[i];
    }

    spec_LSUM = LSUM;
    spec_FirstH = FirstH;
    spec_SecondH = SecondH;

    spec_MedConf = MedConf;

    for (int i = 0; i < (NHIST + 1); i++) {
      spec_NOSKIP[i] = NOSKIP[i];
    }

    spec_LowConf = LowConf;
    spec_HighConf = HighConf;
    spec_AltConf = AltConf;

    for (int i = 0; i < (SIZEUSEALT); i++) {
      spec_use_alt_on_na[i] = use_alt_on_na[i];
    }

    spec_GHIST = GHIST;
    spec_BIM = BIM;
    spec_TICK = TICK;

    for (int i = 0; i < (HISTBUFFERLENGTH); i++) {
      spec_ghist[i] = ghist[i];
    }

    spec_ptghist = ptghist;
    spec_phist = phist;

    for (int i = 0; i < (NHIST + 1); i++) {
      spec_ch_i[i].CLENGTH = ch_i[i].CLENGTH;
      spec_ch_i[i].comp = ch_i[i].comp;
      spec_ch_i[i].OLENGTH = ch_i[i].OLENGTH;
      spec_ch_i[i].OUTPOINT = ch_i[i].OUTPOINT;

      spec_ch_t[0][i].CLENGTH = ch_t[0][i].CLENGTH;
      spec_ch_t[0][i].comp = ch_t[0][i].comp;
      spec_ch_t[0][i].OLENGTH = ch_t[0][i].OLENGTH;
      spec_ch_t[0][i].OUTPOINT = ch_t[0][i].OUTPOINT;

      spec_ch_t[1][i].CLENGTH = ch_t[1][i].CLENGTH;
      spec_ch_t[1][i].comp = ch_t[1][i].comp;
      spec_ch_t[1][i].OLENGTH = ch_t[1][i].OLENGTH;
      spec_ch_t[1][i].OUTPOINT = ch_t[1][i].OUTPOINT;
    }

    for (int i = 0; i < (1 << LOGB); i++) {
      spec_btable[i].pred = btable[i].pred;
      spec_btable[i].hyst = btable[i].hyst;
    }

    for (int i = 0; i < (NHIST + 1); i++) {
      // cout << "Saving " << int(gtable[2][437].ctr) << endl;
      spec_m[i] = m[i];
      spec_TB[i] = TB[i];
      spec_logg[i] = logg[i];
      spec_GI[i] = GI[i];
      spec_GTAG[i] = GTAG[i];
    }

    for (int i = 0; i < NBBANK[0] * (1 << LOGG); i++) {
      spec_gtable[1][i].ctr = gtable[1][i].ctr;
      spec_gtable[1][i].tag = gtable[1][i].tag;
      spec_gtable[1][i].u = gtable[1][i].u;
    }
    for (int i = 0; i < NBBANK[1] * (1 << LOGG); i++) {
      spec_gtable[BORN][i].ctr = gtable[BORN][i].ctr;
      spec_gtable[BORN][i].tag = gtable[BORN][i].tag;
      spec_gtable[BORN][i].u = gtable[BORN][i].u;
    }

    for (int i = BORN + 1; i <= NHIST; i++) {
      for (int j = 0; j < NBBANK[1] * (1 << LOGG); j++) {
        spec_gtable[i][j].ctr = gtable[i][j].ctr;
        spec_gtable[i][j].tag = gtable[i][j].tag;
        spec_gtable[i][j].u = gtable[i][j].u;
      }
    }
    for (int i = 2; i <= BORN - 1; i++) {
      for (int j = 0; j < NBBANK[0] * (1 << LOGG); j++) {
        spec_gtable[i][j].ctr = gtable[i][j].ctr;
        spec_gtable[i][j].tag = gtable[i][j].tag;
        spec_gtable[i][j].u = gtable[i][j].u;
      }
    }

    spec_BI = BI;

    spec_pred_taken = pred_taken; // prediction
    spec_alttaken = alttaken;     // alternate  TAGEprediction
    spec_tage_pred = tage_pred;   // TAGE prediction
    spec_LongestMatchPred = LongestMatchPred;
    spec_HitBank = HitBank;
    spec_AltBank = AltBank;
    spec_Seed = Seed;
    spec_pred_inter = pred_inter;

    for (int i = 0; i < (1 << LOGL); i++) {
      spec_ltable[i].age = ltable[i].age;
      spec_ltable[i].confid = ltable[i].confid;
      spec_ltable[i].CurrentIter = ltable[i].CurrentIter;
      spec_ltable[i].dir = ltable[i].dir;
      spec_ltable[i].NbIter = ltable[i].NbIter;
      spec_ltable[i].TAG = ltable[i].TAG;
    }

    spec_predloop = predloop; // loop predictor prediction

    spec_LIB = LIB;
    spec_LI = LI;
    spec_LHIT = LHIT;         // hitting way in the loop predictor
    spec_LTAG = LTAG;         // tag on the loop predictor
    spec_LVALID = LVALID;     // validity of the loop predictor prediction
    spec_WITHLOOP = WITHLOOP; // counter to monitor whether or not loop prediction is beneficial
    spec_THRES = THRES;
  }

  void speculative_end()
  {

    last_pred_source = spec_last_pred_source;
    last_pointer_for_pred = spec_last_pointer_for_pred;

    spec_state = false;
    IMLIcount = spec_IMLIcount;

    for (int i = 0; i < (1 << LOGBIAS); i++) {
      Bias[i] = spec_Bias[i];
    }
    for (int i = 0; i < (1 << LOGBIAS); i++) {
      BiasSK[i] = spec_BiasSK[i];
    }
    for (int i = 0; i < (1 << LOGBIAS); i++) {
      BiasBank[i] = spec_BiasBank[i];
    }

    for (int i = 0; i < INB; i++) {
      Im[i] = spec_Im[i];
      IGEHL[i] = spec_IGEHL[i];
      for (int j = 0; j < (1 << LOGINB); j++) {
        IGEHLA[i][j] = spec_IGEHLA[i][j];
      }
    }

    for (int i = 0; i < GNB; i++) {
      Gm[i] = spec_Gm[i];
      GGEHL[i] = spec_GGEHL[i];
      for (int j = 0; j < (1 << LOGGNB); j++) {
        GGEHLA[i][j] = spec_GGEHLA[i][j];
      }
    }

    for (int i = 0; i < LNB; i++) {
      Lm[i] = spec_Lm[i];
      LGEHL[i] = spec_LGEHL[i];
      for (int j = 0; j < (1 << LOGLNB); j++) {
        LGEHLA[i][j] = spec_LGEHLA[i][j];
      }
    }

    for (int i = 0; i < NLOCAL; i++) {
      L_shist[i] = spec_L_shist[i];
    }

    updatethreshold = spec_updatethreshold;

    for (int i = 0; i < (1 << LOGSIZEUP); i++) {
      Pupdatethreshold[i] = spec_Pupdatethreshold[i];
    }

    for (int i = 0; i < (1 << LOGSIZEUPS); i++) {
      WG[i] = spec_WG[i];
      WL[i] = spec_WL[i];
      WI[i] = spec_WI[i];
      WB[i] = spec_WB[i];
    }

    LSUM = spec_LSUM;
    FirstH = spec_FirstH;
    SecondH = spec_SecondH;

    MedConf = spec_MedConf;

    for (int i = 0; i < (NHIST + 1); i++) {
      NOSKIP[i] = spec_NOSKIP[i];
    }

    LowConf = spec_LowConf;
    HighConf = spec_HighConf;
    AltConf = spec_AltConf;

    for (int i = 0; i < (SIZEUSEALT); i++) {
      use_alt_on_na[i] = spec_use_alt_on_na[i];
    }

    GHIST = spec_GHIST;
    BIM = spec_BIM;
    TICK = spec_TICK;

    for (int i = 0; i < (HISTBUFFERLENGTH); i++) {
      ghist[i] = spec_ghist[i];
    }

    ptghist = spec_ptghist;
    phist = spec_phist;

    for (int i = 0; i < (NHIST + 1); i++) {
      ch_i[i].CLENGTH = spec_ch_i[i].CLENGTH;
      ch_i[i].comp = spec_ch_i[i].comp;
      ch_i[i].OLENGTH = spec_ch_i[i].OLENGTH;
      ch_i[i].OUTPOINT = spec_ch_i[i].OUTPOINT;

      ch_t[0][i].CLENGTH = spec_ch_t[0][i].CLENGTH;
      ch_t[0][i].comp = spec_ch_t[0][i].comp;
      ch_t[0][i].OLENGTH = spec_ch_t[0][i].OLENGTH;
      ch_t[0][i].OUTPOINT = spec_ch_t[0][i].OUTPOINT;

      ch_t[1][i].CLENGTH = spec_ch_t[1][i].CLENGTH;
      ch_t[1][i].comp = spec_ch_t[1][i].comp;
      ch_t[1][i].OLENGTH = spec_ch_t[1][i].OLENGTH;
      ch_t[1][i].OUTPOINT = spec_ch_t[1][i].OUTPOINT;
    }

    for (int i = 0; i < (1 << LOGB); i++) {
      btable[i].pred = spec_btable[i].pred;
      btable[i].hyst = spec_btable[i].hyst;
    }

    for (int i = 0; i < (NHIST + 1); i++) {
      //
      m[i] = spec_m[i];
      TB[i] = spec_TB[i];
      logg[i] = spec_logg[i];
      GI[i] = spec_GI[i];
      GTAG[i] = spec_GTAG[i];
    }

    for (int i = 0; i < NBBANK[0] * (1 << LOGG); i++) {
      gtable[1][i].ctr = spec_gtable[1][i].ctr;
      gtable[1][i].tag = spec_gtable[1][i].tag;
      gtable[1][i].u = spec_gtable[1][i].u;
    }
    for (int i = 0; i < NBBANK[1] * (1 << LOGG); i++) {
      gtable[BORN][i].ctr = spec_gtable[BORN][i].ctr;
      gtable[BORN][i].tag = spec_gtable[BORN][i].tag;
      gtable[BORN][i].u = spec_gtable[BORN][i].u;
    }

    for (int i = BORN + 1; i <= NHIST; i++) {
      for (int j = 0; j < NBBANK[1] * (1 << LOGG); j++) {
        gtable[i][j].ctr = spec_gtable[i][j].ctr;
        gtable[i][j].tag = spec_gtable[i][j].tag;
        gtable[i][j].u = spec_gtable[i][j].u;
      }
    }
    for (int i = 2; i <= BORN - 1; i++) {
      for (int j = 0; j < NBBANK[0] * (1 << LOGG); j++) {
        gtable[i][j].ctr = spec_gtable[i][j].ctr;
        gtable[i][j].tag = spec_gtable[i][j].tag;
        gtable[i][j].u = spec_gtable[i][j].u;
      }
    }

    // cout << "Restore " << int(gtable[2][437].ctr) << " spec " << int(spec_gtable[2][437].ctr) << endl;

    BI = spec_BI;

    pred_taken = spec_pred_taken; // prediction
    alttaken = spec_alttaken;     // alternate  TAGEprediction
    tage_pred = spec_tage_pred;   // TAGE prediction
    LongestMatchPred = spec_LongestMatchPred;
    HitBank = spec_HitBank;
    AltBank = spec_AltBank;
    Seed = spec_Seed;
    pred_inter = spec_pred_inter;

    for (int i = 0; i < (1 << LOGL); i++) {
      ltable[i].age = spec_ltable[i].age;
      ltable[i].confid = spec_ltable[i].confid;
      ltable[i].CurrentIter = spec_ltable[i].CurrentIter;
      ltable[i].dir = spec_ltable[i].dir;
      ltable[i].NbIter = spec_ltable[i].NbIter;
      ltable[i].TAG = spec_ltable[i].TAG;
    }

    predloop = spec_predloop; // loop predictor prediction

    LIB = spec_LIB;
    LI = spec_LI;
    LHIT = spec_LHIT;         // hitting way in the loop predictor
    LTAG = spec_LTAG;         // tag on the loop predictor
    LVALID = spec_LVALID;     // validity of the loop predictor prediction
    WITHLOOP = spec_WITHLOOP; // counter to monitor whether or not loop prediction is beneficial
    THRES = spec_THRES;
  }
};
#endif
// keep this header's configuration macros from leaking into the including TU
#undef SC
#undef LOOPPREDICTOR
#undef H2P_T
#endif // UCP_ALT_TAGE_H

#ifndef BRANCH_TAGE_SC_L_H
#define BRANCH_TAGE_SC_L_H

#include <cstdint>

#include "address.h"
#include "modules.h"

// The TAGE-SC-L algorithm (class PREDICTOR) is defined in tage_sc_l.cc.
// Ported from the UCP_ISCA24 artifact to the current ChampSim module API.
class PREDICTOR;

struct tage_sc_l : champsim::modules::branch_predictor {
  PREDICTOR* impl = nullptr;
  bool last_prediction = false;

  using branch_predictor::branch_predictor;

  bool predict_branch(champsim::address ip);
  void last_branch_result(champsim::address ip, champsim::address branch_target, bool taken, uint8_t branch_type);
};

#endif

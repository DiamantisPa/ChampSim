#ifndef BTB_BASIC_BTB_INDIRECT_PREDICTOR_H
#define BTB_BASIC_BTB_INDIRECT_PREDICTOR_H

#include <cstdint>
#include <utility>

#include "address.h"

// The indirect-target predictor is an ITTAGE (class my_predictor), ported from
// the UCP_ISCA24 artifact (ittage_64KB.h). It replaces the previous simple
// gshare-indexed table. Forward-declared here; defined in ittage_64KB.h and
// only included by indirect_predictor.cc.
class my_predictor;

struct indirect_predictor {
  my_predictor* ittage = nullptr;

  std::pair<champsim::address, bool> prediction(champsim::address ip);
  void update(champsim::address ip, champsim::address branch_target, bool taken, uint8_t branch_type);
};

#endif

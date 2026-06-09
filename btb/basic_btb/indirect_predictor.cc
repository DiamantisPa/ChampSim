#include "indirect_predictor.h"

#include <cstdint>

#include "ittage_64KB.h" // defines class my_predictor (the ITTAGE)

std::pair<champsim::address, bool> indirect_predictor::prediction(champsim::address ip)
{
  if (ittage == nullptr)
    ittage = new my_predictor();
  uint64_t target = ittage->predict_brindirect(ip.to<uint64_t>());
  // indirect branches are unconditional, so the prediction is always "taken"
  return {champsim::address{target}, true};
}

void indirect_predictor::update(champsim::address ip, champsim::address branch_target, bool taken, uint8_t branch_type)
{
  if (ittage == nullptr)
    ittage = new my_predictor();
  const uint64_t pc = ip.to<uint64_t>();
  const uint64_t target = branch_target.to<uint64_t>();
  // update_brindirect trains only on indirect branches internally;
  // fetch_history_update advances the global history for every branch.
  ittage->update_brindirect(pc, branch_type, taken, target);
  ittage->fetch_history_update(pc, branch_type, taken, target);
}

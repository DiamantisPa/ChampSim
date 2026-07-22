
/*
 * This file implements a basic Branch Target Buffer (BTB) structure.
 * It uses a set-associative BTB to predict the targets of non-return branches,
 * and it uses a small Return Address Stack (RAS) to predict the target of
 * returns.
 */

#include "basic_btb.h"

#include "instruction.h"
#include "ucp_hooks.h"

void basic_btb::initialize_btb()
{
  // UCP alt-path hooks (single-core): BTB-only probe -- no RAS pop, no ITTAGE
  // history perturbation; the walker keeps its own Alt-RAS from the snapshot.
  ucp::btb_probe = [this](uint64_t ip) -> std::optional<ucp::btb_probe_result> {
    auto entry = direct.check_hit(champsim::address{ip});
    if (!entry.has_value())
      return std::nullopt;
    ucp::btb_probe_result r;
    r.branch_type = entry->raw_type;
    r.conditional = (entry->type == direct_predictor::branch_info::CONDITIONAL);
    r.target = (entry->type == direct_predictor::branch_info::RETURN) ? 0 : entry->target.to<uint64_t>();
    return r;
  };
  ucp::btb_ras_snapshot = [this]() {
    std::vector<uint64_t> targets;
    for (const auto& call_ip : ras.stack) { // bottom-to-top; walker pops from the back
      auto size = ras.call_size_trackers[call_ip.slice_lower<champsim::data::bits{champsim::msl::lg2(return_stack::num_call_size_trackers)}>().to<std::size_t>()];
      targets.push_back((call_ip + size).to<uint64_t>());
    }
    return targets;
  };
}

std::pair<champsim::address, bool> basic_btb::btb_prediction(champsim::address ip)
{
  // use BTB for all other branches + direct calls
  auto btb_entry = direct.check_hit(ip);

  // no prediction for this IP
  if (!btb_entry.has_value())
    return {champsim::address{}, false};

  if (btb_entry->type == direct_predictor::branch_info::RETURN)
    return ras.prediction();

  if (btb_entry->type == direct_predictor::branch_info::INDIRECT)
    return indirect.prediction(ip);

  return {btb_entry->target, btb_entry->type != direct_predictor::branch_info::CONDITIONAL};
}

void basic_btb::update_btb(champsim::address ip, champsim::address branch_target, bool taken, uint8_t branch_type)
{
  // add something to the RAS
  if (branch_type == BRANCH_DIRECT_CALL || branch_type == BRANCH_INDIRECT_CALL)
    ras.push(ip);

  // ITTAGE indirect predictor: train on every branch (it trains the indirect
  // tables only for indirect branches internally, and advances global history)
  indirect.update(ip, branch_target, taken, branch_type);

  if (branch_type == BRANCH_RETURN)
    ras.calibrate_call_size(branch_target);

  direct.update(ip, branch_target, branch_type);
}

#ifndef UCP_HOOKS_H
#define UCP_HOOKS_H

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

// Side-channel hooks for the UCP port (alternate-path u-op cache prefetching,
// Singh et al., ISCA'24).  The module system is type-erased, so the tage_sc_l and
// basic_btb modules publish the extra state UCP needs through these globals.
// Single-core only (the fork's tage_sc_l is single-core anyway); everything here
// is inert unless the trace_ucp knob is set.
namespace ucp
{
struct btb_probe_result {
  uint64_t target = 0;      // 0 = no target stored (e.g. RETURN entries)
  uint8_t branch_type = 0;  // raw BRANCH_* constant recorded at update
  bool conditional = false; // direction comes from the Alt-BP
};

// set by basic_btb: BTB-only lookup for alternate-path walking -- touches neither
// the RAS nor the ITTAGE (their alt-path state is walker-local)
inline std::function<std::optional<btb_probe_result>(uint64_t)> btb_probe;

// set by basic_btb: return targets currently on the RAS, bottom-to-top (the
// walker copies this as its Alt-RAS at trigger time, paper Sec. IV-C)
inline std::function<std::vector<uint64_t>()> btb_ras_snapshot;

// set by tage_sc_l on every predict: was the last prediction hard-to-predict per
// the paper's UCP-Conf classification (H2P_TAGE_STYLE_CTR_SC_BIMH)
inline bool bp_last_h2p = false;

} // namespace ucp

#endif

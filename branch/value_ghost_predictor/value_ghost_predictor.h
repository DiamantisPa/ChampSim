#include <cstdint>
#include <unordered_map>
#include <vector>

// Ghost Entry for each PC
struct GhostEntry {
    uint8_t tag;   // 8-bit fingerprint of operand (src1 ⊕ src2)
    bool outcome;  // 1-bit outcome (taken or not)
    uint8_t age;   // 2-bit age for LRU (0=most recent, 3=least recent)
};

// Ghost Predictor for each branch PC
class GhostBranchPredictor {
public:
    static const int GHOST_BUFFER_SIZE = 4;  // Number of entries in the ghost buffer

    // Per-branch ghost buffer (indexed by branch PC)
    std::unordered_map<uint64_t, std::vector<GhostEntry>> ghostTable;

    GhostBranchPredictor();
    bool predict(uint64_t pc, uint64_t src1, uint64_t src2, bool& prediction);
    void update(uint64_t pc, uint64_t src1, uint64_t src2, bool actualOutcome);
    void promote(uint64_t pc, uint64_t src1, uint64_t src2);
};
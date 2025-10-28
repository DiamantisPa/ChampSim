// Updated implementations for do_predict_branch and GhostBranchPredictor
// Assumptions made in this patch:
// - arch_instr now exposes source_register_values: vector<uint64_t> containing runtime operand values at decode/fetch.
//   If you don't have runtime values at this stage, see the notes below: use a value predictor or move ghost prediction to decode.
// - A value predictor object `value_predictor` is available with signature:
//     bool value_predictor.predict(uint64_t pc, unsigned src_idx, uint64_t &pred_value);
//   The patch will use predicted values if actual values are not available.
// - `impl_predict_branch` returns the regular predictor decision (bool) and does not modify global stats itself.
// - Replaces previous GhostBranchPredictor simple vector-per-pc with a bounded hashed bucket table for better memory control.
// - Adds confidence counters and conservative override logic: ghost only overrides when confidence >= CONF_THRESHOLD.

#include <cstdint>
#include <vector>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <algorithm>
#include <iostream>

// --- Tunable constants ---
static constexpr size_t GHOST_TABLE_ENTRIES = 1 << 16; // 65536 buckets
static constexpr size_t GHOST_BUCKET_SIZE   = 4;       // entries per bucket
static constexpr unsigned FINGERPRINT_BITS  = 16;      // fingerprint size (bits)
static constexpr unsigned CONF_MAX          = 3;       // 2-bit/3-bit saturating counter
static constexpr unsigned CONF_THRESHOLD    = 2;       // require confidence >= 2 to override
static constexpr unsigned MIN_SAMPLES       = 50;      // require at least N samples to enable ghost
static constexpr float    MISRATE_THRESHOLD = 0.15f;   // ghost candidate when regular misrate > 15%

// --- Simple helpers ---
static inline uint32_t pc_to_bucket(uint64_t pc) {
    // mix PC bits then fold to table size
    uint64_t x = pc ^ (pc >> 12) ^ (pc << 7);
    return static_cast<uint32_t>(x) & (GHOST_TABLE_ENTRIES - 1);
}

static inline uint32_t make_fingerprint(uint64_t v1, uint64_t v2) {
    // 64-bit mix -> fold to FINGERPRINT_BITS
    uint64_t mix = v1 * 0x9e3779b97f4a7c15ULL;
    mix ^= (v2 + 0x9e3779b97f4a7c15ULL + (mix << 6) + (mix >> 2));
    uint32_t f = static_cast<uint32_t>((mix ^ (mix >> 32)) & ((1u << FINGERPRINT_BITS) - 1));
    if (f == 0) f = 1; // reserve 0 for "empty" (optional)
    return f;
}

// --- Ghost entry and table ---
struct GhostEntry {
    uint32_t tag;    // fingerprint
    uint8_t  outcome; // 0/1
    uint8_t  confidence; // saturating
    uint8_t  age;    // LRU-ish (0 newest)
};

struct GhostBucket {
    std::array<GhostEntry, GHOST_BUCKET_SIZE> entries;
    std::array<bool, GHOST_BUCKET_SIZE> valid;
    GhostBucket() {
        valid.fill(false);
        for (size_t i = 0; i < GHOST_BUCKET_SIZE; ++i) {
            entries[i].tag = 0;
            entries[i].outcome = 0;
            entries[i].confidence = 0;
            entries[i].age = 0xff;
        }
    }
};

class GhostBranchPredictor {
public:
    GhostBranchPredictor();

    // Predict: returns true if ghost has a confident entry and sets `prediction`.
    bool predict(uint64_t pc, uint64_t val1, uint64_t val2, bool &prediction);

    // Update: informs the ghost predictor of the actual outcome and whether regular predictor mispredicted.
    // If reg_mispred is true, the ghost may choose to allocate a new entry for the fingerprint.
    void update(uint64_t pc, uint64_t val1, uint64_t val2, bool actualOutcome, bool reg_mispred, bool used_ghost);
    // Simple instrumentation (counts only)
    size_t ghost_total_uses = 0;
    size_t ghost_correct = 0;
    size_t ghost_wrong = 0;
    size_t ghost_allocs = 0;

private:
    std::vector<GhostBucket> table;

    // find index in bucket, return -1 if not found
    int find_in_bucket(GhostBucket &b, uint32_t tag) {
        for (int i = 0; i < (int)GHOST_BUCKET_SIZE; ++i) {
            if (b.valid[i] && b.entries[i].tag == tag) return i;
        }
        return -1;
    }

    // choose LRU index in bucket for replacement
    int choose_lru(GhostBucket &b) {
        int lru = 0;
        uint8_t max_age = 0;
        for (int i = 0; i < (int)GHOST_BUCKET_SIZE; ++i) {
            if (!b.valid[i]) return i; // empty slot
            if (b.entries[i].age > max_age) { max_age = b.entries[i].age; lru = i; }
        }
        return lru;
    }

    void touch_entry(GhostBucket &b, int idx) {
        // Set idx to MRU (age=0), increment others up to caps
        uint8_t old_age = b.entries[idx].age;
        for (int i = 0; i < (int)GHOST_BUCKET_SIZE; ++i) {
            if (!b.valid[i]) continue;
            if (b.entries[i].age < old_age) b.entries[i].age++;
        }
        b.entries[idx].age = 0;
    }
};
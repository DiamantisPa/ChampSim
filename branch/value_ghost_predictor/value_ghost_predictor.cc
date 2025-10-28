#include "value_ghost_predictor.h"

// --- Implementation ---
GhostBranchPredictor::GhostBranchPredictor() {
    table.resize(GHOST_TABLE_ENTRIES);
}

bool GhostBranchPredictor::predict(uint64_t pc, uint64_t val1, uint64_t val2, bool &prediction) {
    uint32_t bucket_idx = pc_to_bucket(pc);
    uint32_t tag = make_fingerprint(val1, val2);
    GhostBucket &b = table[bucket_idx];

    int idx = find_in_bucket(b, tag);
    if (idx < 0) return false; // miss

    GhostEntry &e = b.entries[idx];

    // Only use ghost prediction if confidence is high enough
    if (e.confidence >= CONF_THRESHOLD) {
        prediction = (e.outcome != 0);
        ghost_total_uses++;
        // update recency
        touch_entry(b, idx);
        return true;
    }
    return false;
}

// New signature: added `used_ghost`
void GhostBranchPredictor::update(uint64_t pc, uint64_t val1, uint64_t val2, bool actualOutcome, bool reg_mispred, bool used_ghost) {
    uint32_t bucket_idx = pc_to_bucket(pc);
    uint32_t tag = make_fingerprint(val1, val2);
    GhostBucket &b = table[bucket_idx];

    int idx = find_in_bucket(b, tag);
    if (idx >= 0) {
        GhostEntry &e = b.entries[idx];
        bool match = ((e.outcome != 0) == actualOutcome);

        if (used_ghost) {
            // Strong update: ghost *made* the prediction, so reward/punish more aggressively
            if (match) {
                // correct override -> stronger reward
                unsigned add = 2;
                unsigned newc = std::min<unsigned>(CONF_MAX, e.confidence + add);
                e.confidence = static_cast<uint8_t>(newc);
                ghost_correct++;
            } else {
                // wrong override -> stronger penalty
                if (e.confidence <= 2) e.confidence = 0;
                else e.confidence = static_cast<uint8_t>(e.confidence - 2);
                ghost_wrong++;
            }
            e.outcome = actualOutcome ? 1 : 0;
            touch_entry(b, idx);
            return;
        } else {
            // Ghost entry existed but wasn't used: soft adaptation
            if (match) {
                if (e.confidence < CONF_MAX) e.confidence++;
                ghost_correct++;
            } else {
                if (e.confidence > 0) e.confidence--;
                ghost_wrong++;
            }
            e.outcome = actualOutcome ? 1 : 0;
            touch_entry(b, idx);
            return;
        }
    }

    // Entry not found:
    // Conservative allocation policy:
    // - allocate ONLY when regular predictor mispredicted (reg_mispred == true)
    // - and only when ghost wasn't claimed to be used (used_ghost should be false here normally)
    // (If used_ghost==true and entry not found, something is inconsistent; do nothing.)
    if (!reg_mispred || used_ghost) return;

    int repl = choose_lru(b);
    b.valid[repl] = true;
    b.entries[repl].tag = tag;
    b.entries[repl].outcome = actualOutcome ? 1 : 0;
    b.entries[repl].confidence = 1; // start low
    b.entries[repl].age = 0;
    // increment age of others
    for (int i = 0; i < (int)GHOST_BUCKET_SIZE; ++i) {
        if (i != repl && b.valid[i] && b.entries[i].age < 0xFE) b.entries[i].age++;
    }
    ghost_allocs++;
}


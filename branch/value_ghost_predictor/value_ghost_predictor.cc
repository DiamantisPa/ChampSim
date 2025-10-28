#include "value_ghost_predictor.h"

bool GhostBranchPredictor::predict(uint64_t pc, uint64_t src1, uint64_t src2, bool& prediction) {
    uint8_t fingerprint = (src1 ^ src2) & 0xFF;  // 8-bit fingerprint (low 8 bits of XOR)

    // Look up the PC's ghost buffer
    auto& buffer = ghostTable[pc];

    for (auto& entry : buffer) {
        if (entry.tag == fingerprint) {
            // Ghost hit, return the predicted outcome
            prediction = entry.outcome;
            return true;  // Found a match
        }
    }

    // Ghost miss: fall back to perceptron or global predictor
    return false;  // Miss
}

void GhostBranchPredictor::update(uint64_t pc, uint64_t src1, uint64_t src2, bool actualOutcome) {
    uint8_t fingerprint = (src1 ^ src2) & 0xFF;

    auto& buffer = ghostTable[pc];

    // Search for the entry to update
    for (auto& entry : buffer) {
        if (entry.tag == fingerprint) {
            // Update the outcome and refresh the age
            entry.outcome = actualOutcome;
            entry.age = 0;  // Most recently used
            // LRU Update: Increment the age for other entries
            for (auto& other : buffer) {
                if (&other != &entry && other.age < 3) {
                    other.age++;
                }
            }
            return;
        }
    }

    // If miss and the predictor was wrong, allocate a new ghost entry
    if (buffer.size() < GHOST_BUFFER_SIZE) {
        buffer.push_back({fingerprint, actualOutcome, 0});
    } else {
        // Evict the LRU (least recently used) entry
        int lruIndex = 0;
        for (int i = 1; i < (int) buffer.size(); ++i) {
            if (buffer[i].age > buffer[lruIndex].age) {
                lruIndex = i;
            }
        }

        // Replace LRU entry
        buffer[lruIndex] = {fingerprint, actualOutcome, 0};
    }

    // LRU Update: Increment the age for all other entries
    for (auto& entry : buffer) {
        if (entry.age < 3) {
            entry.age++;
        }
    }
}

GhostBranchPredictor::GhostBranchPredictor() {}


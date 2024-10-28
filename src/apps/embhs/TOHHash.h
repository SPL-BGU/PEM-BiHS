//
// Created by Lior Siag on 04/01/2024.
// Note that this hashing mechanism works only for up to 32 disks, after which it returns wrong hashes
// The general idea is to divide the state into bucket which holds pegs 0 and 1, and data which holds pegs 2 and 3.
// Each bit i represents the N-i disk, e.g., bit 0 is disk N, bit 1 is disk N-1, ... bit N-1 is disk 1.
// This way, we can simply shift the bits and continuously take the least significant bit, which represents the current
// largest disk, thus allowing us to simply placing it on the peg.
//

#ifndef HOG2_SPL_TOHHASH_H
#define HOG2_SPL_TOHHASH_H

#include <cstdint>
#include <bitset>
#include "TOH.h"

namespace EMBHS {
    template<int N>
    class TOHHash {
    public:
        TOHHash() {}


        uint64_t HashBucket(const TOHState<N> &state) {
            uint64_t hash = 0;
            for (int peg = 0; peg < 2; ++peg) {
                for (int i = 0; i < state.GetDiskCountOnPeg(peg); i++) {
                    hash |= (uint64_t(1) << (32 * peg + N - state.GetDiskOnPeg(peg, i)));
                }
            }
            return hash;
        }

        uint64_t HashData(const TOHState<N> &state) {
            uint64_t hash = 0;
            for (int peg = 2; peg < 4; ++peg) {
                for (int i = 0; i < state.GetDiskCountOnPeg(peg); i++) {
                    hash |= (uint64_t(1) << ((peg - 2) * 32 + N - state.GetDiskOnPeg(peg, i)));
                }
            }
            return hash;
        }

        void RestoreHelper(TOHState<N> &state, uint64_t hash, int peg) {
            for (int i = N; i > 0; --i) {
                if (hash & 0x01) {
                    state.disks[peg][state.counts[peg]] = i;
                    state.counts[peg]++;
                }
                hash >>= 1;
            }
        }

        void Restore(TOHState<N> &state, uint64_t bucketHash, uint64_t dataHash) {
            state.counts[0] = state.counts[1] = state.counts[2] = state.counts[3] = 0;
            RestoreHelper(state, bucketHash, 0);
            RestoreHelper(state, bucketHash >> 32, 1);
            RestoreHelper(state, dataHash, 2);
            RestoreHelper(state, dataHash >> 32, 3);
        }

    protected:
        TOHState<N> goal; // Goal has to be before the PDBs, so it will get initialized before them
    };
}

#endif //HOG2_SPL_TOHHASH_H

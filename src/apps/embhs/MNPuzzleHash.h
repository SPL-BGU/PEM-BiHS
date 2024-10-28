#ifndef HOG2_SPL_MNPUZZLEHASH_H
#define HOG2_SPL_MNPUZZLEHASH_H


#include <cstdint>
#include "MNPuzzle.h"
#include "LexPermutationPDB.h"

namespace EMBHS {
    namespace {
        const std::vector<int> BUCKET5 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        const std::vector<int> DATA5 = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
        const std::vector<int> BUCKET4 = {0, 1, 2, 3, 4, 5, 6};
        const std::vector<int> DATA4 = {7, 8, 9, 10, 11, 12, 13, 14, 15};
    }

    template<int N>
    class STPHash {
    public:
        STPHash() : bucketPDB(&env, goal, GetVector(true)),
                    dataPDB(&env, goal, GetVector(false)) {}

        uint64_t HashBucket(const MNPuzzleState<N, N> &state) {
            return bucketPDB.GetPDBHash(state);
        }

        uint64_t HashData(const MNPuzzleState<N, N> &state) {
            return dataPDB.GetPDBHash(state);
        }

        void Restore(MNPuzzleState<N, N> &state, uint64_t bucketHash, uint64_t dataHash) {
            MNPuzzleState<N, N> bucketState;
            bucketPDB.GetStateFromPDBHash(bucketHash, bucketState);
            MNPuzzleState<N, N> dataState;
            dataPDB.GetStateFromPDBHash(dataHash, dataState);
            state.blank = bucketState.blank;
            for (int i = 0; i < N * N; i++) {
                state.puzzle[i] = (bucketState.puzzle[i] > -1 ? bucketState.puzzle : dataState.puzzle)[i];
            }
        }

    protected:
        static std::vector<int> GetVector(bool isBucket) {
            if (N == 4)
                return isBucket ? BUCKET4 : DATA4;
            if (N == 5)
                return isBucket ? BUCKET5 : DATA5;
            return {};
        }

        MNPuzzle<N, N> env;
        MNPuzzleState<N, N> goal; // Goal has to be before the PDBs, so it will get initialized before them
        LexPermutationPDB<MNPuzzleState<N, N>, slideDir, MNPuzzle<N, N>> bucketPDB;
        LexPermutationPDB<MNPuzzleState<N, N>, slideDir, MNPuzzle<N, N>> dataPDB;
    };
}

#endif //HOG2_SPL_MNPUZZLEHASH_H

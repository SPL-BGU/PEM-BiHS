#ifndef HOG2_SPL_EMBHS_H
#define HOG2_SPL_EMBHS_H

#include <vector>
#include "MNPuzzle.h"
#include "LexPermutationPDB.h"
#include "PancakePuzzle.h"

namespace EMBHS {
    enum Verbosity {
        lNotSet = 0,
        lDebug = 10,
        lInfo = 20,
        lWarn = 30,
        lError = 40,
        lCritical = 50
    };

    template<int N>
    class StoredGapHeuristic : public Heuristic<PancakePuzzleState<N>> {
    public:
        StoredGapHeuristic(PancakePuzzleState<N> goal) {
            env.StoreGoal(goal);
        }

        double HCost(const PancakePuzzleState<N> &a, const PancakePuzzleState<N> &b) const override {
            return env.HCost(a);
        }

    protected:
        PancakePuzzle<N> env;
    };
}

#endif //HOG2_SPL_EMBHS_H

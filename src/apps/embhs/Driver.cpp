#include <iostream>
#include <numeric>
#include "MNPuzzle.h"
#include "SearchEnvironment.h"
#include "LexPermutationPDB.h"
#include "PEBAE.h"
#include "PEMM.h"
#include "PEMASTAR.h"
#include "MNPuzzleHash.h"
#include "TemplateAStar.h"
#include "STPInstances.h"
#include "IDAStar.h"
#include "BAE.h"
#include "EMBHS.h"
#include <sys/stat.h>
#include "PatternDatabases.h"
#include "ParallelIDAStar.h"
#include "TOH.h"
#include "TOHHash.h"

#define RUN_PROBLEM_STP(x) RunProblem<MNPuzzleState<x, x>, slideDir, MNPuzzle<x, x>, EMBHS::STPHash<x>>

//<editor-fold desc="Problem Constants">
const int KORF_OPTIMAL_SOLUTIONS[100] = {57, 55, 59, 56, 56, 52, 52, 50, 46, 59,
                                         57, 45, 46, 59, 62, 42, 66, 55, 46, 52,
                                         54, 59, 49, 54, 52, 58, 53, 52, 54, 47,
                                         50, 59, 60, 52, 55, 52, 58, 53, 49, 54,
                                         54, 42, 64, 50, 51, 49, 47, 49, 59, 53,
                                         56, 56, 64, 56, 41, 55, 50, 51, 57, 66,
                                         45, 57, 56, 51, 47, 61, 50, 51, 53, 52,
                                         44, 56, 49, 56, 48, 57, 54, 53, 42, 57,
                                         53, 62, 49, 55, 44, 45, 52, 65, 54, 50,
                                         57, 57, 46, 53, 50, 49, 44, 54, 57, 54};

const int ARIEL_INSTANCES[50][25] = {
        {14, 5,  9,  2,  18, 8,  23, 19, 12, 17, 15, 0,  10, 20, 4,  6,  11, 21, 1,  7,  24, 3,  16, 22, 13},
        {16, 5,  1,  12, 6,  24, 17, 9,  2,  22, 4,  10, 13, 18, 19, 20, 0,  23, 7,  21, 15, 11, 8,  3,  14},
        {6,  0,  24, 14, 8,  5,  21, 19, 9,  17, 16, 20, 10, 13, 2,  15, 11, 22, 1,  3,  7,  23, 4,  18, 12},
        {18, 14, 0,  9,  8,  3,  7,  19, 2,  15, 5,  12, 1,  13, 24, 23, 4,  21, 10, 20, 16, 22, 11, 6,  17},
        {17, 1,  20, 9,  16, 2,  22, 19, 14, 5,  15, 21, 0,  3,  24, 23, 18, 13, 12, 7,  10, 8,  6,  4,  11},
        {2,  0,  10, 19, 1,  4,  16, 3,  15, 20, 22, 9,  6,  18, 5,  13, 12, 21, 8,  17, 23, 11, 24, 7,  14},
        {21, 22, 15, 9,  24, 12, 16, 23, 2,  8,  5,  18, 17, 7,  10, 14, 13, 4,  0,  6,  20, 11, 3,  1,  19},
        {7,  13, 11, 22, 12, 20, 1,  18, 21, 5,  0,  8,  14, 24, 19, 9,  4,  17, 16, 10, 23, 15, 3,  2,  6},
        {3,  2,  17, 0,  14, 18, 22, 19, 15, 20, 9,  7,  10, 21, 16, 6,  24, 23, 8,  5,  1,  4,  11, 12, 13},
        {23, 14, 0,  24, 17, 9,  20, 21, 2,  18, 10, 13, 22, 1,  3,  11, 4,  16, 6,  5,  7,  12, 8,  15, 19},
        {15, 11, 8,  18, 14, 3,  19, 16, 20, 5,  24, 2,  17, 4,  22, 10, 1,  13, 9,  21, 23, 7,  6,  12, 0},
        {12, 23, 9,  18, 24, 22, 4,  0,  16, 13, 20, 3,  15, 6,  17, 8,  7,  11, 19, 1,  10, 2,  14, 5,  21},
        {21, 24, 8,  1,  19, 22, 12, 9,  7,  18, 4,  0,  23, 14, 10, 6,  3,  11, 16, 5,  15, 2,  20, 13, 17},
        {24, 1,  17, 10, 15, 14, 3,  13, 8,  0,  22, 16, 20, 7,  21, 4,  12, 9,  2,  11, 5,  23, 6,  18, 19},
        {24, 10, 15, 9,  16, 6,  3,  22, 17, 13, 19, 23, 21, 11, 18, 0,  1,  2,  7,  8,  20, 5,  12, 4,  14},
        {18, 24, 17, 11, 12, 10, 19, 15, 6,  1,  5,  21, 22, 9,  7,  3,  2,  16, 14, 4,  20, 23, 0,  8,  13},
        {23, 16, 13, 24, 5,  18, 22, 11, 17, 0,  6,  9,  20, 7,  3,  2,  10, 14, 12, 21, 1,  19, 15, 8,  4},
        {0,  12, 24, 10, 13, 5,  2,  4,  19, 21, 23, 18, 8,  17, 9,  22, 16, 11, 6,  15, 7,  3,  14, 1,  20},
        {16, 13, 6,  23, 9,  8,  3,  5,  24, 15, 22, 12, 21, 17, 1,  19, 10, 7,  11, 4,  18, 2,  14, 20, 0},
        {4,  5,  1,  23, 21, 13, 2,  10, 18, 17, 15, 7,  0,  9,  3,  14, 11, 12, 19, 8,  6,  20, 24, 22, 16},
        {24, 8,  14, 5,  16, 4,  13, 6,  22, 19, 1,  10, 9,  12, 3,  0,  18, 21, 20, 23, 15, 17, 11, 7,  2},
        {7,  6,  3,  22, 15, 19, 21, 2,  13, 0,  8,  10, 9,  4,  18, 16, 11, 24, 5,  12, 17, 1,  23, 14, 20},
        {24, 11, 18, 7,  3,  17, 5,  1,  23, 15, 21, 8,  2,  4,  19, 14, 0,  16, 22, 6,  9,  13, 20, 12, 10},
        {14, 24, 18, 12, 22, 15, 5,  1,  23, 11, 6,  19, 10, 13, 7,  0,  3,  9,  4,  17, 2,  21, 16, 20, 8},
        {3,  17, 9,  8,  24, 1,  11, 12, 14, 0,  5,  4,  22, 13, 16, 21, 15, 6,  7,  10, 20, 23, 2,  18, 19},
        {22, 21, 15, 3,  14, 13, 9,  19, 24, 23, 16, 0,  7,  10, 18, 4,  11, 20, 8,  2,  1,  6,  5,  17, 12},
        {9,  19, 8,  20, 2,  3,  14, 1,  24, 6,  13, 18, 7,  10, 17, 5,  22, 12, 21, 16, 15, 0,  23, 11, 4},
        {17, 15, 7,  12, 8,  3,  4,  9,  21, 5,  16, 6,  19, 20, 1,  22, 24, 18, 11, 14, 23, 10, 2,  13, 0},
        {10, 3,  6,  13, 1,  2,  20, 14, 18, 11, 15, 7,  5,  12, 9,  24, 17, 22, 4,  8,  21, 23, 19, 16, 0},
        {8,  19, 7,  16, 12, 2,  13, 22, 14, 9,  11, 5,  6,  3,  18, 24, 0,  15, 10, 23, 1,  20, 4,  17, 21},
        {19, 20, 12, 21, 7,  0,  16, 10, 5,  9,  14, 23, 3,  11, 4,  2,  6,  1,  8,  15, 17, 13, 22, 24, 18},
        {1,  12, 18, 13, 17, 15, 3,  7,  20, 0,  19, 24, 6,  5,  21, 11, 2,  8,  9,  16, 22, 10, 4,  23, 14},
        {11, 22, 6,  21, 8,  13, 20, 23, 0,  2,  15, 7,  12, 18, 16, 3,  1,  17, 5,  4,  9,  14, 24, 10, 19},
        {5,  18, 3,  21, 22, 17, 13, 24, 0,  7,  15, 14, 11, 2,  9,  10, 1,  8,  6,  16, 19, 4,  20, 23, 12},
        {2,  10, 24, 11, 22, 19, 0,  3,  8,  17, 15, 16, 6,  4,  23, 20, 18, 7,  9,  14, 13, 5,  12, 1,  21},
        {2,  10, 1,  7,  16, 9,  0,  6,  12, 11, 3,  18, 22, 4,  13, 24, 20, 15, 8,  14, 21, 23, 17, 19, 5},
        {23, 22, 5,  3,  9,  6,  18, 15, 10, 2,  21, 13, 19, 12, 20, 7,  0,  1,  16, 24, 17, 4,  14, 8,  11},
        {10, 3,  24, 12, 0,  7,  8,  11, 14, 21, 22, 23, 2,  1,  9,  17, 18, 6,  20, 4,  13, 15, 5,  19, 16},
        {16, 24, 3,  14, 5,  18, 7,  6,  4,  2,  0,  15, 8,  10, 20, 13, 19, 9,  21, 11, 17, 12, 22, 23, 1},
        {2,  17, 4,  13, 7,  12, 10, 3,  0,  16, 21, 24, 8,  5,  18, 20, 15, 19, 14, 9,  22, 11, 6,  1,  23},
        {13, 19, 9,  10, 14, 15, 23, 21, 24, 16, 12, 11, 0,  5,  22, 20, 4,  18, 3,  1,  6,  2,  7,  17, 8},
        {16, 6,  20, 18, 23, 19, 7,  11, 13, 17, 12, 9,  1,  24, 3,  22, 2,  21, 10, 4,  8,  15, 14, 5,  0},
        {7,  4,  19, 12, 16, 20, 15, 23, 8,  10, 1,  18, 2,  17, 14, 24, 9,  5,  0,  21, 6,  3,  11, 13, 22},
        {8,  12, 18, 3,  2,  11, 10, 22, 24, 17, 1,  13, 23, 4,  20, 16, 6,  15, 9,  21, 19, 5,  14, 0,  7},
        {9,  7,  16, 18, 12, 1,  23, 8,  22, 0,  6,  19, 4,  13, 2,  24, 11, 15, 21, 17, 20, 3,  10, 14, 5},
        {1,  16, 10, 14, 17, 13, 0,  3,  5,  7,  4,  15, 19, 2,  21, 9,  23, 8,  12, 6,  11, 24, 22, 20, 18},
        {21, 11, 10, 4,  16, 6,  13, 24, 7,  14, 1,  20, 9,  17, 0,  15, 2,  5,  8,  22, 3,  12, 18, 19, 23},
        {2,  22, 21, 0,  23, 8,  14, 20, 12, 7,  16, 11, 3,  5,  1,  15, 4,  9,  24, 10, 13, 6,  19, 17, 18},
        {2,  21, 3,  7,  0,  8,  5,  14, 18, 6,  12, 11, 23, 20, 10, 15, 17, 4,  9,  16, 13, 19, 24, 22, 1},
        {23, 1,  12, 6,  16, 2,  20, 10, 21, 18, 14, 13, 17, 19, 22, 0,  15, 24, 3,  7,  4,  8,  5,  9,  11}
};

const int ARIEL_OPTIMAL_SOLUTIONS[50] = {95, 96, 97, 98, 100, 101, 104, 108, 113, 114,
                                         106, 109, 101, 111, 103, 96, 109, 110, 106, 92,
                                         103, 95, 104, 107, 81, 105, 99, 98, 88, 92,
                                         99, 97, 106, 102, 98, 90, 100, 96, 104, 82,
                                         106, 108, 104, 93, 101, 100, 92, 107, 100, 113};
//</editor-fold>





template<class Tstate, class Taction, class Tenvironment, class Thash>
int RunProblem(Tstate start, Tstate goal,
               Heuristic<Tstate> *hForward, Heuristic<Tstate> *hBackward,
               EMBHS::Verbosity verbosity, const std::string &algorithm, const std::string &pebae_prefix,
               int numThreadsExpansion, int numThreadsReading) {

    int solutionCost = -1;
    uint64_t nodesExpanded = 0;
    uint64_t necessaryExpanded = 0;
    uint64_t lastBucketSize = 0;
    uint64_t nodesGenerated = 0;
    size_t maxMemory = 0;
    Tenvironment env;
    std::vector<Tstate> solutionPath;
    Timer t;

    if (algorithm == "PEBAE") {
        // Clear and make the directory to store all the PE-BAE* files
        system(("rm -rf " + pebae_prefix).c_str());
        system(("mkdir " + pebae_prefix).c_str());
        EMBHS::PEBAE::PEBAE<Tstate, Taction, Tenvironment, Thash> pebae(pebae_prefix, verbosity, numThreadsExpansion,
                                                                        numThreadsReading);
        pebae.SetHeuristics(hForward, hBackward);

        t.StartTimer();
        pebae.GetPath(start, goal);
        t.EndTimer();

        solutionCost = pebae.GetPathLength();
        nodesExpanded = pebae.GetNodesExpanded();
        necessaryExpanded = pebae.GetNecessaryExpanded();
        lastBucketSize = pebae.GetLastBucketSize();
        nodesGenerated = pebae.GetNodesGenerated();
        maxMemory = pebae.GetMaxMemory();

    } else if (algorithm == "PEMM") {
        // Clear and make the directory to store all the PE-BAE* files
        system(("rm -rf " + pebae_prefix).c_str());
        system(("mkdir " + pebae_prefix).c_str());
        EMBHS::PEMM::PEMM<Tstate, Taction, Tenvironment, Thash> pemm(pebae_prefix, verbosity);
        pemm.SetHeuristics(hForward, hBackward);

        t.StartTimer();
        pemm.GetPath(start, goal);
        t.EndTimer();

        solutionCost = pemm.GetPathLength();
        nodesExpanded = pemm.GetNodesExpanded();
        necessaryExpanded = pemm.GetNecessaryExpanded();
        lastBucketSize = pemm.GetLastBucketSize();
        nodesGenerated = pemm.GetNodesGenerated();

    } else if (algorithm == "PEMASTAR") {
        // Clear and make the directory to store all the PE-BAE* files
        system(("rm -rf " + pebae_prefix).c_str());
        system(("mkdir " + pebae_prefix).c_str());
        EMBHS::PEMASTAR::PEMASTAR<Tstate, Taction, Tenvironment, Thash> pemastar(
                pebae_prefix,
                verbosity);
        pemastar.SetHeuristics(hForward);

        t.StartTimer();
        pemastar.GetPath(start, goal);
        t.EndTimer();

        solutionCost = pemastar.GetPathLength();
        nodesExpanded = pemastar.GetNodesExpanded();
        necessaryExpanded = pemastar.GetNecessaryExpanded();
        lastBucketSize = pemastar.GetLastBucketSize();
        nodesGenerated = pemastar.GetNodesGenerated();

    } else if (algorithm == "RPEMASTAR") {
        // Clear and make the directory to store all the PE-BAE* files
        system(("rm -rf " + pebae_prefix).c_str());
        system(("mkdir " + pebae_prefix).c_str());
        EMBHS::PEMASTAR::PEMASTAR<Tstate, Taction, Tenvironment, Thash> pemastar(
                pebae_prefix,
                verbosity);
        pemastar.SetHeuristics(hBackward);

        t.StartTimer();
        pemastar.GetPath(goal, start);
        t.EndTimer();

        solutionCost = pemastar.GetPathLength();
        nodesExpanded = pemastar.GetNodesExpanded();
        necessaryExpanded = pemastar.GetNecessaryExpanded();
        lastBucketSize = pemastar.GetLastBucketSize();
        nodesGenerated = pemastar.GetNodesGenerated();

    } else if (algorithm == "A") {
        TemplateAStar<Tstate, Taction, Tenvironment> astar;
        if (hForward != nullptr)
            astar.SetHeuristic(hForward);

        t.StartTimer();
        astar.GetPath(&env, start, goal, solutionPath);
        t.EndTimer();

        solutionCost = env.GetPathLength(solutionPath);
        nodesExpanded = astar.GetNodesExpanded();

    } else if (algorithm == "RA") {
        TemplateAStar<Tstate, Taction, Tenvironment> astar;
        if (hBackward != nullptr)
            astar.SetHeuristic(hBackward);

        t.StartTimer();
        astar.GetPath(&env, goal, start, solutionPath);
        t.EndTimer();

        solutionCost = env.GetPathLength(solutionPath);
        nodesExpanded = astar.GetNodesExpanded();

    } else if (algorithm == "IDA") {
        IDAStar<Tstate, Taction> idastar;
        if (hForward != nullptr)
            idastar.SetHeuristic(hForward);

        t.StartTimer();
        idastar.GetPath(&env, start, goal, solutionPath);
        t.EndTimer();

        solutionCost = env.GetPathLength(solutionPath);
        nodesExpanded = idastar.GetNodesExpanded();
        nodesGenerated = idastar.GetNodesTouched();
    } else if (algorithm == "RIDA") {
        IDAStar<Tstate, Taction> idastar;
        if (hBackward != nullptr)
            idastar.SetHeuristic(hBackward);

        t.StartTimer();
        idastar.GetPath(&env, goal, start, solutionPath);
        t.EndTimer();

        solutionCost = env.GetPathLength(solutionPath);
        nodesExpanded = idastar.GetNodesExpanded();
        nodesGenerated = idastar.GetNodesTouched();
    } else if (algorithm == "BAE") {
        EMBHS::BAE::BAE<Tstate, Taction, Tenvironment> bae;
        if (hForward == nullptr)
            hForward = &env;
        if (hBackward == nullptr)
            hBackward = &env;

        t.StartTimer();
        bae.GetPath(&env, start, goal, hForward, hBackward, solutionPath);
        t.EndTimer();

        solutionCost = env.GetPathLength(solutionPath);
        nodesExpanded = bae.GetNodesExpanded();
    } else if (algorithm == "PIDA") {
        ParallelIDAStar<Tenvironment, Tstate, Taction> ida;
        if (hForward == nullptr)
            hForward = &env;
        std::vector<Taction> actionSolutionPath;
        ida.SetHeuristic(hForward);
        t.StartTimer();
        ida.GetPath(&env, start, goal, actionSolutionPath);
        t.EndTimer();

        //Since STP is a unit-cost domain, the path length is the number of actions taken
        solutionCost = (int) actionSolutionPath.size();
        nodesExpanded = ida.GetNodesExpanded();
        necessaryExpanded = ida.GetNecessaryExpanded();
        nodesGenerated = ida.GetNodesTouched();
    } else if (algorithm == "RPIDA") {
        ParallelIDAStar<Tenvironment, Tstate, Taction> ida;
        if (hBackward == nullptr)
            hBackward = &env;
        std::vector<Taction> actionSolutionPath;
        ida.SetHeuristic(hBackward);
        t.StartTimer();
        ida.GetPath(&env, goal, start, actionSolutionPath);
        t.EndTimer();

        //Since STP is a unit-cost domain, the path length is the number of actions taken
        solutionCost = (int) actionSolutionPath.size();
        nodesExpanded = ida.GetNodesExpanded();
        necessaryExpanded = ida.GetNecessaryExpanded();
        nodesGenerated = ida.GetNodesTouched();
    }

    std::cout << "Time to finish: " << t.GetElapsedTime() << std::endl;
    std::cout << "Nodes expanded: " << nodesExpanded << std::endl;
    std::cout << "Necessary expanded: " << necessaryExpanded << std::endl;
    std::cout << "Last Bucket Size: " << lastBucketSize << std::endl;
    std::cout << "Nodes Generated: " << nodesGenerated << std::endl;
    std::cout << "Max Memory: " << maxMemory << std::endl;


    return solutionCost;
}

MNPuzzleState<4, 4> GetKorfProblem(int num) {
    assert(num > -1 && num < 100);
    return STP::GetKorfInstance(num);
}

int GetKorfSolutionCost(int num) {
    assert(num > -1 && num < 100);
    return KORF_OPTIMAL_SOLUTIONS[num];
}

MNPuzzleState<5, 5> GetArielProblem(int num) {
    assert(num > -1 && num < 50);
    MNPuzzleState<5, 5> state;
    for (int i = 0; i < state.puzzle.size(); i++) {
        if (ARIEL_INSTANCES[num][i] == 0)
            state.blank = i;
        state.puzzle[i] = ARIEL_INSTANCES[num][i];
    }
    return state;
}

int GetArielSolutionCost(int num) {
    assert(num > -1 && num < 50);
    return ARIEL_OPTIMAL_SOLUTIONS[num];
}


EMBHS::Verbosity GetVerbosity(const std::string &arg) {
    if (arg == "d" || arg == "debug")
        return EMBHS::lDebug;
    if (arg == "i" || arg == "info")
        return EMBHS::lInfo;
    if (arg == "w" || arg == "warn")
        return EMBHS::lWarn;
    if (arg == "e" || arg == "error")
        return EMBHS::lError;
    if (arg == "c" || arg == "critical")
        return EMBHS::lCritical;
    std::cerr << "Unknown verbosity level" << std::endl;
    exit(-1);
}

void RunKorfInstance(int num, EMBHS::Verbosity verbosity, const std::string &algorithm, const std::string &pdb_prefix,
                     const std::string &pebae_prefix, int numThreadsPdb, int numThreadsExpansion,
                     int numThreadsReading) {
    MNPuzzleState<4, 4> start = GetKorfProblem(num);
    MNPuzzleState<4, 4> goal;
    goal.Reset();

    Heuristic<MNPuzzleState<4, 4>> *h2Forward = nullptr;
    Heuristic<MNPuzzleState<4, 4>> *h2Backward = nullptr;

    if (!pdb_prefix.empty()) {
        h2Forward = EMBHS::GenerateStpPDB(goal, pdb_prefix, numThreadsPdb);
        h2Backward = EMBHS::GenerateStpPDB(start, pdb_prefix, numThreadsPdb);
    }
    int sc = RUN_PROBLEM_STP(4)(start, goal, h2Forward, h2Backward, verbosity, algorithm, pebae_prefix,
                                numThreadsExpansion, numThreadsReading);

    std::cout << "SC: " << sc << " TC: " << GetKorfSolutionCost(num) << std::endl;
    assert(sc == GetKorfSolutionCost(num));
    if (!pdb_prefix.empty()) {
        EMBHS::DeletePDB(h2Forward);
        EMBHS::DeletePDB(h2Backward);
    }
}

void RunArielInstance(int num, EMBHS::Verbosity verbosity, const std::string &algorithm, const std::string &pdb_prefix,
                      const std::string &pebae_prefix, int numThreadsPdb, int numThreadsExpansion,
                      int numThreadsReading) {
    MNPuzzleState<5, 5> start = GetArielProblem(num);
    MNPuzzleState<5, 5> goal;
    goal.Reset();
    Heuristic<MNPuzzleState<5, 5>> *h2Forward = nullptr;
    Heuristic<MNPuzzleState<5, 5>> *h2Backward = nullptr;
    if (!pdb_prefix.empty()) {
        h2Forward = EMBHS::GenerateStpPDB(goal, pdb_prefix, numThreadsPdb);
        h2Backward = EMBHS::GenerateStpPDB(start, pdb_prefix, numThreadsPdb);
    }
    int sc = RUN_PROBLEM_STP(5)(start, goal, h2Forward, h2Backward, verbosity, algorithm, pebae_prefix,
                                numThreadsExpansion, numThreadsReading);

    std::cout << "SC: " << sc << " TC: " << GetArielSolutionCost(num) << std::endl;
    assert(sc == GetArielSolutionCost(num));
    if (!pdb_prefix.empty()) {
        EMBHS::DeletePDB(h2Forward);
        EMBHS::DeletePDB(h2Backward);
    }
}


// This is a helper function that creates a 4X4 state given its tiles in a line seperated by spaces
template<int N>
void GetInstanceFromString(MNPuzzleState<N, N> &state, const std::string &str) {
    std::istringstream is(str);
    int n;
    int i = 0;
    while (is >> n) {
        state.puzzle[i] = n;
        if (n == 0)
            state.blank = i;
        i++;
    }
}

// This is a helper function to run the 5X5 puzzles with the PDBs with reflection.
// This was created out of laziness instead of probably redesigning the experiment call
void RunStp5(int ariel_instance, std::string &algoritm, const std::string &pebae_prefix,
             const std::string &pdb_prefix,
             uint64_t numThreadsExpansion = std::thread::hardware_concurrency(),
             uint64_t numThreadsReading = std::thread::hardware_concurrency()) {
    MNPuzzleState<5, 5> start = GetArielProblem(ariel_instance);
    MNPuzzleState<5, 5> goal;
    Heuristic<MNPuzzleState<5, 5>> *hForward = nullptr;
    Heuristic<MNPuzzleState<5, 5>> *hBackward = nullptr;
    if (algoritm == "PIDA" || algoritm == "PEBAE") {
        hForward = new EMBHS::CanonicalReflectionPDB5(EMBHS::GenerateStpPDB(goal, pdb_prefix, -1));
    }

    if (algoritm == "RPIDA" || algoritm == "PEBAE") {
        hBackward = EMBHS::GenerateStpDoublePDB(start, pdb_prefix);
    }

    int solutionCost = -1;
    uint64_t nodesExpanded = 0;
    uint64_t necessaryExpanded = 0;
    uint64_t lastBucketSize = 0;
    uint64_t nodesGenerated = 0;
    uint64_t maxMemory = 0;
    MNPuzzle<5, 5> env;
    Timer t;

    if (algoritm == "PIDA") {
        ParallelIDAStar<MNPuzzle<5, 5>, MNPuzzleState<5, 5>, slideDir> ida;
        std::vector<slideDir> actionSolutionPath;
        ida.SetHeuristic(hForward);
        ida.SetStopAfterSolution(true);
        t.StartTimer();
        ida.GetPath(&env, start, goal, actionSolutionPath, numThreadsExpansion);
        t.EndTimer();

        //Since STP is a unit-cost domain, the path length is the number of actions taken
        solutionCost = (int) actionSolutionPath.size();
        nodesExpanded = ida.GetNodesExpanded();
        necessaryExpanded = ida.GetNecessaryExpanded();
        nodesGenerated = ida.GetNodesTouched();
    } else if (algoritm == "RPIDA") {
        ParallelIDAStar<MNPuzzle<5, 5>, MNPuzzleState<5, 5>, slideDir> ida;
        std::vector<slideDir> actionSolutionPath;
        ida.SetHeuristic(hBackward);
        ida.SetStopAfterSolution(true);
        t.StartTimer();
        ida.GetPath(&env, goal, start, actionSolutionPath, numThreadsExpansion);
        t.EndTimer();

        //Since STP is a unit-cost domain, the path length is the number of actions taken
        solutionCost = (int) actionSolutionPath.size();
        nodesExpanded = ida.GetNodesExpanded();
        necessaryExpanded = ida.GetNecessaryExpanded();
        nodesGenerated = ida.GetNodesTouched();
    } else if (algoritm == "PEBAE") {
        // Clear and make the directory to store all the PE-BAE* files
        system(("rm -rf " + pebae_prefix).c_str());
        system(("mkdir " + pebae_prefix).c_str());
        EMBHS::PEBAE::PEBAE<MNPuzzleState<5, 5>, slideDir, MNPuzzle<5, 5>, EMBHS::STPHash<5>> pebae(pebae_prefix,
                                                                                                    EMBHS::lInfo,
                                                                                                    (int) numThreadsExpansion,
                                                                                                    (int) numThreadsReading);
        pebae.SetHeuristics(hForward, hBackward);

        t.StartTimer();
        pebae.GetPath(start, goal);
        t.EndTimer();

        solutionCost = pebae.GetPathLength();
        nodesExpanded = pebae.GetNodesExpanded();
        necessaryExpanded = pebae.GetNecessaryExpanded();
        lastBucketSize = pebae.GetLastBucketSize();
        nodesGenerated = pebae.GetNodesGenerated();
        maxMemory = pebae.GetMaxMemory();
    } else {
        std::cerr << "Unknown algorithm" << std::endl;
        exit(-1);
    }

    std::cout << "Time to finish: " << t.GetElapsedTime() << std::endl;
    std::cout << "Nodes expanded: " << nodesExpanded << std::endl;
    std::cout << "Necessary expanded: " << necessaryExpanded << std::endl;
    std::cout << "Last Bucket Size: " << lastBucketSize << std::endl;
    std::cout << "Nodes Generated: " << nodesGenerated << std::endl;
    std::cout << "Max Memory: " << maxMemory << std::endl;
    std::cout << "SC: " << solutionCost << std::endl;

    if (algoritm == "PIDA" || algoritm == "PEBAE") {
        delete hForward;
    }

    if (algoritm == "RPIDA" || algoritm == "PEBAE") {
        for (auto &temp: hBackward->heuristics)
            delete temp;
        delete hBackward;
    }

    assert(solutionCost == GetArielSolutionCost(ariel_instance));
}


template<int N, int T, int B = N - T>
void RunTemplatedToh(int num, EMBHS::Verbosity verbosity, const std::string &algorithm, const std::string &pdb_prefix,
                     const std::string &pebae_prefix, int numThreadsPdb, int numThreadsExpansion,
                     int numThreadsReading) {
    TOH<N> toh;
    TOHState<N> start;
    TOHState<N> goal;

    int table[] = {52058078, 116173544, 208694125, 131936966, 141559500, 133800745, 194246206, 50028346, 167007978,
                   207116816, 163867037, 119897198, 201847476, 210859515, 117688410, 121633885};
    int table2[] = {145008714, 165971878, 154717942, 218927374, 182772845, 5808407, 19155194, 137438954, 13143598,
                    124513215, 132635260, 39667704, 2462244, 41006424, 214146208, 54305743};

    srandom(table[num & 0xF] ^ table2[(num >> 4) & 0xF]);

    start.counts[0] = start.counts[1] = start.counts[2] = start.counts[3] = 0;
    for (int x = N; x > 0; x--) {
        int whichPeg = random() % 4;
        start.disks[whichPeg][start.counts[whichPeg]] = x;
        start.counts[whichPeg]++;
    }

    goal.counts[0] = goal.counts[1] = goal.counts[2] = goal.counts[3] = 0;
    for (int x = N; x > 0; x--) {
        int whichPeg = random() % 4;
        goal.disks[whichPeg][goal.counts[whichPeg]] = x;
        goal.counts[whichPeg]++;
    }

    std::cout << "Start: " << start << std::endl;
    std::cout << "Goal: " << goal << std::endl;

    Heuristic<TOHState<N>> *hForward = EMBHS::BuildTohPDB<N, 4>(pdb_prefix, goal);
    Heuristic<TOHState<N>> *hBackward = EMBHS::BuildTohPDB<N, 4>(pdb_prefix, start);

    int sc = RunProblem<TOHState<N>, TOHMove, TOH<N>, EMBHS::TOHHash<N>>(start, goal, hForward, hBackward, verbosity,
                                                                         algorithm, pebae_prefix, numThreadsExpansion,
                                                                         numThreadsReading);
    std::cout << "SC: " << sc << std::endl;
}

int main(int argc, char *argv[]) {
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout << std::setprecision(3);
    // This is a list of the supported algorithms
    const std::vector<std::string> ALGORITHMS = {"BAE", "A", "RA", "IDA", "RIDA",
                                                 "PEBAE", "PEMM", "PEMASTAR", "RPEMASTAR", "PIDA", "RPIDA"};
    std::vector<std::string> args(argv, argv + argc);
    EMBHS::Verbosity verbosity = EMBHS::lInfo;
    int korf_instance = -1;
    int ariel_instance = -1;
    std::string algorithm = "PEBAE";
    std::string pdb_path;
    std::string pebae_path = "pebae_files/";
    bool stp = false;
    int toh_instance = -2;
    int toh_num_disks = -1;
    int toh_top_pdb_size = -1;

    // Before defaulting to this, make sure that max_threads in PDB is smaller than this, otherwise there will be
    // concurrency issues
    int numThreadsPdb = (int) std::thread::hardware_concurrency();

    int numThreadsExpansion = (int) std::thread::hardware_concurrency();
    int numThreadsReading = (int) std::thread::hardware_concurrency();

    for (int i = 1; i < args.size(); i++) {
        // Verbosity of algorithms. Generally warn or higher is silent, info gives progress information about the
        // algorithm, and debug is for debugging purposes.
        if (args[i] == "-v" || args[i] == "--verbose") {
            i++;
            verbosity = GetVerbosity(args[i]);
        }
            // 100 4X4 instances. Here instances begin at 0 (and not 1 like the paper)
        else if (args[i] == "-k" || args[i] == "--korf") {
            i++;
            korf_instance = std::stoi(args[i]);
        }
            // 50 5X5 instances. Here instances begin at 0 (and not 1 like the paper)
        else if (args[i] == "-a" || args[i] == "--ariel") {
            i++;
            ariel_instance = std::stoi(args[i]);
        }
            // If pdb is set, the algorithm will use PDB instead of the environment's builtin heuristic
        else if (args[i] == "-p" || args[i] == "--pdb") {
            i++;
            pdb_path = args[i];
        }
            // Algorithm name
        else if (args[i] == "-al" || args[i] == "--algorithm") {
            i++;
            algorithm = args[i];
        }
            // This is the prefix for where to store the PEBAE files in case the chosen algorithm is PEBAE
        else if (args[i] == "-pe" || args[i] == "--pebae") {
            i++;
            pebae_path = args[i];
            if (pebae_path.back() != '/' && pebae_path[0] != 0)
                pebae_path += '/';
        }
            // Number of threads to create the PDB.
        else if (args[i] == "-tp" || args[i] == "--tpdb") {
            i++;
            numThreadsPdb = stoi(args[i]);
            // Number of threads to read files into memory.
        } else if (args[i] == "-te" || args[i] == "--texpansion") {
            i++;
            numThreadsExpansion = stoi(args[i]);
            // Number of threads to expand buckets
        } else if (args[i] == "-tr" || args[i] == "--treading") {
            i++;
            numThreadsReading = stoi(args[i]);
        } else if (args[i] == "--stp") {
            stp = true;
        } else if (args[i] == "--toh") {
            i++;
            toh_num_disks = std::stoi(args[i]);
            i++;
            toh_top_pdb_size = std::stoi(args[i]);
            i++;
            toh_instance = std::stoi(args[i]);
        } else {
            std::cerr << "Unhandled argument" << std::endl;
            exit(-1);
        }
    }

    if (toh_instance > -2) {
        if (toh_num_disks <= 0 || toh_top_pdb_size <= 0) {
            std::cerr << "ToH wrong arguments, need '--toh [numDisks] [topPdbSize] [instance]'" << std::endl;
            exit(-1);
        }
        if (toh_num_disks == 20) {
            if (toh_top_pdb_size == 5) {
                RunTemplatedToh<20, 5>(toh_instance, verbosity, algorithm, pdb_path, pebae_path, numThreadsPdb,
                                       numThreadsExpansion, numThreadsReading);
                return 0;
            } else if (toh_top_pdb_size == 4) {
                RunTemplatedToh<20, 4>(toh_instance, verbosity, algorithm, pdb_path, pebae_path, numThreadsPdb,
                                       numThreadsExpansion, numThreadsReading);
                return 0;
            } else {
                std::cerr << "Unhandled ToH PDB size" << std::endl;
                exit(-1);
            }
        } else if (toh_num_disks == 18) {
            if (toh_top_pdb_size == 4) {
                RunTemplatedToh<18, 4>(toh_instance, verbosity, algorithm, pdb_path, pebae_path, numThreadsPdb,
                                       numThreadsExpansion, numThreadsReading);
                return 0;
            } else if (toh_top_pdb_size == 3) {
                RunTemplatedToh<18, 3>(toh_instance, verbosity, algorithm, pdb_path, pebae_path, numThreadsPdb,
                                       numThreadsExpansion, numThreadsReading);
                return 0;
            } else {
                std::cerr << "Unhandled ToH PDB size" << std::endl;
                exit(-1);
            }
        } else if (toh_num_disks == 12) {
            if (toh_top_pdb_size == 2) {
                RunTemplatedToh<12, 2>(toh_instance, verbosity, algorithm, pdb_path, pebae_path, numThreadsPdb,
                                       numThreadsExpansion, numThreadsReading);
                return 0;
            } else {
                std::cerr << "Unhandled ToH PDB size" << std::endl;
                exit(-1);
            }
        } else {
            std::cerr << "Unhandled ToH size" << std::endl;
        }
    }

    if (stp) {
        RunStp5(ariel_instance, algorithm, pebae_path, pdb_path, numThreadsExpansion, numThreadsReading);
        return 0;
    }

    if ((!(ariel_instance + 1) + !(korf_instance + 1)) != 1) {
        std::cerr << ariel_instance << " " << korf_instance << " " << std::endl;
        std::cerr << "Choose exactly one type of problem" << std::endl;
        exit(-1);
    }

    if (std::find(ALGORITHMS.begin(), ALGORITHMS.end(), algorithm) == ALGORITHMS.end()) {
        std::cerr << "Unknown algorithm" << std::endl;
        exit(-1);
    }

    // Note that in both cases, both the forward and backward PDBs will be loaded, even if the algorithm is unidirectional
    if (ariel_instance >= 0)
        RunArielInstance(ariel_instance, verbosity, algorithm, pdb_path, pebae_path, numThreadsPdb,
                         numThreadsExpansion,
                         numThreadsReading);
    else if (korf_instance >= 0)
        RunKorfInstance(korf_instance, verbosity, algorithm, pdb_path, pebae_path, numThreadsPdb,
                        numThreadsExpansion,
                        numThreadsReading);
    else {
        std::cerr << "Choose A problem" << std::endl;
        exit(-1);
    }
    return 0;
}

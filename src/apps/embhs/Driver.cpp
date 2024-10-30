#include <ios>
#include <iostream>
#include <iomanip>
#include <vector>
#include "Utils.h"
#include "MNPuzzle.h"
#include "STPInstances.h"
#include "PatternDatabases.h"
#include "BAE.h"
#include "TemplateAStar.h"
#include "IDAStar.h"
#include "ParallelIDAStar.h"
#include "PEBAE.h"
#include "PEMM.h"
#include "PEMASTAR.h"
#include <sys/stat.h>

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

MNPuzzleState<4, 4> GetStp4Problem(int num) {
    assert(num > -1 && num < 100);
    return STP::GetKorfInstance(num);
}

int GetStp4Solution(int num) {
    assert(num > -1 && num < 100);
    return KORF_OPTIMAL_SOLUTIONS[num];
}

MNPuzzleState<5, 5> GetStp5Problem(int num) {
    assert(num > -1 && num < 50);
    MNPuzzleState<5, 5> state;
    for (int i = 0; i < state.puzzle.size(); i++) {
        if (ARIEL_INSTANCES[num][i] == 0)
            state.blank = i;
        state.puzzle[i] = ARIEL_INSTANCES[num][i];
    }
    return state;
}

int GetStp5Solution(int num) {
    assert(num > -1 && num < 50);
    return ARIEL_OPTIMAL_SOLUTIONS[num];
}

void PrintHelp() {
    std::cout << "USAGE:\n";
    std::cout << "  program [OPTIONS]\n\n";
    std::cout << "OPTIONS:\n";
    std::cout << "  --domain <DOMAIN>           Specify the domain [STP4, STP5, TOH].\n";
    std::cout << "  --heuristic <HEURISTIC>     Set the heuristic method to use.\n";
    std::cout << "  --algorithm <ALG1,ALG2,...> List the algorithms to be used, separated by spaces.\n";
    std::cout << "  --instance <ID>             Set the starting instance ID (non-negative).\n";
    std::cout << "  --num-instances <N>         Number of instances to process (default: 1).\n";
    std::cout << "  --verbosity <LEVEL>         Set verbosity level:\n";
    std::cout << "                                d | debug     (10)\n";
    std::cout << "                                i | info      (20)\n";
    std::cout << "                                w | warn      (30)\n";
    std::cout << "                                e | error     (40)\n";
    std::cout << "                                c | critical  (50)\n";
    std::cout << "  --pdb-dir <DIR>       Directory for pattern databases.\n";
    std::cout << "  --temp-dir <DIR>      Directory for temporary files.\n";
    std::cout << "  --expansion-threads <N>     Set number of threads for expansion (default: hardware concurrency).\n";
    std::cout << "  --reading-threads <N>       Set number of threads for reading (default: hardware concurrency).\n";
    std::cout << "\nEXAMPLES:\n";
    std::cout << "  program --domain STP4 --heuristic MD --algorithm alg1 alg2 --instance 10 \\\n";
    std::cout << "          --num-instances 5 --verbosity info --threads-expansion 4\n";
    std::cout << "\nNOTES:\n";
    std::cout << "  - If using domain STP4, ensure (instance ID + num instances) <= 100.\n";
    std::cout << "  - For STP5, ensure (instance ID + num instances) <= 50.\n";
    std::cout << "  - Default verbosity is 'warn' if not specified.\n";
    std::cout << "  - Check hardware concurrency to determine default thread counts.\n";
    std::cout << "\n";
}

bool ParseArguments(EMBHS::ArgParameters &ap, const std::vector<std::string> &args) {
    if (args.size() == 1) {
        PrintHelp();
        return false;
    }

    for (int i = 1; i < args.size(); ++i) {
        if (args[i] == "-v" || args[i] == "--verbose") {
            ++i;
            ap.SetVerbosity(args[i]);
        } else if (args[i] == "-d" || args[i] == "--domain") {
            ++i;
            ap.domain = args[i];
        } else if (args[i] == "-a" || args[i] == "--algorithm") {
            do {
                ++i;
                ap.algs.push_back(args[i]);
            } while (i + 1 < args.size() && args[i + 1][0] != '-');
        } else if (args[i] == "-i" || args[i] == "--instance") {
            ++i;
            ap.instanceId = std::stoi(args[i]);

        } else if (args[i] == "-n" || args[i] == "--number") {
            ++i;
            ap.numOfInstances = std::stoi(args[i]);
        } else if (args[i] == "-t" || args[i] == "--temp-dir") {
            ++i;
            ap.tempDir = args[i];
            if (ap.tempDir.back() != '/') {
                ap.tempDir += '/';
            }
        } else if (args[i] == "-p" || args[i] == "--pdb-dir") {
            ++i;
            ap.pdbDir = args[i];
            if (ap.pdbDir.back() != '/') {
                ap.pdbDir += '/';
            }
        } else if (args[i] == "-e" || args[i] == "--expansion-threads") {
            ++i;
            ap.numThreadsExpansion = std::stoi(args[i]);
        } else if (args[i] == "-r" || args[i] == "--reading-threads") {
            ++i;
            ap.numThreadsReading = std::stoi(args[i]);
        } else if (args[i] == "-h" || args[i] == "--heuristic") {
            ++i;
            ap.heuristic = args[i];
        } else if (args[i] == "--help") {
            PrintHelp();
            return false;
        } else {
            std::cerr << "Unknown argument: " << args[i] << std::endl;
            return false;
        }
    }
    return true;
}

void HandleTempDir(const std::string &tempDir) {
    if (tempDir.empty()) {
        std::cerr << "Temp dir is not set despite using a PEM algorithm" << std::endl;
        exit(-1);
    }
    struct stat info{};
    if (stat(tempDir.c_str(), &info) == 0) {
        // If dir does not contain any subdirectories (as that means it might not be the
        // correct directory, or it contains data it should not), and only then delete it.
        if (system(("[ -z \"$(find " + tempDir + " -mindepth 1 -type d)\" ] && rm -r " + tempDir).c_str())) {
            std::cerr << "Temp dir file deletion failed" << std::endl;
            exit(-1);
        }
    }

    if (system(("mkdir -p " + tempDir).c_str())) {
        std::cerr << "Temp dir creation failed" << std::endl;
        exit(-1);
    }

}

template<class Tstate, class Taction, class Tenvironment, class Thash>
void RunProblem(Tstate start, Tstate goal, Heuristic<Tstate> *hForward, Heuristic<Tstate> *hBackward,
                const EMBHS::ArgParameters &ap) {
    Timer timer;
    std::vector<Tstate> solutionPath;
    std::vector<Taction> actionSolutionPath;
    Tenvironment env;

    if (ap.HasAlgorithm("BAE")) {
        std::cout << "[A] alg: BAE*; threads: 1\n";
        EMBHS::BAE::BAE<Tstate, Taction, Tenvironment> bae;
        timer.StartTimer();
        bae.GetPath(&env, start, goal, hForward, hBackward, solutionPath);
        timer.EndTimer();
        printf("[R] solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n",
               env.GetPathLength(solutionPath), bae.GetNodesExpanded(),
               bae.GetNodesTouched(), timer.GetElapsedTime());
    }

    if (ap.HasAlgorithm("A")) {
        std::cout << "[A] alg: A*; threads: 1\n";
        TemplateAStar<Tstate, Taction, Tenvironment> astar;
        astar.SetHeuristic(hForward);
        timer.StartTimer();
        astar.GetPath(&env, start, goal, solutionPath);
        timer.EndTimer();
        printf("[R] alg: %s; solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n", "A*",
               env.GetPathLength(solutionPath), astar.GetNodesExpanded(),
               astar.GetNodesTouched(), timer.GetElapsedTime());
    }

    if (ap.HasAlgorithm("RA")) {
        std::cout << "[A] alg: RA*; threads: 1\n";
        TemplateAStar<Tstate, Taction, Tenvironment> astar;
        astar.SetHeuristic(hBackward);
        timer.StartTimer();
        astar.GetPath(&env, goal, start, solutionPath);
        timer.EndTimer();
        printf("[R] alg: %s; solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n", "RA*",
               env.GetPathLength(solutionPath), astar.GetNodesExpanded(),
               astar.GetNodesTouched(), timer.GetElapsedTime());
    }

    if (ap.HasAlgorithm("IDA")) {
        std::cout << "[A] alg: IDA*; threads: 1\n";
        IDAStar<Tstate, Taction> idastar;
        idastar.SetHeuristic(hForward);
        timer.StartTimer();
        idastar.GetPath(&env, start, goal, solutionPath);
        timer.EndTimer();
        printf("[R] alg: %s; solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n", "IDA*",
               env.GetPathLength(solutionPath), idastar.GetNodesExpanded(),
               idastar.GetNodesTouched(), timer.GetElapsedTime());
    }

    if (ap.HasAlgorithm("RIDA")) {
        std::cout << "[A] alg: RIDA*; threads: 1\n";
        IDAStar<Tstate, Taction> idastar;
        idastar.SetHeuristic(hBackward);
        timer.StartTimer();
        idastar.GetPath(&env, goal, start, solutionPath);
        timer.EndTimer();
        printf("[R] alg: %s; solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n", "RIDA*",
               env.GetPathLength(solutionPath), idastar.GetNodesExpanded(),
               idastar.GetNodesTouched(), timer.GetElapsedTime());
    }

    if (ap.HasAlgorithm("AIDA")) {
        std::cout << "[A] alg: AIDA*; threads: " << ap.numThreadsExpansion << "\n";
        ParallelIDAStar<Tenvironment, Tstate, Taction> aida;
        aida.SetHeuristic(hForward);
        timer.StartTimer();
        aida.GetPath(&env, start, goal, actionSolutionPath, ap.numThreadsExpansion);
        timer.EndTimer();
        printf("[R] alg: %s; solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n", "AIDA*",
               env.GetPathLength(start, actionSolutionPath), aida.GetNodesExpanded(),
               aida.GetNodesTouched(), timer.GetElapsedTime());
    }

    if (ap.HasAlgorithm("RAIDA")) {
        std::cout << "[A] alg: RAIDA*; threads: " << ap.numThreadsExpansion << "\n";
        ParallelIDAStar<Tenvironment, Tstate, Taction> aida;
        aida.SetHeuristic(hBackward);
        timer.StartTimer();
        aida.GetPath(&env, goal, start, actionSolutionPath, ap.numThreadsExpansion);
        timer.EndTimer();
        printf("[R] alg: %s; solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n", "RAIDA*",
               env.GetPathLength(goal, actionSolutionPath), aida.GetNodesExpanded(),
               aida.GetNodesTouched(), timer.GetElapsedTime());
    }

    if (ap.HasAlgorithm("PEMBAE")) {
        std::cout << "[A] alg: PEM-BAE*; threads: " << ap.numThreadsExpansion << "\n";
        HandleTempDir(ap.tempDir);
        EMBHS::PEBAE::PEBAE<Tstate, Taction, Tenvironment, Thash> pembae(ap.tempDir, ap.verbosity,
                                                                         ap.numThreadsExpansion,
                                                                         ap.numThreadsReading);
        pembae.SetHeuristics(hForward, hBackward);
        timer.StartTimer();
        pembae.GetPath(start, goal);
        timer.EndTimer();
        printf("[R] alg: %s; solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n", "PEM-BAE*",
               (double) pembae.GetPathLength(), pembae.GetNodesExpanded(),
               pembae.GetNodesGenerated(), timer.GetElapsedTime());
    }

    if (ap.HasAlgorithm("PEMM")) {
        std::cout << "[A] alg: PEMM*; threads: " << ap.numThreadsExpansion << "\n";
        HandleTempDir(ap.tempDir);
        EMBHS::PEMM::PEMM<Tstate, Taction, Tenvironment, Thash> pemm(ap.tempDir, ap.verbosity);
        pemm.SetHeuristics(hForward, hBackward);
        timer.StartTimer();
        pemm.GetPath(start, goal);
        timer.EndTimer();
        printf("[R] alg: %s; solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n", "PEM-BAE*",
               (double) pemm.GetPathLength(), pemm.GetNodesExpanded(),
               pemm.GetNodesGenerated(), timer.GetElapsedTime());
    }
    if (ap.HasAlgorithm("PEMA")) {
        std::cout << "[A] alg: PEM-A*; threads: " << ap.numThreadsExpansion << "\n";
        HandleTempDir(ap.tempDir);
        EMBHS::PEMASTAR::PEMASTAR<Tstate, Taction, Tenvironment, Thash> pemastar(ap.tempDir, ap.verbosity);
        pemastar.SetHeuristics(hForward);

        timer.StartTimer();
        pemastar.GetPath(start, goal);
        timer.EndTimer();

        printf("[R] alg: %s; solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n", "PEM-A*",
               (double) pemastar.GetPathLength(), pemastar.GetNodesExpanded(),
               pemastar.GetNodesGenerated(), timer.GetElapsedTime());
    }

    if (ap.HasAlgorithm("RPEMA")) {
        std::cout << "[A] alg: PEM-RA*; threads: " << ap.numThreadsExpansion << "\n";
        HandleTempDir(ap.tempDir);
        EMBHS::PEMASTAR::PEMASTAR<Tstate, Taction, Tenvironment, Thash> pemastar(ap.tempDir, ap.verbosity);
        pemastar.SetHeuristics(hBackward);

        timer.StartTimer();
        pemastar.GetPath(goal, start);
        timer.EndTimer();

        printf("[R] alg: %s; solution: %1.0f; expanded: %llu; generated: %llu; elapsed: %1.3f\n", "PEM-RA*",
               (double) pemastar.GetPathLength(), pemastar.GetNodesExpanded(),
               pemastar.GetNodesGenerated(), timer.GetElapsedTime());
    }
}

void RunToh(EMBHS::ArgParameters &ap) {
    std::cout << "[D] domain: TOH; heuristic: PDB-" + ap.heuristic << std::endl;
    TOH<20> toh;
    TOHState<20> start;
    TOHState<20> goal;

    int table[] = {52058078, 116173544, 208694125, 131936966, 141559500, 133800745, 194246206, 50028346, 167007978,
                   207116816, 163867037, 119897198, 201847476, 210859515, 117688410, 121633885};
    int table2[] = {145008714, 165971878, 154717942, 218927374, 182772845, 5808407, 19155194, 137438954, 13143598,
                    124513215, 132635260, 39667704, 2462244, 41006424, 214146208, 54305743};

    for (int i = ap.instanceId; i < ap.instanceId + ap.numOfInstances; ++i) {
        srandom(table[i & 0xF] ^ table2[(i >> 4) & 0xF]);

        start.counts[0] = start.counts[1] = start.counts[2] = start.counts[3] = 0;
        for (int x = 20; x > 0; x--) {
            int whichPeg = random() % 4;
            start.disks[whichPeg][start.counts[whichPeg]] = x;
            start.counts[whichPeg]++;
        }

        goal.counts[0] = goal.counts[1] = goal.counts[2] = goal.counts[3] = 0;
        for (int x = 20; x > 0; x--) {
            int whichPeg = random() % 4;
            goal.disks[whichPeg][goal.counts[whichPeg]] = x;
            goal.counts[whichPeg]++;
        }

        std::cout << "[I] instance_id: " << i << "; instance_str: " << start << " " << goal << std::endl;
        Heuristic<TOHState<20>> *hForward;
        Heuristic<TOHState<20>> *hBackward;

        switch (std::stoi(ap.heuristic)) {
            case 4:
                hForward = EMBHS::BuildTohPDB<20, 4>(ap.pdbDir, goal);
                hBackward = EMBHS::BuildTohPDB<20, 4>(ap.pdbDir, start);
                break;
            default:
                std::cerr << "Unavailable TOH Top PDB size" << std::endl;
                exit(-1);
        }
        RunProblem<TOHState<20>, TOHMove, TOH<20>, EMBHS::TOHHash<20>>
                (start, goal, hForward, hBackward, ap);
        EMBHS::DeletePDB(hForward);
        EMBHS::DeletePDB(hBackward);
    }
}

void RunStp4(EMBHS::ArgParameters &ap) {
    std::cout << "[D] domain: STP4; heuristic: " + ap.heuristic << std::endl;
    MNPuzzle<4, 4> env;
    MNPuzzleState<4, 4> goal;
    goal.Reset();

    Heuristic<MNPuzzleState<4, 4>> *fHeuristic;
    Heuristic<MNPuzzleState<4, 4>> *bHeuristic;
    Timer timer;
    std::vector<MNPuzzleState<4, 4>> solutionPath;

    if (ap.heuristic == "4+4+4+4") {
        fHeuristic = EMBHS::GenerateStpPDB(goal, ap.pdbDir, (int) std::thread::hardware_concurrency());
    } else if (ap.heuristic == "MD") {
        fHeuristic = &env;
        bHeuristic = &env;
    } else {
        std::cerr << "Unknown STP4 heuristic: " + ap.heuristic << std::endl;
        exit(-1);
    }

    for (int i = ap.instanceId; i < ap.instanceId + ap.numOfInstances; ++i) {
        MNPuzzleState<4, 4> start = GetStp4Problem(i);
        std::cout << "[I] instance_id: " << i << "; instance_str: " << start << std::endl;
        if (ap.heuristic == "4+4+4+4") {
            bHeuristic = EMBHS::GenerateStpPDB(start, ap.pdbDir, (int) std::thread::hardware_concurrency());
        }

        RUN_PROBLEM_STP(4)(start, goal, fHeuristic, bHeuristic, ap);


        if (ap.heuristic == "4+4+4+4") {
            EMBHS::DeletePDB(bHeuristic);
        }
    }

    if (ap.heuristic == "4+4+4+4") {
        EMBHS::DeletePDB(fHeuristic);
    }
}

void RunStp5(EMBHS::ArgParameters &ap) {
    std::cout << "[D] domain: STP5; heuristic: 6+6+6+6" << std::endl;
    MNPuzzleState<5, 5> goal;
    Heuristic<MNPuzzleState<5, 5>> *hForward = nullptr;
    if (ap.HasAlgorithm("AIDA") || ap.HasAlgorithm("PEMBAE")) {
        hForward = new EMBHS::CanonicalReflectionPDB5(
                EMBHS::GenerateStpPDB(goal, ap.pdbDir, (int) std::thread::hardware_concurrency()));
    }
    Heuristic<MNPuzzleState<5, 5>> *hBackward = nullptr;
    for (int i = ap.instanceId; i < ap.instanceId + ap.numOfInstances; ++i) {
        MNPuzzleState<5, 5> start = GetStp5Problem(i);
        if (ap.HasAlgorithm("RAIDA") || ap.HasAlgorithm("PEMBAE")) {
            hBackward = EMBHS::GenerateStpDoublePDB(start, ap.pdbDir, (int) std::thread::hardware_concurrency());
        }
        RUN_PROBLEM_STP(5)(start, goal, hForward, hBackward, ap);
        if (hBackward != nullptr) {
            for (auto &temp: hBackward->heuristics)
                delete temp;
            delete hBackward;
        }
    }

    delete hForward;

}

int main(int argc, char *argv[]) {
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout << std::setprecision(3);

    std::vector<std::string> args(argv, argv + argc);
    EMBHS::ArgParameters argParameters;
    if (!ParseArguments(argParameters, args)) {
        return 0;
    }
    if (!argParameters.IsValid()) {
        std::cerr << "Command line arguments are not valid, please fix them" << std::endl;
        std::cerr << argParameters << std::endl;
        return -1;
    }

    if (argParameters.domain == "STP4") {
        RunStp4(argParameters);
    } else if (argParameters.domain == "STP5") {
        RunStp5(argParameters);
    } else if (argParameters.domain == "TOH") {
        RunToh(argParameters);
    } else {
        std::cerr << ("Unknown domain: " + argParameters.domain) << std::endl;
        return -1;
    }

}

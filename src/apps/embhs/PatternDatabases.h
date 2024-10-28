#ifndef HOG2_SPL_PATTERNDATABASES_H
#define HOG2_SPL_PATTERNDATABASES_H

#include "MNPuzzle.h"
#include "MNPuzzleHash.h"
#include "TOH.h"
#include "TOHHash.h"
#include <algorithm>
#include <cstdint>
#include <sys/stat.h>
#include <cerrno>

namespace EMBHS {
    const std::vector<int> STP_4_PATTERN_LOCS[4] = {{0,  1,  4,  5},
                                                    {2,  3,  6,  7},
                                                    {8,  9,  12, 13},
                                                    {10, 11, 14, 15}};
    const std::vector<int> STP_5_PATTERN_LOCS[4] = {{0,  1,  5,  6,  10, 11},
                                                    {2,  3,  4,  7,  8,  9},
                                                    {15, 16, 17, 20, 21, 22},
                                                    {13, 14, 18, 19, 23, 24}};
    const std::vector<int> STP_5_REFLECTED_PATTERN_LOCS[4] = {{0,  1,  2,  5,  6,  7},
                                                              {3,  4,  8,  9,  13, 14},
                                                              {10, 11, 15, 16, 20, 21},
                                                              {17, 18, 19, 22, 23, 24}};

    template<int N>
    void GetStpPattern(MNPuzzleState<N, N> &goal, std::vector<int> &pattern, int pid, bool reflected) {
        auto locs = N == 4 ? STP_4_PATTERN_LOCS[pid] :
                    (reflected ? STP_5_REFLECTED_PATTERN_LOCS[pid] : STP_5_PATTERN_LOCS[pid]);
        pattern.resize(0);
        for (int &loc: locs) {
            pattern.push_back(goal.puzzle[loc]);
        }

        // If 0 is not found in the pattern (0 the tile, not the location!), we add it
        if (std::find(pattern.begin(), pattern.end(), 0) == pattern.end()) {
            pattern.push_back(0);
        } else if (N == 5) { // If we are in 5X5 puzzle and 0 is in the pattern, we add the center tile to the pattern
            pattern.push_back(goal.puzzle[12]);
        }
    }

    template<int N>
    std::string GetStpPdbName(MNPuzzleState<N, N> &goal, int pid, const char *prefix, bool reflected) {
        std::string fileName;

        fileName += prefix;
        // For unix systems, the prefix should always end in a trailing slash
        if (fileName.back() != '/' && prefix[0] != 0)
            fileName += '/';
        fileName += std::to_string(N);
        fileName += '-';

        EMBHS::STPHash<N> hasher;

        fileName += std::to_string(hasher.HashBucket(goal));
        fileName += "-";
        fileName += std::to_string(hasher.HashData(goal));
        fileName += "-";
        fileName += std::to_string(pid);
        if (reflected) {
            fileName += "r";
        }
        fileName += "-lex.pdb";
        return fileName;
    }

    // Either loads a PDB to memory or creates it
    template<int N>
    LexPermutationPDB<MNPuzzleState<N, N>, slideDir, MNPuzzle<N, N>> *
    GeneratePartialStpPDB(MNPuzzleState<N, N> &goal, int pid, const char *prefix, int numThreads, bool reflected) {
        MNPuzzle<N, N> env;
        std::vector<int> pattern;
        GetStpPattern(goal, pattern, pid, reflected);


        env.SetPattern(pattern);
        auto *pdb = new LexPermutationPDB<MNPuzzleState<N, N>, slideDir, MNPuzzle<N, N>>(&env, goal, pattern);
        std::string fileName = GetStpPdbName(goal, pid, prefix, reflected);
        if (access(fileName.c_str(), F_OK) != -1) {
            FILE *f = fopen(fileName.c_str(), "r");
            if (!pdb->Load(f)) {
                std::cerr << "Error loading PDB" << std::endl;
                exit(-1);
            }
            fclose(f);
        } else {
            pdb->BuildAdditivePDB(goal, numThreads);
            FILE *f = fopen(fileName.c_str(), "w+");
            pdb->Save(f);
            fclose(f);
        }
        return pdb;
    }

    // Generates a PDB in a regular fashion, or uses its reflected sub-PDBs to create the reflection across the main
    // diagonal
    template<int N>
    Heuristic<MNPuzzleState<N, N>> *
    GenerateStpPDB(MNPuzzleState<N, N> &goal, const std::string &prefix, int numThreads, bool reflected = false) {
        // Generate the folder to hold the PDB files. It does not override it, so we do not need to worry about deleting
        // existing PDBs if we end up loading them
        int status = mkdir(prefix.c_str(), 0777);
        if ((status < 0) && (errno != EEXIST)) {
            std::cerr << "Failed to make a directory";
            exit(-1);
        }

        auto *h = new Heuristic<MNPuzzleState<N, N>>();
        h->lookups.resize(0);
        h->lookups.push_back({kAddNode, 1, 4});
        h->heuristics.resize(0);
        for (unsigned int i = 0; i < 4; i++) {
            h->lookups.push_back({kLeafNode, i, i});
            h->heuristics.push_back(GeneratePartialStpPDB(goal, i, prefix.c_str(), numThreads, reflected));
        }
        return h;
    }


    // This function generates a heuristic that includes the PDB and its reflection across the main diagonal
    template<int N>
    Heuristic<MNPuzzleState<N, N>> *
    GenerateStpDoublePDB(MNPuzzleState<N, N> &goal, const std::string &prefix) {
        // This function assumes that the PDBs exist, and that GeneratePartialStpPDB() will only load them. To make sure
        // we do not create them and cost ourselves a lot of unnecessary building time, they have -1 threads.

        auto *h = new Heuristic<MNPuzzleState<N, N>>();
        h->lookups.resize(0);
        h->lookups.push_back({kMaxNode, 1, 2});
        h->heuristics.resize(0);
        h->lookups.push_back({kLeafNode, 0, 0});
        h->heuristics.push_back(GenerateStpPDB(goal, prefix.c_str(), -1, false));
        h->lookups.push_back({kLeafNode, 1, 1});
        h->heuristics.push_back(GenerateStpPDB(goal, prefix.c_str(), -1, true));
        return h;
    }

    // Since we are working with new, and generally the destructors in hog2 are lackluster, we make sure to clear the
    // memory we use.
    template<int N>
    void DeletePDB(Heuristic<MNPuzzleState<N, N>> *h) {
        for (auto &temp: h->heuristics)
            delete temp;
        delete h;
    }

    // Since it's very simple to use main-diagonal reflection with the canonical goal PDB, we just do it instead of
    // building another PDB
    class CanonicalReflectionPDB5 : public Heuristic<MNPuzzleState<5, 5>> {
    public:
        CanonicalReflectionPDB5(Heuristic<MNPuzzleState<5, 5>> *pdb) {
            this->pdb = pdb;
            GenerateMap();
        }

        double HCost(const MNPuzzleState<5, 5> &a, const MNPuzzleState<5, 5> &b) const override {
            MNPuzzleState<5, 5> c;
            MapState(a, c);
            return std::max(pdb->HCost(a, b), pdb->HCost(c, b));
        }

        void MapState(const MNPuzzleState<5, 5> &original, MNPuzzleState<5, 5> &state) const {
            for (int i = 0; i < 25; ++i) {
                state.puzzle[map.at(i)] = original.puzzle[i];
            }

            for (int i = 0; i < 25; ++i) {
                state.puzzle[i] = map.at(state.puzzle[i]);
                if (state.puzzle[i] == 0) {
                    state.blank = i;
                }
            }
        }

        void GenerateMap() {
            for (int i = 0; i < 5; ++i) {
                for (int j = 0; j < 5; ++j) {
                    map[i * 5 + j] = j * 5 + i;
                }
            }
        }

        ~CanonicalReflectionPDB5() override {
            DeletePDB(this->pdb);
        }


    protected:
        Heuristic<MNPuzzleState<5, 5>> *pdb;
        std::unordered_map<int, int> map;
    };

    template<int numDisks, int pdb1Disks, int pdb2Disks = numDisks - pdb1Disks>
    std::string GetTohPdbName(const std::string &pdb_prefix, const TOHState<numDisks> &goal, bool isTop = true) {
        TOHHash<numDisks> hasher;
        std::string filename = pdb_prefix + "t" + std::to_string(numDisks) + "_";
        filename += std::to_string(isTop ? pdb1Disks : pdb2Disks) + "_";
        filename += std::to_string(hasher.HashBucket(goal)) + "_" + std::to_string(hasher.HashData(goal)) + ".lex";
        return filename;
    }

    template<int numDisks, int pdb1Disks, int pdb2Disks = numDisks - pdb1Disks>
    Heuristic<TOHState<numDisks>> *BuildTohPDB(const std::string &pdb_prefix, const TOHState<numDisks> &goal) {
        TOH<numDisks> toh;
        TOH<pdb1Disks> absToh1;
        TOH<pdb2Disks> absToh2;
        TOHState<pdb1Disks> absTohState1;
        TOHState<pdb2Disks> absTohState2;


        auto *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
        auto *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks

        std::string topPdbName = GetTohPdbName<numDisks, pdb1Disks, pdb2Disks>(pdb_prefix, goal, true);
        std::string bottomPdbName = GetTohPdbName<numDisks, pdb1Disks, pdb2Disks>(pdb_prefix, goal, false);

        if (access(topPdbName.c_str(), F_OK) != -1) {
            FILE *f = fopen(topPdbName.c_str(), "r");
            pdb1->PDBHeuristic<TOHState<pdb1Disks>, TOHMove, TOH<pdb1Disks>, TOHState<numDisks>>::Load(f);
            fclose(f);
        } else {
            pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
            FILE *f = fopen(topPdbName.c_str(), "w+");
            pdb1->PDBHeuristic<TOHState<pdb1Disks>, TOHMove, TOH<pdb1Disks>, TOHState<numDisks>>::Save(f);
            fclose(f);
        }

        if (access(bottomPdbName.c_str(), F_OK) != -1) {
            FILE *f = fopen(bottomPdbName.c_str(), "r");
            pdb2->PDBHeuristic<TOHState<pdb2Disks>, TOHMove, TOH<pdb2Disks>, TOHState<numDisks>>::Load(f);
            fclose(f);
        } else {
            pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
            FILE *f = fopen(bottomPdbName.c_str(), "w+");
            pdb2->PDBHeuristic<TOHState<pdb2Disks>, TOHMove, TOH<pdb2Disks>, TOHState<numDisks>>::Save(f);
            fclose(f);
        }

        auto *h = new Heuristic<TOHState<numDisks>>;

        h->lookups.resize(0);
        h->lookups.push_back({kAddNode, 1, 2});
        h->lookups.push_back({kLeafNode, 0, 0});
        h->lookups.push_back({kLeafNode, 1, 1});
        h->heuristics.resize(0);
        h->heuristics.push_back(pdb1);
        h->heuristics.push_back(pdb2);

        return h;
    }

    template<int N>
    void DeletePDB(Heuristic<TOHState<N>> *h) {
        for (auto &temp: h->heuristics)
            delete temp;
        delete h;
    }
}


#endif //HOG2_SPL_PATTERNDATABASES_H

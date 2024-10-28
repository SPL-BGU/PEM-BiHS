//
// Created by Lior Siag on 01/10/2023.
//

#ifndef HOG2_SPL_PEBAE_H
#define HOG2_SPL_PEBAE_H

#include "Heuristic.h"
#include "FixedSizeSetMultiThreaded.h"
#include "Timer.h"
#include "EMBHS.h"
#include <limits>
#include <mutex>
#include <unordered_map>
#include <cstdio>
#include <ostream>
#include <utility>
#include <algorithm>
#include <iostream>
#include <thread>
#include <cmath>


namespace EMBHS { namespace PEBAE {

    // DiskState represents the data of a state that goes into the file itself.
    struct DiskState {
        uint64_t bucket;
        uint64_t data;
    };


    static bool operator==(const DiskState &a, const DiskState &b) {
        return (a.bucket == b.bucket && a.data == b.data);
    }

    // Since the hash need to fit into 64-bits, we cannot include both bucket and data, so just data will be enough for
    // the purposes of FixedSizeSet, which just need to prevent a lot of collisions.
    struct DiskStateHash {
        std::size_t operator()(const DiskState &x) const {
            const uint64_t m1 = UINT64_C(0xc6a4a7935bd1e995);
            const uint64_t m2 = UINT64_C(0x5bd1e9955bd1e995);
            const int r1 = 47;
            const int r2 = 31;

            uint64_t p1 = x.bucket;
            uint64_t p2 = x.data;

            p1 ^= p1 >> r1;
            p1 *= m1;
            p1 ^= p1 >> r1;

            p2 ^= p2 >> r2;
            p2 *= m2;
            p2 ^= p2 >> r2;

            // Combine p1 and p2
            p1 *= m2;
            p1 ^= p2;

            // Final mixing
            p1 ^= p1 >> r1;
            p1 *= m1;
            p1 ^= p1 >> r1;

            return p1;
        }
    };


    enum tSearchDirection {
        kForward,
        kBackward,
    };

    static std::ostream &operator<<(std::ostream &out, const tSearchDirection &d) {
        switch (d) {
            case kForward:
                out << "Forward";
                break;
            case kBackward:
                out << "Backward";
                break;
        }
        return out;
    }


    // Represents the data of a bucket. In PEMM this would include the bucket hash, but we have that in DiskState. If this
    // bucketing system does not work, we will add bucket hash to it to separate files even further.
    struct OpenData {
        uint8_t gCost;
        uint8_t hCost;
        uint8_t hBarCost;
    };

    static bool operator==(const OpenData &a, const OpenData &b) {
        return (a.gCost == b.gCost && a.hCost == b.hCost && a.hBarCost == b.hBarCost);
    }


    static std::ostream &operator<<(std::ostream &out, const OpenData &d) {
        out << "[g:" << +d.gCost << ", h:" << +d.hCost << ", hb:" << +d.hBarCost << "] ";
        return out;
    }

    // Represents an open file (paired with OpenData) which logs how many states we have written as well
    struct OpenFile {
        OpenFile(const std::string &filename = "") : filename(filename), writtenStates(0) {}

        std::string filename;
        uint64_t writtenStates;
    };

    // Since every value is no larger than 256 (at least not in the 5X5 STP) we can fit everything into a perfect hash
    struct OpenDataHash {
        std::size_t operator()(const OpenData &x) const {
            return x.gCost | (x.hCost << 8) | (x.hBarCost << 16);
        }
    };

    // Since we only need the closed files for duplicate detection, we can store the direction inside the struct instead of
    // having to closed queues.
    struct ClosedData {
        uint8_t gCost;
        uint8_t hCost;
        uint8_t hBarCost;
        tSearchDirection dir;
    };

    static bool operator==(const ClosedData &a, const ClosedData &b) {
        return (a.dir == b.dir && a.gCost == b.gCost && a.hCost == b.hCost && a.hBarCost == b.hBarCost);
    }

    static std::ostream &operator<<(std::ostream &out, const ClosedData &d) {
        out << "[" << ((d.dir == kForward) ? "forward" : "backward") << ", g:" << +d.gCost << ", h:" << +d.hCost;
        out << ", hb:" << +d.hBarCost << "]";
        return out;
    }

    struct ClosedFile {
        ClosedFile(const std::string filename = "") : filename(filename) {}

        std::string filename;
    };

    struct ClosedDataHash {
        std::size_t operator()(const ClosedData &x) const {
            return (x.dir) | (x.gCost << 1) | (x.hCost << 9) | (x.hBarCost << 17);
        }
    };

    // A helper struct for bound calculation
    struct BoundValues {
        uint16_t minG;
        uint16_t minF;
        uint16_t minD;
        uint16_t minB;
    };

    static std::ostream &operator<<(std::ostream &out, const BoundValues &bv) {
        out << "[" << bv.minG << ", " << bv.minF << ", " << bv.minD << ", " << bv.minB << "]";
        return out;
    }

    //typedef FixedSizeSet<DiskState, DiskStateHash> BucketSet;
    typedef FixedSizeSetMultiThreaded<DiskState, DiskStateHash> BucketSet;


    template<class Tstate, class Taction, class Tenvironment, class Thash>
    class PEBAE {
    public:
        PEBAE(std::string cPrefix = "pebae_files/", EMBHS::Verbosity cVerbosity = EMBHS::lInfo,
              unsigned int expansionThreads = 0, unsigned int readingThreads = 0) :
                prefix(std::move(cPrefix)),
                stateHashers(expansionThreads == 0 ? (int) std::thread::hardware_concurrency() : expansionThreads),
                verbosity(cVerbosity) {
            epsilon = 1;
            nodesExpanded = 0;
            isSearchFinished = false;
            currExpansionDir = kForward;
            this->verbosity = cVerbosity;
            this->numExpansionThreads = expansionThreads == 0 ? std::thread::hardware_concurrency() : expansionThreads;
            this->numReadingThreads = readingThreads == 0 ? std::thread::hardware_concurrency() : readingThreads;
            SetHeuristics();
            mutexes = new std::mutex[maxCost * maxCost * maxCost];
        }

        ~PEBAE() { delete[] mutexes; }

        void GetPath(Tstate &, Tstate &, uint8_t = 1);

        void SetHeuristics(Heuristic<Tstate> *hF = nullptr, Heuristic<Tstate> *hB = nullptr);

        void AddStateToQueue(const Tstate &, tSearchDirection, uint8_t);

        void GetOpenAndDiskData(const Tstate &, tSearchDirection, uint8_t, OpenData &, DiskState &, int = 0);

        inline void AddStateToQueue(OpenData &openData, DiskState diskState, tSearchDirection dir) {
            AddStatesToQueue(openData, &diskState, dir, 1);
        }

        inline std::unordered_map<OpenData, OpenFile, OpenDataHash> &GetOpenQueue(const tSearchDirection &dir) {
            return dir == kForward ? openForwardQueue : openBackwardQueue;
        }

        void AddStatesToQueue(const OpenData &, DiskState *, tSearchDirection, size_t);

        std::string GetOpenName(const OpenData &, tSearchDirection);

        void ExpandNextFile();

        OpenData GetBestFile();

        bool CanTerminateSearch();

        BoundValues GetBoundValues(tSearchDirection);

        void ReadAndDDBucket(BucketSet &, const OpenData &, tSearchDirection);

        void ReadBucket(BucketSet &, const OpenData &, tSearchDirection);

        void RemoveDuplicates(BucketSet &, const OpenData &, tSearchDirection);

        void WriteToClosed(BucketSet &, const OpenData &, tSearchDirection);

        std::string GetClosedName(const ClosedData &);

        void CheckSolution(tSearchDirection, OpenData, const BucketSet &);

        void FindSolution(const OpenData &, const OpenFile &, uint8_t, const BucketSet &);

        void ParallelExpandBucket(const OpenData &, BucketSet &, tSearchDirection, int, int);

        void ReadBucketThread(BucketSet &, const OpenData &, tSearchDirection, int, int);

        uint16_t GetPathLength() {
            return bestSolution;
        }

        uint64_t GetNodesExpanded() {
            return nodesExpanded;
        }

        uint64_t GetNodesGenerated() {
            return nodesGenerated;
        }

        uint64_t GetLastBucketSize() {
            return lastBucketSize;
        }

        uint64_t GetNecessaryExpanded() {
            uint64_t necessary = 0;
            for (const auto &count: counts) {
                if (count.first < bestSolution)
                    necessary += count.second;
            }
            return necessary;
        }


        Tstate GetMeetingState() {
            return solutionState;
        }

        size_t GetMaxMemory() {
            return maxMemory;
        }

        void PrintStates(const OpenData &openData, DiskState *diskState, size_t count) {
            printLock.lock();
            std::cout << "O[" << unsigned(openData.gCost) << "," << unsigned(openData.hCost) << ","
                      << unsigned(openData.hBarCost) << "]\n";
            for (int i = 0; i < count; ++i, diskState++) {
                std::cout << diskState->bucket << "," << diskState->data << "\n";
            }
            printLock.unlock();
        }

        inline FILE *GetFilePointer(const std::string &filename, const char *mode = "rb") {
            FILE *f = fopen(filename.c_str(), mode);
            if (f == nullptr) {
                if (verbosity <= EMBHS::lCritical) {
                    printf("Error opening %s; Aborting!\n", filename.c_str());
                    perror("Reason: ");
                    exit(-1);
                }
            }
            return f;
        }

    protected:
        // General information for the search
        Tenvironment env;
        std::vector<Thash> stateHashers;
        Tstate start;
        Tstate goal;
        Heuristic<Tstate> *hForward;
        Heuristic<Tstate> *hBackward;
        uint8_t epsilon;
        unsigned int numExpansionThreads;
        unsigned int numReadingThreads;
        std::mutex *mutexes;
        int maxCost = 100;

        // Open and closed lists
        std::unordered_map<OpenData, OpenFile, OpenDataHash> openForwardQueue;
        std::unordered_map<OpenData, OpenFile, OpenDataHash> openBackwardQueue;
        std::unordered_map<ClosedData, ClosedFile, ClosedDataHash> closedQueue;

        // Information about current progress
        uint16_t bestSolution = UINT16_MAX;
        DiskState solutionState{};
        std::vector<uint64_t> gDistForward;
        std::vector<uint64_t> gDistBackward;
        uint64_t nodesExpanded;
        uint64_t nodesGenerated;
        std::map<double, uint64_t> counts;
        bool isSearchFinished;
        tSearchDirection currExpansionDir;
        uint16_t lb;
        uint16_t lastBucketSize;
        size_t maxMemory;

        // Mutex locks
        std::mutex printLock;
        std::mutex countLock;
        std::mutex openLock;
        std::mutex statesLock;

        // General
        std::string prefix;
        EMBHS::Verbosity verbosity;
    };

    // The main method to get the path. Sets the different values, adds start and goal, and starts expanding.
    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::GetPath(Tstate &cStart, Tstate &cGoal, uint8_t eps) {
        this->start = cStart;
        this->goal = cGoal;
        this->isSearchFinished = false;
        this->currExpansionDir = kForward;
        this->epsilon = eps;
        // Clear and initialize our tracker of the g-values we encounter during the search
        gDistForward.clear();
        gDistForward.resize(256);
        gDistBackward.clear();
        gDistBackward.resize(256);
        this->nodesExpanded = 0;
        this->lastBucketSize = 0;
        this->nodesGenerated = 0;
        this->maxMemory = 0;
        AddStateToQueue(this->start, kForward, 0);
        AddStateToQueue(this->goal, kBackward, 0);

        while ((!openForwardQueue.empty() || !openBackwardQueue.empty()) && !this->isSearchFinished) {
            ExpandNextFile();
        }
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::SetHeuristics(Heuristic<Tstate> *hF, Heuristic<Tstate> *hB) {
        this->hForward = hF == nullptr ? &this->env : hF;
        this->hBackward = hB == nullptr ? &this->env : hB;
    }

    // Here we want to add a state in its regular representation to our queue. But since regular state and disk state are
    // different, we first need to understand wo which bucket it goes (OpenData) and then find its disk representation
    // (DiskState). Only then can we add it.
    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void
    PEBAE<Tstate, Taction, Tenvironment, Thash>::AddStateToQueue(const Tstate &state, tSearchDirection direction,
                                                                 uint8_t gCost) {
        OpenData openData{};
        DiskState diskState{};
        GetOpenAndDiskData(state, direction, gCost, openData, diskState);
        AddStateToQueue(openData, diskState, direction);
    }


    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::GetOpenAndDiskData(const Tstate &state, tSearchDirection dir,
                                                                         uint8_t gCost, OpenData &openData,
                                                                         DiskState &diskState, int myThread) {
        diskState.bucket = stateHashers[myThread].HashBucket(state);
        diskState.data = stateHashers[myThread].HashData(state);

        openData.gCost = gCost;

        if (dir == kForward) {
            openData.hCost = hForward->HCost(state, this->goal);
            openData.hBarCost = hBackward->HCost(state, this->start);

        } else { //kBackward
            openData.hCost = hBackward->HCost(state, this->start);
            openData.hBarCost = hForward->HCost(state, this->goal);
        }
        if (openData.gCost < openData.hBarCost) {
            printLock.lock();
            std::cout << "Error: " << dir << " " << state << " g:" << unsigned(openData.gCost) << " hb: "
                      << unsigned(openData.hBarCost) << this->start << " sanity " << this->goal
                      << std::endl;
            printLock.unlock();
        }
    }

    // Adds states to a particular file, given its OpenData.
    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void
    PEBAE<Tstate, Taction, Tenvironment, Thash>::AddStatesToQueue(const OpenData &openData, DiskState *diskState,
                                                                  tSearchDirection dir, size_t count) {
        int mutexPosition = (openData.gCost % maxCost) + maxCost * (openData.hCost % maxCost) +
                            maxCost * maxCost * (openData.hBarCost % maxCost);
        openLock.lock();
        auto &openQueue = GetOpenQueue(dir);
        auto iter = openQueue.find(openData);
        // If the file does not exist, we want to create it
        if (iter == openQueue.end()) {
            openQueue[openData] = OpenFile(GetOpenName(openData, dir));
        }

        auto &open_file = openQueue[openData];
        openLock.unlock();
        mutexes[mutexPosition].lock();
        FILE *f = GetFilePointer(open_file.filename, "ab");
        size_t written = fwrite(diskState, sizeof(DiskState), count, f);
        open_file.writtenStates += written;
        fclose(f);
        mutexes[mutexPosition].unlock();
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    std::string
    PEBAE<Tstate, Taction, Tenvironment, Thash>::GetOpenName(const OpenData &openData, tSearchDirection dir) {
        std::string s;
        s += this->prefix;
        s += dir == kForward ? "forward-" : "backward-";
        s += std::to_string(openData.gCost);
        s += "-";
        s += std::to_string(openData.hCost);
        s += "-";
        s += std::to_string(openData.hBarCost);
        s += ".open";
        return s;
    }

    // Method that is responsible to choose and expand the next file.
    // It first checks if the search can terminate. Then it loads the bucket and removes nodes that appear multiple times
    // within it and then removes duplicates from previous files. Then it checks for a solution and at the same time expands
    // the bucket without checking for duplicates (delayed duplicate-detection).
    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::ExpandNextFile() {
        static BucketSet states(0);
        static OpenData next;

        if (CanTerminateSearch())
            return;

        uint64_t prevExpansions = this->nodesExpanded;

        // get best file (minimum b-value in the current search direction)
        auto &openQueue = GetOpenQueue(currExpansionDir);
        OpenData openData = GetBestFile();

        if (verbosity <= EMBHS::lInfo) {
            printLock.lock();
            std::cout << "Next: [" << currExpansionDir << "] " << openData;
            printLock.unlock();
        }

        Timer timer;

        ReadAndDDBucket(states, openData, currExpansionDir);

        if (verbosity <= EMBHS::lInfo) {
            printLock.lock();
            std::cout << " (" << states.size() << " entries " << openQueue.find(openData)->second.writtenStates << ")";
            printLock.unlock();
            if (verbosity <= EMBHS::lDebug)
                std::cout << "\n";
            else
                std::cout << " ";
        }

        timer.StartTimer();
        openQueue.erase(openQueue.find(openData));

        // 2. Read in opposite buckets to check for solutions in parallel to expanding this bucket
        openLock.lock();
        std::thread t(&PEBAE::CheckSolution, this, currExpansionDir, openData, std::ref(states));
        openLock.unlock();

        // 3. expand all states in current bucket & write out successors
        //const auto numThreads = 1;
        std::vector<std::thread *> threads;
        for (size_t x = 0; x < numExpansionThreads; x++)
            threads.push_back(
                    new std::thread(&PEBAE::ParallelExpandBucket, this, openData, std::ref(states),
                                    currExpansionDir, x, numExpansionThreads));
        for (auto &thread: threads) {
            thread->join();
            delete thread;
        }

        timer.EndTimer();
        if (verbosity <= EMBHS::lInfo) {
            printLock.lock();
            std::cout << "[" << timer.GetElapsedTime() << "s expanding] ";
            printLock.unlock();
        }

        // Close thread that is doing DSD
        timer.StartTimer();
        t.join();
        timer.EndTimer();
        if (verbosity <= EMBHS::lInfo) {
            printLock.lock();
            std::cout << "[" << timer.GetElapsedTime() << "s DSD]\n";
            printLock.unlock();
        }
        lastBucketSize = this->nodesExpanded - prevExpansions;
        currExpansionDir = (currExpansionDir == kForward ? kBackward : kForward);
        maxMemory = std::max(maxMemory, states.GetCurrentSize());
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    OpenData PEBAE<Tstate, Taction, Tenvironment, Thash>::GetBestFile() {
        auto &openQueue = GetOpenQueue(currExpansionDir);
        OpenData best = (openQueue.begin())->first;
        // We use the uint16_t 2 to convert this entire thing to prevent overflows
        uint16_t bestPriority = ((uint16_t) 2) * best.gCost + best.hCost - best.hBarCost;
        uint16_t currPriority;
        for (const auto &s: openQueue) {
            currPriority = ((uint16_t) 2) * s.first.gCost + s.first.hCost - s.first.hBarCost;
            // Always prefer lower b-values, and tiebreak in favor of lower g-values. This promises we always expand
            //  the parent before the child
            if (currPriority < bestPriority || (currPriority == bestPriority && s.first.gCost < best.gCost) ||
                (currPriority == bestPriority && s.first.gCost == best.gCost && s.first.hCost < best.hCost)) {
                bestPriority = currPriority;
                best = s.first;
            }
        }
        return best;
    }

// This method collects all the minimal values (g,f,d,b) that we need to bound calculations.
    template<class Tstate, class Taction, class Tenvironment, class Thash>
    BoundValues PEBAE<Tstate, Taction, Tenvironment, Thash>::GetBoundValues(tSearchDirection dir) {
        auto &openQueue = GetOpenQueue(dir);
        BoundValues bv{UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX};
        for (const auto &s: openQueue) {
            if (s.first.gCost < bv.minG)
                bv.minG = s.first.gCost;
            if (((uint16_t) s.first.gCost) + s.first.hCost < bv.minF)
                bv.minF = ((uint16_t) s.first.gCost) + s.first.hCost;
            if (((uint16_t) s.first.gCost) - s.first.hBarCost < bv.minD)
                bv.minD = ((uint16_t) s.first.gCost) - s.first.hBarCost;
            if (((uint16_t) 2) * s.first.gCost + s.first.hCost - s.first.hBarCost < bv.minB)
                bv.minB = ((uint16_t) 2) * s.first.gCost + s.first.hCost - s.first.hBarCost;
        }
        return bv;
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    bool PEBAE<Tstate, Taction, Tenvironment, Thash>::CanTerminateSearch() {
        BoundValues forwardBV = GetBoundValues(kForward);
        BoundValues backwardBV = GetBoundValues(kBackward);
        if (verbosity <= EMBHS::lDebug) {
            printLock.lock();
            std::cout << "Forward BV: " << forwardBV << std::endl;
            std::cout << "Backward BV: " << backwardBV << std::endl;
            printLock.unlock();
        }
        uint16_t gBound = forwardBV.minG + backwardBV.minG;
        uint16_t fdBound = forwardBV.minF + backwardBV.minD;
        uint16_t dfBound = forwardBV.minD + backwardBV.minF;
        // This uses the gcd trick from Vidal's paper
        auto bBound = (uint16_t) ceil(double((forwardBV.minB + backwardBV.minB)) / 2);
        lb = std::max({gBound, fdBound, dfBound, bBound});
        if (verbosity <= EMBHS::lInfo) {
            std::cout << "Solution: ";
            if (this->bestSolution == UINT16_MAX)
                std::cout << "NF";
            else
                std::cout << unsigned(this->bestSolution);
            std::cout << " Bounds: " << unsigned(gBound) << "," << unsigned(fdBound) << ","
                      << unsigned(dfBound) << "," << unsigned(bBound) << "\n";
        }

        if (this->bestSolution <= lb) {
            isSearchFinished = true;
            if (verbosity <= EMBHS::lInfo) {
                printf("Done!\n");
                printf("gBound: %d; fdBound: %d; dfBound: %d; bBound: %d; solution: %d\n",
                       gBound, fdBound, dfBound, bBound, bestSolution);
                printf("%d nodes expanded\n", this->nodesExpanded);

                Thash hasher;
                Tstate tmp;
                hasher.Restore(tmp, solutionState.bucket, solutionState.data);
                std::cout << "Solution State: " << tmp << std::endl;
                printf("Forward Distribution:\n");
                for (int x = 0; x < gDistForward.size(); x++)
                    if (gDistForward[x] != 0)
                        printf("%d\t%llu\n", x, gDistForward[x]);
                printf("Backward Distribution:\n");
                for (int x = 0; x < gDistBackward.size(); x++)
                    if (gDistBackward[x] != 0)
                        printf("%d\t%llu\n", x, gDistBackward[x]);
                if (lb == gBound)
                    printf("-Triggered by gBound\n");
                if (lb == fdBound)
                    printf("-Triggered by fdBound\n");
                if (lb == dfBound)
                    printf("-Triggered by dfBound\n");
                if (lb == bBound)
                    printf("-Triggered by bBound\n");
            }
            return true;
        }
        return false;
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::ReadAndDDBucket(BucketSet &states, const OpenData &openData,
                                                                      tSearchDirection dir) {
        Timer t1, t2, t3;
        t1.StartTimer();
        ReadBucket(states, openData, dir);
        t1.EndTimer();
        t2.StartTimer();
        RemoveDuplicates(states, openData, dir); // delayed duplicate detection
        t2.EndTimer();
        t3.StartTimer();
        WriteToClosed(states, openData, dir);
        t3.EndTimer();
        if (verbosity <= EMBHS::lInfo) {
            printLock.lock();
            std::cout << "[" << t1.GetElapsedTime() << "s reading] ["
                      << t2.GetElapsedTime() << "s dd] ["
                      << t3.GetElapsedTime() << "s writing]";
            printLock.unlock();
        }
    }

// Reading nodes from a file involves both actually reading them but also inserting them into a set. By doing this, we
// get only a single copy of a state even if it appeared multiple times in the file.
    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::ReadBucket(BucketSet &states, const OpenData &openData,
                                                                 tSearchDirection dir) {
        auto &openQueue = GetOpenQueue(dir);
        states.resize(openQueue[openData].writtenStates);
        //const auto numThreads = 1;
        std::vector<std::thread *> threads;
        for (size_t x = 0; x < numReadingThreads; x++)
            threads.push_back(new std::thread(&PEBAE::ReadBucketThread, this, std::ref(states), openData, dir, x,
                                              numReadingThreads));
        for (auto &thread: threads) {
            thread->join();
            delete thread;
        }
        remove(GetOpenName(openData, dir).c_str());
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::ReadBucketThread(BucketSet &states, const OpenData &openData,
                                                                       tSearchDirection dir, int myThread,
                                                                       int totalThreads) {
        const size_t bufferSize = 1024;

        auto &openQueue = GetOpenQueue(dir);
        int64_t toRead = openQueue[openData].writtenStates / totalThreads;
        DiskState buffer[bufferSize];

        FILE *f = fopen(openQueue[openData].filename.c_str(), "rb");

        fseek(f, myThread * toRead * sizeof(DiskState), SEEK_SET);

        if (myThread == totalThreads - 1) {
            toRead += openQueue[openData].writtenStates % totalThreads;
        }

        size_t numRead;
        for (int i = 0; i < toRead / bufferSize; ++i) {
            numRead = fread(buffer, sizeof(DiskState), bufferSize, f);
            for (int j = 0; j < bufferSize; ++j) {
                states.insert(buffer[j]);
            }
        }

        if (toRead % bufferSize > 0) {
            numRead = fread(buffer, sizeof(DiskState), toRead % bufferSize, f);
            for (int j = 0; j < toRead % bufferSize; ++j) {
                states.insert(buffer[j]);
            }
        }
        fclose(f);
    }

// This is the delayed-duplicate-detection that searches for the current states in previous closed buckets with better
// g-values. Since we work in unit edge cost domains (for now), we only need to check 2 backwards.
    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::RemoveDuplicates(BucketSet &states, const OpenData &openData,
                                                                       tSearchDirection dir) {

        uint8_t diff = 2;
        if (openData.gCost == 0)
            diff = 0;
        if (openData.gCost == 1)
            diff = 1;
        // The h-values stay the same, so we only have to check for the g-values when looking for potential buckets to have
        // duplicates of our current states.
        for (uint8_t depth = openData.gCost - diff; depth < openData.gCost; depth++) {
            ClosedData closedData{};
            closedData.gCost = depth;
            closedData.hCost = openData.hCost;
            closedData.hBarCost = openData.hBarCost;
            closedData.dir = dir;

            if (closedQueue.find(closedData) == closedQueue.end())
                continue;
            FILE *f = GetFilePointer(closedQueue[closedData].filename);
            const size_t bufferSize = 1024;
            DiskState buffer[bufferSize];
            size_t numRead;
            do {
                numRead = fread(buffer, sizeof(DiskState), bufferSize, f);
                for (size_t x = 0; x < numRead; x++) {
                    auto i = states.find(buffer[x]);
                    if (i != states.end()) {
                        // Note that erasing does not actually erase the state from the bucket, but only marks it as
                        // invalid. This is to speed things up since we do not care for actually removing it, only for
                        // ignoring it.
                        states.erase(i);
                    }
                }
            } while (numRead == bufferSize);
            fclose(f);
        }
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::WriteToClosed(BucketSet &states, const OpenData &openData,
                                                                    tSearchDirection dir) {
        ClosedData closedData{};
        closedData.gCost = openData.gCost;
        closedData.hCost = openData.hCost;
        closedData.hBarCost = openData.hBarCost;
        closedData.dir = dir;

        closedQueue[closedData] = ClosedFile(GetClosedName(closedData));
        FILE *f = GetFilePointer(closedQueue[closedData].filename, "ab");
        for (const auto &i: states) {
            if (i.valid) {
                fwrite(&i, sizeof(DiskState), 1, f);
            }
        }
        fclose(f);
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    std::string PEBAE<Tstate, Taction, Tenvironment, Thash>::GetClosedName(const ClosedData &closedData) {
        std::string s;
        s += this->prefix;
        s += closedData.dir == kForward ? "forward-" : "backward-";
        s += std::to_string(closedData.gCost);
        s += "-";
        s += std::to_string(closedData.hCost);
        s += "-";
        s += std::to_string(closedData.hBarCost);
        s += ".closed";
        return s;
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::CheckSolution(tSearchDirection currDir, OpenData openData,
                                                                    const BucketSet &states) {
        auto &oppositeOpenQueue = GetOpenQueue(currDir == kForward ? kBackward : kForward);
        for (const auto &s: oppositeOpenQueue) {
            // Only need to check if we find a better solution (g+g < U)
            //  Also note that since h is static, the hCost=hBarCost and vice versa, therefore, if they are not equal,
            //  there are no nodes in our current bucket that can be found in the open bucket we are looking at
            if (openData.hCost == s.first.hBarCost && openData.hBarCost == s.first.hCost &&
                openData.gCost + s.first.gCost >= lb && openData.gCost + s.first.gCost < bestSolution) {
                FindSolution(s.first, s.second, openData.gCost, states);
            }
        }
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void PEBAE<Tstate, Taction, Tenvironment, Thash>::FindSolution(const OpenData &openData, const OpenFile &openFile,
                                                                   uint8_t gCost, const BucketSet &states) {
        const size_t bufferSize = 128;
        DiskState buffer[bufferSize];

        FILE *f = GetFilePointer(openFile.filename);
        size_t numRead;

        do {
            numRead = fread(buffer, sizeof(DiskState), bufferSize, f);
            // If any of the nodes that we read in the opposite direction appear in the current states, we have a solution
            // (though not necessarily optimal). Note that find() returns end() if the state is invalid.
            for (int x = 0; x < numRead; x++) {
                if (states.find(buffer[x]) != states.end()) {
                    uint16_t currentSolution = ((uint16_t) 0) + openData.gCost + gCost;
                    if (currentSolution < bestSolution) {
                        bestSolution = std::min(currentSolution, bestSolution);
                        solutionState = buffer[x];
                    }


                    if (verbosity <= EMBHS::lDebug) {
                        printLock.lock();
                        printf("\nFound solution cost %d+%d=%d\n", openData.gCost, gCost, currentSolution);
                        printf("Current best solution: %d\n", bestSolution);
                        printLock.unlock();
                    }
                }
            }
        } while (numRead == bufferSize);
        fclose(f);
    }

    template<class Tstate, class Taction, class Tenvironment, class Thash>
    void
    PEBAE<Tstate, Taction, Tenvironment, Thash>::ParallelExpandBucket(const OpenData &openData, BucketSet &states,
                                                                      tSearchDirection dir, int myThread,
                                                                      int totalThreads) {
        // Controls how many states we agree to store for a particular OpenData before flushing from the cache.
        const int cacheSize = 8192;
        // stores a cache of states, so we will not have to write for every node expansion.
        std::unordered_map<OpenData, std::vector<DiskState>, OpenDataHash> cache;
        // Instead of actually creating the different states we can have a temporary state that we apply an action to,
        // record its OpenData and DiskState, add it to the cache, and then revert the action. This is possible because we
        // are not actually interested in the regular state representation.
        Tstate tmp;
        uint64_t localExpanded = 0;
        int count = 0;
        uint64_t localGenerated = 0;
        for (const auto &values: states) {
            count++;
            if (myThread != count % totalThreads)
                continue;

            DiskState v{};
            if (!values.valid)
                continue;

            v = values.item;
            localExpanded++;
            stateHashers[myThread].Restore(tmp, v.bucket, v.data);
            std::vector<Taction> validActions;
            env.GetActions(tmp, validActions);

            for (Taction &action: validActions) {
                env.ApplyAction(tmp, action);
                OpenData newOpenData{};
                DiskState newDiskState{};
                GetOpenAndDiskData(tmp, dir, openData.gCost + 1, newOpenData, newDiskState, myThread);
                localGenerated++;
                std::vector<DiskState> &c = cache[newOpenData];
                c.push_back(newDiskState);

                // Flush the cache to the relevant file
                if (c.size() > cacheSize) {
                    AddStatesToQueue(newOpenData, &c[0], dir, c.size());
                    if (verbosity <= EMBHS::lDebug)
                        PrintStates(newOpenData, &c[0], c.size()); // States logging
                    c.clear();
                }
                env.UndoAction(tmp, action);
            }
        }

        // Flush the rest of the cache if there are leftovers.
        for (auto &i: cache) {
            if (!i.second.empty()) {
                AddStatesToQueue(i.first, &(i.second[0]), dir, i.second.size());
                if (verbosity <= EMBHS::lDebug)
                    PrintStates(i.first, &(i.second[0]), i.second.size()); // States logging
            }

        }

        countLock.lock();
        nodesExpanded += localExpanded;
        if (dir == kForward) {
            gDistForward[openData.gCost] += localExpanded;
        } else {
            gDistBackward[openData.gCost] += localExpanded;
        }
        counts[lb] += localExpanded;
        nodesGenerated += localGenerated;
        countLock.unlock();
    }
}}

#endif //HOG2_SPL_PEBAE_H
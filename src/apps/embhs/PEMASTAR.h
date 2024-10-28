//
// Created by Shahaf Shperberg on 15/11/2023.
//

#ifndef HOG2_SPL_PEMASTAR_H
#define HOG2_SPL_PEMASTAR_H

#include "Heuristic.h"
#include "Timer.h"
#include <limits>
#include <mutex>
#include <unordered_map>
#include <cstdio>
#include <ostream>
#include <utility>
#include <algorithm>
#include <iostream>
#include <thread>
#include "EMBHS.h"
#include "FixedSizeSetMultiThreaded.h"


namespace EMBHS {
    namespace PEMASTAR {
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


// Represents the data of a bucket.
        struct BucketData {
            uint8_t gCost;
            uint8_t hCost;
        };

        static bool operator==(const BucketData &a, const BucketData &b) {
            return (a.gCost == b.gCost && a.hCost == b.hCost);
        }


        static std::ostream &operator<<(std::ostream &out, const BucketData &d) {
            out << "[g:" << +d.gCost << ", h:" << +d.hCost << "]";
            return out;
        }

// Represents an open file (paired with BucketData) which logs how many states we have written as well
        struct BucketFile {
            BucketFile(const std::string &filename = "") : filename(filename), writtenStates(0) {}

            std::string filename;
            uint64_t writtenStates;
        };

// Since every value is no larger than 256 (at least not in the 5X5 STP) we can fit everything into a perfect hash
        struct BucketDataHash {
            std::size_t operator()(const BucketData &x) const {
                return x.gCost | (x.hCost << 8);
            }
        };


        typedef FixedSizeSetMultiThreaded<DiskState, DiskStateHash> BucketSet;

        template<class Tstate, class Taction, class Tenvironment, class Thash>
        class PEMASTAR {
        public:
            PEMASTAR(std::string cPrefix = "PEMASTAR_files/", EMBHS::Verbosity cVerbosity = EMBHS::lInfo) :
                    prefix(std::move(cPrefix)),
                    stateHashers((int) std::thread::hardware_concurrency()) {
                epsilon = 1;
                nodesExpanded = 0;
                isSearchFinished = false;
                this->verbosity = cVerbosity;
                SetHeuristics();
                mutexes = new std::mutex[maxCost * maxCost];
            }

            ~PEMASTAR() { delete[] mutexes; }

            void GetPath(Tstate &, Tstate &, uint8_t = 1);

            void SetHeuristics(Heuristic<Tstate> *hF = nullptr);

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

            void AddStateToQueue(const Tstate &, uint8_t);

            void GetOpenAndDiskData(const Tstate &, uint8_t, BucketData &, DiskState &, int = 0);

            inline void AddStateToQueue(BucketData &openData, DiskState diskState) {
                AddStatesToQueue(openData, &diskState, 1);
            }


            void AddStatesToQueue(const BucketData &, DiskState *, size_t);

            std::string GetOpenName(const BucketData &);

            void ExpandNextFile();

            BucketData GetBestFile();

            bool CanTerminateSearch();

            uint16_t GetBoundValues();

            void ReadAndDDBucket(BucketSet &, const BucketData &);

            void ReadBucket(BucketSet &, const BucketData &);

            void RemoveDuplicates(BucketSet &, const BucketData &);

            void WriteToClosed(BucketSet &, const BucketData &);

            std::string GetClosedName(const BucketData &);

            void ParallelExpandBucket(const BucketData &, BucketSet &, int, int);

            void ReadBucketThread(BucketSet &, const BucketData &, int, int);

            uint16_t GetPathLength() {
                return bestSolution;
            }

            void PrintStates(const BucketData &openData, DiskState *diskState, size_t count) {
                printLock.lock();
                std::cout << "O[" << unsigned(openData.gCost) << "," << unsigned(openData.hCost) << "]\n";
                for (int i = 0; i < count; ++i, diskState++) {
                    std::cout << diskState->bucket << "," << diskState->data << "\n";
                }
                printLock.unlock();
            }

            template<class QueueType>
            void CloseQueue(QueueType queue) {
                for (const auto &s: queue) {
                    if (s.second.f != nullptr) {
                        fclose(s.second.f);
                    }
                }
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

            // Open and closed lists
            std::unordered_map<BucketData, BucketFile, BucketDataHash> openQueue;
            std::unordered_map<BucketData, BucketFile, BucketDataHash> closedQueue;

            // Information about current progress
            uint16_t bestSolution = UINT16_MAX;
            DiskState solutionState;
            std::vector<uint64_t> gDist;
            uint64_t nodesExpanded;
            uint64_t nodesGenerated;
            bool isSearchFinished;
            uint16_t minF;
            uint16_t lb;
            uint16_t lastBucketSize;
            std::map<double, uint64_t> counts;
            std::mutex *mutexes;
            int maxCost = 100;

            // Mutex locks
            std::mutex printLock;
            std::mutex countLock;
            std::mutex openLock;
            std::mutex bestSolutionLock;

            // General
            std::string prefix;
            EMBHS::Verbosity verbosity;
        };

// The main method to get the path. Sets the different values, adds start and goal, and starts expanding.
        template<class Tstate, class Taction, class Tenvironment, class Thash>
        void PEMASTAR<Tstate, Taction, Tenvironment, Thash>::GetPath(Tstate &cStart, Tstate &cGoal, uint8_t eps) {
            this->start = cStart;
            this->goal = cGoal;
            this->isSearchFinished = false;
            this->epsilon = eps;
            // Clear and initialize our tracker of the g-values we encounter during the search
            gDist.clear();
            gDist.resize(256);
            this->nodesExpanded = 0;
            this->nodesGenerated = 0;
            this->lastBucketSize = 0;
            AddStateToQueue(this->start, 0);
            while (!openQueue.empty() && !this->isSearchFinished) {
                ExpandNextFile();
            }
        }

        template<class Tstate, class Taction, class Tenvironment, class Thash>
        void PEMASTAR<Tstate, Taction, Tenvironment, Thash>::SetHeuristics(Heuristic<Tstate> *hF) {
            this->hForward = hF == nullptr ? &this->env : hF;
        }

// Here we want to add a state in its regular representation to our queue. But since regular state and disk state are
// different, we first need to understand wo which bucket it goes (BucketData) and then find its disk representation
// (DiskState). Only then can we add it.
        template<class Tstate, class Taction, class Tenvironment, class Thash>
        void PEMASTAR<Tstate, Taction, Tenvironment, Thash>::AddStateToQueue(const Tstate &state,
                                                                             uint8_t gCost) {
            BucketData openData{};
            DiskState diskState{};
            GetOpenAndDiskData(state, gCost, openData, diskState);
            AddStateToQueue(openData, diskState);
        }


        template<class Tstate, class Taction, class Tenvironment, class Thash>
        void PEMASTAR<Tstate, Taction, Tenvironment, Thash>::GetOpenAndDiskData(const Tstate &state,
                                                                                uint8_t gCost, BucketData &openData,
                                                                                DiskState &diskState, int myThread) {
            diskState.bucket = stateHashers[myThread].HashBucket(state);
            diskState.data = stateHashers[myThread].HashData(state);

            openData.gCost = gCost;
            openData.hCost = hForward->HCost(state, this->goal);

        }

// Adds states to a particular file, given its BucketData.
        template<class Tstate, class Taction, class Tenvironment, class Thash>
        void
        PEMASTAR<Tstate, Taction, Tenvironment, Thash>::AddStatesToQueue(const BucketData &openData,
                                                                         DiskState *diskState,
                                                                         size_t count) {
            int mutexPosition = (openData.gCost % maxCost) + maxCost * (openData.hCost % maxCost);
            openLock.lock();
            auto iter = openQueue.find(openData);
            // If the file does not exist, we want to create it
            if (iter == openQueue.end()) {
                openQueue[openData] = BucketFile(GetOpenName(openData));
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
        std::string PEMASTAR<Tstate, Taction, Tenvironment, Thash>::GetOpenName(const BucketData &openData) {
            std::string s;
            s += this->prefix;
            s += std::to_string(openData.gCost);
            s += "-";
            s += "-";
            s += std::to_string(openData.hCost);
            s += ".open";
            return s;
        }

// Method that is responsible to choose and expand the next file.
// It first checks if the search can terminate. Then it loads the bucket and removes nodes that appear multiple times
// within it and then removes duplicates from previous files. Then it checks for a solution and at the same time expands
// the bucket without checking for duplicates (delayed duplicate-detection).
        template<class Tstate, class Taction, class Tenvironment, class Thash>
        void PEMASTAR<Tstate, Taction, Tenvironment, Thash>::ExpandNextFile() {
            static BucketSet states(0);
            static BucketData next;

            if (CanTerminateSearch())
                return;
            uint64_t prevExpansions = this->nodesExpanded;

            // get best file (minimum f-value)
            BucketData openData = GetBestFile();

            Timer timer;
            timer.StartTimer();

            ReadAndDDBucket(states, openData);

            timer.EndTimer();


            if (verbosity <= EMBHS::lInfo) {
                printLock.lock();
                std::cout << openData << " (" << states.size() << " entries ";
                std::cout << openQueue.find(openData)->second.writtenStates << ") [" << timer.GetElapsedTime()
                          << "s reading/dd]";
                if (verbosity <= EMBHS::lDebug) {
                    std::cout << "\n";
                }
                printLock.unlock();
            }

            openQueue.erase(openQueue.find(openData));

            timer.StartTimer();

            // expand all states in current bucket & write out successors
            const auto numThreads = std::thread::hardware_concurrency();
            std::vector<std::thread *> threads;
            for (size_t x = 0; x < numThreads; x++)
                threads.push_back(
                        new std::thread(&PEMASTAR::ParallelExpandBucket, this, openData, std::ref(states), x,
                                        numThreads));
            for (auto &thread: threads) {
                thread->join();
                delete thread;
            }
            timer.EndTimer();
            if (verbosity <= EMBHS::lInfo) {
                printLock.lock();
                std::cout << "[" << timer.GetElapsedTime() << "s expanding]+";
                printLock.unlock();
            }
            lastBucketSize = this->nodesExpanded - prevExpansions;

        }

        template<class Tstate, class Taction, class Tenvironment, class Thash>
        BucketData PEMASTAR<Tstate, Taction, Tenvironment, Thash>::GetBestFile() {
            BucketData best = (openQueue.begin())->first;
            // We use the uint16_t 2 to convert this entire thing to prevent overflows
            uint16_t bestPriority = best.gCost + best.hCost;
            uint16_t currPriority;
            for (const auto &s: openQueue) {
                currPriority = s.first.gCost + s.first.hCost;
                // Always prefer lower f-values, and tiebreak in favor of lower g-values. This promises we always expand
                //  the parent before the child
                if (currPriority < bestPriority || (currPriority == bestPriority && s.first.gCost < best.gCost)) {
                    bestPriority = currPriority;
                    best = s.first;
                }
            }
            return best;
        }

// This method collects all the minimal values (g,f,d,b) that we need to bound calculations.
        template<class Tstate, class Taction, class Tenvironment, class Thash>
        uint16_t PEMASTAR<Tstate, Taction, Tenvironment, Thash>::GetBoundValues() {
            uint16_t currMinF = UINT16_MAX;
            for (const auto &s: openQueue) {
                if (((uint16_t) s.first.gCost) + s.first.hCost < currMinF)
                    currMinF = ((uint16_t) s.first.gCost) + s.first.hCost;
            }
            return currMinF;
        }

        template<class Tstate, class Taction, class Tenvironment, class Thash>
        bool PEMASTAR<Tstate, Taction, Tenvironment, Thash>::CanTerminateSearch() {
            lb = GetBoundValues(); //minimal f-value in Open
            if (verbosity <= EMBHS::lInfo) {
                std::cout << "S: ";
                if (this->bestSolution == UINT16_MAX)
                    std::cout << "NF";
                else
                    std::cout << unsigned(this->bestSolution);
                std::cout << " LB: " << unsigned(lb) << "\n";
            }

            if (this->bestSolution <= lb) {
                isSearchFinished = true;
                if (verbosity <= EMBHS::lInfo) {
                    printf("Done!\n");
                    printf("fBound: %d; solution: %d\n",
                           lb, bestSolution);
                    printf("%d nodes expanded\n", this->nodesExpanded);

                    Thash hasher;
                    Tstate tmp;
                    hasher.Restore(tmp, solutionState.bucket, solutionState.data);
                    std::cout << "Solution State: " << tmp << std::endl;
                    printf("Distribution:\n");
                    for (int x = 0; x < gDist.size(); x++)
                        if (gDist[x] != 0)
                            printf("%d\t%llu\n", x, gDist[x]);
                }
                return true;
            }
            return false;
        }

        template<class Tstate, class Taction, class Tenvironment, class Thash>
        void
        PEMASTAR<Tstate, Taction, Tenvironment, Thash>::ReadAndDDBucket(BucketSet &states, const BucketData &openData) {
            ReadBucket(states, openData);
            RemoveDuplicates(states, openData); // delayed duplicate detection
            WriteToClosed(states, openData);
        }

// Reading nodes from a file involves both actually reading them but also inserting them into a set. By doing this, we
// get only a single copy of a state even if it appeared multiple times in the file.
        template<class Tstate, class Taction, class Tenvironment, class Thash>
        void PEMASTAR<Tstate, Taction, Tenvironment, Thash>::ReadBucket(BucketSet &states, const BucketData &openData) {
            states.resize(openQueue[openData].writtenStates);
            //const auto numThreads = 1;
            std::vector<std::thread *> threads;
            const int numReadingThreads = (int) std::thread::hardware_concurrency();
            for (size_t x = 0; x < numReadingThreads; x++)
                threads.push_back(new std::thread(&PEMASTAR::ReadBucketThread, this, std::ref(states), openData, x,
                                                  numReadingThreads));
            for (auto &thread: threads) {
                thread->join();
                delete thread;
            }
            remove(GetOpenName(openData).c_str());
        }

        template<class Tstate, class Taction, class Tenvironment, class Thash>
        void PEMASTAR<Tstate, Taction, Tenvironment, Thash>::ReadBucketThread(BucketSet &states,
                                                                              const BucketData &openData,
                                                                              int myThread, int totalThreads) {
            const size_t bufferSize = 1024;

            int64_t toRead = openQueue[openData].writtenStates / totalThreads;
            int64_t readCounter = 0;
            DiskState buffer[bufferSize];

            FILE *f = fopen(openQueue[openData].filename.c_str(), "rb");
            if (f == nullptr) {
                std::cout << "failed to open" << std::endl;
            }

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
        void PEMASTAR<Tstate, Taction, Tenvironment, Thash>::RemoveDuplicates(BucketSet &states,
                                                                              const BucketData &openData) {

            uint8_t diff = 2;
            if (openData.gCost == 0)
                diff = 0;
            if (openData.gCost == 1)
                diff = 1;

            // The h-values stay the same, so we only have to check for the g-values when looking for potential buckets to have
            // duplicates of our current states.
            for (uint8_t depth = openData.gCost - diff; depth < openData.gCost; depth++) {
                BucketData closedData{};
                closedData.gCost = depth;
                closedData.hCost = openData.hCost;

                if (closedQueue.find(closedData) == closedQueue.end()) {
                    continue;
                }
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
        void
        PEMASTAR<Tstate, Taction, Tenvironment, Thash>::WriteToClosed(BucketSet &states, const BucketData &openData) {
            BucketData closedData{};
            closedData.gCost = openData.gCost;
            closedData.hCost = openData.hCost;

            closedQueue[closedData] = BucketFile(GetClosedName(closedData));
            FILE *f = GetFilePointer(closedQueue[closedData].filename, "ab");
            for (const auto &i: states) {
                fwrite(&i, sizeof(DiskState), 1, f);
            }
            fclose(f);
        }

        template<class Tstate, class Taction, class Tenvironment, class Thash>
        std::string PEMASTAR<Tstate, Taction, Tenvironment, Thash>::GetClosedName(const BucketData &closedData) {
            std::string s;
            s += this->prefix;
            s += std::to_string(closedData.gCost);
            s += "-";
            s += std::to_string(closedData.hCost);
            s += ".closed";
            return s;
        }


        template<class Tstate, class Taction, class Tenvironment, class Thash>
        void PEMASTAR<Tstate, Taction, Tenvironment, Thash>::ParallelExpandBucket(const BucketData &openData,
                                                                                  BucketSet &states, int myThread,
                                                                                  int totalThreads) {
            // Controls how many states we agree to store for a particular BucketData before flushing from the cache.
            const int cacheSize = 8192;
            // stores a cache of states, so we will not have to write for every node expansion.
            std::unordered_map<BucketData, std::vector<DiskState>, BucketDataHash> cache;
            // Instead of actually creating the different states we can have a temporary state that we apply an action to,
            // record its BucketData and DiskState, add it to the cache, and then revert the action. This is possible because we
            // are not actually interested in the regular state representation.
            Tstate tmp;
            uint64_t localExpanded = 0;
            uint64_t localGenerated = 0;
            int count = 0;
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
                    BucketData newBucketData{};
                    DiskState newDiskState{};
                    GetOpenAndDiskData(tmp, openData.gCost + 1, newBucketData, newDiskState, myThread);
                    localGenerated++;
                    std::vector<DiskState> &c = cache[newBucketData];
                    c.push_back(newDiskState);
                    if (tmp == goal) {
                        bestSolutionLock.lock();
                        if (openData.gCost + 1 < bestSolution) {
                            bestSolution = openData.gCost + 1;
                        }
                        bestSolutionLock.unlock();
                    }

                    // Flush the cache to the relevant file
                    if (c.size() > cacheSize) {
                        AddStatesToQueue(newBucketData, &c[0], c.size());
                        if (verbosity <= EMBHS::lDebug)
                            PrintStates(newBucketData, &c[0], c.size()); // States logging
                        c.clear();
                    }
                    env.UndoAction(tmp, action);
                }
            }

            // Flush the rest of the cache if there are leftovers.
            for (auto &i: cache) {
                if (!i.second.empty()) {
                    AddStatesToQueue(i.first, &(i.second[0]), i.second.size());
                    if (verbosity <= EMBHS::lDebug)
                        PrintStates(i.first, &(i.second[0]), i.second.size()); // States logging
                }

            }

            countLock.lock();
            nodesExpanded += localExpanded;
            gDist[openData.gCost] += localExpanded;
            counts[lb] += localExpanded;
            nodesGenerated += localGenerated;
            countLock.unlock();
        }
    }
}
#endif //HOG2_SPL_PEMASTAR_H
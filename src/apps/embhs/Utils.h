#ifndef SRC_APPS_EMBHS_UTILS_H
#define SRC_APPS_EMBHS_UTILS_H

#include <string>
#include <vector>
#include <algorithm>
#include <thread>

namespace EMBHS {
    enum Verbosity {
        lNotSet = 0,
        lDebug = 10,
        lInfo = 20,
        lWarn = 30,
        lError = 40,
        lCritical = 50
    };

    class ArgParameters {
    public:
        ArgParameters() {}

        bool IsValid() const {
            if (instanceId < 0)
                return false;
            if (algs.empty() || domain.empty() || heuristic.empty())
                return false;
            if (domain == "STP4" && instanceId + numOfInstances > 100)
                return false;
            if (domain == "STP5" && instanceId + numOfInstances > 50)
                return false;
            return true;
        }

        bool HasAlgorithm(const std::string &alg) const {
            return std::find(algs.begin(), algs.end(), alg) != algs.end();
        }

        void SetVerbosity(const std::string &arg) {
            if (arg == "d" || arg == "debug") { verbosity = EMBHS::lDebug; }
            else if (arg == "i" || arg == "info") { verbosity = EMBHS::lInfo; }
            else if (arg == "w" || arg == "warn") { verbosity = EMBHS::lWarn; }
            else if (arg == "e" || arg == "error") { verbosity = EMBHS::lError; }
            else if (arg == "c" || arg == "critical") { verbosity = EMBHS::lCritical; }
            else { std::cerr << "Unknown verbosity level, defaulting to warn" << std::endl; }
        }

        friend std::ostream &operator<<(std::ostream &os, const ArgParameters &params) {
            os << "ArgParameters { domain: " << params.domain
               << ", heuristic: " << params.heuristic
               << ", algorithms: [";

            for (size_t i = 0; i < params.algs.size(); ++i) {
                os << params.algs[i] << (i < params.algs.size() - 1 ? ", " : "");
            }

            os << "], instanceId: " << params.instanceId
               << ", numOfInstances: " << params.numOfInstances
               << ", pdbDir: " << params.pdbDir
               << ", tempDir: " << params.tempDir
               << ", verbosity: " << params.verbosity
               << ", numThreadsExpansion: " << params.numThreadsExpansion
               << ", numThreadsReading: " << params.numThreadsReading << " }";
            return os;
        }

        std::string domain;
        std::string heuristic;
        std::vector<std::string> algs;
        int instanceId = -1;
        int numOfInstances = 1;
        std::string pdbDir;
        std::string tempDir;
        Verbosity verbosity = EMBHS::lWarn;
        int numThreadsExpansion = (int) std::thread::hardware_concurrency();
        int numThreadsReading = (int) std::thread::hardware_concurrency();
    };
}

#endif //SRC_APPS_EMBHS_UTILS_H

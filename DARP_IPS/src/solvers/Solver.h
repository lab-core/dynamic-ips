//
// Created by Elahe Amiri on 2022-10-26.
//

#ifndef SOLVER_H
#define SOLVER_H


#include "solvers/MasterAlgorithm.h"
#include "../CplexSolver/MIPSolver.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"

//-----------------------------------------------------------------------------
//  solver class
//  Main algorithms to solve the problem in anytime, fixed epoch or static mode
//-----------------------------------------------------------------------------

class Solver {
public:
    float elapsedTime_;
    float avgEpochRuntime_;

    int nbOnePick_;
    int nbTwoPick_;
    int nbThreePick_;
    int nbRecycle_;

    float objValue_;
    float heuristicCG_;
    int greedyRebalanceTime_;

    int epoch_;

    myTools::SharedVector<PLabel> labelsPool_;
    PCG_Algorithm CG_Model_;
    PISUD_Algorithm ISUD_Model_;

    PMIPSolver MIPModel_;
    PGreedyModeler GreedyModel_;
    PSolverOption subProOptions_;
    PRuntimeMetrics runtimeMetrics_;

    myTools::Timer *simulationTime_;
    myTools::Timer *rebalancingTime_;
    myTools::Timer *subProblemTime_;
    myTools::Timer *preprocessTime_;
    Tools::LogOutput* pLogRunTimesStream_;
    Tools::LogOutput* pLogEpochSubRuntimeStream_;
//    Tools::LogOutput* pLogEpochSubRouteStream_;
//    Tools::LogOutput* pLogSolutionChange_;


    Solver(const PInstance & mainInst, InputPaths &inputPaths);

    virtual ~Solver();

    static void selectVehiclesForSubproblem(const PInstance &EpochInst, int iter);

    // this function is to solve the epoch instance with CG using ISUD
    void solveCG_Epoch(PInstance & EpochInst, PInstance & mainInst, InputPaths &inputPaths);
    void solveICG_Epoch(PInstance & EpochInst, PInstance & mainInst, InputPaths &inputPaths);
    void solve_Epoch(PInstance & EpochInst, PInstance & mainInst, InputPaths &inputPaths);

    bool solve_SP_Label(PInstance & EpochInst, PInstance & mainInst, int &iter, int &nbNegativeFound,
        vector2D<PRoute> &availableRoutes, float availableTime, int &nbRoutes);

    bool solve_SP_CPLEX(PInstance & EpochInst, PInstance & mainInst, int &iter, int &nbNegativeFound,
        vector2D<PRoute> &availableRoutes, float availableTime, int &nbRoutes);

    // these functions are to solve the main instance in anytime /dynamic /static mode
    void anyTimeSolver(PInstance & mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);
    void staticSolver(PInstance & mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);
    void dynamicSolver(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);

    // function to print epoch runTime to file
    std::string saveRuntimes(const PInstance & EpochInst);
    std::string saveRuntimesGreedy(const PInstance & EpochInst);

    static void CreateOneStopRoutes(const PVehicle &vehicle, std::vector<PRoute> &availableRoutes, const PInstance & pInst,
                                    const PInstance &EpochInst, int &nbNegative);

    static void updateAvailableRoutes(boost::dynamic_bitset<> &removedRequests, vector2D<PRoute> &availableRoutes);

    void reconstructAvailableRoutes(const PInstance &mainInst, vector2D<PRoute> &availableRoutes);

    void returnVehicles(const PInstance & EpochInst) const;
    void returnVehiclesZone(const PInstance & EpochInst) const;
    void returnVehiclesAssign(const PInstance & EpochInst) const;
    void returnVehiclesAlonsoCplex(const PInstance & EpochInst) const;
    void returnVehiclesAlonso(const PInstance & EpochInst) const;

    std::string toString(const PInstance & mainInst) const;
    void solve(PInstance & mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);
};

// Metrics structure to hold all runtime data
struct RuntimeMetrics {
    // Basic epoch information
    float epochRuntime_;

    // Master problem times
    float masterEpochTime_;
    float RPEpochTime_;
    float CPEpochTime_;
    float RPEpochBuildTime_;
    float CPEpochBuildTime_;
    float isudMIPEpochTime_;

    // Subproblem and preprocessing times
    float subproEpochTime_;
    float GreedyTime_;
    float AssignTime_;

    // Algorithm metrics
    int nbDominated_;                           // number of labels removed via Domination Rules
    int nbGenerated_;                           // number of generated labels
    int nbPrunedArcs_;
    int nbPrunedPath_;                          // number of labels detected as Unreachable by soft time window
    int nbEliminated_ ;                         // number of labels detected as Unreachable due to travel time
    int nbRecycledColumns_;
    int nbNegativeFound_;

    // Constructor with default values
    RuntimeMetrics() : epochRuntime_(0.0f), masterEpochTime_(0), RPEpochTime_(0), CPEpochTime_(0), RPEpochBuildTime_(0),
                       CPEpochBuildTime_(0),isudMIPEpochTime_(0), subproEpochTime_(0.0f), GreedyTime_(0),
                       AssignTime_(0), nbDominated_(0), nbGenerated_(0), nbPrunedArcs_(0),
                       nbPrunedPath_(0),nbEliminated_(0),nbRecycledColumns_(0), nbNegativeFound_(0){}
    void resetSubproblemMetrics();
    void updateSubproblemMetrics(PLabelingSubPro & subProblem );
    void updateSubproblemMetrics(PCplexSubPro & subProblem );
};
#endif //SOLVER_H

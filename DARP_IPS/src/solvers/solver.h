//
// Created by Elahe Amiri on 2022-10-26.
//

#ifndef SOLVER_H
#define SOLVER_H


#include "solvers/MasterAlgorithm.h"
#include "solvers/MIPSolver.h"
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
    float epochRuntime_;
    float SubproEpochTime_;
    float masterEpochTime_;
    float RPEpochTime_;
    float CPEpochTime_;
    float RPEpochBuildTime_;
    float CPEpochBuildTime_;
    float GreedyTime_;
    float AssignTime_;
    float isudMIPEpochTime_;
    int nbDominated_;                           // number of labels removed via Domination Rules
    int nbGenerated_;                           // number of generated labels
    int nbPrunedArcs_;
    int nbPrunedPath_;                  // number of labels detected as Unreachable by soft time window
    int nbEliminated_ ;                    // number of labels detected as Unreachable due to travel time
    int nbRecycledColumns_;
    int nbNegativeFound_;
    int nbOnePick_;
    int nbTwoPick_;
    int nbThreePick_;
    int greedyRebalanceTime_;

    int epoch_;

    myTools::SharedVector<PLabel> labelsPool_;

    std::shared_ptr<MasterAlgorithm> masterModel_;
    std::shared_ptr<MIPSolver> MIPModel_;
    PGreedyModeler GreedyModel_;
    PSolverOption subProOptions_;

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

    void solveCG_Epoch_CPLEX(PInstance &EpochInst, PInstance & mainInst, InputPaths &inputPaths);
    // this function is to solve the main instance in anytime mode
    void anyTimeSolver(PInstance & mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);

    // this function is to solve the main instance in static mode
    void staticSolver(PInstance & mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);
    // this function is to solve the main instance in dynamic mode iteratively with fixed epoch
    void dynamicSolver(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);

    // function to print epoch runTime to file
    std::string saveRuntimes(const PInstance & EpochInst);

    static void CreateOneStopRoutes(const PVehicle &vehicle, std::vector<PRoute> &availableRoutes, const PInstance & pInst,
                                    const PInstance &EpochInst, int &nbNegative);

    static void updateAvailableRoutes(boost::dynamic_bitset<> &removedRequests, vector2D<PRoute> &availableRoutes);

    void returnVehicles(const PInstance & EpochInst) const;
    void returnVehiclesZone(const PInstance & EpochInst) const;
    void returnVehiclesAssign(const PInstance & EpochInst) const;
    void returnVehiclesAlonso(const PInstance & EpochInst) const;

    // Display results
    std::string toString(const PInstance & mainInst) const;
    void solve(PInstance & mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);
};


#endif //SOLVER_H

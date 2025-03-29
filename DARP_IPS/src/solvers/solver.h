//
// Created by Elahe Amiri on 2022-10-26.
//

#ifndef SOLVER_H
#define SOLVER_H

#include "data/Instance.h"
#include "solvers/MasterAlgorithm.h"
#include "solvers/CPLEXSubProblem.h"
#include "solvers/LabelingSubProblem.h"
#include "solvers/MIPSolver.h"
#include "solvers/GreedyModeler.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"
extern vector2D<float> durationMatrix_;
// extern Tools::PThreadsPool pPool;

//-----------------------------------------------------------------------------
//  solver class
//  Main algorithms to solve the problem in anytime, fixed epoch or static mode
//-----------------------------------------------------------------------------

class solver {
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

    int epoch_;

    myTools::SharedVector<PLabel> labelsPool_;

    std::shared_ptr<MasterAlgorithm> masterModel_;
    std::shared_ptr<MIPSolver> MIPModel_;
    PGreedyModeler GreedyModel_;
    PSolverOption subProOptions_;

    myTools::Timer *simulationTime_;
    myTools::Timer *subProblemTime_;
    myTools::Timer *preprocessTime_;
    Tools::LogOutput* pLogRunTimesStream_;
    Tools::LogOutput* pLogEpochSubRuntimeStream_;
//    Tools::LogOutput* pLogEpochSubRouteStream_;
//    Tools::LogOutput* pLogSolutionChange_;


    solver(PInstance & mainInst, InputPaths &inputPaths);

    virtual ~solver();

    void selectVehiclesForSubproblem(PInstance &EpochInst, int iter);

    // this function is to solve the epoch instance with CG using ISUD
    void solveCG_Epoch(PInstance & EpochInst, PInstance & mainInst, InputPaths &inputPaths);
    // this function is to solve the main instance in anytime mode
    void anyTimeSolver(PInstance & mainInst, InputPaths &inputPaths, std::string& instNum, bool middleSave, float saveTime);

    // this function is to solve the main instance in static mode
    void staticSolver(PInstance & mainInst, InputPaths &inputPaths, std::string& instNum, bool middleSave, float saveTime);
    // this function is to solve the main instance in dynamic mode iteratively with fixed epoch
    void dynamicSolver(PInstance & mainInst, InputPaths &inputPaths, std::string instNum, bool middleSave, float saveTime);

    // function to print epoch runTime to file
    std::string saveRuntimes(PInstance & EpochInst);

    void CreateOneStopRoutes(PVehicle &vehicle, std::vector<PRoute> &availableRoutes, PInstance & pInst,
                             PInstance &EpochInst, int &nbNegative);

    void updateAvailableRoutes(std::bitset<MAX_BIT_SIZE> &removedRequests, vector2D<PRoute> &availableRoutes);

    void returnVehicles(PInstance & EpochInst);
    void returnVehiclesZone(PInstance & EpochInst);

    // Display results
    std::string toString(PInstance & mainInst) const;
};


#endif //SOLVER_H

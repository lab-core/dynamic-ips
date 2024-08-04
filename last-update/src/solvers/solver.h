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
    double elapsedTime_;
    double avgEpochRuntime_;
    double epochRuntime_;
    double SubproEpochTime_;
    double masterEpochTime_;
    double RPEpochTime_;
    double CPEpochTime_;
    double RPEpochBuildTime_;
    double CPEpochBuildTime_;
    double GreedyTime_;
    double AssignTime_;
    double isudMIPEpochTime_;
    int nbDominated_;                           // number of labels removed via Domination Rules
    int nbGenerated_;                           // number of generated labels
    int nbUnreachableDelay_;                  // number of labels detected as Unreachable by soft time window
    int nbUnreachableDTrip_;                    // number of labels detected as Unreachable due to travel time
    int nbNegativeFound_;

    int epoch_;

    myTools::SharedVector<PLabel> labelsPool_;

    std::shared_ptr<MasterAlgorithm> masterModel_;
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

    // this function is to solve the epoch instance with CG using ISUD
    void solveCG_Epoch(PInstance & EpochInst, PInstance & mainInst, InputPaths &inputPaths);
    void solveCG_Epoch1(PInstance & EpochInst, PInstance & mainInst, InputPaths &inputPaths);

    // this function is to solve the main instance in anytime mode
    void anyTimeSolver(PInstance & mainInst, InputPaths &inputPaths, std::string& instNum, bool middleSave, float saveTime);

    void anyTimeSolverEvent(PInstance & mainInst, InputPaths &inputPaths);

    // this function is to solve the main instance in static mode
    void staticSolver(PInstance & mainInst, InputPaths &inputPaths, std::string& instNum, bool middleSave, float saveTime);
    // this function is to solve the main instance in dynamic mode iteratively with fixed epoch
    void dynamicSolver(PInstance & mainInst, InputPaths &inputPaths, std::string instNum, bool middleSave, float saveTime);

    // function to print epoch runTime to file
    std::string saveRuntimes(PInstance & EpochInst);


    // Display results
    std::string toString(PInstance & mainInst) const;
};


#endif //SOLVER_H

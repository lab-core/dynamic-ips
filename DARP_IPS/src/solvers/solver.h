//
// Created by Elahe Amiri on 2022-10-26.
//

#ifndef SOLVER_H
#define SOLVER_H

#include "data/Instance.h"
#include "solvers/ISUDAlgorithm.h"
#include "solvers/CPLEXSubProblem.h"
#include "solvers/LabelingSubProblem.h"
#include "solvers/MIPSolver.h"
#include "solvers/GreedyModeler.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"

// extern Tools::PThreadsPool pPool;

class solver {
public:
    double elapsedTime_;
    double avgEpochRuntime_;
    double epochRuntime_;
    double SubproEpochTime_;
    double isudEpochTime_;
    double RPEpochTime_;
    double CPEpochTime_;
    double isudMIPEpochTime_;
    int epoch_;

    std::shared_ptr<ISUDAlgorithm> isudObj_;
    PGreedyModeler GreedyModel_;
    PSolverOption subProOptions_;

    myTools::Timer *simulationTime_;
    myTools::Timer *subProblemTime_;
    myTools::Timer *preprocessTime_;
    Tools::LogOutput* pLogRunTimesStream_;
    Tools::LogOutput* pLogEpochSolutionStream_;


    solver(PInstance & mainInst, InputPaths &inputPaths);

    virtual ~solver();

    // this function is to solve the epoch instance with CG using ISUD
    void solveCG_ISUD(PInstance & EpochInst, InputPaths &inputPaths);

    // this function is to solve the main instance in dynamic mode iteratively with fixed epoch
    void dynamicSolver(PInstance & mainInst, InputPaths &inputPaths);

    // this function is to solve the main instance in anytime mode
    void anyTimeSolver(PInstance & mainInst, InputPaths &inputPaths);

    // this function is to solve the main instance in static mode
    void staticSolver(PInstance & mainInst, InputPaths &inputPaths, std::string instNum, bool middleSave, float saveTime);

    // function to print epoch runTime to file
    void saveRuntimes(PInstance & EpochInst, const std::string& EpochRunTimeDir);
    std::string saveRuntimes(PInstance & EpochInst);


    // Display results
    std::string toString(PInstance & mainInst) const;
};


#endif //SOLVER_H

//
// Created by Elahe Amiri on 2022-10-26.
//

#ifndef SOLVER_H
#define SOLVER_H

#include "data/Instance.h"
#include "solver/MasterAlgorithm.h"
#include "solver/LabelingSubProblem.h"
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
    double isudMIPEpochTime_;
    int nbDominated_;                           // number of labels removed via Domination Rules
    int nbEliminated_;                          // number of labels removed via Elimination Rules
    int nbGenerated_;                           // number of generated labels
    int nbNegativeFound_;
    int epoch_;

    myTools::SharedVector<PLabel> labelsPool_;

    std::shared_ptr<MasterAlgorithm> masterModel_;
    PSolverOption subProOptions_;

    myTools::Timer *simulationTime_;
    myTools::Timer *subProblemTime_;
    myTools::Timer *preprocessTime_;


    solver(PInstance & mainInst, InputPaths &inputPaths);

    virtual ~solver();

    // this function is to solve the epoch instance with CG using ISUD
    void solveCG_Epoch(PInstance & EpochInst, PInstance & mainInst, InputPaths &inputPaths);
};


#endif //SOLVER_H

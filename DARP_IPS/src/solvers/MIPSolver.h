//
// Created by Ella on 2021-10-18.
//

#ifndef MIPSOLVER_H
#define MIPSOLVER_H

#include "data/Instance.h"
#include "utilities/InputPaths.h"
#include "utilities/MyTools.h"

//-----------------------------------------------------------------------------
//  Contains MIP formulation to solve with Cplex
//  This is just for testing the results and comparison
//-----------------------------------------------------------------------------

class MIPSolver {
public:
    IloEnv env_;
    IloModel Model_;
    IloCplex Cplex_;
    IloRangeArray constraints_;


    // Variables
    IloNumVar3D X_;
    IloNumVar2D U_;
    IloNumVar2D W_;
    IloNumVarArray Z_;

    float objValue_;
    int nbNodes_;
    int nbRequests_;
    int nbVehicles_;

    myTools::Timer *solveTime_;                             // time to solve the model
    // Solution containers
    std::vector<PRoute> routeSolution_;
    std::vector<PRequest> zSolution_;

    MIPSolver();

    virtual ~MIPSolver();

    void initializeModel(PInstance &pInst);
    void buildModel(PInstance &pInst);
    void solveModel(PInstance &pInst, InputPaths &inputPaths);
    void SolveMIP(PInstance &pInst, InputPaths &inputPaths);

};

#endif //MIPSOLVER_H

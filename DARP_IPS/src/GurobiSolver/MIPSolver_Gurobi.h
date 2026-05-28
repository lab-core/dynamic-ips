//
// Gurobi port of MIPSolver_Cplex
//

#ifndef GUROBI_MIPSOLVER_H
#define GUROBI_MIPSOLVER_H

#include "data/Instance.h"
#include "utilities/InputPaths.h"
#include "utilities/MyTools.h"
#include "gurobi_c++.h"
#include <vector>

//-----------------------------------------------------------------------------
//  Contains MIP formulation to solve with Gurobi.
//  This is the master MIP — same model as MIPSolver_Cplex but using the Gurobi API.
//-----------------------------------------------------------------------------

class MIPSolver_Gurobi {
public:
    // Reference to the process-wide Gurobi env (see solvers/SolverEnv.h).
    GRBEnv&  env_;
    GRBModel Model_;
    std::vector<GRBConstr> constraints_;

    // Variables
    std::vector<std::vector<std::vector<GRBVar>>> X_;   // X_[v][i][j]
    std::vector<std::vector<GRBVar>>              U_;   // U_[v][i]
    std::vector<std::vector<GRBVar>>              W_;   // W_[v][i]
    std::vector<GRBVar>                           Z_;   // Z_[i]

    float objValue_;
    int   nbNodes_;
    int   nbRequests_;
    int   nbVehicles_;

    myTools::Timer *solveTime_;                          // time to solve the model

    // Solution containers
    std::vector<PRoute>   routeSolution_;
    std::vector<PRequest> zSolution_;

    MIPSolver_Gurobi();
    virtual ~MIPSolver_Gurobi();

    void initializeModel(PInstance &pInst);
    void buildModel(PInstance &pInst);
    void configureGurobi();
    void solveModel(PInstance &pInst, InputPaths &inputPaths);
    void diagnoseInfeasibility();
    void extractSolution(PInstance &pInst);
    void SolveMIP(PInstance &pInst, InputPaths &inputPaths);
};

#endif // GUROBI_MIPSOLVER_H

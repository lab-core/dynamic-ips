//
// Gurobi port of SubProblem_Cplex
//

#ifndef _GUROBI_SUBPROBLEM_H
#define _GUROBI_SUBPROBLEM_H

#include "data/Instance.h"
#include "solvers/SubproModeler.h"
#include "gurobi_c++.h"
#include <vector>

//-----------------------------------------------------------------------------
//  SubProblem_Gurobi class
//  Pricing subproblem solver (Figure 3 of the paper) using Gurobi.
//-----------------------------------------------------------------------------
class SubProblem_Gurobi : public SubproModeler {
public:

    // Reference to the process-wide Gurobi env (see solvers/SolverEnv.h).
    // Subproblems are created once per vehicle per epoch — sharing the env
    // here is the single biggest license-acquisition reduction in the code.
    GRBEnv&  env_;
    GRBModel SubProModel_;

    // Decision variables
    std::vector<std::vector<GRBVar>> X_;   // arc usage
    std::vector<GRBVar>              U_;   // arrival times
    std::vector<GRBVar>              W_;   // vehicle load

    int nbNodes_;
    int nbRequests_;

    double bestObjectiveValue_;  // best objective (reduced cost) found
    bool   solutionFound_;       // true once an incumbent exists

    // Constructor / destructor
    explicit SubProblem_Gurobi(PVehicle &vehicle);
    ~SubProblem_Gurobi() override = default;

    // Display
    std::string toString() const;


    // Solution extraction
    void extractPoolSolutions(PInstance &pInst, std::vector<PRoute> &availableRoutes);
    void extractSingleSolution(PInstance &pInst, std::vector<PRoute> &availableRoutes, int solutionIndex);

    // Accessors
    double getBestObjectiveValue() const { return bestObjectiveValue_; }
    bool   hasSolution()           const { return solutionFound_; }

    // Solver configuration
    void configureGurobi();
    void configurePool();

    // Build and solve the subproblem with Gurobi
    void solveSP(PInstance &pInst, std::vector<PRoute> &availableRoutes);
private:
    void initializeModel(PInstance &pInst);
    void buildModel(PInstance &pInst);
    void solveModel(PInstance &pInst, std::vector<PRoute> &availableRoutes);
};

typedef std::shared_ptr<SubProblem_Gurobi> PGurobiSubPro;

#endif // _GUROBI_SUBPROBLEM_H

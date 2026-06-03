//
// Created by Ella on 2021-10-13.
//

#ifndef _SUBPROBLEM_H
#define _SUBPROBLEM_H

#include "data/Instance.h"
#include "solvers/SubproModeler.h"
#include <ilcplex/ilocplex.h>

//-----------------------------------------------------------------------------
//  SubProblem_Cplex class
//  Algorithms for solving the subProblems and getting results
//-----------------------------------------------------------------------------
using IloNumVar2D = IloArray<IloNumVarArray>;
using IloNumVar3D = IloArray<IloNumVar2D>;

class SubProblem_Cplex : public SubproModeler{
public:
    // defining objects on the CPLEX model
    IloEnv env_;
    IloModel SubProModel_;
    IloCplex Cplex_;

    IloNumVar2D X_;
    IloNumVarArray U_;
    IloNumVarArray W_;

    int nbNodes_;
    int nbRequests_;

    double bestObjectiveValue_;  // Store the best objective value found
    bool solutionFound_;         // Flag to check if solution was found

    // Constructor and Destructor
    explicit SubProblem_Cplex(PVehicle &vehicle);

    ~SubProblem_Cplex() override;

    // Display
    std::string toString() const;

    // Solution extraction
    void extractPoolSolutions(PInstance &pInst, std::vector<PRoute> &availableRoutes);
    void extractSingleSolution(PInstance &pInst, std::vector<PRoute> &availableRoutes, int solutionIndex);

    // Accessors
    double getBestObjectiveValue() const { return bestObjectiveValue_; }
    bool hasSolution() const { return solutionFound_; }


    // Solver configuration
    void configureCplex();
    void configurePool();

    // Build and solve the subProblem with CPLEX
    void solveSP(PInstance &pInst, std::vector<PRoute> &availableRoutess);
private:
    void initializeModel(PInstance &pInst);
    void buildModel(PInstance &pInst);
    void solveModel(PInstance &pInst, std::vector<PRoute> &availableRoutess);

};
typedef std::shared_ptr<SubProblem_Cplex> PCplexSubPro;

#endif //_SUBPROBLEM_H

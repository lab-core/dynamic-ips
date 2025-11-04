//
// Created by Ella on 2021-10-13.
//

#ifndef _SUBPROBLEM_H
#define _SUBPROBLEM_H

#include "data/Instance.h"
#include "solvers/SubproModeler.h"
#include <ilcplex/ilocplex.h>

//-----------------------------------------------------------------------------
//  CPLEXSubProblem class
//  Algorithms for solving the subProblems and getting results
//-----------------------------------------------------------------------------
using IloNumVar2D = IloArray<IloNumVarArray>;
using IloNumVar3D = IloArray<IloNumVar2D>;

class CPLEXSubProblem : public SubproModeler{
public:

    int nbGenerated_;

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
    explicit CPLEXSubProblem(PVehicle &vehicle);

    ~CPLEXSubProblem() override;

    // Build and solve the subProblem with CPLEX
    void initializeModel(PInstance &pInst);
    void buildModel(PInstance &pInst);
    void solveModel(PInstance &pInst, std::vector<PRoute> &availableRoutess);
    void solveSP(PInstance &pInst, std::vector<PRoute> &availableRoutess);

    // Display function
    std::string toString() const;
    void extractPoolSolutions(PInstance &pInst, std::vector<PRoute> &availableRoutes);
    void extractSingleSolution(PInstance &pInst, std::vector<PRoute> &availableRoutes, int solutionIndex);
    double getBestObjectiveValue() const { return bestObjectiveValue_; }
    bool hasSolution() const { return solutionFound_; }
    void solveForColumnGeneration(PInstance &pInst, std::vector<PRoute> &availableRoutes);
    void setInitialIncumbent();
    std::string toStringOut(int epoch) const;


    void addSimpleMIPStart();

    void addSmartMIPStart();

};
typedef std::shared_ptr<CPLEXSubProblem> PCplexSubPro;

#endif //_SUBPROBLEM_H

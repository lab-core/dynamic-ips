//
// Created by Ella on 2021-10-13.
//

#ifndef _SUBPROBLEM_H
#define _SUBPROBLEM_H

#include "data/Instance.h"
#include "solvers/SubproModeler.h"
#include "solvers/CPLEXModeler.h"

//-----------------------------------------------------------------------------
//  CPLEXSubProblem class
//  Algorithms for solving the subProblems and getting results
//-----------------------------------------------------------------------------


class CPLEXSubProblem : public SubproModeler{
public:

    // defining objects on the CPLEX model
    IloEnv env_;
    IloModel SubProModel_;
    IloCplex SubProbCplex_;

    IloNumVar2D X;
    IloNumVarArray U;
    IloNumVarArray W;


    // Constructor and Destructor
    explicit CPLEXSubProblem(PVehicle &vehicle);

    ~CPLEXSubProblem() override;

    // Build and solve the subProblem with CPLEX
    void BuildModelCPLEX(int maxPickUp);
    void SolveCPLEX();

    // function to convert solution to routes and save them in vehicle object
    void SolutionToRoutes(std::vector<PRoute> &availableRoutes);

    // Display function
    std::string toString() const;

};
typedef std::shared_ptr<CPLEXSubProblem> PCplexSubPro;

#endif //_SUBPROBLEM_H

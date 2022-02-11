//
// Created by Ella on 2021-10-13.
//

#ifndef _SUBPROBLEM_H
#define _SUBPROBLEM_H

#include "data/Instance.h"
#include "solvers/CPLEXModeler.h"
#include "utilities/MyTools.h"
//-----------------------------------------------------------------------------
//  CPLEXSubProblem class
//  Algorithms for solving the subProblems and getting results
//-----------------------------------------------------------------------------


class CPLEXSubProblem {
public:
    PVehicle* Vehicle_;                     // the vehicle for which we are solving the sub problem
    PGraph subGraph_;                       // the graph of the feasible solution for the vehicle
    std::vector<PRequest> subRequests_;     // List of requests

    // defining objects on the CPLEX model
    IloEnv env_;
    IloModel SubProModel_;
    IloCplex SubProbCplex_;

    IloNumVar2D X;
    IloNumVarArray U;
    IloNumVarArray W;


    // Constructor and Destructor
    CPLEXSubProblem(PVehicle &vehicle);

    virtual ~CPLEXSubProblem();

    // calculation of penalties and initialization of the subgraph
    void initSubGraph(PInstance &pInst, SubSolveStatus status);

    // Build and solve the subProblem with CPLEX
    void BuildModelCPLEX(IloNumArray& requestDuals, IloNum& vehicleDual, std::map<int, int>& requestToOrder, int maxPickUp);
    void SolveCPLEX();

    // function to convert solution to routes and save them in vehicle object
    void SolutionToRoutes(std::vector<PRoute> &availableRoutes, std::map<std::string , PRoute> &generatedRoutes);

    // Display function
    std::string toString() const;

};
typedef std::shared_ptr<CPLEXSubProblem> PCPLEXsubPro;


#endif //_SUBPROBLEM_H

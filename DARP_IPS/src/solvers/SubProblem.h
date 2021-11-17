//
// Created by Ella on 2021-10-13.
//

#ifndef _SUBPROBLEM_H
#define _SUBPROBLEM_H

#include "data/Instance.h"
#include "solvers/CPLEXModeler.h"
#include "utilities/MyTools.h"
//-----------------------------------------------------------------------------
//  SubProblem class
//  Algorithms for solving the subProblems and getting results
//-----------------------------------------------------------------------------


class SubProblem {
public:
    PVehicle* Vehicle_;                     // the vehicle for which we are solving the sub problem
    PGraph subGraph_;                       // the graph of the feasible solution for the vehicle
    std::vector<PRequest> subRequests_;     // List of requests
    int numRoutes_;                         // number of routes found that match conditions
    float bestReducedCost_;                 // best reduced cost found
    std::vector<PRoute> generatedRoutes_;   // list of generated routes after solving

    // defining objects on the CPLEX model
    IloEnv env_;
    IloModel SubProModel_;
    IloCplex SubProbCplex_;

    IloNumVar2D X;
    IloNumVarArray U;
    IloNumVarArray W;


    // Constructor and Destructor
    SubProblem(PVehicle &vehicle);

    virtual ~SubProblem();

    // calculation of penalties and initialization of the subgraph
    void initSubGraph(PInstance &pInst, int epoch);

    // Build and solve the subProblem with CPLEX
    void BuildModelCPLEX(IloNumArray& requestDuals, IloNum& vehicleDual, std::map<int, int>& requestToOrder);
    void SolveCPLEX();

    // function to convert solution to routes and save them in vehicle object
    void SolutionToRoutes(std::vector<PRoute> &availableRoutes, std::map<std::string , PRoute> &generatedRoutes);

    // Display function
    std::string toString() const;

};
typedef std::shared_ptr<SubProblem> PSubProblem;


#endif //_SUBPROBLEM_H

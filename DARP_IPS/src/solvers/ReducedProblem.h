//
// Created by Ella on 2021-11-10.
//

#ifndef DARP_IPS_REDUCEDPROBLEM_H
#define DARP_IPS_REDUCEDPROBLEM_H

#include "utilities/MyTools.h"
#include "solvers/CPLEXModeler.h"
#include "data/Route.h"
#include "data/Instance.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------


class ReducedProblem {
public:
    IloEnv env_;
    IloModel RPModel_;
    IloCplex RPCplex_;
    IloObjective reducedObj_;

    // Variables
    IloNumVarArray routeVar_;
    IloNumVarArray zVar_;

    // dual costs
    IloNumArray requestDuals_;
    IloNumArray vehicleDuals_;

    // right-hand-side of constraints
    IloNumArray requestRHS_;
    IloNumArray vehicleRHS_;

    // set of constraints
    IloRangeArray requestConst_;
    IloRangeArray vehicleConst_;

    vector<int> orderToRequest_;
    std::map<int, int> requestToOrder_;

    std::vector<PRoute> routesToAdd_;

    // Constructor and Destructor
    ReducedProblem(PInstance &pInst);
    virtual ~ReducedProblem();

    // this function reset the model based the current set of routes and changed the set of constraints (size)
    void updateRequestOrder(PInstance &pInst);

    // this function clear all objects from the model at the start of each epoch
    void clearModel(PInstance &pInst);

    // this function initialized the model and define empty set of constraints
    void initializeModel(PInstance &pInst);

    // this function adds routeVar to the model
    void addRouteVar(PRoute &newRoute);

    // this function adds zVar to the model used for the routes that served only one request
    void addZVar(PRequest &request);

    // this function add one route at each iteration of the algorithm during one epoch
    void updateModel(PInstance &pInst, std::vector<PRoute> &routeSolution);

    // this function build the model at the start of each epoch
    void buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution);

    // this function solve the model and remove all columns except than the current base
    void solveModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    std::map<std::string , PRoute> &generatedRoutes);

    // Display function
    std::string toString() const;
};


#endif //DARP_IPS_REDUCEDPROBLEM_H

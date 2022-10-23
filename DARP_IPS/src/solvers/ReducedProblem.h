//
// Created by Ella on 2021-11-10.
//

#ifndef DARP_IPS_REDUCEDPROBLEM_H
#define DARP_IPS_REDUCEDPROBLEM_H

#include "solvers/MasterModeler.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------


class ReducedProblem : public MasterModeler{
public:
    // Variables
    IloNumVarArray routeVar_;
    IloNumVarArray zVar_;

    std::vector<PRoute> compRoutes_;

    // Constructor and Destructor
    ReducedProblem();


    // this function initialized the model and define empty set of constraints
    void ResetRPModel();

    // this function adds routeVar to the model
    void addRouteVar(PRoute &newRoute);

    // this function adds zVar to the model used for the routes that served only one request
    void addZVar(PRequest &request);

    // this function add one route at each iteration of the algorithm during one epoch
    void updateModel(PInstance &pInst, std::vector<PRoute> &routeSolution);

    // this function build the model at the start of each epoch
    void buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution);

     // this function solve the model and remove all columns except than the current base
     virtual void solveModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    std::unordered_map<std::string , PRoute> &generatedRoutes);

    // function to check whether two routes are column disjoint or not
    static bool isColumnDisjoint(std::vector<PRoute> &routeSet, PRoute &newRoute, std::unordered_map<unsigned int, int>& requestToOrder);

    // function to check whether the route is repeated before
    static bool isColumnRepeat(std::vector<PRoute> &routeSet, PRoute &newRoute, std::unordered_map<unsigned int, int>& requestToOrder);

    // Display function
    std::string toString() const override;
    void restartRP();
};


#endif //DARP_IPS_REDUCEDPROBLEM_H
